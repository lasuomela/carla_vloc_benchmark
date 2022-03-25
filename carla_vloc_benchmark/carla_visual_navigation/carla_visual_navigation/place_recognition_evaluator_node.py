import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import message_filters
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

import os
import json

from std_msgs.msg import ColorRGBA, Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker
from carla_visual_navigation_interfaces.srv import LogScenarioResults
from visual_localization_interfaces.msg import KBestPRMatches

from carla_visual_navigation.geometry_utils import distance_from_pose

from visual_robot_localization.coordinate_transforms import SensorOffsetCompensator

class PlaceRecognitionEvaluator(Node):

    def __init__(self):
        super().__init__('place_recognition_evaluator')
        self.declare_parameter("odometry_topic", "/carla/ego_vehicle/odometry")
        odometry_topic_name = self.get_parameter('odometry_topic').get_parameter_value().string_value

        self.declare_parameter("place_reg_pose_topic", '/carla/ego_vehicle/place_reg_loc')
        pose_publish_topic = self.get_parameter('place_reg_pose_topic').get_parameter_value().string_value

        self.declare_parameter("place_reg_markers_topic", '/carla/ego_vehicle/place_reg_markers')
        pr_loc_marker_topic_name = self.get_parameter('place_reg_markers_topic').get_parameter_value().string_value

        self.declare_parameter("synchrony_diff", 0.01)
        synchrony_diff = self.get_parameter('synchrony_diff').get_parameter_value().double_value

        self.declare_parameter("compensate_sensor_offset", True)
        self.compensate_sensor_offset = self.get_parameter('compensate_sensor_offset').get_parameter_value().bool_value

        self.declare_parameter("base_frame", "ego_vehicle")
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.declare_parameter("sensor_frame", "ego_vehicle/rgb_front")
        sensor_frame = self.get_parameter('sensor_frame').get_parameter_value().string_value

        self.odometry_subscription = message_filters.Subscriber(self, Odometry, odometry_topic_name)
        self.pr_loc_subscription = message_filters.Subscriber(self, KBestPRMatches, pose_publish_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.odometry_subscription, self.pr_loc_subscription], 10, synchrony_diff)
        self.ts.registerCallback(self.localization_callback)

        self.log_srv = self.create_service(LogScenarioResults, "log_scenario_results", self.scenario_finished_callback)

        self.publisher = self.create_publisher(Marker, pr_loc_marker_topic_name, 10)

        self.accuracy_categories_meters = [{'lim_t': 25, 'true_count':0, 'false_count':0},
                                           {'lim_t': 10, 'true_count':0, 'false_count':0},
                                           {'lim_t': 5, 'true_count':0, 'false_count':0},
                                           {'lim_t': 2, 'true_count':0, 'false_count':0}]

        if self.compensate_sensor_offset:
            self.sensor_offset_compensator = SensorOffsetCompensator(base_frame, sensor_frame, True)

    def localization_callback(self, odometry_msg, top_k_gallery_matches_msg):
        self.publish_place_recognition_poses(top_k_gallery_matches_msg)
        self.update_stats(odometry_msg, top_k_gallery_matches_msg)

    def scenario_finished_callback(self, request, response):
        scenario_info_msg = request.scenario_results
        self.save_stats( scenario_info_msg )
        response.logging_result = True
        return response
                
    def save_stats(self, scenario_info_msg):

        scenario_info_dict = message_to_ordereddict(scenario_info_msg)

        print('Top 1 recall:')
        for accuracy_category in self.accuracy_categories_meters:
            prct = accuracy_category['true_count']/(accuracy_category['true_count']+accuracy_category['false_count'])

            category_string = 'Top1 recall @{}m: {:.3f}'.format(accuracy_category['lim_t'], prct)
            print(category_string)

            scenario_info_dict[category_string] = prct

        if scenario_info_dict['log_results']:
            result_list = []
            if os.path.exists(scenario_info_dict['log_file_path']):
                if (os.stat(scenario_info_dict['log_file_path']).st_size != 0):
                    with open(scenario_info_dict['log_file_path'], 'r+') as f:
                        result_list = json.load(f)

            result_list.append(scenario_info_dict)
            scenario_info_dict.update({'idx':len(result_list)})

            with open(scenario_info_dict['log_file_path'], 'w') as f:
                json.dump(result_list, f, indent=2)

    def update_stats(self, odometry_msg, top_k_gallery_matches_msg):
        if self.compensate_sensor_offset:
            odometry_msg_offset_corrected = odometry_msg
            odometry_msg_offset_corrected.pose.pose = self.sensor_offset_compensator.add_offset(odometry_msg_offset_corrected.pose.pose)
            odometry_msg = odometry_msg_offset_corrected

        odometry_pose = odometry_msg.pose.pose
        top1_pose = top_k_gallery_matches_msg.locations.poses[0]

        dist = distance_from_pose(odometry_pose, top1_pose)
        print('Diff T: {:.3f} m'.format(dist))
        for accuracy_category in self.accuracy_categories_meters:
            if dist <= accuracy_category['lim_t']:
                accuracy_category['true_count'] = accuracy_category['true_count']+ 1
            else:
                accuracy_category['false_count'] = accuracy_category['false_count']+ 1 

    def publish_place_recognition_poses(self, top_k_gallery_matches_msg):
        top_k_dict = message_to_ordereddict(top_k_gallery_matches_msg)
        poses = top_k_dict['locations']['poses']

        header = Header()
        set_message_fields(header, top_k_dict['header'])
        marker = Marker(header=header, scale=Vector3(x=1.0,y=1.0,z=1.0), type=8, action=0, color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
        for i, pose in enumerate(poses):
            if i == 0:
                color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            else:
                color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

            marker.colors.append(color)

            point = Point()
            set_message_fields(point, pose['position'])
            marker.points.append(point)

        self.publisher.set_data(marker)


def main(args=None):
    rclpy.init(args=args)

    executor = SingleThreadedExecutor()
    evaluator = PlaceRecognitionEvaluator()
    executor.add_node(evaluator)
    executor.spin()

    evaluator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
