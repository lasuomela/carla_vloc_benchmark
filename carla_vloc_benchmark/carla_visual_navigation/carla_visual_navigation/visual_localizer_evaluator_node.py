import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import message_filters
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

import transforms3d as t3d
import numpy as np
import os
import json
import time
from matplotlib import cm

from std_msgs.msg import ColorRGBA, Header, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, PoseStamped, Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
from carla_visual_navigation_interfaces.msg import ScenarioInfo
from carla_visual_navigation_interfaces.srv import LogScenarioResults
from visual_localization_interfaces.msg import VisualPoseEstimate

from carla_visual_navigation.geometry_utils import distance_from_pose, angular_distance_from_pose
from visual_robot_localization.visual_localizer_node import VisualLocalizer 


class VisualLocalizerEvaluator(Node):

    def __init__(self, pub=None):
        super().__init__('visual_localizer_evaluator')
        self.declare_parameter("odometry_topic", "/carla/ego_vehicle/odometry")
        odometry_topic_name = self.get_parameter('odometry_topic').get_parameter_value().string_value

        self.declare_parameter("true_pose_publish_topic", "/carla/ego_vehicle/true_pose")
        true_pose_publish_topic = self.get_parameter('true_pose_publish_topic').get_parameter_value().string_value

        self.declare_parameter("visual_pose_publish_topic", "/carla/ego_vehicle/visual_pose_estimate_visualization")
        visual_pose_publish_topic = self.get_parameter('visual_pose_publish_topic').get_parameter_value().string_value

        self.declare_parameter("place_recognition_publish_topic", "/carla/ego_vehicle/place_recognition_visualization")
        place_recognition_publish_topic = self.get_parameter('place_recognition_publish_topic').get_parameter_value().string_value

        self.declare_parameter("visual_pose_topic", '/carla/ego_vehicle/visual_pose_estimate')
        visual_pose_topic_name = self.get_parameter('visual_pose_topic').get_parameter_value().string_value

        self.declare_parameter("synchrony_diff", 0.01)
        synchrony_diff = self.get_parameter('synchrony_diff').get_parameter_value().double_value

        self.odometry_subscription = message_filters.Subscriber(self, Odometry, odometry_topic_name)
        self.vis_loc_subscription = message_filters.Subscriber(self, VisualPoseEstimate, visual_pose_topic_name)

        # With odometry frequency 20Hz queue size 100 corresponds to searching up to 5 sec old odometry messages (in simulation time)
        # If the localization algorith is very slow, you might need to increase this
        queue_size = 100
        self.ts = message_filters.ApproximateTimeSynchronizer([self.odometry_subscription, self.vis_loc_subscription], queue_size, synchrony_diff)
        self.ts.registerCallback(self.localization_callback)

        self.latest_vloc_time_ros = None
        self.latest_vloc_walltime = None

        self.log_srv = self.create_service(LogScenarioResults, "log_scenario_results", self.scenario_finished_callback)

        self.true_pose_publisher = self.create_publisher(PoseStamped, true_pose_publish_topic, 10)
        self.place_recognition_publisher = self.create_publisher(Marker, place_recognition_publish_topic, 10)
        self.pnp_estimate_publisher = self.create_publisher(PoseArray, visual_pose_publish_topic, 10)

        self.accuracy_categories_meters = [{'lim_t': 20, 'lim_r': 180, 'true_count':0, 'false_count':0},
                                            {'lim_t': 5, 'lim_r': 10, 'true_count':0, 'false_count':0},
                                           {'lim_t': 0.5, 'lim_r': 5, 'true_count':0, 'false_count':0},
                                           {'lim_t': 0.25, 'lim_r': 2, 'true_count':0, 'false_count':0}]

        self.colormap = cm.get_cmap('Accent')

        # Thresholds to determine when the node starts publishing place recognition top1 result
        # instead of the geometrically estimated pose
        self.pycolmap_failure_tolerance = 10
        self.pycolmap_failure_count = 0

    def localization_callback(self, odometry_msg, visual_pose_msg):
        self.update_stats(odometry_msg, visual_pose_msg)

    def scenario_finished_callback(self, request, response):
        scenario_info_msg = request.scenario_results
        self.save_stats( scenario_info_msg )
        response.logging_result = True
        return response
                
    def save_stats(self, scenario_info_msg):

        scenario_info_dict = message_to_ordereddict(scenario_info_msg)
        
        print('Route {}'.format( 'success' if (scenario_info_dict['scenario_success']) else 'failed' ))
        print('Localized within threshold:')

        # for accuracy_category in self.accuracy_categories_meters:
        #     prct = accuracy_category['true_count']/(accuracy_category['true_count']+accuracy_category['false_count'])
        #     category_string = '@ {}m, {}°: {:.3f}'.format(accuracy_category['lim_t'], accuracy_category['lim_r'], prct)
        #     print(category_string)

        #     category_name = 'accuracy_{}m_{}deg'.format(accuracy_category['lim_t'], accuracy_category['lim_r'])
        #     scenario_info_dict[category_name] = prct 
        scenario_info_dict['accuracy_categories_vloc'] = self.accuracy_categories_meters 
        scenario_info_dict['pycolmap_failure_count'] = self.pycolmap_failure_count

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
    
    def update_stats(self, odometry_msg, visual_pose_msg):

        best_pose, best_pose_idx = VisualLocalizer.choose_best_estimate(visual_pose_msg)

        # To replicate paper experiments start using place recognition pose after 10 PnP failures
        if best_pose_idx is None:
            self.get_logger().warn('Pycolmap absolute_pose_estimation failed...')
            self.pycolmap_failure_count += 1


        if (best_pose_idx is not None):

            vloc_walltime = time.time()
            timestamp = visual_pose_msg.header.stamp
            vloc_time_ros = timestamp.sec + timestamp.nanosec*(1e-9)

            dist = distance_from_pose(odometry_msg.pose.pose, best_pose)
            angle, ang_dist = angular_distance_from_pose(odometry_msg.pose.pose, best_pose)

            vloc_freq_ros = 1/(vloc_time_ros - self.latest_vloc_time_ros) if (self.latest_vloc_time_ros is not None) else 0
            vloc_freq_walltime = 1/(vloc_walltime - self.latest_vloc_walltime) if (self.latest_vloc_walltime is not None) else 0
            print('Diff T: {:.3f}m, R: {:.3f}, {:.2f}Hz @ sim time, {:.2f}Hz @ wall time'.format(dist, ang_dist, vloc_freq_ros, vloc_freq_walltime))
            self.latest_vloc_time_ros = vloc_time_ros
            self.latest_vloc_walltime = vloc_walltime

            for accuracy_category in self.accuracy_categories_meters:
                if (dist <= accuracy_category['lim_t']) and (ang_dist <= accuracy_category['lim_r']):
                    accuracy_category['true_count'] = accuracy_category['true_count']+ 1
                else:
                    accuracy_category['false_count'] = accuracy_category['false_count']+ 1 

            # Place recognition & PnP localization visualizations
            marker = Marker(header=odometry_msg.header, scale=Vector3(x=1.0,y=1.0,z=1.0), type=8, action=0, color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
            poses = PoseArray(header=odometry_msg.header, poses=[])
            for i, estimate in enumerate(visual_pose_msg.pnp_estimates):
                if i == best_pose_idx:
                    color = self.colormap(0)
                else:
                    color = self.colormap(i+1)
                color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])

                place_recognition_idxs = estimate.place_recognition_idx
                for idx in place_recognition_idxs:
                    marker.colors.append(color)
                    marker.points.append(visual_pose_msg.place_recognition_poses[idx.data].position)

                if estimate.success.data == True:
                    poses.poses.append(estimate.pose)

            self.place_recognition_publisher.publish(marker)
            self.pnp_estimate_publisher.publish(poses)
            self.true_pose_publisher.publish(PoseStamped(header = odometry_msg.header, pose=odometry_msg.pose.pose))


def main(args=None):
    rclpy.init(args=args)

    executor = SingleThreadedExecutor()
    evaluator = VisualLocalizerEvaluator()
    executor.add_node(evaluator)
    executor.spin()

    evaluator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
