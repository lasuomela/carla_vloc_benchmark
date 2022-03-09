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
from copy import deepcopy
import threading

from std_msgs.msg import Float64, Int32
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Vector3, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from carla_visual_navigation_interfaces.msg import CrashInfo
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

        self.declare_parameter("visual_pose_topic", '/carla/ego_vehicle/best_vloc_estimate')
        visual_pose_topic_name = self.get_parameter('visual_pose_topic').get_parameter_value().string_value

        self.declare_parameter("synchrony_diff", 0.01)
        synchrony_diff = self.get_parameter('synchrony_diff').get_parameter_value().double_value

        self.filter_initialized_subscription =  self.create_subscription(
            PoseWithCovarianceStamped,
            '/carla/ego_vehicle/filter_reinit_pose',
            self.filter_initialized_callback,
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=1))

        self.crash_subscription =  self.create_subscription(
            CrashInfo,
            '/carla/ego_vehicle/log_segment',
            self.crash_callback,
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=1))

        self.travelled_distance_subscription =  self.create_subscription(
            Float64,
            '/carla/ego_vehicle/travelled_distance',
            self.travelled_distance_callback,
            10)

        self.path_tracking_error_subscription =  self.create_subscription(
            Float64,
            '/carla/ego_vehicle/path_tracking_error',
            self.path_tracking_error_callback,
            10)

        self.vloc_computation_time_subscription =  self.create_subscription(
            Float64,
            '/carla/ego_vehicle/vloc_computation_delay',
            self.vloc_computation_time_callback,
            10)

        self.path_subscriber =  self.create_subscription(
            Path,
            "/carla/ego_vehicle/waypoints",
            self.path_callback,
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=10))

        # self.reinit_waypoint_idx =  self.create_subscription(
        #     Int32,
        #     "/carla/ego_vehicle/reinit_waypoint_idx",
        #     self.farthest_waypoint_idx_callback,
        #     qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=1))

        self.true_odometry_subscription = message_filters.Subscriber(self, Odometry, odometry_topic_name)
        self.vis_loc_subscription = message_filters.Subscriber(self, PoseWithCovarianceStamped, visual_pose_topic_name)
        self.filtered_loc_subscription = message_filters.Subscriber(self, Odometry, '/odometry/filtered')

        # With odometry frequency 20Hz queue size 100 corresponds to searching up to 5 sec old odometry messages (in simulation time)
        # If the localization algorith is very slow, you might need to increase this
        queue_size = 100
        self.ts = message_filters.ApproximateTimeSynchronizer([self.true_odometry_subscription, self.vis_loc_subscription], queue_size, synchrony_diff)
        self.ts.registerCallback(self.vloc_callback)

        self.ts_filtered = message_filters.ApproximateTimeSynchronizer([self.true_odometry_subscription, self.filtered_loc_subscription], queue_size, synchrony_diff)
        self.ts_filtered.registerCallback(self.filtered_location_callback)

        self.latest_vloc_time_ros = None
        self.latest_vloc_walltime = None

        self.log_srv = self.create_service(LogScenarioResults, "log_scenario_results", self.scenario_finished_callback)

        self.true_pose_publisher = self.create_publisher(PoseStamped, true_pose_publish_topic, 10)
        self.place_recognition_publisher = self.create_publisher(Marker, place_recognition_publish_topic, 10)
        self.pnp_estimate_publisher = self.create_publisher(PoseArray, visual_pose_publish_topic, 10)

        self.accuracy_categories_template = [{'lim_t': 20, 'lim_r': 180, 'true_count':0, 'false_count':0},
                                            {'lim_t': 5, 'lim_r': 10, 'true_count':0, 'false_count':0},
                                            {'lim_t': 0.5, 'lim_r': 5, 'true_count':0, 'false_count':0},
                                            {'lim_t': 0.25, 'lim_r': 2, 'true_count':0, 'false_count':0}]
        self.accuracy_categories_vloc = deepcopy(self.accuracy_categories_template)
        self.accuracy_categories_filtered = deepcopy(self.accuracy_categories_template)
        self.avg_tracking_error = None
        self.num_tracking_measurements = 0

        self.avg_vloc_computation_time = 0
        self.num_vloc_computations = 0

        self.segment_dicts = []
        self.segment_information = None

        self.total_waypoint_number = None

        self.logger = self.get_logger()

        self.latest_filter_error_metric = 0.0
        self.latest_filter_error_angular = 0.0

        self.lock = threading.Lock()

    def path_callback(self, path_msg):
        self.total_waypoint_number = len(path_msg.poses)

    # def farthest_waypoint_idx_callback(self, idx_msg):
    #     with self.lock:
    #         self.farthest_waypoint_idx = idx_msg.data

    def path_tracking_error_callback(self, tracking_error_msg):
        #with self.lock:
        if self.segment_information is not None:

            if self.num_tracking_measurements != 0:
                self.num_tracking_measurements += 1
                self.avg_tracking_error = (self.num_tracking_measurements - 1)/self.num_tracking_measurements * self.avg_tracking_error \
                    + 1/self.num_tracking_measurements * tracking_error_msg.data
            else:
                self.num_tracking_measurements += 1
                self.avg_tracking_error = tracking_error_msg.data

            if self.segment_information['avg_tracking_error'] is not None:
                self.segment_information['num_tracking_measurements'] += 1
                self.segment_information['avg_tracking_error'] = (self.segment_information['num_tracking_measurements'] - 1)/self.segment_information['num_tracking_measurements'] * self.segment_information['avg_tracking_error'] \
                    + 1/self.segment_information['num_tracking_measurements'] * tracking_error_msg.data
            else:
                self.segment_information['num_tracking_measurements'] += 1
                self.segment_information['avg_tracking_error'] = tracking_error_msg.data

    def travelled_distance_callback(self, distance_msg):
        # with self.lock:
        self.travelled_distance = distance_msg.data

    def vloc_callback(self, odometry_msg, visual_pose_msg):
        # with self.lock:
            # Update total and segment stats
        self.update_vloc_stats(odometry_msg, visual_pose_msg)

    def vloc_computation_time_callback(self, delay_msg):

        if self.num_vloc_computations != 0:
            self.num_vloc_computations += 1
            self.avg_vloc_computation_time = (self.num_vloc_computations - 1)/self.num_vloc_computations * self.avg_vloc_computation_time \
                + 1/self.num_vloc_computations * delay_msg.data
        else:
            self.num_vloc_computations += 1
            self.avg_vloc_computation_time = delay_msg.data

    def filtered_location_callback(self, true_odometry_msg, filtered_odometry_msg):
       # with self.lock:
        self.update_filtered_stats(true_odometry_msg, filtered_odometry_msg)

    def filter_initialized_callback(self, init_pose_msg):
      #  with self.lock:
        self.travelled_distance = 0.0
        self.latest_filter_error_metric = 0.0
        self.latest_filter_error_angular = 0.0
        start_pose = message_to_ordereddict(init_pose_msg.pose.pose.position)
        self.segment_information = {'segment_start_location': start_pose,
                                    'segment_end_location': None,
                                    'segment_length': None,
                                    'avg_tracking_error': None,
                                    'num_tracking_measurements': 0,
                                    'pose_final_error': {'metric': 0,'angular': 0},
                                    'segment_reached_waypoint_idx': None,
                                    'accuracies_vloc': deepcopy(self.accuracy_categories_template),
                                    'accuracies_filtered': deepcopy(self.accuracy_categories_template)}
        self.get_logger().info('Begun a new segment')


    def crash_callback(self, crash_info_msg):
        #with self.lock:
        if self.segment_information is not None:
            self.segment_information['segment_end_location'] = message_to_ordereddict(crash_info_msg.crash_location.point)
            self.segment_information['segment_length'] = self.travelled_distance
            self.segment_information['pose_final_error'] = {'metric': self.latest_filter_error_metric,'angular': self.latest_filter_error_angular}
            self.segment_information['segment_reached_waypoint_idx'] = crash_info_msg.closest_waypoint_idx.data

            self.segment_dicts.append(self.segment_information)
            self.segment_information = None
            self.logger.info('Stored a segment')
            self.logger.info('{} segments in total'.format(len(self.segment_dicts)))
        else:
            self.logger.info('Got a crash but no segment is initialized')


    def scenario_finished_callback(self, request, response):
        #with self.lock:
        self.segment_information['segment_end_location'] = None
        self.segment_information['segment_length'] = self.travelled_distance
        self.segment_information['pose_final_error'] = {'metric': self.latest_filter_error_metric,'angular': self.latest_filter_error_angular}
        self.segment_dicts.append(self.segment_information)
        self.logger.info('Saving all segments')

        scenario_info_msg = request.scenario_results
        self.save_stats( scenario_info_msg )
        response.logging_result = True
        return response

        #########


    def save_stats(self, scenario_info_msg):
        with self.lock:
            self.logger.info('{}'.format(self.segment_dicts))

            scenario_info_dict = message_to_ordereddict(scenario_info_msg)
            
            print('Localized within threshold:')

            percents = {}
            for accuracy_category in self.accuracy_categories_vloc:
                if (accuracy_category['true_count']+accuracy_category['false_count']) > 0:

                    prct = accuracy_category['true_count']/(accuracy_category['true_count']+accuracy_category['false_count'])
                    category_string = '@ {}m, {}°: {:.3f}'.format(accuracy_category['lim_t'], accuracy_category['lim_r'], prct)
                    print(category_string)

                    category_name = 'accuracy_{}m_{}deg'.format(accuracy_category['lim_t'], accuracy_category['lim_r'])
                    percents[category_name] = prct 

            scenario_info_dict['accuracies_vloc'] = percents

            percents = {}
            for accuracy_category in self.accuracy_categories_filtered:
                if (accuracy_category['true_count']+accuracy_category['false_count']) > 0:
                    prct = accuracy_category['true_count']/(accuracy_category['true_count']+accuracy_category['false_count'])
                    category_string = '@ {}m, {}°: {:.3f}'.format(accuracy_category['lim_t'], accuracy_category['lim_r'], prct)
                    #print(category_string)
                    category_name = 'accuracy_{}m_{}deg'.format(accuracy_category['lim_t'], accuracy_category['lim_r'])
                    percents[category_name] = prct 

            scenario_info_dict['accuracies_filtered'] = percents

            processed_segment_dicts = []
            for segment_info in self.segment_dicts:
                segment_dict = {}
                for measure_type in ['accuracies_vloc', 'accuracies_filtered']:
                    percents = {}
                    for accuracy_category in segment_info[measure_type]:
                        if (accuracy_category['true_count']+accuracy_category['false_count']) > 0:
                            prct = accuracy_category['true_count']/(accuracy_category['true_count']+accuracy_category['false_count'])
                            category_string = '@ {}m, {}°: {:.3f}'.format(accuracy_category['lim_t'], accuracy_category['lim_r'], prct)
                            #print(category_string)
                            category_name = 'accuracy_{}m_{}deg'.format(accuracy_category['lim_t'], accuracy_category['lim_r'])
                            percents[category_name] = prct 

                    segment_dict[measure_type] = percents
                segment_dict['segment_start_location'] = segment_info['segment_start_location']
                segment_dict['segment_end_location'] = segment_info['segment_end_location'] 
                segment_dict['segment_length'] = segment_info['segment_length']
                segment_dict['avg_tracking_error'] = segment_info['avg_tracking_error']
                segment_dict['pose_final_error'] = segment_info['pose_final_error']
                segment_dict['segment_reached_waypoint_idx'] = segment_info['segment_reached_waypoint_idx']
                processed_segment_dicts.append(segment_dict)

            scenario_info_dict['segments'] = processed_segment_dicts

            scenario_info_dict['avg_tracking_error'] = self.avg_tracking_error
            scenario_info_dict['path_waypoint_count'] = self.total_waypoint_number
            scenario_info_dict['avg_vloc_computation_time'] = self.avg_vloc_computation_time

            if scenario_info_dict['log_results']:
                result_list = []
                if os.path.exists(scenario_info_dict['log_file_path']):
                    if (os.stat(scenario_info_dict['log_file_path']).st_size != 0):
                        with open(scenario_info_dict['log_file_path'], 'r+') as f:
                            result_list = json.load(f)

                result_list.append(scenario_info_dict)
                scenario_info_dict.update({'idx':len(result_list)})

                with open(scenario_info_dict['log_file_path'], 'w') as f:
                    json.dump(result_list, f, indent=4)
    
    def update_vloc_stats(self, odometry_msg, visual_pose_msg):
        #with self.lock:
        if self.segment_information is not None:

            vloc_walltime = time.time()
            timestamp = visual_pose_msg.header.stamp
            vloc_time_ros = timestamp.sec + timestamp.nanosec*(1e-9)

            dist = distance_from_pose(odometry_msg.pose.pose, visual_pose_msg.pose.pose)
            angle, ang_dist = angular_distance_from_pose(odometry_msg.pose.pose, visual_pose_msg.pose.pose)

            vloc_freq_ros = 1/(vloc_time_ros - self.latest_vloc_time_ros) if (self.latest_vloc_time_ros is not None) else 0
            vloc_freq_walltime = 1/(vloc_walltime - self.latest_vloc_walltime) if (self.latest_vloc_walltime is not None) else 0
            print('Diff T: {:.3f}m, R: {:.3f}, {:.2f}Hz @ sim time, {:.2f}Hz @ wall time'.format(dist, ang_dist, vloc_freq_ros, vloc_freq_walltime))
            self.latest_vloc_time_ros = vloc_time_ros
            self.latest_vloc_walltime = vloc_walltime

            for accuracy_log in [self.accuracy_categories_vloc, self.segment_information['accuracies_vloc']]:
                for accuracy_category in accuracy_log:
                    if (dist <= accuracy_category['lim_t']) and (ang_dist <= accuracy_category['lim_r']):
                        accuracy_category['true_count'] += 1
                    else:
                        accuracy_category['false_count'] += 1
            # else:
            #     self.get_logger().warn('Segment logger not initialized!')

    def update_filtered_stats(self, true_odometry_msg, filtered_odometry_msg):
       # with self.lock:
        if self.segment_information is not None:
            dist = distance_from_pose(true_odometry_msg.pose.pose, filtered_odometry_msg.pose.pose)
            angle, ang_dist = angular_distance_from_pose(true_odometry_msg.pose.pose, filtered_odometry_msg.pose.pose)

            self.latest_filter_error_metric = dist
            self.latest_filter_error_angular = ang_dist

            for accuracy_log in [self.accuracy_categories_filtered, self.segment_information['accuracies_filtered']]:
                for accuracy_category in accuracy_log:
                    if (dist <= accuracy_category['lim_t']) and (ang_dist <= accuracy_category['lim_r']):
                        accuracy_category['true_count'] += 1
                    else:
                        accuracy_category['false_count'] += 1
        # else:
        #     self.get_logger().warn('Segment logger not initialized!')

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
