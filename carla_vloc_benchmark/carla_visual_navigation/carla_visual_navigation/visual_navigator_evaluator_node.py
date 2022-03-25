import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import message_filters
from rosidl_runtime_py.convert import message_to_ordereddict

import os
import json
import time
from copy import deepcopy
import threading

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from carla_visual_navigation_interfaces.msg import CrashInfo
from carla_visual_navigation_interfaces.srv import LogScenarioResults
from visual_localization_interfaces.msg import VisualPoseEstimate

from carla_visual_navigation.geometry_utils import distance_from_pose, angular_distance_from_pose


class VisualLocalizerEvaluator(Node):

    def __init__(self, pub=None):
        super().__init__('visual_localizer_evaluator')

        self.declare_parameter("role_name", "ego_vehicle")
        role_name = self.get_parameter('role_name').get_parameter_value().string_value

        self.declare_parameter("odometry_topic", "/carla/{}/odometry".format(role_name))
        odometry_topic_name = self.get_parameter('odometry_topic').get_parameter_value().string_value

        self.declare_parameter("visual_pose_topic", '/carla/{}/visual_pose_estimate'.format(role_name))
        visual_pose_topic_name = self.get_parameter('visual_pose_topic').get_parameter_value().string_value

        self.declare_parameter("ekf_odometry_topic", '/odometry/filtered')
        ekf_odometry_topic_name = self.get_parameter('ekf_odometry_topic').get_parameter_value().string_value

        self.filter_initialized_subscription =  self.create_subscription(
            PoseWithCovarianceStamped,
            '/carla/{}/filter_reinit_pose'.format(role_name),
            self.filter_initialized_callback,
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=1))

        self.crash_subscription =  self.create_subscription(
            CrashInfo,
            '/carla/{}/log_segment'.format(role_name),
            self.crash_callback,
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=1))

        self.travelled_distance_subscription =  self.create_subscription(
            Float64,
            '/carla/{}/travelled_distance'.format(role_name),
            self.travelled_distance_callback,
            10)

        self.path_tracking_error_subscription =  self.create_subscription(
            Float64,
            '/carla/{}/path_tracking_error'.format(role_name),
            self.path_tracking_error_callback,
            10)

        self.path_subscriber =  self.create_subscription(
            Path,
            "/carla/{}/waypoints".format(role_name),
            self.path_callback,
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=10))

        self.true_odometry_subscription = message_filters.Subscriber(self, Odometry, odometry_topic_name)
        self.vis_loc_subscription = message_filters.Subscriber(self, VisualPoseEstimate, visual_pose_topic_name)
        self.ekf_loc_subscription = message_filters.Subscriber(self, Odometry, ekf_odometry_topic_name)

        # With odometry frequency 20Hz queue size 100 corresponds to searching up to 5 sec old odometry messages (in simulation time)
        # If the localization algorith is very slow, you might need to increase this
        queue_size = 100
        synchrony_diff = 0.01
        self.ts = message_filters.ApproximateTimeSynchronizer([self.true_odometry_subscription, self.vis_loc_subscription], queue_size, synchrony_diff)
        self.ts.registerCallback(self.vloc_callback)

        self.ts_ekf = message_filters.ApproximateTimeSynchronizer([self.true_odometry_subscription, self.ekf_loc_subscription], queue_size, synchrony_diff)
        self.ts_ekf.registerCallback(self.ekf_location_callback)

        self.latest_vloc_rostime = None
        self.latest_vloc_walltime = None

        self.log_srv = self.create_service(LogScenarioResults, "log_scenario_results", self.scenario_finished_callback)

        self.accuracy_categories_template = [{'lim_t': 20, 'lim_r': 180, 'true_count':0, 'false_count':0},
                                            {'lim_t': 5, 'lim_r': 10, 'true_count':0, 'false_count':0},
                                            {'lim_t': 0.5, 'lim_r': 5, 'true_count':0, 'false_count':0},
                                            {'lim_t': 0.25, 'lim_r': 2, 'true_count':0, 'false_count':0}]
        self.accuracy_categories_vloc = deepcopy(self.accuracy_categories_template)
        self.accuracy_categories_ekf = deepcopy(self.accuracy_categories_template)

        self.avg_vloc_computation_time = 0
        self.num_vloc_computations = 0

        self.segment_dicts = []
        self.segment_information = None

        self.avg_tracking_error = None
        self.num_tracking_measurements = 0

        self.total_waypoint_number = None

        self.logger = self.get_logger()

        self.lock = threading.Lock()

    def path_callback(self, path_msg):
        self.total_waypoint_number = len(path_msg.poses)

    def path_tracking_error_callback(self, tracking_error_msg):
        if self.segment_information is not None:
            self.avg_tracking_error, self.num_tracking_measurements = running_average(self.avg_tracking_error,
                                                                                    self.num_tracking_measurements,
                                                                                    tracking_error_msg.data)
            self.segment_information['avg_tracking_error'], \
                self.segment_information['num_tracking_measurements'] = running_average(self.segment_information['avg_tracking_error'],
                                                                                        self.segment_information['num_tracking_measurements'],
                                                                                        tracking_error_msg.data)

    def travelled_distance_callback(self, distance_msg):
        if self.segment_information is not None:
            self.segment_information['segment_length'] = distance_msg.data

    def vloc_callback(self, odometry_msg, visual_pose_msg):
        # Update total and segment stats
        self.update_vloc_stats(odometry_msg, visual_pose_msg)

    def ekf_location_callback(self, true_odometry_msg, ekf_odometry_msg):
        self.update_ekf_stats(true_odometry_msg, ekf_odometry_msg)

    def filter_initialized_callback(self, init_pose_msg):
        # Initialize a new segment for logging results
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

        self.logger.info('Begun a new segment')


    def crash_callback(self, crash_info_msg):
        if self.segment_information is not None:
            self.segment_information['segment_end_location'] = message_to_ordereddict(crash_info_msg.crash_location.point)
            self.segment_information['segment_reached_waypoint_idx'] = crash_info_msg.closest_waypoint_idx.data

            del self.segment_information['num_tracking_measurements']

            self.segment_dicts.append(self.segment_information)
            self.segment_information = None

            self.logger.info('Stored a segment')
            self.logger.info('{} segments in total'.format(len(self.segment_dicts)))
        else:
            self.logger.info('Got a crash but no segment is initialized')

    def scenario_finished_callback(self, request, response):

        del self.segment_information['num_tracking_measurements']

        self.segment_dicts.append(self.segment_information)
        self.segment_information = None
        self.logger.info('Saving all segments')

        scenario_info_msg = request.scenario_results
        self.save_stats( scenario_info_msg )
        response.logging_result = True
        return response


    def update_vloc_stats(self, odometry_msg, visual_pose_msg):

        if self.segment_information is not None:

            walltime = time.time()
            rostime = self.get_clock().now().nanoseconds

            if visual_pose_msg.pnp_success.data == True:
                dist = distance_from_pose(odometry_msg.pose.pose, visual_pose_msg.pose.pose)
                angle, ang_dist = angular_distance_from_pose(odometry_msg.pose.pose, visual_pose_msg.pose.pose)

                for accuracy_log in [self.accuracy_categories_vloc, self.segment_information['accuracies_vloc']]:
                    for accuracy_category in accuracy_log:
                        if (dist <= accuracy_category['lim_t']) and (ang_dist <= accuracy_category['lim_r']):
                            accuracy_category['true_count'] += 1
                        else:
                            accuracy_category['false_count'] += 1

                # Calculate the computation delay
                if self.latest_vloc_rostime is not None:
                    # Transform the delay expressed in walltime to simulation time by computing the ratio of simulation and walltimes
                    vloc_computation_delay_wall = rclpy.duration.Duration.from_msg(visual_pose_msg.computation_delay).nanoseconds*1e-9
                    sim_wall_ratio = ((rostime-self.latest_vloc_rostime) / (time.time() - self.latest_vloc_walltime))*1e-9
                    vloc_computation_delay = vloc_computation_delay_wall * sim_wall_ratio
                else:
                    vloc_computation_delay = 0

                self.avg_vloc_computation_time, self.num_vloc_computations = running_average(self.avg_vloc_computation_time,
                                                                                            self.num_vloc_computations,
                                                                                            vloc_computation_delay)
                if self.latest_vloc_rostime is None:
                    vloc_freq_ros = 0
                elif rostime == self.latest_vloc_rostime:
                    vloc_freq_ros = 0
                else:
                    vloc_freq_ros = 1/((rostime - self.latest_vloc_rostime)*1e-9)

                vloc_freq_walltime = 1/(walltime - self.latest_vloc_walltime) if (self.latest_vloc_walltime is not None) else 0

                if visual_pose_msg.pnp_success.data == True:
                    # Print the error and frequency at which the vloc is being run
                    print('Diff T: {:.3f}m, R: {:.3f}, Running {:.2f}Hz @ sim time, {:.2f}Hz @ wall time, computation delay {:.3f}s @ sim time'.format(dist,
                                                                                                                                                    ang_dist,
                                                                                                                                                    vloc_freq_ros,
                                                                                                                                                    vloc_freq_walltime,
                                                                                                                                                    vloc_computation_delay))
            else:
                for accuracy_log in [self.accuracy_categories_vloc, self.segment_information['accuracies_vloc']]:
                    for accuracy_category in accuracy_log:
                        accuracy_category['false_count'] += 1

                print('Pycolmap PnP estimation failed...')

            self.latest_vloc_rostime = rostime
            self.latest_vloc_walltime = walltime


    def update_ekf_stats(self, true_odometry_msg, ekf_odometry_msg):
        if self.segment_information is not None:
            dist = distance_from_pose(true_odometry_msg.pose.pose, ekf_odometry_msg.pose.pose)
            angle, ang_dist = angular_distance_from_pose(true_odometry_msg.pose.pose, ekf_odometry_msg.pose.pose)

            for accuracy_log in [self.accuracy_categories_ekf, self.segment_information['accuracies_filtered']]:
                for accuracy_category in accuracy_log:
                    if (dist <= accuracy_category['lim_t']) and (ang_dist <= accuracy_category['lim_r']):
                        accuracy_category['true_count'] += 1
                    else:
                        accuracy_category['false_count'] += 1

            self.segment_information['pose_final_error'] = {'metric': dist,'angular': ang_dist}

    def save_stats(self, scenario_info_msg):
        with self.lock:
            self.logger.info('{}'.format(self.segment_dicts))

            scenario_info_dict = message_to_ordereddict(scenario_info_msg)
            
            print('Saving stats...')

            # Compute visual localization accuracy over whole scenario route
            scenario_info_dict['accuracies_vloc'] = compute_recall_categories(self.accuracy_categories_vloc)

            # Compute Kalman filter output accuracy over whole scenario route
            scenario_info_dict['accuracies_filtered'] = compute_recall_categories(self.accuracy_categories_ekf)

            # Compute visual localization and filter output accuracies over the route segments
            for segment_info in self.segment_dicts:
                for measure_type in ['accuracies_vloc', 'accuracies_filtered']:
                    segment_info[measure_type] = compute_recall_categories(segment_info[measure_type])

            scenario_info_dict['segments'] = self.segment_dicts
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


def compute_recall_categories(recall_category_counts):
    recall_percents = {}
    for recall_category in recall_category_counts:
        if (recall_category['true_count']+recall_category['false_count']) > 0:
            prct = recall_category['true_count']/(recall_category['true_count']+recall_category['false_count'])
            category_name = 'accuracy_{}m_{}deg'.format(recall_category['lim_t'], recall_category['lim_r'])
            recall_percents[category_name] = prct
    return recall_percents


def running_average(current_average, num_measurements, new_measurement):

    if num_measurements != 0:
        num_measurements += 1
        new_average = (num_measurements-1)/num_measurements * current_average + (1/num_measurements) * new_measurement
    else:
        num_measurements += 1
        new_average = new_measurement
    return new_average, num_measurements


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
