#!/usr/bin/env python
#
#
"""
This module contains a local planner to perform
low-level waypoint following based on PID controllers.
"""

import collections
import math
import threading
import numpy as np
import time

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
import message_filters

from carla_visual_navigation_agent.vehicle_pp_controller import VehiclePPController
from carla_visual_navigation_agent.misc import distance_vehicle
from visual_robot_localization.visual_localizer_node import VisualLocalizer 

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo  # pylint: disable=import-error
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, Float32, String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose

from carla_ackermann_control import carla_control_physics as phys

from carla_visual_navigation_interfaces.srv import StopLocalPlanner
from visual_localization_interfaces.msg import VisualPoseEstimate


class LocalPlanner(CompatibleNode):
    """
    LocalPlanner implements the basic behavior of following a trajectory of waypoints that is
    generated on-the-fly. The low-level motion of the vehicle is computed by using two PID
    controllers, one is used for the lateral control and the other for the longitudinal
    control (cruise speed).

    When multiple paths are available (intersections) this local planner makes a random choice.
    """

    # minimum distance to target waypoint as a percentage (e.g. within 90% of
    # total distance)
    MIN_DISTANCE_SECONDS = 1.5

    def __init__(self):
        super(LocalPlanner, self).__init__("local_planner")

        role_name = self.get_param("role_name", "ego_vehicle")
        self.control_time_step = self.get_param("control_time_step", 0.05)

        args_lateral_dict = {}
        args_lateral_dict['vehicle_wheelbase'] = self.get_param("vehicle_wheelbase", 3.1) # 2.875 Real Tesla mk3 wheelbase
        vehicle_info = CarlaEgoVehicleInfo()
        args_lateral_dict['vehicle_max_steering_angle'] = phys.get_vehicle_max_steering_angle(vehicle_info)

        args_longitudinal_dict = {}
        args_longitudinal_dict['K_P'] = self.get_param("Kp_longitudinal", 0.206)
        args_longitudinal_dict['K_I'] = self.get_param("Ki_longitudinal", 0.0206)
        args_longitudinal_dict['K_D'] = self.get_param("Kd_longitudinal", 0.515)

        self.data_lock = threading.Lock()

        self._current_pose = None
        self._current_speed = None
        self._target_speed = 0.0
        self._target_pose = None
        self.previous_vloc_pose = None
        self.stop = False

        self._waypoints_queue = []
        self._waypoint_array = np.array([])

        visual_pose_topic_name = self.get_param('visual_pose_topic', '/carla/{}/visual_pose_estimate'.format(role_name))
        speedometer_topic_name = self.get_param('speedometer_topic', '/carla/{}/speedometer'.format(role_name))

        # subscribers

        self.visual_pose_subscription =  self.create_subscription( VisualPoseEstimate, visual_pose_topic_name, self.vloc_cb, 10)
        self.speedometer_subscription = self.create_subscription( Float32, speedometer_topic_name, self.speedometer_cb, 10)

        self._path_subscriber = self.new_subscription(
            Path,
            "/carla/{}/waypoints".format(role_name),
            self.path_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._target_speed_subscriber = self.new_subscription(
            Float64,
            "/carla/{}/speed_command".format(role_name),
            self.target_speed_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.planner_stop_srv = self.create_service(StopLocalPlanner, "/stop_local_planner", self.planner_stop_cb)

        # publishers
        self._target_pose_publisher = self.new_publisher(
            Marker,
            "/carla/{}/next_target".format(role_name),
            qos_profile=10)
        self._control_cmd_publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/{}/vehicle_control_cmd".format(role_name),
            qos_profile=10)

        # initializing controller
        self._vehicle_controller = VehiclePPController(
            self, args_lateral=args_lateral_dict, args_longitudinal=args_longitudinal_dict)

        # Thresholds to determine when the node starts publishing place recognition top1 result
        # instead of the geometrically estimated pose
        self.pycolmap_failure_tolerance = 10
        self.pycolmap_failure_count = 0

    def vloc_cb(self, vloc_pose_msg):
        with self.data_lock:

            best_pose, best_pose_idx = VisualLocalizer.choose_best_estimate(vloc_pose_msg)

            # To replicate paper experiments start using place recognition pose after 10 PnP failures
            if best_pose_idx is None:
                self.get_logger().warn('Pycolmap absolute_pose_estimation failed...')
                self.pycolmap_failure_count += 1
            else:
                self.pycolmap_failure_count = 0

            if (self.pycolmap_failure_count > self.pycolmap_failure_tolerance) | (best_pose_idx is not None):
                if best_pose != self.previous_vloc_pose:
                    self.previous_vloc_pose = best_pose
                    self._current_pose = best_pose

                    if self._waypoints_queue:
                        self._target_pose = self._compute_target_waypoint(self._current_pose)
                    else:
                        self.loginfo("Waiting for a route...")

                    if self._target_pose is not None:
                        self._target_pose_publisher.publish(self.pose_to_marker_msg(self._target_pose))


    def speedometer_cb(self, speedometer_msg):
        with self.data_lock:
            self._current_speed = speedometer_msg.data*3.6

    def target_speed_cb(self, target_speed_msg):
        with self.data_lock:
            self._target_speed = target_speed_msg.data

    def path_cb(self, path_msg):
        with self.data_lock:
            self._target_pose = None
            self._waypoints_queue = [pose.pose for pose in path_msg.poses]
            self._waypoint_array = self._waypoint_locations_to_numpy(self._waypoints_queue)

    def planner_stop_cb(self, request, response):
        with self.data_lock:
            self.emergency_stop()
            self.stop = True
            response.stop_result = True
            return response

    def _waypoint_locations_to_numpy(self, waypoint_queue):
        locations = []
        for pose in waypoint_queue:
            locations.append( [pose.position.x, pose.position.y, pose.position.z] )
        locations = np.array(locations)
        return locations

    def _compute_target_waypoint(self, current_pose):

        min_distance = self._target_speed * self.MIN_DISTANCE_SECONDS / 3.6  # discard waypoints closer than this

        current_location_np = np.array([[ current_pose.position.x, current_pose.position.y, current_pose.position.z ]])
        distances = np.squeeze(np.linalg.norm( self._waypoint_array - current_location_np, axis=1))

        # If last waypoint is the within min_dist, set it as goal
        distance_to_goal = distances[-1]

        if distance_to_goal > min_distance:

            # Set the distances smaller than min_distance to an insanely high value
            distances[ distances < min_distance ] = np.finfo(np.float).max

            # Indices of the two smallest distances that are larger than min_dist
            idx_part = np.argpartition(distances, 2)[:(2)]
            idx_partsort = np.argsort(distances[idx_part])
            idx_ksmallest = idx_part[idx_partsort]

            # Out of the two closest waypoints select the one that is further in the route
            # (to avoid backtracking)
            if idx_ksmallest[1] > idx_ksmallest[0]:
                target_waypoint_idx = idx_ksmallest[1]
            else:
                target_waypoint_idx = idx_ksmallest[0]
        else:
            target_waypoint_idx = -1

        target_waypoint = self._waypoints_queue[target_waypoint_idx]
        return target_waypoint

    def pose_to_marker_msg(self, pose):
        marker_msg = Marker()
        marker_msg.type = 0
        marker_msg.header.frame_id = "map"
        marker_msg.pose = pose
        marker_msg.scale.x = 1.0
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2
        marker_msg.color.r = 255.0
        marker_msg.color.a = 1.0
        return marker_msg

    def run_step(self):
        """
        Executes one step of local planning which involves running the longitudinal
        and lateral PID controllers to follow the waypoints trajectory.
        """
        with self.data_lock:
            if self.stop:
                #self.loginfo("Stopped by scenario...")
                self.emergency_stop()
                return

            if not self._waypoints_queue:
                self.loginfo("Waiting for a route...")
                self.emergency_stop()
                return

            # when target speed is 0, brake.
            if self._target_speed == 0.0:
                self.emergency_stop()
                return

            if self._current_speed is None:
                self.emergency_stop()
                return

            if self._current_pose is None:
                #self.loginfo('Waiting for the current pose')
                self.emergency_stop()
                return

            if self._target_pose is None:
                #self.loginfo('Waiting for the current pose')
                self.emergency_stop()
                return

            # move using PID controllers
            control_msg, pursuit_angle = self._vehicle_controller.run_step(
                self._target_speed, self._current_speed, self._current_pose, self._target_pose)

            self._control_cmd_publisher.publish(control_msg)

    def emergency_stop(self):
        control_msg = CarlaEgoVehicleControl()
        control_msg.steer = 0.0
        control_msg.throttle = 0.0
        control_msg.brake = 1.0
        control_msg.hand_brake = False
        control_msg.manual_gear_shift = False
        self._control_cmd_publisher.publish(control_msg)


def main(args=None):
    """

    main function

    :return:
    """
    roscomp.init("local_planner", args=args)

    local_planner = None
    update_timer = None
    try:
        local_planner = LocalPlanner()
        roscomp.on_shutdown(local_planner.emergency_stop)

        update_timer = local_planner.new_timer(
            local_planner.control_time_step, lambda timer_event=None: local_planner.run_step())

        local_planner.spin()

    except KeyboardInterrupt:
        pass

    finally:
        roscomp.loginfo('Local planner shutting down.')
        roscomp.shutdown()

if __name__ == "__main__":
    main()
