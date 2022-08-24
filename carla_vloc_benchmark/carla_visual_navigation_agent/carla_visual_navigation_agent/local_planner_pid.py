#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
This module contains a local planner to perform
low-level waypoint following based on PID controllers.
"""

import collections
import math
import threading

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_visual_navigation_agent.vehicle_pid_controller import VehiclePIDController
from carla_ad_agent.misc import distance_vehicle

from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=import-error
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker

from carla_visual_navigation_interfaces.srv import ToggleLocalPlanner

import numpy as np

class LocalPlanner(CompatibleNode):
    """
    LocalPlanner implements the basic behavior of following a trajectory of waypoints. 
    The low-level motion of the vehicle is computed by using two PID
    controllers, one is used for the lateral control and the other for the longitudinal
    control (cruise speed).
    """

    # minimum distance to target waypoint as a percentage (e.g. within 90% of
    # total distance)
    MIN_DISTANCE_SECONDS = 0.9
    MAX_DISTANCE_SECONDS = 3.0

    def __init__(self):
        super(LocalPlanner, self).__init__("local_planner")

        role_name = self.get_param("role_name", "ego_vehicle")
        odometry_topic_name = self.get_param('odometry_topic', "/odometry/filtered")

        self.control_time_step = self.get_param("control_time_step", 0.05)

        self.args_lateral_dict = {}
        self.args_lateral_dict['K_P'] = self.get_param("Kp_lateral", 0.9)
        self.args_lateral_dict['K_I'] = self.get_param("Ki_lateral", 0.0)
        self.args_lateral_dict['K_D'] = self.get_param("Kd_lateral", 0.0)

        self.args_longitudinal_dict = {}
        self.args_longitudinal_dict['K_P'] = self.get_param("Kp_longitudinal", 0.206)
        self.args_longitudinal_dict['K_I'] = self.get_param("Ki_longitudinal", 0.0206)
        self.args_longitudinal_dict['K_D'] = self.get_param("Kd_longitudinal", 0.515)

        self.data_lock = threading.Lock()

        self._current_pose = None
        self._current_speed = None
        self._target_speed = 0.0
        self._target_pose = None
        self._target_pose_idx = None


        self._buffer_size = 5
        self._waypoints_queue = collections.deque(maxlen=20000)
        self._waypoint_buffer = collections.deque(maxlen=self._buffer_size)
        
        self.min_distance = None
        self.max_distance = None

        self.stop = False

        self._odometry_subscriber = self.new_subscription(
            Odometry,
            odometry_topic_name,
            self.odometry_cb,
            qos_profile=10)

        self._path_subscriber = self.new_subscription(
            Path,
            "/carla/{}/waypoints".format(role_name),
            self.path_cb,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # Originally (in Carla Python API) subscribes to /speed_command which comes from ad_agent.
        # ad_agent only takes cares of hazards (stops in case of blocking traffic lights/vehicles)
        # since we don't need that at the moment, we subscribe directly to /target_speed
        # This enables us to run without launching ros bridge waypoint_publisher which
        # the ad_agent base class agent depends on for waypoint queries
        self._target_speed_subscriber = self.new_subscription(
            Float64,
            "/carla/{}/target_speed".format(role_name),
            self.target_speed_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # publishers
        self._target_pose_publisher = self.new_publisher(
            Marker,
            "/carla/{}/next_target".format(role_name),
            qos_profile=10)
        self._control_cmd_publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/{}/vehicle_control_cmd".format(role_name),
            qos_profile=10)

        self.planner_toggle_srv = self.create_service(ToggleLocalPlanner, "/toggle_local_planner", self.planner_toggle_cb)

        # initializing controller
        self._vehicle_controller = VehiclePIDController(
            self, args_lateral=self.args_lateral_dict, args_longitudinal=self.args_longitudinal_dict)

    def planner_toggle_cb(self, request, response):
        with self.data_lock:
            if request.stop == True:
                self._target_pose = None
                self._target_pose_idx = None
                self.emergency_stop()
                self.stop = True
                response.stop_result = True
                
            elif request.stop == False:
                self._current_pose = None
                self._current_speed = None
                self._target_pose = None
                self._target_pose_idx = None

                self._vehicle_controller = VehiclePIDController(
                    self, args_lateral=self.args_lateral_dict, args_longitudinal=self.args_longitudinal_dict)

                self.stop = False
                response.stop_result = False
            return response

    def odometry_cb(self, odometry_msg):
        with self.data_lock:
            self._current_pose = odometry_msg.pose.pose
            self._current_speed = math.sqrt(odometry_msg.twist.twist.linear.x ** 2 +
                                            odometry_msg.twist.twist.linear.y ** 2 +
                                            odometry_msg.twist.twist.linear.z ** 2) * 3.6

    def target_speed_cb(self, target_speed_msg):
        with self.data_lock:
            self._target_speed = target_speed_msg.data * 3.6
            self.min_distance = self._target_speed * self.MIN_DISTANCE_SECONDS / 3.6
            self.max_distance = self._target_speed * self.MAX_DISTANCE_SECONDS / 3.6

    def path_cb(self, path_msg):
        with self.data_lock:

            self._target_pose = None
            self._target_pose_idx = None
            self._waypoints_queue = [pose.pose for pose in path_msg.poses]
            self._waypoint_array = self._waypoint_locations_to_numpy(self._waypoints_queue)


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
                self.emergency_stop()
                return

            if not self._waypoint_buffer and not self._waypoints_queue:
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
                self.emergency_stop()
                return

            if self._target_pose is None:
                self._target_pose = self._compute_target_waypoint(self._current_pose)

            while (self._target_pose_idx+1 != len(self._waypoints_queue)) & (distance_vehicle(self._target_pose, self._current_pose.position) < self.min_distance):
                self._target_pose_idx += 1
                self._target_pose = self._waypoints_queue[self._target_pose_idx]

            if distance_vehicle(self._target_pose, self._current_pose.position) > self.max_distance:
                self._target_pose = self._compute_target_waypoint(self._current_pose)

            self._target_pose_publisher.publish(self.pose_to_marker_msg(self._target_pose))

            # move using PID controllers
            control_msg = self._vehicle_controller.run_step(
                self._target_speed, self._current_speed, self._current_pose, self._target_pose)

            self._control_cmd_publisher.publish(control_msg)

    def offset_route(self, target_pose, distance):

        # Test offsetting the target pose by d
        if self._target_pose_idx+1 != len(self._waypoints_queue):

            prev_target_pose_tmp = self._waypoints_queue[self._target_pose_idx].position
            current_target_pose = [self._waypoints_queue[self._target_pose_idx+1].position.x, self._waypoints_queue[self._target_pose_idx+1].position.y]
            prev_target_pose = [prev_target_pose_tmp.x, prev_target_pose_tmp.y]

            try:
                slope = (current_target_pose[1]-prev_target_pose[1])/(current_target_pose[0]-prev_target_pose[0])
                pslope = -1/slope
            except ZeroDivisionError:
                slope = (current_target_pose[1]-prev_target_pose[1])/((current_target_pose[0]-prev_target_pose[0])+0.00001)
                pslope = -1/(slope+0.00001)

            sign_x = ((pslope > 0) == (prev_target_pose[0] > current_target_pose[0])) * 2 - 1
            sign_y = ((pslope > 0) == (prev_target_pose[1] > current_target_pose[1])) * 2 - 1

            delta_x = sign_x * -distance / ((1 + pslope**2)**0.5)
            delta_y = pslope * delta_x

            target_pose.position.x = float(np.round(current_target_pose[0] + delta_x, 2))
            target_pose.position.y = float(np.round(current_target_pose[1] + delta_y,2 ))

        return target_pose


    def emergency_stop(self):
        control_msg = CarlaEgoVehicleControl()
        control_msg.steer = 0.0
        control_msg.throttle = 0.0
        control_msg.brake = 1.0
        control_msg.hand_brake = False
        control_msg.manual_gear_shift = False
        self._control_cmd_publisher.publish(control_msg)

    def _compute_target_waypoint(self, current_pose):

        min_distance = self._target_speed * self.MIN_DISTANCE_SECONDS / 3.6  # discard waypoints closer than this

        current_location_np = np.array([[ current_pose.position.x, current_pose.position.y, current_pose.position.z ]])
        distances = np.squeeze(np.linalg.norm( self._waypoint_array - current_location_np, axis=1))

        # If last waypoint is the within min_dist, set it as goal
        distance_to_goal = distances[-1]

        if distance_to_goal > min_distance:
            target_waypoint_idx = np.argmin(distances)
        else:
            target_waypoint_idx = -1

        target_waypoint = self._waypoints_queue[target_waypoint_idx]
        self._target_pose_idx = target_waypoint_idx
        return target_waypoint

    def _waypoint_locations_to_numpy(self, waypoint_queue):
        locations = []
        for pose in waypoint_queue:
            locations.append( [pose.position.x, pose.position.y, pose.position.z] )
        locations = np.array(locations)
        return locations

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
