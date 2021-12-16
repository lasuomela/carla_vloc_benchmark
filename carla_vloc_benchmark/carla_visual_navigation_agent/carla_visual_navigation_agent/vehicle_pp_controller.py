#!/usr/bin/env python
#
#

""" This module contains PID and PP controllers to perform lateral and longitudinal control. """

from collections import deque
import math
import numpy as np
from transforms3d.euler import quat2euler
from geometry_msgs.msg import Point  # pylint: disable=import-error
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo  # pylint: disable=import-error
from carla_ackermann_control import carla_control_physics as phys

class VehiclePPController(object):  # pylint: disable=too-few-public-methods
    """
    VehiclePPController is a combination of two controllers (lateral Pure Pursuit and longitudinal PID)
    to perform the low level control a vehicle from client side
    """

    def __init__(self, node, args_lateral=None, args_longitudinal=None):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral Pure Pursuit controller using
                             the following semantics:
                             vehicle_wheelbase -- vehicle axle distance
                             vehicle_max_steering_angle -- The angle which defines the minimum turning radius

        :param args_longitudinal: dictionary of arguments to set the longitudinal PID
                                  controller using the following semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        """
        if not args_lateral:
            vehicle_info = CarlaEgoVehicleInfo()
            args_lateral = {'vehicle_wheelbase': 2.875, 'vehicle_max_steering_angle': phys.get_vehicle_max_steering_angle(vehicle_info)}
        if not args_longitudinal:
            args_longitudinal = {'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}

        self.node = node
        self._lon_controller = PIDLongitudinalController(**args_longitudinal)
        self._lat_controller = PPLateralController(**args_lateral)

    def run_step(self, target_speed, current_speed, current_pose, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal
        controllers to reach a target waypoint at a given target_speed.

        :param target_speed: desired vehicle speed
        :param waypoint: target location encoded as a waypoint
        :return: control signal (throttle and steering)
        """
        control = CarlaEgoVehicleControl()
        throttle = self._lon_controller.run_step(target_speed, current_speed)
        steering, pursuit_angle = self._lat_controller.run_step(current_pose, waypoint)
        control.steer = -steering
        control.throttle = throttle
        control.brake = 0.0
        control.hand_brake = False
        control.manual_gear_shift = False

        return control, pursuit_angle


class PIDLongitudinalController(object):  # pylint: disable=too-few-public-methods
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, K_P=1.0, K_D=0.0, K_I=0.0):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0

    def run_step(self, target_speed, current_speed):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """
        previous_error = self.error
        self.error = target_speed - current_speed
        # restrict integral term to avoid integral windup
        self.error_integral = np.clip(self.error_integral + self.error, -40.0, 40.0)
        self.error_derivative = self.error - previous_error
        output = self._K_P * self.error + self._K_I * self.error_integral + self._K_D * self.error_derivative
        return np.clip(output, 0.0, 1.0)


class PPLateralController(object):  # pylint: disable=too-few-public-methods
    """
    PPLateralController implements lateral control using a Pure Pursuit scheme.

    See:
    Automatic Steering Methods for Autonomous Automobile Path Tracking
    Jarrod M. Snider
    https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
    """

    def __init__(self, vehicle_wheelbase, vehicle_max_steering_angle):
        """
        """
        self.max_steering_angle = vehicle_max_steering_angle
        self.L = vehicle_wheelbase

    def run_step(self, current_pose, waypoint):
        """
        Estimate the steering angle of the vehicle based on the Pure Pursuit equation

        :param waypoint: target waypoint
        :param current_pose: current pose of the vehicle
        :return: steering control in the range [-1, 1]
        """
        v_begin = current_pose.position
        quaternion = (
            current_pose.orientation.w,
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z
        )
        _, _, yaw = quat2euler(quaternion)
        v_end = Point()
        v_end.x = v_begin.x + math.cos(yaw)
        v_end.y = v_begin.y + math.sin(yaw)

        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([waypoint.position.x -
                          v_begin.x, waypoint.position.y -
                          v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))
                                 
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        ld = np.linalg.norm(w_vec)
        pursuit_angle = math.atan( 2*self.L*math.sin(_dot)/ ld )
        pursuit_steer = pursuit_angle/self.max_steering_angle
        output = np.clip(pursuit_steer, -1.0, 1.0)

        return output, pursuit_angle
