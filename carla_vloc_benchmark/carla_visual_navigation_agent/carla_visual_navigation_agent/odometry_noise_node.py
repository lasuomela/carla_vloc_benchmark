import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import SingleThreadedExecutor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from std_msgs.msg import Float64
from visual_localization_interfaces.msg import VisualPoseEstimate
from carla_visual_navigation_interfaces.srv import ReinitializeFilter

from robot_localization.srv import SetPose

from visual_robot_localization.visual_localizer_node import VisualLocalizer 

import numpy as np
from numpy.random import default_rng
import threading
import time
from copy import deepcopy

from rclpy.callback_groups import ReentrantCallbackGroup


class OdometryNoiseSimulator(Node):

    def __init__(self):
        super().__init__('odometry_noise_node')

        self.declare_parameter("add_noise", True)
        self.add_noise = self.get_parameter('add_noise').get_parameter_value().bool_value

        self.declare_parameter("reinit_filter", True)
        self.reinit_filter = self.get_parameter('reinit_filter').get_parameter_value().bool_value

        self.odometry_subscription =  self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odometry_cb,
            10)

        self.vloc_subscription =  self.create_subscription(
            VisualPoseEstimate,
            '/carla/ego_vehicle/visual_pose_estimate',
            self.vloc_cb,
            10)

        self.noisy_odom_publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            '/carla/ego_vehicle/odometry_noisy',
            10)

        self.true_odom_publisher = self.create_publisher(
            Odometry,
            '/carla/ego_vehicle/odometry_controlled',
            10)

        self.vloc_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/carla/ego_vehicle/best_vloc_estimate',
            10)

        self.vloc_delay_publisher = self.create_publisher(
            Float64,
            '/carla/ego_vehicle/vloc_computation_delay',
            10)

        self.filter_reinitialized_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/carla/ego_vehicle/filter_reinit_pose',
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=10))

        if self.reinit_filter:
            self.set_filter_pose_client = self.create_client(SetPose, 'set_pose', callback_group=ReentrantCallbackGroup())
            while not self.set_filter_pose_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

        #self.reinitialize_filter_srv = self.create_service(ReinitializeFilter, "/reinitialize_filter", self.reinitialize_filter_cb, callback_group=ReentrantCallbackGroup())

        if self.reinit_filter:
            self.filter_initialized = False
        else:
            self.filter_initialized = True
        self.future = None

        self.rng = default_rng()

        loc_var = 0.1
        loc_cov = 0.0
        or_var = 0.1
        or_cov = 0.0

        self.vloc_estimate_covariance =    [loc_var,    loc_cov,    loc_cov,    0.0,    0.0,    0.0,
                                            loc_cov,    loc_var,    loc_cov,    0.0,    0.0,    0.0,
                                            loc_cov,    loc_cov,    loc_var,    0.0,    0.0,    0.0,
                                            0.0,    0.0,    0.0,    or_var,    or_cov,    or_cov,
                                            0.0,    0.0,    0.0,    or_cov,    or_var,    or_cov,
                                            0.0,    0.0,    0.0,    or_cov,    or_cov,    or_var]
        self.vloc_estimate_covariance = np.array(self.vloc_estimate_covariance)
        self.vloc_published = False

        target_speed = 4
        self.odometry_noise_magnitude = 0.04
        self.odometry_noise_eps = 0.08 #(np.pi/64)/30

        loc_odo_var = target_speed * self.odometry_noise_magnitude
        loc_odo_cov = 0 #target_speed * self.odometry_noise_magnitude/10
        or_odo_var =  target_speed * self.odometry_noise_magnitude + self.odometry_noise_eps
        or_odo_cov =  0 #(target_speed * self.odometry_noise_magnitude + self.odometry_noise_eps)/10

        self.odometry_twist_covariance = [loc_odo_var,    loc_odo_cov,    loc_odo_cov,    0.0,    0.0,    0.0,
                                        loc_odo_cov,    loc_odo_var,    loc_odo_cov,    0.0,    0.0,    0.0,
                                        loc_odo_cov,    loc_odo_cov,    loc_odo_var,    0.0,    0.0,    0.0,
                                        0.0,    0.0,    0.0,    or_odo_var,    or_odo_cov,    or_odo_cov,
                                        0.0,    0.0,    0.0,    or_odo_cov,    or_odo_var,    or_odo_cov,
                                        0.0,    0.0,    0.0,    or_odo_cov,    or_odo_cov,    or_odo_var]
        self.odometry_twist_covariance = np.array(self.odometry_twist_covariance)

        self.lock = threading.Lock()


    def odometry_cb(self, msg):

        # with self.lock:
        if not(self.filter_initialized):
            if self.future is None:
                self.filter_init_request = SetPose.Request()
                self.filter_init_request.pose = PoseWithCovarianceStamped(header=msg.header, pose = msg.pose)
                self.get_logger().info('Setting filter pose to {}'.format(msg.pose.pose))
                self.future = self.set_filter_pose_client.call_async(self.filter_init_request)
            else:
                if self.future.done():
                    self.filter_initialized = True
                    self.get_logger().info('Filter initialized')
                    self.filter_reinitialized_publisher.publish(self.filter_init_request.pose)
        else:
            if self.add_noise:
                msg.twist.twist.linear.x = msg.twist.twist.linear.x + float(self.rng.normal(0, abs(msg.twist.twist.linear.x)*self.odometry_noise_magnitude, size=1))
                msg.twist.twist.angular.z = msg.twist.twist.angular.z + float(self.rng.normal(0, abs(msg.twist.twist.linear.x)*self.odometry_noise_magnitude + self.odometry_noise_eps, size=1))
                msg.twist.covariance = self.odometry_twist_covariance
                msg.header.frame_id = 'ego_vehicle'
                msg = TwistWithCovarianceStamped(header = msg.header, twist=msg.twist)
                if self.vloc_published:
                    self.noisy_odom_publisher.publish(msg)
            else:
                if self.vloc_published:
                    self.true_odom_publisher.publish(msg)


    def vloc_cb(self, msg):
        if self.filter_initialized:
            if (self.vloc_published):

                image_capture_time = Time.from_msg(msg.header.stamp)
                current_time = self.get_clock().now()

                vloc_computation_time = (current_time-image_capture_time).nanoseconds *1e-9
                self.vloc_delay_publisher.publish(Float64(data= vloc_computation_time))

                best_pose, best_pose_idx = VisualLocalizer.choose_best_estimate(msg)
                if best_pose_idx is not None:
                    pose_msg = PoseWithCovarianceStamped()
                    pose_msg.header = msg.header
                    pose_msg.pose.pose = best_pose
                    pose_msg.pose.covariance = self.vloc_estimate_covariance
                    self.vloc_publisher.publish(pose_msg)
            else:
                self.vloc_published = True


        
def main(args=None):

    rclpy.init(args=args)

    try:
        noiser = OdometryNoiseSimulator()
        executor = SingleThreadedExecutor()
        executor.add_node(noiser)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        noiser.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()