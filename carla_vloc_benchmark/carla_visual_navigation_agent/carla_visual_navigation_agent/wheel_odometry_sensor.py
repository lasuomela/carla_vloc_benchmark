import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped

import numpy as np

class WheelOdometrySensor(Node):

    '''
    A pseudo sensor to turn the ground truth Carla vehicle odometry
    into simulated, noisy wheel odometry
    '''

    def __init__(self):
        super().__init__('wheel_odometry_sensor')

        self.declare_parameter("role_name", "ego_vehicle")
        self.role_name  = self.get_parameter('role_name').get_parameter_value().string_value

        self.odometry_subscription =  self.create_subscription(
            Odometry,
            '/carla/{}/odometry'.format(self.role_name),
            self.odometry_cb,
            10)

        self.wheel_odometry_publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            '/carla/{}/wheel_odometry'.format(self.role_name),
            10)

        self.rng = np.random.default_rng()

        # Noise parameters for the simulated wheel odometry
        target_speed = 4
        self.odometry_noise_magnitude = 0.04
        self.odometry_noise_eps = 0.08

        loc_odo_var = target_speed * self.odometry_noise_magnitude
        loc_odo_cov = 0
        or_odo_var =  target_speed * self.odometry_noise_magnitude + self.odometry_noise_eps
        or_odo_cov =  0

        self.odometry_twist_covariance = [loc_odo_var,    loc_odo_cov,    loc_odo_cov,    0.0,    0.0,    0.0,
                                        loc_odo_cov,    loc_odo_var,    loc_odo_cov,    0.0,    0.0,    0.0,
                                        loc_odo_cov,    loc_odo_cov,    loc_odo_var,    0.0,    0.0,    0.0,
                                        0.0,    0.0,    0.0,    or_odo_var,    or_odo_cov,    or_odo_cov,
                                        0.0,    0.0,    0.0,    or_odo_cov,    or_odo_var,    or_odo_cov,
                                        0.0,    0.0,    0.0,    or_odo_cov,    or_odo_cov,    or_odo_var]
        self.odometry_twist_covariance = np.array(self.odometry_twist_covariance)

    def odometry_cb(self, msg):

        msg.twist.twist.linear.x = msg.twist.twist.linear.x + float(self.rng.normal(0, abs(msg.twist.twist.linear.x)*self.odometry_noise_magnitude, size=1))
        msg.twist.twist.angular.z = msg.twist.twist.angular.z + float(self.rng.normal(0, abs(msg.twist.twist.linear.x)*self.odometry_noise_magnitude + self.odometry_noise_eps, size=1))
        msg.twist.covariance = self.odometry_twist_covariance
        msg.header.frame_id = msg.child_frame_id
        msg = TwistWithCovarianceStamped(header = msg.header, twist=msg.twist)
        self.wheel_odometry_publisher.publish(msg)
 

def main(args=None):

    rclpy.init(args=args)
    try:
        wheel_odometry = WheelOdometrySensor()
        executor = SingleThreadedExecutor()
        executor.add_node(wheel_odometry)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        wheel_odometry.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
