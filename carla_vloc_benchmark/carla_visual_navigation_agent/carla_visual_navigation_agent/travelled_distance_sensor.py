import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

import math

class TravelledDistanceSensor(Node):
    '''
    Keeps track of the distance a Carla vehicle has travelled since last reinitialization
    by the vehicle_reinitializer_node.
    '''

    def __init__(self):
        super().__init__('travelled_distance_sensor_node')

        self.declare_parameter("role_name", "ego_vehicle")
        role_name = self.get_parameter('role_name').get_parameter_value().string_value

        self.odometry_subscriber = self.create_subscription(
            Odometry,
            "/carla/{}/odometry".format(role_name),
            self.odometry_callback,
            10)

        self.init_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            "/carla/ego_vehicle/filter_reinit_pose".format(role_name),
            self.init_callback,
            rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=10))

        self.distance_publisher = self.create_publisher(
            Float64,
            '/carla/{}/travelled_distance'.format(role_name),
            10)

        self.travelled_distance = 0.0
        self.previous_location = None

    def _point_distance(self, pos1, pos2):
        dist = math.sqrt( math.pow(pos1.x-pos2.x, 2) +
                        math.pow(pos1.y-pos2.y, 2) +
                        math.pow(pos1.z-pos2.z, 2) )
        return dist

    def odometry_callback(self, odometry_msg):
        new_pose = odometry_msg.pose.pose.position
        if self.previous_location is not None:
            distance = self._point_distance(new_pose, self.previous_location)
            self.travelled_distance += distance
        self.previous_location = new_pose
        self.distance_publisher.publish( Float64(data=self.travelled_distance))

    def init_callback(self, msg):
        self.previous_location = None
        self.travelled_distance = 0.0


def main(args=None):

    rclpy.init(args=args)
    try:
        sensor = TravelledDistanceSensor()
        executor = SingleThreadedExecutor()
        executor.add_node(sensor)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()