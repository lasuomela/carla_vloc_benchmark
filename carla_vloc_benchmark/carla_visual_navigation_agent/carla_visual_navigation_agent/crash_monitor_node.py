
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time

import math
import message_filters
from queue import SimpleQueue

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaCollisionEvent

class CrashMonitor(Node):

    '''
    Filter the crash messages from the ROS bridge crash sensor in order to not flood the
    vehicle_reinitializer_node.
    '''

    def __init__(self):
        super().__init__('crash_monitor_node')

        self.declare_parameter("role_name", "ego_vehicle")
        self.role_name  = self.get_parameter('role_name').get_parameter_value().string_value

        self.true_odometry_subscription = message_filters.Subscriber(self, Odometry, "/carla/{}/odometry".format(self.role_name))
        self.collision_subscription = message_filters.Subscriber(self, CarlaCollisionEvent, "/carla/{}/collision".format(self.role_name))

        # With odometry frequency 20Hz queue size 10 corresponds to searching up to 0.5 sec old odometry messages (in simulation time)
        queue_size = 10
        synchrony_diff = 0.01
        self.ts = message_filters.ApproximateTimeSynchronizer([self.true_odometry_subscription, self.collision_subscription], queue_size, synchrony_diff)
        self.ts.registerCallback(self.collision_callback)

        self.crash_event_durability = Duration(seconds = 5)
        self.crash_event_proximity_filter = 5

        self.latest_crash_location = None
        self.crash_publish_queue = SimpleQueue()

        self.timer = self.create_timer(0.1, self.crash_publish_cycle)

        self.crash_publisher = self.create_publisher(
            PointStamped,
            '/carla/{}/crash_location'.format(self.role_name),
            1)

    def crash_publish_cycle(self):
        if not self.crash_publish_queue.empty():
            self.crash_publisher.publish(self.crash_publish_queue.get())

    def collision_callback(self, odometry_msg, collision_msg):
        '''
        Filter the crash messages from the ROS bridge crash sensor:

        After a crash, do not report another crash within radius 
        self.crash_event_proximity_filter before a duration
        self.crash_event_durability has passed.
        '''

        crash_location = odometry_msg.pose.pose.position

        if self.latest_crash_location is not None:
            time_since_last = Time.from_msg(collision_msg.header.stamp) - Time.from_msg(self.latest_crash_location.header.stamp)
            if time_since_last > self.crash_event_durability:
                self.latest_crash_location = None

        if self.latest_crash_location is not None:
            dist = self.point_distance(crash_location, self.latest_crash_location.point)
        else:
            dist = self.crash_event_proximity_filter

        if dist >= self.crash_event_proximity_filter:
            crash_location_msg = PointStamped(header=collision_msg.header, point=crash_location)
            self.crash_publish_queue.put( crash_location_msg )
            self.latest_crash_location = crash_location_msg 

    def point_distance(self, point1, point2):
        distance = math.sqrt(math.pow(point1.x - point2.x, 2) + math.pow(point1.y - point2.y, 2) + math.pow(point1.z - point2.z, 2))
        return distance

def main(args=None):

    rclpy.init(args=args)
    try:
        monitor = CrashMonitor()
        executor = SingleThreadedExecutor()
        executor.add_node(monitor)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()