
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time

import weakref
import math
import sys
import os
import threading
import message_filters
from queue import SimpleQueue

import carla
import carla_common.transforms as trans
sys.path.append(os.environ['SCENARIO_RUNNER_PATH'])
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.traffic_events import TrafficEvent, TrafficEventType

from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaCollisionEvent

class CrashMonitor(Node):

    def __init__(self):
        super().__init__('crash_monitor_node')

        self.role_name = 'ego_vehicle'

        self.true_odometry_subscription = message_filters.Subscriber(self, Odometry, "/carla/{}/odometry".format(self.role_name))
        self.collision_subscription = message_filters.Subscriber(self, CarlaCollisionEvent, "/carla/{}/collision".format(self.role_name))

        # With odometry frequency 20Hz queue size 100 corresponds to searching up to 5 sec old odometry messages (in simulation time)
        # If the localization algorith is very slow, you might need to increase this
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

        self.lock = threading.Lock()

    def crash_publish_cycle(self):
        if not self.crash_publish_queue.empty():
            self.crash_publisher.publish(self.crash_publish_queue.get())

    def collision_callback(self, odometry_msg, collision_msg):

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

# class CrashMonitor(Node):

#     MIN_AREA_OF_COLLISION = 3       # If closer than this distance, the collision is ignored
#     MAX_AREA_OF_COLLISION = 5       # If further than this distance, the area is forgotten
#     MAX_ID_TIME = Duration(seconds=5)                 # Amount of time the last collision if is remembered


#     def __init__(self):
#         super().__init__('crash_monitor_node')

#         self.role_name = 'ego_vehicle'

#         self.client = carla.Client('localhost', 2000)
#         self.client.set_timeout(10.0)
#         self.world=self.client.get_world()
#         self.world.wait_for_tick()


#         # Retrieve an existing collision sensor instead of creating new one
#         # Creating and destroying sensors seems prone to making carla crash
#         self.ego_vehicle = None
#         self._collision_sensor = None
#         actors = self.world.get_actors()
#         for actor in actors:
#             if 'role_name' in actor.attributes:
#                 if actor.attributes['role_name'] == self.role_name:
#                     self.ego_vehicle = actor
#                 if actor.attributes['role_name'] == 'collision':
#                     self._collision_sensor = actor

#         if self._collision_sensor is None:
#             RuntimeError('No collision sensor found!')


#         self._collision_sensor.listen(lambda event: self._count_collisions(weakref.ref(self), event))
#         self.other_actor = None
#         self.other_actor_type = 'miscellaneous'
#         self.registered_collisions = []
#         self.last_id = None
#         self.collision_time = None
#         self.actual_value = 0
#         self.list_traffic_events = []

#         self.timer = self.create_timer(0.5, self.update)

#         self.crash_publisher = self.create_publisher(
#             PointStamped,
#             '/carla/{}/crash_location'.format(self.role_name),
#             1)
#         self.last_published = None

#         self.lock = threading.Lock()

#     def destroy_node(self):
#         with self.lock:
#             self._collision_sensor.stop()
#             super(CrashMonitor, self).destroy_node()
    
#     def update(self):
#         """
#         Check collision count
#         """
#         with self.lock:

#             if self.registered_collisions:
#                 actor_location = self.ego_vehicle.get_location()

#             new_registered_collisions = []


#             # Loops through all the previous registered collisions
#             for collision_time, collision_location in self.registered_collisions:

#                 time_diff = (self.get_clock().now() - collision_time)
#                 #self.get_logger().info('Timediff {}'.format(time_diff))

#                 if time_diff < self.MAX_ID_TIME:

#                     # Get the distance to the collision point
#                     distance_vector = actor_location - collision_location
#                     distance = math.sqrt(math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))

#                     # If far away from a previous collision, forget it
#                     if distance <= self.MAX_AREA_OF_COLLISION:
#                         new_registered_collisions.append((collision_time, collision_location))

#                         if self.last_published is not None:
#                             distance_vector = self.last_published - collision_location
#                             distance = math.sqrt(math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))
#                         else:
#                             distance = self.MAX_AREA_OF_COLLISION
#                             # self.get_logger().info('Set distance to None')

#                         if distance > self.MIN_AREA_OF_COLLISION:
#                             self.last_published = collision_location
#                             collision_point = trans.carla_location_to_ros_point(collision_location)
#                             header = Header(stamp = self.get_clock().now().to_msg(), frame_id='map')
#                             collision_point_msg = PointStamped(header=header, point=collision_point)
#                             self.crash_publisher.publish(collision_point_msg)
#                         # else:
#                         #     self.get_logger().info('Its too close to previous')

#             self.registered_collisions = new_registered_collisions

#             if self.collision_time:
#                 if ((self.get_clock().now() - self.collision_time) > self.MAX_ID_TIME):
#                     self.last_id = None
#                     self.last_published = None

#     @staticmethod
#     def _count_collisions(weak_self, event):     # pylint: disable=too-many-return-statements
#         """
#         Callback to update collision count
#         """
#         self = weak_self()
#         if not self:
#             return

#         with self.lock:

#             actor_location = self.ego_vehicle.get_location()

#             # Ignore the current one if it is the same id as before
#             if self.last_id == event.other_actor.id:
#                 return

#             # Filter to only a specific actor
#             if self.other_actor and self.other_actor.id != event.other_actor.id:
#                 return

#             # Filter to only a specific type
#             if self.other_actor_type:
#                 if self.other_actor_type == "miscellaneous":
#                     if "traffic" not in event.other_actor.type_id \
#                             and "static" not in event.other_actor.type_id:
#                         return
#                 else:
#                     if self.other_actor_type not in event.other_actor.type_id:
#                         return

#             # Ignore it if its too close to a previous collision (avoid micro collisions)
#             for _, collision_location in self.registered_collisions:

#                 distance_vector = actor_location - collision_location
#                 distance = math.sqrt(math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))

#                 if distance <= self.MIN_AREA_OF_COLLISION:
#                     return

#             self.actual_value += 1
#             time = self.get_clock().now()
#             self.collision_time = time

#             self.registered_collisions.append( (time, actor_location))

#             self.last_id = event.other_actor.id


# def main(args=None):

#     rclpy.init(args=args)
#     try:
#         monitor = CrashMonitor()
#         executor = SingleThreadedExecutor()
#         executor.add_node(monitor)
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         monitor.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()