import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Float64, Int32
from nav_msgs.msg import Path, Odometry

import numpy as np
import threading

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point

class PathTrackingSensor(Node):

    def __init__(self):
        super().__init__('path_tracking_sensor')

        self.role_name = 'ego_vehicle'

        self.odometry_subscription =  self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odometry_cb,
            10)

        self.route_waypoints = None
        self.route_waypoint_locations_array = None
        self.path_subscriber =  self.create_subscription(
            Path,
            "/carla/{}/waypoints".format(self.role_name),
            self.path_cb,
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=10))

        self.tracking_error_publisher = self.create_publisher(
            Float64,
            '/carla/{}/path_tracking_error'.format(self.role_name),
            10)

        self.closest_wp_idx_publisher = self.create_publisher(
            Int32,
            '/carla/{}/closest_wp'.format(self.role_name),
            10)

        self.timer = self.create_timer(0.5, self.distance_to_path)
        self.latest_odometry = None
        self.lock = threading.Lock()

        self.wp_publisher = self.create_publisher(Marker, '/carla/ego_vehicle/closest_wps', 10)

    def path_cb(self, path_msg):
        with self.lock:
            self.route_waypoints = [pose.pose for pose in path_msg.poses]
            self.route_waypoint_locations_array = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in self.route_waypoints])

    def odometry_cb(self, odometry_msg):
        with self.lock:
            self.latest_odometry = odometry_msg
    
    def distance_to_path(self):
        with self.lock:
            if (self.latest_odometry is not None) and (self.route_waypoint_locations_array is not None):
                current_pose = self.latest_odometry
                current_location_np = np.array([[ current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z ]])
                distances = np.squeeze(np.linalg.norm( self.route_waypoint_locations_array - current_location_np, axis=1))

                # Indices of the two smallest distances
                idx_part = np.argpartition(distances, 2)[:(2)]
                idx_partsort = np.argsort(distances[idx_part])
                idx_ksmallest = idx_part[idx_partsort]

                wp_1 = self.route_waypoint_locations_array[idx_ksmallest[0]]
                wp_2 = self.route_waypoint_locations_array[idx_ksmallest[1]]

                a = wp_1
                ap = current_location_np - wp_1
                ab = wp_2 - wp_1
                projection = a + np.dot(ap, ab) / np.dot(ab, ab) * ab
                dist = np.linalg.norm(current_location_np - projection)

                self.tracking_error_publisher.publish(Float64(data=dist))

                if dist < 4.0:
                    self.closest_wp_idx_publisher.publish(Int32(data=int(idx_ksmallest[0])))

                marker = Marker(header=self.latest_odometry.header, scale=Vector3(x=1.0,y=1.0,z=1.0), type=8, action=0, color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
                marker.points.append(self.route_waypoints[idx_ksmallest[0]].position)
                marker.colors.append( ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
                marker.points.append(self.route_waypoints[idx_ksmallest[1]].position)
                marker.colors.append( ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))

                marker.points.append(current_pose.pose.pose.position)
                marker.colors.append( ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))

                marker.points.append(Point(x=projection[0], y=projection[1], z=projection[2]))
                marker.colors.append( ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0))

                self.wp_publisher.publish(marker)


def main(args=None):

    rclpy.init(args=args)
    try:
        path_tracker = PathTrackingSensor()
        executor = SingleThreadedExecutor()
        executor.add_node(path_tracker)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        path_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
