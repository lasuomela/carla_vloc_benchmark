from builtins import RuntimeError
import sys
import os
sys.path.append(os.environ['SCENARIO_RUNNER_PATH'])

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Int32, Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from carla_visual_navigation_interfaces.msg import CrashInfo
from carla_visual_navigation_interfaces.srv import ReinitializeVehicle, ToggleLocalPlanner
from robot_localization.srv import SetPose

import carla
import carla_common.transforms as trans

import numpy as np
import time
import threading


class VehicleReinitializer(Node):

    '''
    A node which teleports the Carla vehicle to the position of the farthest reached route waypoint
    in case the vehicle collides with an object in the environment.
    '''

    def __init__(self):
        super().__init__('vehicle_reinitializer_node')

        self.declare_parameter("role_name", "ego_vehicle")
        role_name = self.get_parameter('role_name').get_parameter_value().string_value

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world=self.client.get_world()
        self.world.wait_for_tick()

        cb_group = ReentrantCallbackGroup()

        self.set_filter_pose_client = self.create_client(SetPose, 'set_pose', callback_group=cb_group)
        while not self.set_filter_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("'Set filter pose' service not available, waiting again...")

        self.toggle_planner_client = self.create_client(ToggleLocalPlanner, 'toggle_local_planner', callback_group=cb_group)
        while not self.toggle_planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("'Toggle planner' service not available, waiting again...")


        self.reposition_vehicle_srv = self.create_service(ReinitializeVehicle, "/reposition_vehicle", self.set_vehicle_pose, callback_group=ReentrantCallbackGroup())
        self.reposition_vehicle_client = self.create_client(ReinitializeVehicle, 'reposition_vehicle', callback_group=cb_group)
        while not self.reposition_vehicle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("'Reposition vehicle' service not available, waiting again...")


        self.crash_subscriber = self.create_subscription(
            PointStamped,
            "/carla/{}/crash_location".format(role_name),
            self.reinitialize_vehicle_cb,
            1)

        self.path_subscriber =  self.create_subscription(
            Path,
            "/carla/{}/waypoints".format(role_name),
            self.path_cb,
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=10))

        self.closest_wp_subscriber =  self.create_subscription(
            Int32,
            "/carla/{}/closest_wp".format(role_name),
            self.closest_waypoint_cb,
            10)



        self.farthest_reached_waypoint_idx_publisher = self.create_publisher(
            Int32,
            '/carla/{}/reinit_waypoint_idx'.format(role_name),
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=10))

        self.filter_reinitialized_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/carla/ego_vehicle/filter_reinit_pose',
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=1))

        self.log_segment_publisher = self.create_publisher(
            CrashInfo,
            '/carla/ego_vehicle/log_segment',
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=1))

        self.ego_vehicle = None
        actors = self.world.get_actors()
        for actor in actors:
            if 'role_name' in actor.attributes:
                if actor.attributes['role_name'] == role_name:
                    self.ego_vehicle = actor

        
        self.route_waypoints = None
        self.route_waypoint_locations_array = None
        self.farthest_reached_waypoint_idx = 0

        self.lock = threading.Lock()


    def carla_transfrom_distance(self, transfrom1, transfrom2):
        loc1 = trans.carla_location_to_numpy_vector(transfrom1.location)
        loc2 = trans.carla_location_to_numpy_vector(transfrom2.location)

        dist = np.linalg.norm(loc2-loc1)
        return dist


    def set_vehicle_pose(self, req, response):

        waypoint = self.route_waypoints[self.farthest_reached_waypoint_idx]
        waypoint = trans.ros_pose_to_carla_transform( waypoint )

        self.ego_vehicle.set_simulate_physics(False)
        self.ego_vehicle.set_transform(waypoint)

        while self.carla_transfrom_distance(self.ego_vehicle.get_transform(), waypoint) > 0.5:
            time.sleep(0.2)

        self.ego_vehicle.set_simulate_physics(True)

        while np.linalg.norm(trans.carla_velocity_to_numpy_vector( self.ego_vehicle.get_velocity() )) > 0.01:
            time.sleep(0.2)

        response.reinitialize_result = trans.carla_transform_to_ros_pose( self.ego_vehicle.get_transform() )

        return response


    async def reinitialize_vehicle_cb(self, crash_location_msg):
        with self.lock:
            # Send a message to the performance evaluator node indicating that the vehicle has crashed and the stats for the current segment should be saved
            self.log_segment_publisher.publish(CrashInfo(crash_location=crash_location_msg, closest_waypoint_idx=Int32(data=self.farthest_reached_waypoint_idx)))

            if self.route_waypoints is None:
                raise RuntimeError("Haven't received route waypoints!")

            # Ask vehicle local planner to stop
            req = ToggleLocalPlanner.Request()
            req.stop = True
            future = self.toggle_planner_client.call_async(req)
            await future

            # Reposition the vehicle back to route
            req = ReinitializeVehicle.Request()
            req.reinitialize_request = True
            future = self.reposition_vehicle_client.call_async(req)
            result = await future
            pose = result.reinitialize_result

            # Reinitialize the Kalman filter at the new position
            req = SetPose.Request()
            req.pose = PoseWithCovarianceStamped()
            req.pose.header = Header( stamp=self.get_clock().now().to_msg(), frame_id='map')
            req.pose.pose.pose = pose
            await self.set_filter_pose_client.call_async(req)
            self.filter_reinitialized_publisher.publish(req.pose)

            # Ask vehicle local planner to start running again
            req = ToggleLocalPlanner.Request()
            req.stop = False
            future = self.toggle_planner_client.call_async(req)
            await future

            self.get_logger().info('Reinitialized vehicle')

    def path_cb(self, path_msg):
        with self.lock:
            self.route_waypoints = [pose.pose for pose in path_msg.poses]
            self.route_waypoint_locations_array = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in self.route_waypoints])

    def closest_waypoint_cb(self, wp_idx_msg):
        with self.lock:
            if self.farthest_reached_waypoint_idx < wp_idx_msg.data:
                # Prevent skipping parts of route by using shortcuts
                if wp_idx_msg.data - self.farthest_reached_waypoint_idx < 75:
                    self.farthest_reached_waypoint_idx = wp_idx_msg.data

def main(args=None):

    rclpy.init(args=args)
    try:
        reinitializer = VehicleReinitializer()
        executor = SingleThreadedExecutor()
        executor.add_node(reinitializer)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        reinitializer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()