import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from std_msgs.msg import Float64
from visual_localization_interfaces.msg import VisualPoseEstimate

from robot_localization.srv import SetPose


class LocalizationSensorSynchronizer(Node):

    '''
    Sync the different localization sensor modalities that feed into
    Kalman filter or local planner.

    The intent is not to start publishing (wheel) odometry before the visual
    localization node is initialized and publishing.
    '''

    def __init__(self):
        super().__init__('odometry_noise_node')

        self.declare_parameter("role_name", "ego_vehicle")
        self.role_name  = self.get_parameter('role_name').get_parameter_value().string_value

        self.declare_parameter("add_noise", True)
        self.add_noise = self.get_parameter('add_noise').get_parameter_value().bool_value

        self.declare_parameter("reinit_filter", True)
        self.reinit_filter = self.get_parameter('reinit_filter').get_parameter_value().bool_value

        self.odometry_subscription =  self.create_subscription(
            Odometry,
            '/carla/{}/odometry'.format(self.role_name),
            self.odometry_cb,
            10)

        self.wheel_odometry_subscription =  self.create_subscription(
            TwistWithCovarianceStamped,
            '/carla/{}/wheel_odometry'.format(self.role_name),
            self.wheel_odometry_cb,
            10)

        self.vloc_subscription =  self.create_subscription(
            VisualPoseEstimate,
            '/carla/{}/visual_pose_estimate'.format(self.role_name),
            self.vloc_cb,
            10)

        self.true_odom_publisher = self.create_publisher(
            Odometry,
            '/carla/{}/odometry_vlocsynced'.format(self.role_name),
            10)

        self.wheel_odometry_publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            '/carla/{}/wheel_odometry_vlocsynced'.format(self.role_name),
            10)

        # Move these to vloc package
        self.vloc_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/carla/{}/vloc_estimate_synced'.format(self.role_name),
            10)

        self.vloc_delay_publisher = self.create_publisher(
            Float64,
            '/carla/{}/vloc_computation_delay'.format(self.role_name),
            10)

        self.filter_reinitialized_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/carla/{}/filter_reinit_pose'.format(self.role_name),
            qos_profile=rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=10))

        if self.reinit_filter:
            self.set_filter_pose_client = self.create_client(SetPose, 'set_pose', callback_group=ReentrantCallbackGroup())
            while not self.set_filter_pose_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

        self.filter_initialized = False
        self.future = None
        self.vloc_published = False

    def odometry_cb(self, msg):

        if not(self.filter_initialized):
            if self.reinit_filter:
                self._initialize_kalman_filter(msg)
            else:
                # In case we aren't using the Kalman filter, only send a filter_reinitialized message to
                # signal the performance evaluator node to start logging
                self.filter_reinitialized_publisher.publish(PoseWithCovarianceStamped(header=msg.header, pose=msg.pose))
                self.filter_initialized = True
        else:
            if self.vloc_published:
                self.true_odom_publisher.publish(msg)

    def wheel_odometry_cb(self, msg):
        if self.filter_initialized & self.vloc_published:
            self.wheel_odometry_publisher.publish(msg)

    def vloc_cb(self, msg):
        if self.filter_initialized:
            if (self.vloc_published):
                if msg.pnp_success.data == True:
                    self.vloc_publisher.publish(PoseWithCovarianceStamped(header=msg.header, pose=msg.pose))
            else:
                self.vloc_published = True

    def _initialize_kalman_filter(self, msg):
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

def main(args=None):

    rclpy.init(args=args)

    try:
        syncer = LocalizationSensorSynchronizer()
        executor = SingleThreadedExecutor()
        executor.add_node(syncer)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        syncer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()