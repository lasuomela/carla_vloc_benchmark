import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor 

import message_filters
from cv_bridge import CvBridge, CvBridgeError
from rosidl_runtime_py.convert import message_to_ordereddict

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

import os
from datetime import datetime, timezone
import numpy as np
import cv2
import json
import transforms3d as t3d
import copy

from carla_visual_navigation.geometry_utils import distance_from_odometry
from visual_robot_localization.coordinate_transforms import SensorOffsetCompensator

def make_save_dir(gallery_path):
    save_dir = '{}/{}'.format(gallery_path, datetime.now().strftime("%Y-%m-%d_%H:%M:%S"))
    os.mkdir(save_dir)
    return save_dir

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture')

        self.declare_parameter("image_save_path", "/image-gallery")
        image_gallery_path = self.get_parameter('image_save_path').get_parameter_value().string_value
        self.image_save_path = make_save_dir(image_gallery_path)
        self.image_prefix = 'im'

        self.declare_parameter("image_node", "/carla/ego_vehicle/rgb_front/image")
        image_topic_name = self.get_parameter('image_node').get_parameter_value().string_value

        self.declare_parameter("odometry_node", "/carla/ego_vehicle/odometry")
        odometry_topic_name = self.get_parameter('odometry_node').get_parameter_value().string_value

        self.declare_parameter("image_density", 2.0)
        self.image_density = self.get_parameter('image_density').get_parameter_value().double_value

        self.declare_parameter("base_frame", "ego_vehicle")
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.declare_parameter("sensor_frame", "ego_vehicle/rgb_front")
        sensor_frame = self.get_parameter('sensor_frame').get_parameter_value().string_value

        self.declare_parameter("synchrony_diff", 0.01)
        synchrony_diff = self.get_parameter('synchrony_diff').get_parameter_value().double_value

        self.odometry_subscription = message_filters.Subscriber(self, Odometry, odometry_topic_name)
        self.image_subscription = message_filters.Subscriber(self, Image, image_topic_name)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.odometry_subscription, self.image_subscription], 10, synchrony_diff)
        self.ts.registerCallback(self.callback)

        self.cv_bridge = CvBridge()
        self.latest_saved_odometry = None
        
        self.publisher = self.create_publisher(Odometry, '/carla/ego_vehicle/sensor_position', 10)

        self.sensor_offset_compensator = SensorOffsetCompensator(base_frame, sensor_frame, True)


    def callback(self, odometry_msg, image_msg):

        if self.latest_saved_odometry is not None:
            dist = distance_from_odometry(self.latest_saved_odometry, odometry_msg)
        else:
            dist = None

        if (dist is None) or (dist >= self.image_density):
            self.latest_saved_odometry = odometry_msg

            time = image_msg.header.stamp
            save_name = '{}/{}_{:04d}_{}'.format(self.image_save_path, self.image_prefix, time.sec, str(time.nanosec))

            try:
                # Convert your ROS Image message to OpenCV2
                cv2_img = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
            except CvBridgeError as e:
                print(e)
            else:
                # Save your OpenCV2 image as a png 
                cv2.imwrite(save_name+'.png', cv2_img)

            odometry_msg_camera_pose = copy.deepcopy(odometry_msg)
            odometry_msg_camera_pose.pose.pose = self.sensor_offset_compensator.add_offset(odometry_msg_camera_pose.pose.pose)

            if self.publisher is not None:
                self.publisher.set_data(odometry_msg_camera_pose)

            odometry_camera_ordereddict = message_to_ordereddict(odometry_msg_camera_pose)
            with open(save_name+'_odometry_camera.json','w') as file:
                file.write(json.dumps(odometry_camera_ordereddict))

            odometry_ordereddict = message_to_ordereddict(odometry_msg)
            with open(save_name+'_odometry.json','w') as file:
                file.write(json.dumps(odometry_ordereddict))

def main(args=None):
    rclpy.init(args=args)

    executor = SingleThreadedExecutor()
    node = ImageCapture()

    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
