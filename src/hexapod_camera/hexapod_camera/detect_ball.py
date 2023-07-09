#!/usr/bin/env python3
# Copyright 2023 Josh Newans
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Modified script of Josh Newans
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image, CompressedImage
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from .submodules.process_image import wait_on_gui, create_tuning_window, get_tuning_params, find_circles
import numpy as np
import cv2


class DetectBall(Node):

    def __init__(self, compression_quality=0):
        super().__init__('detect_ball')
        self.compression_quality = compression_quality
        self.get_logger().info('Looking for the ball...')
        # self.image_sub = self.create_subscription(Image,"/image_in",self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        # self.hexapod_camera_subscriber_ = self.create_subscription(Image,"/hexapod_camera_topic",self.camera_callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.hexapod_camera_compressed_subscriber_ = self.create_subscription(CompressedImage,"/hexapod_camera_compressed_topic",self.camera_compressed_callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)
        # self.image_compressed_out_pub = self.create_publisher(CompressedImage, "/image_compressed_out", 1)
        # self.image_compressed_tuning_pub = self.create_publisher(CompressedImage, "/image_compressed_tuning", 1)
        self.ball_pub  = self.create_publisher(Point,"/detected_ball",1)

        self.declare_parameter('tuning_mode', False)

        # self.declare_parameter("x_min",0)
        # self.declare_parameter("x_max",100)
        # self.declare_parameter("y_min",0)
        # self.declare_parameter("y_max",100)
        # self.declare_parameter("h_min",0)
        # self.declare_parameter("h_max",180)
        # self.declare_parameter("s_min",0)
        # self.declare_parameter("s_max",255)
        # self.declare_parameter("v_min",0)
        # self.declare_parameter("v_max",255)
        # self.declare_parameter("sz_min",0)
        # self.declare_parameter("sz_max",100)
        
        # manual entered in params
        self.declare_parameter("x_min",0)
        self.declare_parameter("x_max",100)
        self.declare_parameter("y_min",0)
        self.declare_parameter("y_max",100)
        self.declare_parameter("h_min",78)
        self.declare_parameter("h_max",173)
        self.declare_parameter("s_min",0)
        self.declare_parameter("s_max",255)
        self.declare_parameter("v_min",174)
        self.declare_parameter("v_max",255)
        self.declare_parameter("sz_min",5)
        self.declare_parameter("sz_max",30)

        # self.declare_parameter("x_min",0)
        # self.declare_parameter("x_max",100)
        # self.declare_parameter("y_min",0)
        # self.declare_parameter("y_max",100)
        # self.declare_parameter("h_min",78)
        # self.declare_parameter("h_max",160)
        # self.declare_parameter("s_min",0)
        # self.declare_parameter("s_max",110)
        # self.declare_parameter("v_min",227)
        # self.declare_parameter("v_max",255)
        # self.declare_parameter("sz_min",5)
        # self.declare_parameter("sz_max",30)
        
        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
            'h_min': self.get_parameter('h_min').get_parameter_value().integer_value,
            'h_max': self.get_parameter('h_max').get_parameter_value().integer_value,
            's_min': self.get_parameter('s_min').get_parameter_value().integer_value,
            's_max': self.get_parameter('s_max').get_parameter_value().integer_value,
            'v_min': self.get_parameter('v_min').get_parameter_value().integer_value,
            'v_max': self.get_parameter('v_max').get_parameter_value().integer_value,
            'sz_min': self.get_parameter('sz_min').get_parameter_value().integer_value,
            'sz_max': self.get_parameter('sz_max').get_parameter_value().integer_value
        }

        self.bridge = CvBridge()
        # self.tuning_mode = True
        if self.tuning_mode:
            create_tuning_window(self.tuning_params)

    def camera_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            if (self.tuning_mode):
                self.tuning_params = get_tuning_params()

            keypoints_norm, out_image, tuning_image = find_circles(cv_image, self.tuning_params)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = data.header
            self.image_tuning_pub.publish(img_to_pub)

            point_out = Point()

            # Keep the biggest point
            # They are already converted to normalised coordinates
            for i, kp in enumerate(keypoints_norm):
                x = kp.pt[0]
                y = kp.pt[1]
                s = kp.size

                self.get_logger().info(f"Pt {i}: ({x},{y},{s})")

                if (s > point_out.z):                    
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s

            if (point_out.z > 0):
                self.ball_pub.publish(point_out) 
        except CvBridgeError as e:
            print(e)  


    def camera_compressed_callback(self,img_msg):
        try:
            # decompress the image
            np_arr = np.frombuffer(img_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        try:
            if (self.tuning_mode):
                self.tuning_params = get_tuning_params()

            keypoints_norm, out_image, tuning_image = find_circles(cv_image, self.tuning_params)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = img_msg.header
            self.image_out_pub.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = img_msg.header
            self.image_tuning_pub.publish(img_to_pub)

            # # OUT IMAGE
            # # compress the image using PNG - 0 = max compression, 9 = least compression
            # success, encoded_image = cv2.imencode('.png', out_image, [cv2.IMWRITE_PNG_COMPRESSION, self.compression_quality])

            # # check if compression was successful
            # if not success:
            #     self.get_logger().error('Failed to compress image')

            # # create compressed image message
            # msg = CompressedImage()
            # msg.header.stamp = self.get_clock().now().to_msg()
            # msg.format = 'png'
            # msg.data = encoded_image.tobytes()

            # # publish the message
            # self.image_compressed_out_pub.publish(msg)


            # # TUNING IMAGE
            # # compress the image using PNG - 0 = max compression, 9 = least compression
            # success, encoded_image = cv2.imencode('.png', tuning_image, [cv2.IMWRITE_PNG_COMPRESSION, self.compression_quality])

            # # check if compression was successful
            # if not success:
            #     self.get_logger().error('Failed to compress image')

            # # create compressed image message
            # msg = CompressedImage()
            # msg.header.stamp = self.get_clock().now().to_msg()
            # msg.format = 'png'
            # msg.data = encoded_image.tobytes()

            # # publish the message
            # self.image_compressed_tuning_pub.publish(msg)

            
            point_out = Point()

            # Keep the biggest point
            # They are already converted to normalised coordinates
            for i, kp in enumerate(keypoints_norm):
                x = kp.pt[0]
                y = kp.pt[1]
                s = kp.size

                self.get_logger().info(f"Pt {i}: ({x},{y},{s})")

                if (s > point_out.z):                    
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s

            if (point_out.z > 0):
                self.ball_pub.publish(point_out) 
        except CvBridgeError as e:
            print(e)  

def main(args=None):

    rclpy.init(args=args)

    detect_ball = DetectBall()
    while rclpy.ok():
        rclpy.spin_once(detect_ball)
        wait_on_gui()

    detect_ball.destroy_node()
    rclpy.shutdown()

