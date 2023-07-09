#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge
import numpy as np


class HexapodObjectTracking(Node):
    def __init__(self):
        """
        Initialization of Hexapod Object Tracking
        """
        super().__init__('hexapod_object_tracking')
        self.hexapod_object_tracking_publisher_ = self.create_publisher(String, '/hexapod_object_tracking_topic', 10)
        self.get_logger().info("Hexapod Object Tracking has been started")   
        
        # Initialize the CvBridge object
        self.bridge = CvBridge()
        # self.hexapod_camera_subscriber_ = self.create_subscription(Image,"/hexapod_camera_topic",self.camera_callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        # self.hexapod_camera_compressed_subscriber_ = self.create_subscription(CompressedImage,"/hexapod_camera_compressed_topic",self.camera_compressed_callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        # self.detect_ball_image_out_subscriber_ = self.create_subscription(Image, "/image_tuning", self.camera_callback, 1)
        self.detect_ball_image_out_subscriber_ = self.create_subscription(Image, "/image_out", self.camera_callback, 1)
        # self.detect_ball_image_compressed_out_subscriber_ = self.create_subscription(CompressedImage, "/image_compressed_out", self.camera_compressed_callback, 1)
        # self.detect_ball_point_subscriber_ = self.create_subscription(Point,"/detected_ball", self.get_point_callback, 1)
        self.detect_ball3d_subscriber_  = self.create_subscription(Point,"/detected_ball_3d", self.get_point_callback,1)
        # self.detect_ball3d_marker_subscriber_  = self.create_subscription(Marker,"/ball_3d_marker", self.get_marker_callback, 1)

    def get_marker_callback(self, msg):
        """
        Display ball marker
        """
        print(f"{msg.pose.position}")

    def get_point_callback(self, msg):
        """
        Display ball's coordinates
        """
        print(f"Point({msg.x}, {msg.y}, {msg.z})")

    def camera_callback(self, img_msg):
        """
        Display latest frame from camera in shared memory
        """
        # Convert the ROS message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    
        # Display the resulting frame
        cv2.imshow('Image Window', frame)
        
        # close window with the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyWindow('Image Window')  

    def camera_compressed_callback(self, img_msg):
        """
        Display latest compressed (PNG) frame from camera in shared memory
        """    
        # decompress the image
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # display the image
        cv2.imshow('Image Window', image)
        
        # close window with the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyWindow('Image Window')
        

def main(args=None):
    rclpy.init(args=args)
    node = HexapodObjectTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()