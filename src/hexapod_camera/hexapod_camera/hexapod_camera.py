#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from .submodules.camera_read import CameraRead


class HexapodCamera(Node):
    """
    Reads camera feed from shared memory and publishes it in both raw and compressed (PNG) format
    """
    def __init__(self, publish_frequency=10, compression_quality=0):
        """
        Initialization of Hexapod Camera
        """
        super().__init__('hexapod_object_tracking')
        self.compression_quality = compression_quality
        self.hexapod_camera_publisher_ = self.create_publisher(Image, '/hexapod_camera_topic', rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.hexapod_camera_compressed_publisher_ = self.create_publisher(CompressedImage, '/hexapod_camera_compressed_topic', rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.get_logger().info("Hexapod Camera has been started")
        self.camera = CameraRead() 


        # Create a Timer object to limit the publishing rate to 10 frames per second
        # self.timer = self.create_timer(1.0 / publish_frequency, self.publish_image)
        self.timer = self.create_timer(1.0 / publish_frequency, self.publish_image_compressed)


    def publish_image(self):
        """
        Display latest frame from camera in shared memory
        """
        # Initialize the CvBridge object
        bridge = CvBridge()

        # Get latest frame from shared memory
        frame = self.camera.get_camera_frame()

        # Convert the frame to a ROS message
        img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Set the header fields of the message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_frame"
        img_msg.header = header

        # publish the frame
        self.hexapod_camera_publisher_.publish(img_msg)

        # Display the resulting frame
        cv2.imshow('Hexapod Camera', frame)

        # close window with the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.timer.cancel()
            cv2.destroyAllWindows()
            return
        

    def publish_image_compressed(self):
        """
        Display latest frame from camera in shared memory in PNG format (lossless compression)
        """
       # Get latest frame from shared memory
        frame = self.camera.get_camera_frame()

        # compress the image using PNG - 0 = max compression, 9 = least compression
        success, encoded_image = cv2.imencode('.png', frame, [cv2.IMWRITE_PNG_COMPRESSION, self.compression_quality])

        # Display the resulting frame
        # cv2.imshow('Hexapod Camera', frame)

        # # close window with the 'q' key
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     self.timer.cancel()
        #     cv2.destroyAllWindows()
        #     return
        

        # check if compression was successful
        if not success:
            self.get_logger().error('Failed to compress image')

        # create compressed image message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'png'
        msg.data = encoded_image.tobytes()

        # publish the message
        self.hexapod_camera_compressed_publisher_.publish(msg)
        # self.get_logger().info('Published image message')


    def destroy_node(self):
        self.timer.cancel()
        self.camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HexapodCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
