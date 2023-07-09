#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO


class Torch(Node):
    def __init__(self):
        super().__init__('torch')
        self.hexapod_object_tracking_publisher_ = self.create_publisher(String, '/torch_topic', 10)
        self.hexapod_controller_subscriber_ = self.create_subscription(String, '/hexapod_controller_topic', self.on_torch, 10)
        self.hexapod_voice_commands_subscriber_ = self.create_subscription(String, '/hexapod_sound_voice_commands_topic', self.on_torch, 10)
        # self.Relay_Ch1 = 26
        # self.Relay_Ch2 = 20
        self.Relay_Ch3 = 21

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # GPIO.setup(self.Relay_Ch1,GPIO.OUT)
        # GPIO.setup(self.Relay_Ch2,GPIO.OUT)
        GPIO.setup(self.Relay_Ch3,GPIO.OUT)
        self.get_logger().info("The torch has been has been initialized.")

    def on_torch(self, msg):
        """
        On the torch if follow or on command given
        """
        input_str = msg.data
        try:
            # process valid commands only
            if input_str in ["follow", "on", "off"]:
                # LOW = NC closed, the light is attahced to NC so this turns the light on
                if input_str == "follow" or input_str == "on":
                    GPIO.output(self.Relay_Ch3, GPIO.LOW)
                    print("Channel 3:The Common Contact is access to the Normal Closed Contact!\n")
                # HIGH = NO closed, dummy load attached here
                else:
                    GPIO.output(self.Relay_Ch3, GPIO.HIGH)
                    print("Channel 3:The Common Contact is access to the Normal Open Contact!")
           
        except Exception as e:
            print(e)
            GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = Torch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
