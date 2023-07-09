#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from playsound import playsound
import time


class HexapodSounds(Node):
    """
    Luma sounds that play when they perform certain actions.
    """
    def __init__(self):
        super().__init__('hexapod_sounds')        
        self.get_logger().info("Hexapod Sounds Initialized.")
        self.hexapod_voice_commands_subscriber_ = self.create_subscription(String, '/hexapod_sound_voice_commands_topic', self.play_audio_file, 10)
        self.hexapod_controller_subscriber_ = self.create_subscription(String, '/hexapod_controller_topic', self.play_audio_file, 10)


    def play_audio_file(self, msg):
        # Process the received voice input
        input_str = msg.data
        match input_str:
            case "stand":
                playsound('/home/pi-ubuntu/ros2_hexapod/sounds/affection.wav')
                time.sleep(3)
                playsound('/home/pi-ubuntu/ros2_hexapod/sounds/activate.wav')
            case "sit":
                playsound('/home/pi-ubuntu/ros2_hexapod/sounds/sad.wav')
                time.sleep(3)
                playsound('/home/pi-ubuntu/ros2_hexapod/sounds/deactivate.wav')
            case "follow":
                playsound('/home/pi-ubuntu/ros2_hexapod/sounds/happy.wav')
                time.sleep(3)
                playsound('/home/pi-ubuntu/ros2_hexapod/sounds/laugh.wav')
            case "rotate_left" | "rotate_right":
                playsound('/home/pi-ubuntu/ros2_hexapod/sounds/confused.wav')
            case "forward":
                playsound('/home/pi-ubuntu/ros2_hexapod/sounds/perform_action.wav')
            case _:     
                print("No such sound.")


def main(args=None):
    rclpy.init(args=args)
    node = HexapodSounds()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
