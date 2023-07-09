#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import getch 


class HexapodController(Node):
    def __init__(self):
        """
        Initialization of Hexapod Controller
        """
        super().__init__('hexapod_controller')
        self.hexapod_controller_publisher_ = self.create_publisher(String, '/hexapod_controller_topic', 10)
        self.hexapod_movement_subscriber_ = self.create_subscription(String, '/hexapod_movement_topic', self.hexapod_movement_callback, 10)
        self.get_logger().info("Hexapod controller has been started")
        self.buffer = []
        self.read_keyboard_input()
        

    def hexapod_movement_callback(self, msg):
        # print("here")
        input_str = msg.data
        self.get_logger().info(input_str)

    def read_keyboard_input(self):
        while True:
            # Read a single character from the terminal
            char = getch.getch()

            if char == '\x1b':  # Check for escape character (ASCII code 27)
                # Check for arrow keys
                seq = char + getch.getch() + getch.getch()
                self.buffer.append(seq)
            else:
                self.buffer.append(char)
            if len(self.buffer) > 1:
                # If more than one character is buffered, remove the older characters
                self.buffer.pop(0)

            if len(self.buffer) > 0:
                input_str = None
                key = self.buffer[0]
                if "[" in key:
                    match key[-1]:
                        case'A':  # Arrow up key
                            input_str = 'forward'
                        case 'B':  # Arrow down key
                            input_str = 'back'
                        case 'C':  # Arrow right key
                            input_str = 'rotate_right'
                        case 'D':  # Arrow left key
                            input_str = 'rotate_left'

                elif key.isdigit():  # Check for numeric keys
                    match key[-1]:
                        case '1': 
                            input_str = 'sit'
                        case '2':  
                            input_str = 'stand'
                        case '3':  
                            input_str = 'neutral_standing'
                        case '4':  
                            input_str = 'follow'
                        case '5':  
                            input_str = 'on'
                        case '6':  
                            input_str = 'off'
                            
                        case '0':  
                            input_str = 'stop'

                # Publish the input as a String message
                if input_str is not None:
                    msg = String()
                    msg.data = input_str
                    self.hexapod_controller_publisher_.publish(msg)
                    self.buffer.pop(0)                
        

def main(args=None):
    rclpy.init(args=args)
    node = HexapodController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
