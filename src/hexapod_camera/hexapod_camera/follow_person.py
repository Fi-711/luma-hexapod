#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class FollowPerson(Node):

    def __init__(self):
        super().__init__('follow_person')
        self.subscription = self.create_subscription(
            String,
            '/detected_person',
            self.follow_person,
            10)

        self.search_for_person = False
        self.can_move = True

        self.hexapod_controller_publisher_ = self.create_publisher(String, '/hexapod_controller_topic', 10)
        self.hexapod_voice_commands_subscriber_ = self.create_subscription(String, '/hexapod_sound_voice_commands_topic', self.hexapod_voice_controller_callback, 10)
        self.hexapod_controller_subscriber_ = self.create_subscription(String, '/hexapod_controller_topic', self.hexapod_voice_controller_callback, 10)
        self.hexapod_movement_subscriber_ = self.create_subscription(String, '/hexapod_movement_topic', self.hexapod_movement_callback, 10)
        self.get_logger().info("Hexapod follow person initialized.")

    def follow_person(self, res):
        command = res.data
        # print(command)
        if self.search_for_person and self.can_move:
            msg = String()
            if command == "person_found":
                msg.data= "forward"
            else:
                self.get_logger().info('Target lost')
                msg.data= "rotate_right"
            
            # publish msg
            self.hexapod_controller_publisher_.publish(msg)
            self.can_move = False
            # time.sleep(1)

    def hexapod_voice_controller_callback(self, msg):
        # Process the received keyboard input
        input_str = msg.data
  
        match input_str:
            case'follow':
                self.search_for_person = True
            case'stop':
                self.search_for_person = False

        self.get_logger().info(input_str)


    def hexapod_movement_callback(self, msg):
        match msg.data:
            case "bezier_tripod":
                self.can_move = True

def main(args=None):
    rclpy.init(args=args)
    follow_person = FollowPerson()
    rclpy.spin(follow_person)
    follow_person.destroy_node()
    rclpy.shutdown()