#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import time


class VoiceCommands(Node):
    def __init__(self):
        """
        Initialization of Voice Commands Module
        """
        super().__init__('hexapod_voice_commands')
        self.get_logger().info("Voice commands module started.")
        self.hexapod_voice_commands_publisher_ = self.create_publisher(String, '/hexapod_sound_voice_commands_topic', 10)
        self.has_printed_start_message = False
        self.speech_to_text()

    def speech_to_text(self):
        msg = String()

        if not self.has_printed_start_message:
            msg.data = "Speech module successfully started. Accepting commands."
            self.hexapod_voice_commands_publisher_.publish(msg)
            self.has_printed_start_message = True
        r = sr.Recognizer()
        mic = sr.Microphone()

        while True:
            try:
                with mic as source:
                    audio = r.listen(source)
                words = r.recognize_google(audio)

                if len(words) > 1:
                    print(f"=====================\n\n\n{words}\n\n\n===============================")
                    command = self.filter_words(words)
                    print(f"=====================\n\n\n{command}\n\n\n===============================")
                    # Only accept commands with Luma in it
                    if command is not None:
                        msg.data = command
                        self.hexapod_voice_commands_publisher_.publish(msg)
                time.sleep(0.5)
            except Exception as e:
                pass
  


    def filter_words(self, words):
        """
        Filter sentences for commands
        """
        # lower case letters and split into separate words
        res = words.lower().split(" ")

        if "luma" in res:
            idx = res.index("luma")
            command = res[idx::]
        elif "loomah" in res:
            idx = res.index("loomah")
            command = res[idx::]
        elif "looma" in res:
            idx = res.index("looma")
            command = res[idx::]
        elif "lumos" in res:
            idx = res.index("lumos")
            command = res[idx::]
        
        # command must be at least length two
        if len(command) > 2:
            match f"{command[1]}_{command[2]}":
                case 'rotate_right':
                    command = "rotate_right"
                case 'rotate_left':
                    command = "rotate_left"
                case 'neutral_standing': 
                    command = "neutral_standing"
                case _:  # default
                    command = None
        elif len(command) > 1:
             match command[1]:
                case'forward': # up
                    command = "forward"
                    print('working')
                case 'back': # down
                    command = "back"
                case 'sit': # 1
                    command = "sit"
                case 'stand':  # 2
                    command = "stand"
                case 'stop':  # default
                    command = "stop"
                case 'follow':
                     command = "follow"
                case 'on':
                     command = "on"
                case 'off':
                     command = "off"     
                case _:
                     command = None      

        return command


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommands()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
