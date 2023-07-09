import os
import sys
import time
import unittest
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

import rclpy

import std_msgs.msg


# launch feature node

def generate_test_description():
    file_path = '/home/pi-ubuntu/ros2_hexapod/src/'
    torch_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(
            file_path, "hexapod_light", "hexapod_light", "torch.py"
        )],
        additional_env = {'PYTHONUNBUFFERED': '1'},
        # parameters=[{
        #     "topic": "listener_chatter"
        # }]
    )

    hexapod_controller_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(
            file_path, "hexapod", "hexapod", "hexapod_controller.py"
        )],
        additional_env = {'PYTHONUNBUFFERED': '1'},
        # parameters=[{
        #     "topic": "/hexapod_controller_topic"
        # }]
    )

    # hexapod_voice_commands_node = launch_ros.actions.Node(
    #     executable=sys.executable,
    #     arguments=[os.path.join(
    #         file_path, "hexapod_sound", "hexapod_sound", "hexapod_voice_commands.py"
    #     )],
    #     additional_env = {'PYTHONUNBUFFERED': '1'},
    #     parameters=[{
    #         "topic": "/hexapod_sound_voice_commands_topic"
    #     }]
    # )

    return (
        launch.LaunchDescription([
            torch_node,
            hexapod_controller_node,
            # hexapod_voice_commands_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'torch': torch_node,
            'hexapod_controller': hexapod_controller_node,
            # 'hexapod_voice_commands': hexapod_voice_commands_node
        }
    )


class TestTalkerListenerLink(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_talker_listener_link')

    def tearDown(self):
        self.node.destroy_node()

    def test_talker_transmits(self, hexapod_controller, proc_output):
        msgs_rx = []

        sub = self.node.create_subscription(
            std_msgs.msg.String, '/hexapod_controller_topic', lambda msg: msgs_rx.append(msg), 10
        )

        try:
            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msgs_rx) > 2:
                    break

            self.assertGreater(len(msgs_rx), 2)

            # check output os from right node
            for msg in msgs_rx:
                proc_output.assertWaitFor(
                    expected_output = msg.data, process = hexapod_controller
                )
        finally:
            self.node.destroy_subscription(sub)

    def test_listener_receives(self, torch, proc_output):
        pub = self.node.create_publisher(std_msgs.msg.String, '/hexapod_controller_topic', 10)
        # time.sleep(2)
        try:
            msg = std_msgs.msg.String()
            msg.data = str(uuid.uuid4())

            for _ in range(10):
                pub.publish(msg)
                success = proc_output.waitFor(
                    expected_output = msg.data,
                    process = torch,
                    timeout = 1.0,
                )

                if success:
                    break
            assert success, "Waiting for output timed out"
        finally:
            self.node.destroy_publisher(pub)