from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # movement
    hexapod_movement_node = Node(
        package="hexapod",
        executable="hexapod_movement"
    )

    hexapod_controller_node = Node(
        package="hexapod",
        executable="hexapod_controller"
    )

    # sounds
    hexapod_sounds_node = Node(
        package="hexapod_sound",
        executable="hexapod_sounds"
    )

    voice_commands_node = Node(
        package="hexapod_sound",
        executable="hexapod_voice_commands"
    )

    # light
    hexapod_light_node = Node(
        package="hexapod_light",
        executable="torch"
    )

    # camera - object/ person tracking
    hexapod_camera_node = Node(
        package="hexapod_camera",
        executable="camera"
    )

    detect_ball_node = Node(
        package="hexapod_camera",
        executable="detect_ball"
    )

    detect_ball_3d_node = Node(
        package="hexapod_camera",
        executable="detect_ball_3d"
    )

    detect_person_node = Node(
        package="hexapod_camera",
        executable="detect_person"
    )

    follow_ball_node = Node(
        package="hexapod_camera",
        executable="follow_ball"
    )

    follow_person_node = Node(
        package="hexapod_camera",
        executable="follow_person"
    )

    # testing node
    hexapod_object_tracking_node = Node(
        package="hexapod_camera",
        executable="hexapod_object_tracking"
    )

    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        parameters=[
            {"image_size": [640,480]},
            {"camera_frame_id": "camera_link_optical"}
        ]
    )
    
    
    # ld.add_action(v4l2_camera_node)
    ld.add_action(hexapod_movement_node)
    ld.add_action(hexapod_controller_node)

    # ld.add_action(voice_commands_node)
    ld.add_action(hexapod_sounds_node)

    # ld.add_action(hexapod_light_node)

    # ld.add_action(hexapod_camera_node)

    # follow ball  
    # ld.add_action(detect_ball_node)
    # ld.add_action(detect_ball_3d_node)
    # ld.add_action(follow_ball_node)
    # ld.add_action(hexapod_object_tracking_node)

    # follow person
    # ld.add_action(detect_person_node)
    # ld.add_action(follow_person_node)
    
    return ld