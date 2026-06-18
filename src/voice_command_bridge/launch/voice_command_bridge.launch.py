from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="voice_command_bridge",
            executable="voice_command_bridge_node",
            name="voice_command_bridge_node",
            output="screen",
        ),
    ])
