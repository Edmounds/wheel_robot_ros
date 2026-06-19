import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("voice_command_bridge")
    default_config_file = os.path.join(package_dir, "config", "voice_command_bridge.yaml")
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config_file),
        Node(
            package="voice_command_bridge",
            executable="voice_command_bridge_node",
            name="voice_command_bridge_node",
            output="screen",
            parameters=[
                config_file,
            ],
        ),
    ])
