from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    camera_path = LaunchConfiguration("camera_path")
    start_height_stub = LaunchConfiguration("start_height_stub")

    default_params = PathJoinSubstitution(
        [FindPackageShare("gesture_control"), "config", "gesture_control.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML parameters for gesture_control_node.",
            ),
            DeclareLaunchArgument(
                "camera_path",
                default_value="/dev/video2",
                description="OpenCV camera path or index.",
            ),
            DeclareLaunchArgument(
                "start_height_stub",
                default_value="false",
                description="Start a local SetBodyHeight stub service for bench testing.",
            ),
            Node(
                package="gesture_control",
                executable="gesture_control_node",
                name="gesture_control_node",
                output="screen",
                parameters=[
                    params_file,
                    {"camera_path": camera_path},
                ],
            ),
            Node(
                package="gesture_control",
                executable="height_service_stub_node",
                name="height_service_stub_node",
                output="screen",
                condition=IfCondition(start_height_stub),
                parameters=[params_file],
            ),
        ]
    )
