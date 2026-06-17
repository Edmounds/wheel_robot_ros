import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory("sim")
    default_world = os.path.join(sim_dir, "worlds", "ab_navigation.world")
    robot_urdf = os.path.join(sim_dir, "urdf", "two_wheel_diff_robot.urdf")

    with open(robot_urdf, "r", encoding="utf-8") as robot_file:
        robot_description = robot_file.read()

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    paused = LaunchConfiguration("paused")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("x", default_value="-2.0"),
        DeclareLaunchArgument("y", default_value="-1.4"),
        DeclareLaunchArgument("z", default_value="0.08"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        ExecuteProcess(
            cmd=[
                "gzserver",
                "--verbose",
                "--pause",
                "-s",
                "libgazebo_ros_init.so",
                "-s",
                "libgazebo_ros_factory.so",
                world,
            ],
            output="screen",
            condition=IfCondition(paused),
        ),
        ExecuteProcess(
            cmd=[
                "gzserver",
                "--verbose",
                "-s",
                "libgazebo_ros_init.so",
                "-s",
                "libgazebo_ros_factory.so",
                world,
            ],
            output="screen",
            condition=UnlessCondition(paused),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"robot_description": robot_description},
            ],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_wheel_robot",
            output="screen",
            arguments=[
                "-topic",
                "robot_description",
                "-entity",
                "wheel_robot",
                "-x",
                x,
                "-y",
                y,
                "-z",
                z,
                "-Y",
                yaw,
            ],
        ),
    ])
