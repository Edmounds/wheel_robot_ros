import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_dir = get_package_share_directory("sim")
    navigation_dir = get_package_share_directory("navigation")
    xbox_teleop_dir = get_package_share_directory("xbox_teleop")
    sim_world_launch = os.path.join(sim_dir, "launch", "sim_world.launch.py")
    mapping_launch = os.path.join(navigation_dir, "launch", "mapping_workflow.launch.py")
    xbox_teleop_launch = os.path.join(xbox_teleop_dir, "launch", "xbox_series_teleop.launch.py")
    default_world = os.path.join(sim_dir, "worlds", "ab_navigation.world")
    default_rviz_config = os.path.join(navigation_dir, "rviz", "sim_mapping_workflow.rviz")
    default_xbox_config = os.path.join(xbox_teleop_dir, "config", "xbox_series_teleop.yaml")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    paused = LaunchConfiguration("paused")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    map_name = LaunchConfiguration("map_name")
    maps_dir = LaunchConfiguration("maps_dir")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_voice = LaunchConfiguration("use_voice")
    use_xbox_teleop = LaunchConfiguration("use_xbox_teleop")
    xbox_config_file = LaunchConfiguration("xbox_config_file")
    joy_dev = LaunchConfiguration("joy_dev")
    joy_topic = LaunchConfiguration("joy_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("x", default_value="-2.0"),
        DeclareLaunchArgument("y", default_value="-1.4"),
        DeclareLaunchArgument("z", default_value="0.08"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("map_name", default_value="sim_lab"),
        DeclareLaunchArgument("maps_dir", default_value=os.path.expanduser("~/.ros/wheel_robot_maps")),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument("use_voice", default_value="true"),
        DeclareLaunchArgument("use_xbox_teleop", default_value="true"),
        DeclareLaunchArgument("xbox_config_file", default_value=default_xbox_config),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("joy_topic", default_value="/joy"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_world_launch),
            launch_arguments={
                "world": world,
                "use_sim_time": "true",
                "headless": headless,
                "paused": paused,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                "map_name": map_name,
                "maps_dir": maps_dir,
                "use_sim_time": "true",
                "use_rviz": use_rviz,
                "rviz_config": rviz_config,
                "use_voice": use_voice,
                "use_lidar_driver": "false",
                "scan_topic": "/scan",
                "imu_topic": "/imu/data",
                "configuration_basename": "sim_2d.lua",
                "publish_static_laser_tf": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(xbox_teleop_launch),
            condition=IfCondition(use_xbox_teleop),
            launch_arguments={
                "config_file": xbox_config_file,
                "joy_dev": joy_dev,
                "joy_topic": joy_topic,
                "cmd_vel_topic": cmd_vel_topic,
            }.items(),
        ),
    ])
