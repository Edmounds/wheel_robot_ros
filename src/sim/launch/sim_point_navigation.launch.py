import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def package_data_dir(package_name, relative_dir):
    package_share = get_package_share_directory(package_name)
    workspace_root = os.path.abspath(os.path.join(package_share, "..", "..", "..", ".."))
    candidates = [
        os.path.join(workspace_root, package_name, relative_dir),
        os.path.join(workspace_root, "src", package_name, relative_dir),
        os.path.join(package_share, relative_dir),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return os.path.join(package_share, relative_dir)


def generate_launch_description():
    sim_dir = get_package_share_directory("sim")
    navigation_dir = get_package_share_directory("navigation")
    sim_world_launch = os.path.join(sim_dir, "launch", "sim_world.launch.py")
    point_navigation_launch = os.path.join(navigation_dir, "launch", "point_navigation.launch.py")
    default_world = os.path.join(sim_dir, "worlds", "ab_navigation.world")
    default_navigation_map_dir = package_data_dir("navigation", "map")
    default_map_dir = os.path.join(default_navigation_map_dir, "sim_lab")
    default_map_yaml = os.path.join(default_map_dir, "map.yaml")
    default_pbstream = os.path.join(default_map_dir, "sim_lab.pbstream")
    default_rviz_config = os.path.join(navigation_dir, "rviz", "sim_point_navigation.rviz")
    default_missions_dir = package_data_dir("navigation", "missions")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    paused = LaunchConfiguration("paused")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    map_yaml = LaunchConfiguration("map")
    pbstream = LaunchConfiguration("pbstream")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_voice = LaunchConfiguration("use_voice")
    voice_websocket_enabled = LaunchConfiguration("voice_websocket_enabled")
    voice_websocket_host = LaunchConfiguration("voice_websocket_host")
    voice_websocket_port = LaunchConfiguration("voice_websocket_port")
    missions_dir = LaunchConfiguration("missions_dir")
    mission_name = LaunchConfiguration("mission_name")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("x", default_value="-2.0"),
        DeclareLaunchArgument("y", default_value="-1.4"),
        DeclareLaunchArgument("z", default_value="0.08"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("map", default_value=default_map_yaml),
        DeclareLaunchArgument("pbstream", default_value=default_pbstream),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument("use_voice", default_value="true"),
        DeclareLaunchArgument("voice_websocket_enabled", default_value="true"),
        DeclareLaunchArgument("voice_websocket_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("voice_websocket_port", default_value="8765"),
        DeclareLaunchArgument("missions_dir", default_value=default_missions_dir),
        DeclareLaunchArgument("mission_name", default_value="default"),
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
            PythonLaunchDescriptionSource(point_navigation_launch),
            launch_arguments={
                "map": map_yaml,
                "pbstream": pbstream,
                "use_sim_time": "true",
                "use_rviz": use_rviz,
                "rviz_config": rviz_config,
                "use_voice": use_voice,
                "voice_websocket_enabled": voice_websocket_enabled,
                "voice_websocket_host": voice_websocket_host,
                "voice_websocket_port": voice_websocket_port,
                "use_lidar_driver": "false",
                "scan_topic": "/scan",
                "imu_topic": "/imu/data",
                "configuration_basename": "sim_2d_localization.lua",
                "publish_static_laser_tf": "false",
                "missions_dir": missions_dir,
                "mission_name": mission_name,
                "odom_topic": "/odom",
            }.items(),
        ),
    ])
