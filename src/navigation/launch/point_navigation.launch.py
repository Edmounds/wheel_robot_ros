import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    navigation_dir = get_package_share_directory("navigation")
    navigation_launch = os.path.join(navigation_dir, "launch", "navigation.launch.py")
    default_params_file = os.path.join(navigation_dir, "config", "nav2_minimal.yaml")
    default_navigation_map_dir = package_data_dir("navigation", "map")
    default_map_dir = os.path.join(default_navigation_map_dir, "sim_lab")
    default_map_yaml = os.path.join(default_map_dir, "map.yaml")
    default_pbstream = os.path.join(default_map_dir, "sim_lab.pbstream")
    default_rviz_config = os.path.join(navigation_dir, "rviz", "point_navigation.rviz")
    default_missions_dir = package_data_dir("navigation", "missions")

    map_yaml = LaunchConfiguration("map")
    pbstream = LaunchConfiguration("pbstream")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    scan_topic = LaunchConfiguration("scan_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    configuration_basename = LaunchConfiguration("configuration_basename")
    use_lidar_driver = LaunchConfiguration("use_lidar_driver")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    publish_static_laser_tf = LaunchConfiguration("publish_static_laser_tf")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_voice = LaunchConfiguration("use_voice")
    voice_websocket_enabled = LaunchConfiguration("voice_websocket_enabled")
    voice_websocket_host = LaunchConfiguration("voice_websocket_host")
    voice_websocket_port = LaunchConfiguration("voice_websocket_port")
    missions_dir = LaunchConfiguration("missions_dir")
    mission_name = LaunchConfiguration("mission_name")
    odom_topic = LaunchConfiguration("odom_topic")

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map_yaml),
        DeclareLaunchArgument("pbstream", default_value=default_pbstream),
        DeclareLaunchArgument("params_file", default_value=default_params_file),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("configuration_basename", default_value="n10p_2d_localization.lua"),
        DeclareLaunchArgument("use_lidar_driver", default_value="true"),
        DeclareLaunchArgument("lidar_serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("publish_static_laser_tf", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument("use_voice", default_value="true"),
        DeclareLaunchArgument("voice_websocket_enabled", default_value="true"),
        DeclareLaunchArgument("voice_websocket_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("voice_websocket_port", default_value="8765"),
        DeclareLaunchArgument("missions_dir", default_value=default_missions_dir),
        DeclareLaunchArgument("mission_name", default_value="default"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "map": map_yaml,
                "pbstream": pbstream,
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "scan_topic": scan_topic,
                "imu_topic": imu_topic,
                "configuration_basename": configuration_basename,
                "use_lidar_driver": use_lidar_driver,
                "lidar_serial_port": lidar_serial_port,
                "publish_static_laser_tf": publish_static_laser_tf,
                "use_rviz": "false",
            }.items(),
        ),
        Node(
            package="navigation",
            executable="mission_manager_node",
            name="mission_manager_node",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "missions_dir": missions_dir,
                    "odom_topic": odom_topic,
                    "frame_id": "map",
                    "planning_mode": "straight_line",
                },
            ],
        ),
        Node(
            package="navigation",
            executable="mission_nav2_player",
            name="mission_nav2_player",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "mission_name": mission_name,
                    "load_mission_service": "/load_mission",
                    "navigate_through_poses_action_name": "/navigate_through_poses",
                },
            ],
        ),
        Node(
            package="voice_command_bridge",
            executable="voice_command_bridge_node",
            name="voice_command_bridge_node",
            output="screen",
            condition=IfCondition(use_voice),
            parameters=[
                {
                    "websocket_enabled": voice_websocket_enabled,
                    "websocket_host": voice_websocket_host,
                    "websocket_port": voice_websocket_port,
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_point_navigation",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
