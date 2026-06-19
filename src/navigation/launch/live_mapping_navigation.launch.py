import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
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
    mapping_workflow_launch = os.path.join(
        navigation_dir,
        "launch",
        "mapping_workflow.launch.py",
    )
    default_params_file = os.path.join(
        navigation_dir,
        "config",
        "nav2_minimal.yaml",
    )
    default_rviz_config = os.path.join(
        navigation_dir,
        "rviz",
        "point_navigation.rviz",
    )
    default_maps_dir = package_data_dir("navigation", "map")

    map_name = LaunchConfiguration("map_name")
    maps_dir = LaunchConfiguration("maps_dir")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    scan_topic = LaunchConfiguration("scan_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    configuration_basename = LaunchConfiguration("configuration_basename")
    use_lidar_driver = LaunchConfiguration("use_lidar_driver")
    publish_static_laser_tf = LaunchConfiguration("publish_static_laser_tf")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_voice = LaunchConfiguration("use_voice")
    voice_websocket_enabled = LaunchConfiguration("voice_websocket_enabled")
    voice_websocket_host = LaunchConfiguration("voice_websocket_host")
    voice_websocket_port = LaunchConfiguration("voice_websocket_port")

    configured_nodes = [
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value="live_map"),
        DeclareLaunchArgument("maps_dir", default_value=default_maps_dir),
        DeclareLaunchArgument("params_file", default_value=default_params_file),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("configuration_basename", default_value="n10p_2d.lua"),
        DeclareLaunchArgument("use_lidar_driver", default_value="true"),
        DeclareLaunchArgument("publish_static_laser_tf", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument("use_voice", default_value="true"),
        DeclareLaunchArgument("voice_websocket_enabled", default_value="true"),
        DeclareLaunchArgument("voice_websocket_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("voice_websocket_port", default_value="8765"),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(mapping_workflow_launch),
                    launch_arguments={
                        "map_name": map_name,
                        "maps_dir": maps_dir,
                        "use_sim_time": use_sim_time,
                        "use_rviz": "false",
                        "use_voice": "false",
                        "use_lidar_driver": use_lidar_driver,
                        "scan_topic": scan_topic,
                        "imu_topic": imu_topic,
                        "configuration_basename": configuration_basename,
                        "publish_static_laser_tf": publish_static_laser_tf,
                    }.items(),
                ),
            ],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": configured_nodes,
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
                    "use_sim_time": use_sim_time,
                    "websocket_enabled": voice_websocket_enabled,
                    "websocket_host": voice_websocket_host,
                    "websocket_port": voice_websocket_port,
                    "persist_waypoints": False,
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_live_mapping_navigation",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
