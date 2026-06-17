import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_dir = get_package_share_directory("navigation")
    cartographer_dir = get_package_share_directory("cartographer_config")
    localization_launch = os.path.join(
        cartographer_dir,
        "launch",
        "cartographer_localization.launch.py",
    )
    default_params_file = os.path.join(
        navigation_dir,
        "config",
        "nav2_minimal.yaml",
    )
    default_rviz_config = os.path.join(
        cartographer_dir,
        "rviz",
        "demo_2d.rviz",
    )

    map_yaml = LaunchConfiguration("map")
    pbstream = LaunchConfiguration("pbstream")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    scan_topic = LaunchConfiguration("scan_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    configuration_basename = LaunchConfiguration("configuration_basename")
    use_lidar_driver = LaunchConfiguration("use_lidar_driver")
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    publish_static_laser_tf = LaunchConfiguration("publish_static_laser_tf")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_yaw = LaunchConfiguration("laser_yaw")
    laser_pitch = LaunchConfiguration("laser_pitch")
    laser_roll = LaunchConfiguration("laser_roll")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    configured_nodes = [
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Path to the Nav2 occupancy-grid map YAML file.",
        ),
        DeclareLaunchArgument(
            "pbstream",
            default_value="",
            description="Path to the Cartographer frozen state .pbstream file.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to the minimal Nav2 parameter file.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument(
            "configuration_basename",
            default_value="n10p_2d_localization.lua",
        ),
        DeclareLaunchArgument("use_lidar_driver", default_value="true"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("laser_frame", default_value="laser_link"),
        DeclareLaunchArgument("publish_static_laser_tf", default_value="true"),
        DeclareLaunchArgument("laser_x", default_value="0.0"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.0"),
        DeclareLaunchArgument("laser_yaw", default_value="0.0"),
        DeclareLaunchArgument("laser_pitch", default_value="0.0"),
        DeclareLaunchArgument("laser_roll", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "scan_topic": scan_topic,
                "imu_topic": imu_topic,
                "configuration_basename": configuration_basename,
                "load_state_filename": pbstream,
                "load_frozen_state": "true",
                "use_lidar_driver": use_lidar_driver,
                "base_frame": base_frame,
                "laser_frame": laser_frame,
                "publish_static_laser_tf": publish_static_laser_tf,
                "laser_x": laser_x,
                "laser_y": laser_y,
                "laser_z": laser_z,
                "laser_yaw": laser_yaw,
                "laser_pitch": laser_pitch,
                "laser_roll": laser_roll,
            }.items(),
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                    "yaml_filename": map_yaml,
                },
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
            package="rviz2",
            executable="rviz2",
            name="rviz2_navigation",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
