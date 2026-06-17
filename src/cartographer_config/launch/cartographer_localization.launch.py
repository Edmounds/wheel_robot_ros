import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_dir = get_package_share_directory("cartographer_config")
    config_dir = os.path.join(package_dir, "config")
    lslidar_launch = PathJoinSubstitution([
        FindPackageShare("lslidar_driver"),
        "launch",
        "lslidar_launch.py",
    ])

    configuration_directory = LaunchConfiguration("configuration_directory")
    configuration_basename = LaunchConfiguration("configuration_basename")
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    load_state_filename = LaunchConfiguration("load_state_filename")
    load_frozen_state = LaunchConfiguration("load_frozen_state")
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
    lidar_use_rviz = LaunchConfiguration("lidar_use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "configuration_directory",
            default_value=config_dir,
        ),
        DeclareLaunchArgument(
            "configuration_basename",
            default_value="n10p_2d_localization.lua",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/imu/data",
        ),
        DeclareLaunchArgument(
            "load_state_filename",
            default_value="",
        ),
        DeclareLaunchArgument(
            "load_frozen_state",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_lidar_driver",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
        ),
        DeclareLaunchArgument(
            "laser_frame",
            default_value="laser_link",
        ),
        DeclareLaunchArgument(
            "publish_static_laser_tf",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "laser_x",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "laser_y",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "laser_z",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "laser_yaw",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "laser_pitch",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "laser_roll",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "lidar_use_rviz",
            default_value="false",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="laser_to_base_tf",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'",
                publish_static_laser_tf,
                "' == 'true' and '",
                base_frame,
                "' != '",
                laser_frame,
                "'",
            ])),
            arguments=[
                laser_x,
                laser_y,
                laser_z,
                laser_yaw,
                laser_pitch,
                laser_roll,
                base_frame,
                laser_frame,
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lslidar_launch),
            launch_arguments={"use_rviz": lidar_use_rviz}.items(),
            condition=IfCondition(use_lidar_driver),
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory",
                configuration_directory,
                "-configuration_basename",
                configuration_basename,
                "-load_state_filename",
                load_state_filename,
                "-load_frozen_state",
                load_frozen_state,
            ],
            remappings=[
                ("scan", scan_topic),
                ("imu", imu_topic),
            ],
        ),
    ])
