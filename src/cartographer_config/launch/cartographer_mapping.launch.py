import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("cartographer_config")
    config_dir = os.path.join(package_dir, "config")
    lslidar_dir = get_package_share_directory("lslidar_driver")
    lslidar_launch = os.path.join(
        lslidar_dir,
        "launch",
        "lslidar_launch.py",
    )

    configuration_directory = LaunchConfiguration("configuration_directory")
    configuration_basename = LaunchConfiguration("configuration_basename")
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")
    rviz_config = LaunchConfiguration("rviz_config")
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")
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
            default_value="n10p_2d.lua",
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
            "resolution",
            default_value="0.05",
        ),
        DeclareLaunchArgument(
            "publish_period_sec",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                package_dir,
                "rviz",
                "demo_2d.rviz",
            ),
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
            condition=UnlessCondition(PythonExpression([
                "'",
                base_frame,
                "' == '",
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
            ],
            remappings=[("scan", scan_topic)],
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="occupancy_grid_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"resolution": resolution},
                {"publish_period_sec": publish_period_sec},
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
