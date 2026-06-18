import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_dir = get_package_share_directory("sim")
    sim_world_launch = os.path.join(sim_dir, "launch", "sim_world.launch.py")
    default_world = os.path.join(sim_dir, "worlds", "ab_navigation.world")

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
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("x", default_value="-2.0"),
        DeclareLaunchArgument("y", default_value="-1.4"),
        DeclareLaunchArgument("z", default_value="0.08"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_world_launch),
            launch_arguments={
                "world": world,
                "use_sim_time": use_sim_time,
                "headless": headless,
                "paused": paused,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
            }.items(),
        ),
    ])
