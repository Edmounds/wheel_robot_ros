import os
import glob
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _run_debug_command(command):
    if shutil.which(command[0]) is None:
        return f"{command[0]}: not found"

    try:
        result = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=2.0,
        )
    except subprocess.TimeoutExpired:
        return f"{' '.join(command)}: timed out"

    output = result.stdout.strip()
    if not output:
        return f"{' '.join(command)}: no output (exit {result.returncode})"
    return output


def _ros2_executable_summary(package, executable):
    if shutil.which("ros2") is None:
        return "ros2: not found"

    try:
        result = subprocess.run(
            ["ros2", "pkg", "executables", package],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=2.0,
        )
    except subprocess.TimeoutExpired:
        return f"ros2 pkg executables {package}: timed out"

    executable_lines = [
        line.strip() for line in result.stdout.splitlines()
        if line.strip()
    ]
    expected = f"{package} {executable}"
    if expected in executable_lines:
        return f"{expected} (available via ros2 package index)"
    if executable_lines:
        return (
            f"{expected} not listed; available {package} executables: "
            f"{', '.join(executable_lines)}"
        )
    return f"{expected} not listed (exit {result.returncode}): {result.stdout.strip()}"


def _input_device_summary():
    lines = []
    dev_path = "/dev/input"
    if os.path.isdir(dev_path):
        entries = sorted(os.listdir(dev_path))
        lines.append(f"{dev_path}: {' '.join(entries) if entries else '(empty)'}")
        for pattern in ("js*", "event*", "by-id/*", "by-path/*"):
            for path in sorted(glob.glob(os.path.join(dev_path, pattern))):
                try:
                    target = os.path.realpath(path)
                    stat = os.stat(path)
                    lines.append(
                        f"  {path} -> {target} mode={oct(stat.st_mode & 0o777)} "
                        f"uid={stat.st_uid} gid={stat.st_gid}"
                    )
                except OSError as exc:
                    lines.append(f"  {path}: {exc}")
    else:
        lines.append(f"{dev_path}: missing")

    proc_devices = "/proc/bus/input/devices"
    if os.path.exists(proc_devices):
        try:
            with open(proc_devices, "r", encoding="utf-8", errors="replace") as handle:
                blocks = [block.strip() for block in handle.read().split("\n\n") if block.strip()]
        except OSError as exc:
            lines.append(f"{proc_devices}: {exc}")
        else:
            candidates = [
                block for block in blocks
                if any(token in block.lower() for token in ("js", "xbox", "controller", "gamepad"))
            ]
            lines.append(f"{proc_devices}: {len(blocks)} devices, {len(candidates)} joystick candidates")
            for block in candidates[:8]:
                for row in block.splitlines():
                    if row.startswith(("N:", "H:", "B:")):
                        lines.append(f"  {row}")
    else:
        lines.append(f"{proc_devices}: missing")

    return "\n".join(lines)


def _bluetooth_summary():
    lines = [
        "bluetoothctl show:",
        _run_debug_command(["bluetoothctl", "show"]),
        "bluetoothctl devices:",
        _run_debug_command(["bluetoothctl", "devices"]),
    ]
    return "\n".join(lines)


def _emit_debug_info(context):
    joy_dev = LaunchConfiguration("joy_dev").perform(context)
    joy_topic = LaunchConfiguration("joy_topic").perform(context)
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)
    use_joy_node = LaunchConfiguration("use_joy_node").perform(context)
    joy_executable = LaunchConfiguration("joy_executable").perform(context)
    require_enable_button = LaunchConfiguration("require_enable_button").perform(context)
    enable_button = LaunchConfiguration("enable_button").perform(context)
    turbo_button = LaunchConfiguration("turbo_button").perform(context)
    linear_axis = LaunchConfiguration("linear_axis").perform(context)
    angular_axis = LaunchConfiguration("angular_axis").perform(context)

    lines = [
        "===== xbox_teleop debug info =====",
        f"launch args: use_joy_node={use_joy_node} joy_executable={joy_executable} joy_dev={joy_dev} "
        f"joy_topic={joy_topic} cmd_vel_topic={cmd_vel_topic}",
        f"teleop mapping: require_enable_button={require_enable_button} "
        f"enable_button={enable_button} turbo_button={turbo_button} "
        f"linear_axis={linear_axis} angular_axis={angular_axis}",
        f"joy_dev exists: {os.path.exists(joy_dev)}",
        f"joy executable: {_ros2_executable_summary('joy', joy_executable)}",
        f"ros2 executable: {shutil.which('ros2') or 'not found in PATH'}",
        _input_device_summary(),
        _bluetooth_summary(),
        "Expected flow: bluetooth controller -> /dev/input/jsX -> joy_node -> "
        f"{joy_topic} -> xbox_series_teleop_node -> {cmd_vel_topic}",
        "If bluetoothctl shows connected but no /dev/input/jsX exists, the issue is below ROS "
        "(kernel input device mapping, xpad/xpadneo, or permissions).",
        "===== end xbox_teleop debug info =====",
    ]
    return [LogInfo(msg="\n".join(lines))]


def generate_launch_description():
    package_dir = get_package_share_directory("xbox_teleop")
    default_config = os.path.join(package_dir, "config", "xbox_series_teleop.yaml")

    config_file = LaunchConfiguration("config_file")
    debug_info = LaunchConfiguration("debug_info")
    use_joy_node = LaunchConfiguration("use_joy_node")
    joy_executable = LaunchConfiguration("joy_executable")
    joy_dev = LaunchConfiguration("joy_dev")
    joy_topic = LaunchConfiguration("joy_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    require_enable_button = LaunchConfiguration("require_enable_button")
    enable_button = LaunchConfiguration("enable_button")
    turbo_button = LaunchConfiguration("turbo_button")
    linear_axis = LaunchConfiguration("linear_axis")
    angular_axis = LaunchConfiguration("angular_axis")
    deadzone = LaunchConfiguration("deadzone")
    autorepeat_rate = LaunchConfiguration("autorepeat_rate")
    coalesce_interval_ms = LaunchConfiguration("coalesce_interval_ms")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("debug_info", default_value="false"),
        DeclareLaunchArgument("use_joy_node", default_value="true"),
        DeclareLaunchArgument("joy_executable", default_value="joy_node"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("joy_topic", default_value="/joy"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("require_enable_button", default_value="false"),
        DeclareLaunchArgument("enable_button", default_value="5"),
        DeclareLaunchArgument("turbo_button", default_value="4"),
        DeclareLaunchArgument("linear_axis", default_value="1"),
        DeclareLaunchArgument("angular_axis", default_value="2"),
        DeclareLaunchArgument("deadzone", default_value="0.05"),
        DeclareLaunchArgument("autorepeat_rate", default_value="30.0"),
        DeclareLaunchArgument("coalesce_interval_ms", default_value="1"),
        OpaqueFunction(
            function=_emit_debug_info,
            condition=IfCondition(debug_info),
        ),
        Node(
            package="joy",
            executable=joy_executable,
            name="joy_node",
            output="screen",
            condition=IfCondition(use_joy_node),
            parameters=[
                {
                    "dev": joy_dev,
                    "deadzone": deadzone,
                    "autorepeat_rate": autorepeat_rate,
                    "coalesce_interval_ms": coalesce_interval_ms,
                },
            ],
            remappings=[("joy", joy_topic)],
        ),
        Node(
            package="xbox_teleop",
            executable="xbox_series_teleop_node",
            name="xbox_series_teleop_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "joy_topic": joy_topic,
                    "cmd_vel_topic": cmd_vel_topic,
                    "require_enable_button": require_enable_button,
                    "enable_button": enable_button,
                    "turbo_button": turbo_button,
                    "linear_axis": linear_axis,
                    "angular_axis": angular_axis,
                },
            ],
        ),
    ])
