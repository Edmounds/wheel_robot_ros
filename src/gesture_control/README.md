# gesture_control

ROS2 camera gesture control package for the wheel-leg robot.

The node reads a monocular camera through OpenCV, runs the bundled Qualcomm
MediaPipe-Hand QNN assets, converts hand landmarks into simple gesture classes,
and publishes robot control commands.

## Gesture Mapping

- `Open_Palm`: stop, publish zero `/cmd_vel`
- `Closed_Fist`: turn in place
- `Pointing_Up`: move forward
- `Thumb_Up`: call `SetBodyHeight` with `command=1`
- `Victory`: call `SetBodyHeight` with `command=0`

## Body Height Service

```srv
int8 command
---
bool ok
string message
```

`command=1` means raise, and `command=0` means lower.

## Run

```bash
colcon build --symlink-install --packages-select gesture_control
source install/setup.bash
ros2 launch gesture_control gesture_control.launch.py
```

The default camera path is `/dev/video2`. Override it with
`camera_path:=/dev/video0` when needed.

For bench testing without the real height controller:

```bash
ros2 launch gesture_control gesture_control.launch.py start_height_stub:=true
```

The current PC may not have the Qualcomm QNN/aidlite runtime. In that case the
node logs the backend error and stays alive. On the Qualcomm target, keep the
bundled W8A8 model names or adjust `config/gesture_control.yaml`.
