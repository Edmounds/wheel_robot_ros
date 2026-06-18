# 项目说明
这是一个轮腿机器人的控制的ROS根目录

## 功能需求
- 接收麦克风语音然后转为文字，然后输入到外部的一个模型中转为json指令，然后根据这个json指令去执行对应的任务，比如导航到A点等
- 基于nav2的theta star规划，导航，根据cartographer里程计，进行导航到A点/B点这样的任务
- 摄像头手势识别然后转换为对应的json指令

## 一晚调通的最小导航方案

当前先只做二轮车室内低速 A/B 点导航：

- 2D 激光雷达发布 `/scan`
- Cartographer 负责建图和定位，并发布 `map -> odom -> base_link` 虚拟 odom TF
- `laser_link` 通过静态 TF 挂到 `base_link`
- Nav2 只保留地图服务、全局规划、DWB 局部控制、局部/全局 costmap、基础行为树导航
- Nav2 输出速度到 `/cmd_vel`

先不做语音、手势、EKF、AMCL、多地图和复杂恢复行为。

包职责：

- `cartographer_config`：雷达启动、建图、定位、Cartographer 虚拟 odom
- `navigation`：Nav2 参数和导航启动
- `gesture_control`：单目摄像头手势识别，发布 `/cmd_vel` 并通过 service 控制轮腿升降

### 手势控制

`gesture_control` 内置 Qualcomm MediaPipe-Hand QNN 必要资产。当前电脑没有 QNN/aidlite 运行环境时，节点会提示 backend 错误；上板后通常只需要在配置里改摄像头路径。

```bash
colcon build --symlink-install --packages-select gesture_control
source install/setup.bash
ros2 launch gesture_control gesture_control.launch.py camera_path:=/dev/video0
```

升降 service 为 `gesture_control/srv/SetBodyHeight`，`command=1` 表示升高，`command=0` 表示降低。

### 依赖

如果本机还没有 Nav2：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

如果要启动 Gazebo 图形界面体验仿真流程：

```bash
colcon build --symlink-install --packages-select sim
source install/setup.bash
ros2 launch sim sim_gui.launch.py
```

另开一个终端可以直接给仿真车发速度：

```bash
source install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.3}}" -r 10
```

### 建图

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch cartographer_config cartographer_mapping.launch.py
```

如果雷达不在车体中心，启动时传入雷达到车体中心的静态 TF，例如：

```bash
ros2 launch cartographer_config cartographer_mapping.launch.py laser_x:=0.10 laser_y:=0.0 laser_yaw:=0.0
```

建图完成后保存 Cartographer 状态和 Nav2 地图：

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/tmp/n10p_map.pbstream', include_unfinished_submaps: true}"
ros2 run nav2_map_server map_saver_cli -f /tmp/n10p_map
```

### 定位 + 导航

```bash
ros2 launch navigation navigation.launch.py \
  pbstream:=/tmp/n10p_map.pbstream \
  map:=/tmp/n10p_map.yaml
```

如果雷达安装位姿不是零，同样传入 `laser_x`、`laser_y`、`laser_z`、`laser_yaw`、`laser_pitch`、`laser_roll`。

### 调参优先级

1. 先确认 TF 树是 `map -> odom -> base_link -> laser_link`
2. 在 RViz 里确认 `/scan` 方向正确，前方障碍显示在车前方
3. 先发 1 米以内短目标，确认 `/cmd_vel` 方向正确
4. 再调 `src/navigation/config/nav2_minimal.yaml` 里的速度、机器人半径和 inflation 半径
