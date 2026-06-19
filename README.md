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
- `navigation`：建图工作流、Nav2 参数、导航启动、任务点录制和回放
- `gesture_control`：单目摄像头手势识别，发布 `/cmd_vel` 并通过 service 控制轮腿升降

### 手势控制

`gesture_control` 内置 Qualcomm MediaPipe-Hand QNN 必要资产。当前电脑没有 QNN/aidlite 运行环境时，节点会提示 backend 错误；上板后通常只需要在配置里改摄像头路径。

```bash
colcon build --symlink-install --packages-select gesture_control
source install/setup.bash
ros2 launch gesture_control gesture_control.launch.py
```

默认摄像头路径是 `/dev/video2`，需要时可用 `camera_path:=/dev/video0` 覆盖。

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

### 建图到导航完整流程

下面以地图名 `lab1` 为例。保存一次地图会得到两类文件：

- `src/navigation/map/lab1/lab1.pbstream`：Cartographer 纯定位加载的状态文件
- `src/navigation/map/lab1/map.yaml` 和 `map.pgm`：Nav2 `map_server` 加载的栅格地图

#### 1. 编译和加载环境

```bash
colcon build --symlink-install
source install/setup.bash
```

每个新终端都需要先执行：

```bash
source install/setup.bash
```

#### 2. 启动建图

推荐使用完整建图工作流，它会同时启动 Cartographer、雷达驱动、RViz、`map_saver_server` 和 `/cartographer/save_map` 保存服务：

```bash
ros2 launch navigation mapping_workflow.launch.py map_name:=lab1 use_voice:=false
```

如果需要语音控制，把 `use_voice:=false` 去掉或改成 `use_voice:=true`。

建图时用遥控或手柄低速走完整区域，RViz 里确认 `/map` 基本闭合、墙体没有明显重影后再保存。

如果只想启动最底层 Cartographer 建图，可以用：

```bash
ros2 launch cartographer_config cartographer_mapping.launch.py
```

雷达不在车体中心时，底层建图 launch 支持传入雷达到车体中心的静态 TF，例如：

```bash
ros2 launch cartographer_config cartographer_mapping.launch.py laser_x:=0.10 laser_y:=0.0 laser_yaw:=0.0
```

#### 3. 保存地图

如果你现在是用 `navigation mapping_workflow.launch.py` 在建图，另开一个终端执行下面这条就能保存：

```bash
source install/setup.bash
ros2 service call /cartographer/save_map std_srvs/srv/Trigger "{}"
```

成功时返回的 `message` 会包含保存路径，默认是：

```text
src/navigation/map/lab1/lab1.pbstream
src/navigation/map/lab1/map.yaml
src/navigation/map/lab1/map.pgm
```

也可以直接检查文件：

```bash
ls -lh "$PWD/src/navigation/map/lab1"
```

如果 `/cartographer/save_map` 不存在，说明当前不是用完整建图工作流启动的。只用 `cartographer_config cartographer_mapping.launch.py` 时，用手动方式保存：

```bash
MAP_DIR="$PWD/src/navigation/map/lab1"
mkdir -p "$MAP_DIR"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${MAP_DIR}/lab1.pbstream', include_unfinished_submaps: true}"
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/map"
```

#### 4. 启动定位和 Nav2 导航

保存完成后先停止建图进程，再启动定位导航。不要同时运行两个 `cartographer_node`。

```bash
source install/setup.bash
MAP_DIR="$PWD/src/navigation/map/lab1"
ros2 launch navigation navigation.launch.py \
  pbstream:="$MAP_DIR/lab1.pbstream" \
  map:="$MAP_DIR/map.yaml" \
  use_rviz:=true
```

如果雷达安装位姿不是零，导航时也要传同一组 `laser_x`、`laser_y`、`laser_z`、`laser_yaw`、`laser_pitch`、`laser_roll`。

RViz 打开后用 `Nav2 Goal` 在地图上发目标点。也可以用命令发一个 `NavigateToPose` 目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, behavior_tree: ''}" \
  --feedback
```

#### 5. 可选：录制和回放固定任务路线

如果要录制 A 点、途径点、终点并保存为任务，启动带任务管理的导航工作流：

```bash
source install/setup.bash
MAP_DIR="$PWD/src/navigation/map/lab1"
ros2 launch navigation point_navigation.launch.py \
  pbstream:="$MAP_DIR/lab1.pbstream" \
  map:="$MAP_DIR/map.yaml" \
  mission_name:=lab1_route \
  use_voice:=false
```

把车移动到对应位置后，依次调用服务采点：

```bash
ros2 service call /record_start_pose std_srvs/srv/Trigger "{}"
ros2 service call /record_waypoint std_srvs/srv/Trigger "{}"
ros2 service call /record_goal_pose std_srvs/srv/Trigger "{}"
ros2 service call /append_draft_segment std_srvs/srv/Trigger "{}"
ros2 service call /save_mission navigation/srv/SaveMission "{mission_name: 'lab1_route'}"
```

回放这条任务路线：

```bash
ros2 param set /mission_nav2_player mission_name lab1_route
ros2 service call /start_navigation_mission std_srvs/srv/Trigger "{}"
```

停止任务：

```bash
ros2 service call /stop_navigation_mission std_srvs/srv/Trigger "{}"
```

#### 6. 仿真流程

仿真建图：

```bash
source install/setup.bash
ros2 launch sim sim_mapping_workflow.launch.py map_name:=sim_lab use_voice:=false
```

保存仿真地图：

```bash
ros2 service call /cartographer/save_map std_srvs/srv/Trigger "{}"
```

仿真导航：

```bash
MAP_DIR="$PWD/src/navigation/map/sim_lab"
ros2 launch sim sim_point_navigation.launch.py \
  pbstream:="$MAP_DIR/sim_lab.pbstream" \
  map:="$MAP_DIR/map.yaml" \
  use_voice:=false
```

### 调参优先级

1. 先确认 TF 树是 `map -> odom -> base_link -> laser_link`
2. 在 RViz 里确认 `/scan` 方向正确，前方障碍显示在车前方
3. 先发 1 米以内短目标，确认 `/cmd_vel` 方向正确
4. 再调 `src/navigation/config/nav2_minimal.yaml` 里的速度、机器人半径和 inflation 半径
