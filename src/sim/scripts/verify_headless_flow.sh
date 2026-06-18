#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_CANDIDATE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
if [ -f "${ROOT_CANDIDATE}/.colcon_install_layout" ]; then
  INSTALL_DIR="${ROOT_CANDIDATE}"
  ROOT_DIR="$(cd "${INSTALL_DIR}/.." && pwd)"
else
  ROOT_DIR="${ROOT_CANDIDATE}"
  INSTALL_DIR="${ROOT_DIR}/install"
fi
RUN_DIR="${RUN_DIR:-/tmp/wheel_robot_sim_verify_$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="${RUN_DIR}/logs"
MAP_PREFIX="${RUN_DIR}/ab_map"
PBSTREAM="${RUN_DIR}/ab_map.pbstream"
MAPS_DIR="${RUN_DIR}/maps"
MAP_NAME="ab_map"
SAVED_MAP_DIR="${MAPS_DIR}/${MAP_NAME}"

A_X="${A_X:--2.0}"
A_Y="${A_Y:--1.4}"
A_YAW="${A_YAW:-0.0}"
B_X="${B_X:-0.8}"
B_Y="${B_Y:-0.0}"
B_YAW="${B_YAW:-0.0}"

mkdir -p "${LOG_DIR}"
export ROS_LOG_DIR="${LOG_DIR}/ros"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:$((12000 + RANDOM % 20000))}"
mkdir -p "${ROS_LOG_DIR}"

log() {
  printf '[sim-verify] %s\n' "$*"
}

die() {
  log "FAILED: $*"
  exit 1
}

have_pkg() {
  ros2 pkg prefix "$1" >/dev/null 2>&1
}

require_runtime() {
  command -v ros2 >/dev/null 2>&1 || die "ros2 command not found; source /opt/ros/humble/setup.bash first"
  command -v gzserver >/dev/null 2>&1 || die "gzserver not found; install Gazebo Classic runtime"
  have_pkg gazebo_ros || die "ROS package gazebo_ros not found; install ros-humble-gazebo-ros-pkgs"
  have_pkg gazebo_plugins || die "ROS package gazebo_plugins not found; install ros-humble-gazebo-ros-pkgs"
  have_pkg nav2_theta_star_planner || die "ROS package nav2_theta_star_planner not found"
  have_pkg nav2_regulated_pure_pursuit_controller || die "ROS package nav2_regulated_pure_pursuit_controller not found"
  have_pkg cartographer_ros || die "ROS package cartographer_ros not found"
  have_pkg nav2_map_server || die "ROS package nav2_map_server not found"
}

source_workspace() {
  if [ -f "${INSTALL_DIR}/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1091
    source "${INSTALL_DIR}/setup.bash"
    set -u
  fi
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  log "waiting for topic ${topic}"
  timeout "${timeout_sec}" bash -c "until ros2 topic list | grep -qx '${topic}'; do sleep 1; done" \
    || die "topic ${topic} did not appear within ${timeout_sec}s"
}

wait_for_topic_msg() {
  local topic="$1"
  local timeout_sec="$2"
  local log_name="$3"
  log "waiting for message on ${topic}"
  timeout "${timeout_sec}" ros2 topic echo "${topic}" --once >"${LOG_DIR}/${log_name}" 2>&1 \
    || die "topic ${topic} did not publish a message within ${timeout_sec}s"
}

wait_for_service() {
  local service="$1"
  local timeout_sec="$2"
  log "waiting for service ${service}"
  timeout "${timeout_sec}" bash -c "until ros2 service list | grep -qx '${service}'; do sleep 1; done" \
    || die "service ${service} did not appear within ${timeout_sec}s"
}

wait_for_lifecycle_active() {
  local node="$1"
  local timeout_sec="$2"
  local log_file="${LOG_DIR}/${node}_lifecycle.log"
  log "waiting for ${node} lifecycle active"
  timeout "${timeout_sec}" bash -c \
    "until ros2 service call /${node}/get_state lifecycle_msgs/srv/GetState '{}' 2>/dev/null | tee '${log_file}' | grep -q 'active'; do sleep 2; done" \
    || die "${node} did not become active within ${timeout_sec}s"
}

stop_process() {
  local pid="${1:-}"
  if [ -n "${pid}" ] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "-${pid}" >/dev/null 2>&1 || kill "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
  fi
}

stop_sim_stack() {
  pkill -f "gzserver --verbose .*ab_navigation.world" >/dev/null 2>&1 || true
  pkill -f "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher --ros-args -r __node:=robot_state_publisher" >/dev/null 2>&1 || true
  pkill -f "/opt/ros/humble/lib/gazebo_ros/spawn_entity.py .* -entity wheel_robot" >/dev/null 2>&1 || true
  pkill -f "/opt/ros/humble/lib/cartographer_ros/cartographer_node .*sim_2d" >/dev/null 2>&1 || true
  pkill -f "/opt/ros/humble/lib/cartographer_ros/cartographer_occupancy_grid_node" >/dev/null 2>&1 || true
}

yaw_quaternion_zw() {
  awk -v yaw="$1" 'BEGIN {printf "%.12f %.12f\n", sin(yaw / 2.0), cos(yaw / 2.0)}'
}

cleanup() {
  stop_process "${PLAN_ECHO_PID:-}"
  stop_process "${SIM_PID:-}"
  stop_sim_stack
}
trap cleanup EXIT

require_runtime
source_workspace

if ! have_pkg sim; then
  log "sim package is not installed yet; building workspace packages"
  (cd "${ROOT_DIR}" && colcon build --packages-select cartographer_config navigation sim)
  source_workspace
fi

log "run directory: ${RUN_DIR}"
log "gazebo master: ${GAZEBO_MASTER_URI}"

log "starting headless Gazebo for mapping"
setsid ros2 launch sim sim_mapping_workflow.launch.py \
  x:="${A_X}" y:="${A_Y}" yaw:="${A_YAW}" \
  headless:=true \
  use_rviz:=false \
  use_voice:=false \
  map_name:="${MAP_NAME}" \
  maps_dir:="${MAPS_DIR}" \
  >"${LOG_DIR}/sim_mapping_workflow.log" 2>&1 &
SIM_PID=$!
wait_for_topic /clock 45
wait_for_topic /scan 60
wait_for_topic /imu/data 60
wait_for_topic_msg /clock 15 clock_mapping.log
wait_for_topic_msg /scan 30 scan_mapping.log
wait_for_topic_msg /imu/data 30 imu_mapping.log
wait_for_service /unpause_physics 30
ros2 service call /unpause_physics std_srvs/srv/Empty '{}' >"${LOG_DIR}/unpause_mapping.log" 2>&1 || true

wait_for_service /write_state 60
wait_for_service /cartographer/save_map 60
wait_for_topic /map 90

log "driving an exploration loop for mapping"
timeout 42 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.18}, angular: {z: 0.25}}" \
  >"${LOG_DIR}/mapping_drive_forward_left.log" 2>&1 || true
timeout 20 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.12}, angular: {z: -0.45}}" \
  >"${LOG_DIR}/mapping_drive_right.log" 2>&1 || true
timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" \
  >"${LOG_DIR}/mapping_drive_stop.log" 2>&1 || true

log "saving Cartographer state and Nav2 occupancy map through workflow service"
ros2 service call /cartographer/save_map std_srvs/srv/Trigger '{}' \
  >"${LOG_DIR}/cartographer_save_map.log" 2>&1
grep -q "success=True" "${LOG_DIR}/cartographer_save_map.log" || grep -q "success: true" "${LOG_DIR}/cartographer_save_map.log" \
  || die "/cartographer/save_map did not report success"
PBSTREAM="${SAVED_MAP_DIR}/${MAP_NAME}.pbstream"
MAP_PREFIX="${SAVED_MAP_DIR}/map"
test -s "${PBSTREAM}" || die "pbstream file was not created: ${PBSTREAM}"
test -s "${MAP_PREFIX}.yaml" || die "map yaml was not created"
test -s "${MAP_PREFIX}.pgm" || test -s "${MAP_PREFIX}.png" || die "map image was not created"

ros2 service call /pause_physics std_srvs/srv/Empty '{}' >"${LOG_DIR}/pause_mapping.log" 2>&1 || true
stop_process "${SIM_PID}"
SIM_PID=""
stop_sim_stack
sleep 3

log "starting point navigation workflow from waypoint A for pure localization"
setsid ros2 launch sim sim_point_navigation.launch.py \
  x:="${A_X}" y:="${A_Y}" yaw:="${A_YAW}" \
  headless:=true \
  use_rviz:=false \
  use_voice:=false \
  map:="${MAP_PREFIX}.yaml" \
  pbstream:="${PBSTREAM}" \
  >"${LOG_DIR}/sim_point_navigation.log" 2>&1 &
SIM_PID=$!
wait_for_topic /clock 45
wait_for_topic /scan 60
wait_for_topic /imu/data 60
wait_for_topic_msg /clock 15 clock_navigation.log
wait_for_topic_msg /scan 30 scan_navigation.log
wait_for_topic_msg /imu/data 30 imu_navigation.log
wait_for_service /unpause_physics 30
ros2 service call /unpause_physics std_srvs/srv/Empty '{}' >"${LOG_DIR}/unpause_navigation.log" 2>&1 || true

wait_for_service /planner_server/get_state 90
wait_for_lifecycle_active planner_server 90
wait_for_lifecycle_active controller_server 90
wait_for_lifecycle_active bt_navigator 90

ros2 param get /planner_server GridBased.plugin >"${LOG_DIR}/planner_plugin.log" 2>&1 \
  || die "failed to read planner plugin parameter"
grep -q "nav2_theta_star_planner/ThetaStarPlanner" "${LOG_DIR}/planner_plugin.log" \
  || die "planner_server is not using ThetaStarPlanner"

wait_for_topic /plan 90
wait_for_topic /odom 60
wait_for_topic_msg /odom 30 odom_navigation.log

log "sending NavigateToPose goal from A to B"
ros2 topic echo /plan >"${LOG_DIR}/plan_echo.log" 2>&1 &
PLAN_ECHO_PID=$!
read -r B_QZ B_QW < <(yaw_quaternion_zw "${B_YAW}")
NAV_GOAL="{pose: {header: {frame_id: 'map'}, pose: {position: {x: ${B_X}, y: ${B_Y}, z: 0.0}, orientation: {z: ${B_QZ}, w: ${B_QW}}}}, behavior_tree: ''}"
timeout 180 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "${NAV_GOAL}" --feedback \
  >"${LOG_DIR}/navigate_to_pose.log" 2>&1 \
  || die "NavigateToPose action timed out or failed to return"
stop_process "${PLAN_ECHO_PID}"
PLAN_ECHO_PID=""

grep -q "Goal accepted" "${LOG_DIR}/navigate_to_pose.log" \
  || die "NavigateToPose goal was not accepted"
grep -Eq "status: STATUS_SUCCEEDED|STATUS_SUCCEEDED|SUCCEEDED" "${LOG_DIR}/navigate_to_pose.log" \
  || die "NavigateToPose did not report success; see ${LOG_DIR}/navigate_to_pose.log"
grep -q "poses:" "${LOG_DIR}/plan_echo.log" \
  || die "no path message was captured on /plan"

log "SUCCESS: mapping, pure localization, Theta* navigation, and A-to-B action completed"
log "artifacts: ${RUN_DIR}"
