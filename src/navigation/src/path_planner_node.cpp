#include "path_planner/path_planner_node.hpp"

#include <cmath>
#include <thread>

#include "path_planner/math_utils.hpp"
#include "tf2/utils.h"

namespace path_planner {

namespace {

constexpr char kThetaStarMode[] = "theta_star";
constexpr char kStraightLineMode[] = "straight_line";

double yaw_from_pose(const geometry_msgs::msg::Pose& pose) {
  const auto& q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double interpolate_yaw(double start_yaw, double end_yaw, double ratio) {
  return normalize_angle(start_yaw + normalize_angle(end_yaw - start_yaw) * ratio);
}

builtin_interfaces::msg::Duration to_duration_msg(
    const rclcpp::Duration& duration) {
  const int64_t nanoseconds = duration.nanoseconds();
  builtin_interfaces::msg::Duration msg;
  msg.sec = static_cast<int32_t>(nanoseconds / 1000000000);
  msg.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000);
  return msg;
}

}  // namespace

PathPlannerNode::PathPlannerNode(const rclcpp::NodeOptions& options)
    : Node("path_planner_node", options),
      current_position_({0.0, 0.0}),
      current_yaw_(0.0),
      odom_received_(false) {
  declare_and_load_parameters();

  if (planning_mode_ != kThetaStarMode && planning_mode_ != kStraightLineMode) {
    RCLCPP_FATAL(this->get_logger(), "不支持的 planning_mode: %s",
                 planning_mode_.c_str());
    rclcpp::shutdown();
    return;
  }

  if (planning_mode_ == kThetaStarMode) {
    try {
      grid_map_ = std::make_shared<GridMap>(GridMap::from_map_yaml(
          map_yaml_path_, inflation_radius_, lethal_cost_threshold_, allow_unknown_));
      RCLCPP_INFO(this->get_logger(), "成功加载地图: %s, 膨胀半径: %.2fm",
                  map_yaml_path_.c_str(), inflation_radius_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "初始化 GridMap 失败: %s", e.what());
      rclcpp::shutdown();
    }

    theta_star_ = std::make_unique<ThetaStar>(weight_, max_search_iterations_);
  } else {
    RCLCPP_INFO(this->get_logger(), "启用 straight_line 规划模式，不加载静态地图");
  }

  // 发布路径，使用 TRANSIENT_LOCAL QoS 便于 RViz 和后续节点读取最新路径
  rclcpp::QoS plan_qos(rclcpp::KeepLast(1));
  plan_qos.transient_local();
  plan_qos.reliable();
  plan_pub_ = this->create_publisher<nav_msgs::msg::Path>(plan_topic_, plan_qos);

  if (planning_mode_ == kThetaStarMode) {
    // 发布静态 costmap
    rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
    costmap_qos.transient_local();
    costmap_qos.reliable();
    costmap_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_publish_topic_, costmap_qos);

    auto msg = grid_map_->to_occupancy_grid();
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    costmap_pub_->publish(msg);
  }

  // 订阅里程计
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&PathPlannerNode::odom_callback, this, std::placeholders::_1));

  // 订阅 RViz 2D Goal Pose
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      std::bind(&PathPlannerNode::goal_callback, this, std::placeholders::_1));

  // Service
  plan_service_ = this->create_service<navigation::srv::PlanPath>(
      "/plan_path",
      std::bind(&PathPlannerNode::plan_path_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  compute_path_action_server_ =
      rclcpp_action::create_server<ComputePathThroughPoses>(
          this,
          compute_path_action_name_,
          std::bind(&PathPlannerNode::handle_compute_path_goal, this,
                    std::placeholders::_1, std::placeholders::_2),
          std::bind(&PathPlannerNode::handle_compute_path_cancel, this,
                    std::placeholders::_1),
          std::bind(&PathPlannerNode::handle_compute_path_accepted, this,
                    std::placeholders::_1));

  clear_costmap_service_ =
      this->create_service<nav2_msgs::srv::ClearCostmapAroundRobot>(
          clear_costmap_service_name_,
          std::bind(&PathPlannerNode::clear_costmap_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "PathPlannerNode 已启动");
}

void PathPlannerNode::declare_and_load_parameters() {
  // 话题配置
  this->declare_parameter("map_yaml", "indoor_flat.yaml");
  this->declare_parameter("planning_mode", kStraightLineMode);
  this->declare_parameter("costmap_publish_topic", "/costmap");
  this->declare_parameter("odom_topic", "/Odometry");
  this->declare_parameter("goal_topic", "/direct_goal_pose");
  this->declare_parameter("plan_topic", "/plan");
  this->declare_parameter("planning_frame_id", "map");
  this->declare_parameter("compute_path_action_name", "/compute_path_through_poses");
  this->declare_parameter("clear_costmap_service_name", "/local_costmap/clear_around_robot");

  // Theta* 参数
  this->declare_parameter("inflation_radius", 0.55);
  this->declare_parameter("lethal_cost_threshold", 65);
  this->declare_parameter("allow_unknown", false);
  this->declare_parameter("weight", 1.0);
  this->declare_parameter("max_search_iterations", 250000);
  this->declare_parameter("interpolation_resolution", 0.05);

  map_yaml_path_ = this->get_parameter("map_yaml").as_string();
  planning_mode_ = this->get_parameter("planning_mode").as_string();
  costmap_publish_topic_ = this->get_parameter("costmap_publish_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  goal_topic_ = this->get_parameter("goal_topic").as_string();
  plan_topic_ = this->get_parameter("plan_topic").as_string();
  planning_frame_id_ = this->get_parameter("planning_frame_id").as_string();
  compute_path_action_name_ =
      this->get_parameter("compute_path_action_name").as_string();
  clear_costmap_service_name_ =
      this->get_parameter("clear_costmap_service_name").as_string();
  inflation_radius_ = this->get_parameter("inflation_radius").as_double();
  lethal_cost_threshold_ =
      static_cast<int8_t>(this->get_parameter("lethal_cost_threshold").as_int());
  allow_unknown_ = this->get_parameter("allow_unknown").as_bool();
  weight_ = this->get_parameter("weight").as_double();
  max_search_iterations_ = static_cast<std::size_t>(std::max<int64_t>(
      1, this->get_parameter("max_search_iterations").as_int()));
  interpolation_resolution_ =
      this->get_parameter("interpolation_resolution").as_double();
}

void PathPlannerNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  current_position_.x = msg->pose.pose.position.x;
  current_position_.y = msg->pose.pose.position.y;

  // 从四元数提取 yaw
  double siny_cosp = 2.0 * (msg->pose.pose.orientation.w *
                                 msg->pose.pose.orientation.z +
                             msg->pose.pose.orientation.x *
                                 msg->pose.pose.orientation.y);
  double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y *
                                       msg->pose.pose.orientation.y +
                                   msg->pose.pose.orientation.z *
                                       msg->pose.pose.orientation.z);
  current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  odom_received_ = true;
}

void PathPlannerNode::goal_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  WorldPoint start;
  double start_yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!odom_received_) {
      RCLCPP_WARN(this->get_logger(), "尚未收到 odom，无法使用当前位姿作为起点");
      return;
    }
    start = current_position_;
    start_yaw = current_yaw_;
  }

  WorldPoint goal = {msg->pose.position.x, msg->pose.position.y};

  RCLCPP_INFO(this->get_logger(),
              "收到 RViz 目标: (%.2f, %.2f) -> (%.2f, %.2f)",
              start.x, start.y, goal.x, goal.y);

  nav_msgs::msg::Path path_msg;
  const std::vector<WorldPoint> checkpoints {start, goal};
  const std::vector<double> checkpoint_yaws {start_yaw, yaw_from_pose(msg->pose)};
  if (plan_checkpoints(checkpoints, &checkpoint_yaws, path_msg)) {
    publish_plan(path_msg);
    RCLCPP_INFO(this->get_logger(), "路径规划成功，%zu 个点",
                path_msg.poses.size());
  } else {
    RCLCPP_WARN(this->get_logger(), "路径规划失败");
  }
}

rclcpp_action::GoalResponse PathPlannerNode::handle_compute_path_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const ComputePathThroughPoses::Goal> goal) {
  (void)uuid;
  if (goal->goals.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "拒绝 ComputePathThroughPoses 请求：goals 为空");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathPlannerNode::handle_compute_path_cancel(
    const std::shared_ptr<GoalHandleComputePathThroughPoses> goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PathPlannerNode::handle_compute_path_accepted(
    const std::shared_ptr<GoalHandleComputePathThroughPoses> goal_handle) {
  std::thread{std::bind(&PathPlannerNode::execute_compute_path, this,
                        std::placeholders::_1),
              goal_handle}
      .detach();
}

void PathPlannerNode::execute_compute_path(
    const std::shared_ptr<GoalHandleComputePathThroughPoses> goal_handle) {
  auto result = std::make_shared<ComputePathThroughPoses::Result>();
  std::string error_message;

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    return;
  }

  if (plan_compute_path_through_poses(*goal_handle->get_goal(), *result,
                                      error_message)) {
    publish_plan(result->path);
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "ComputePathThroughPoses 规划成功，%zu 个点",
                result->path.poses.size());
    return;
  }

  RCLCPP_WARN(this->get_logger(), "ComputePathThroughPoses 规划失败: %s",
              error_message.c_str());
  goal_handle->abort(result);
}

void PathPlannerNode::clear_costmap_callback(
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Request> request,
    std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Response> response) {
  (void)response;
  RCLCPP_INFO(this->get_logger(),
              "收到 ClearCostmapAroundRobot 请求 reset_distance=%.2f，point_nav 下为空操作",
              request->reset_distance);
}

void PathPlannerNode::plan_path_callback(
    const std::shared_ptr<navigation::srv::PlanPath::Request> request,
    std::shared_ptr<navigation::srv::PlanPath::Response> response) {
  // 确定起点：use_current_start 为 true 时使用当前 odom 位姿
  WorldPoint start;
  double start_yaw = 0.0;
  if (request->use_current_start) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!odom_received_) {
      response->success = false;
      response->message = "尚未收到 odom，无法确定起点";
      return;
    }
    start = current_position_;
    start_yaw = current_yaw_;
  } else {
    start = {request->start.pose.position.x, request->start.pose.position.y};
    start_yaw = yaw_from_pose(request->start.pose);
  }

  WorldPoint goal = {request->goal.pose.position.x,
                     request->goal.pose.position.y};

  // 提取途径点
  std::vector<WorldPoint> waypoints;
  std::vector<WorldPoint> checkpoints;
  std::vector<double> checkpoint_yaws;
  checkpoints.push_back(start);
  checkpoint_yaws.push_back(start_yaw);
  for (const auto& wp : request->waypoints) {
    waypoints.push_back({wp.pose.position.x, wp.pose.position.y});
    checkpoints.push_back({wp.pose.position.x, wp.pose.position.y});
    checkpoint_yaws.push_back(yaw_from_pose(wp.pose));
  }
  checkpoints.push_back(goal);
  checkpoint_yaws.push_back(yaw_from_pose(request->goal.pose));

  RCLCPP_INFO(this->get_logger(),
              "Service 请求: 起点(%.2f, %.2f) -> 终点(%.2f, %.2f), %zu 个途径点",
              start.x, start.y, goal.x, goal.y, waypoints.size());

  nav_msgs::msg::Path path_msg;
  if (plan_checkpoints(checkpoints, &checkpoint_yaws, path_msg)) {
    response->path = path_msg;
    response->success = true;
    response->message = "规划成功";
    RCLCPP_INFO(this->get_logger(), "Service 规划成功，%zu 个点",
                path_msg.poses.size());
  } else {
    response->success = false;
    response->message = "路径规划失败，可能起点或终点在障碍物内，或无可达路径";
  }
}

bool PathPlannerNode::plan(const WorldPoint& start,
                           const WorldPoint& goal,
                           const std::vector<WorldPoint>& waypoints,
                           nav_msgs::msg::Path& path_out) {
  std::vector<WorldPoint> checkpoints;
  checkpoints.push_back(start);
  for (const auto& wp : waypoints) {
    checkpoints.push_back(wp);
  }
  checkpoints.push_back(goal);
  return plan_checkpoints(checkpoints, nullptr, path_out);
}

bool PathPlannerNode::plan_checkpoints(
    const std::vector<WorldPoint>& checkpoints,
    const std::vector<double>* checkpoint_yaws,
    nav_msgs::msg::Path& path_out) {
  if (checkpoints.size() < 2) {
    return false;
  }
  if (checkpoint_yaws != nullptr && checkpoint_yaws->size() != checkpoints.size()) {
    return false;
  }

  std::shared_ptr<GridMap> grid_map = grid_map_;

  if (planning_mode_ == kThetaStarMode && !grid_map) {
    RCLCPP_ERROR(this->get_logger(), "全局 costmap 尚未加载");
    return false;
  }

  // 分段规划并拼接
  std::vector<WorldPoint> full_path;
  std::vector<double> full_yaws;
  for (size_t i = 0; i + 1 < checkpoints.size(); ++i) {
    std::vector<WorldPoint> segment;
    if (planning_mode_ == kStraightLineMode) {
      segment = {checkpoints[i], checkpoints[i + 1]};
    } else {
      segment = theta_star_->search(*grid_map, checkpoints[i], checkpoints[i + 1]);
    }
    if (segment.empty()) {
      const auto& search_failure_reason = theta_star_->last_failure_reason();
      RCLCPP_ERROR(this->get_logger(),
                   "分段规划失败: (%.2f, %.2f) -> (%.2f, %.2f)",
                   checkpoints[i].x, checkpoints[i].y,
                   checkpoints[i + 1].x, checkpoints[i + 1].y);
      if (!search_failure_reason.empty()) {
        RCLCPP_ERROR(this->get_logger(), "%s", search_failure_reason.c_str());
      }
      return false;
    }

    segment = interpolate_path(segment, interpolation_resolution_);
    std::vector<double> segment_yaws;
    if (checkpoint_yaws != nullptr) {
      const auto lengths = [&segment]() {
        std::vector<double> values;
        values.push_back(0.0);
        for (size_t index = 1; index < segment.size(); ++index) {
          const double dx = segment[index].x - segment[index - 1].x;
          const double dy = segment[index].y - segment[index - 1].y;
          values.push_back(values.back() + std::hypot(dx, dy));
        }
        return values;
      }();
      const double total_length = lengths.empty() ? 0.0 : lengths.back();
      for (size_t index = 0; index < segment.size(); ++index) {
        const double ratio = total_length <= 1e-9 ? 0.0 : lengths[index] / total_length;
        segment_yaws.push_back(
            interpolate_yaw((*checkpoint_yaws)[i], (*checkpoint_yaws)[i + 1], ratio));
      }
    }

    // 避免重复添加衔接点
    if (!full_path.empty() && !segment.empty()) {
      segment.erase(segment.begin());
      if (!segment_yaws.empty()) {
        segment_yaws.erase(segment_yaws.begin());
      }
    }
    full_path.insert(full_path.end(), segment.begin(), segment.end());
    full_yaws.insert(full_yaws.end(), segment_yaws.begin(), segment_yaws.end());
  }

  path_out = to_path_msg(full_path, checkpoint_yaws == nullptr ? nullptr : &full_yaws);
  return true;
}

bool PathPlannerNode::plan_compute_path_through_poses(
    const ComputePathThroughPoses::Goal& goal,
    ComputePathThroughPoses::Result& result,
    std::string& error_message) {
  const auto planning_started = this->now();
  result = ComputePathThroughPoses::Result();

  if (goal.goals.empty()) {
    error_message = "goals is empty";
    result.planning_time = to_duration_msg(this->now() - planning_started);
    return false;
  }

  WorldPoint start;
  double start_yaw = 0.0;
  if (goal.use_start) {
    start = {goal.start.pose.position.x, goal.start.pose.position.y};
    start_yaw = yaw_from_pose(goal.start.pose);
  } else {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!odom_received_) {
      error_message = "odom has not been received";
      result.planning_time = to_duration_msg(this->now() - planning_started);
      return false;
    }
    start = current_position_;
    start_yaw = current_yaw_;
  }

  std::vector<WorldPoint> checkpoints;
  std::vector<double> checkpoint_yaws;
  checkpoints.push_back(start);
  checkpoint_yaws.push_back(start_yaw);

  for (const auto& goal_pose : goal.goals) {
    checkpoints.push_back({goal_pose.pose.position.x, goal_pose.pose.position.y});
    checkpoint_yaws.push_back(yaw_from_pose(goal_pose.pose));
  }

  if (!plan_checkpoints(checkpoints, &checkpoint_yaws, result.path)) {
    error_message = "path planning failed";
    result.planning_time = to_duration_msg(this->now() - planning_started);
    return false;
  }

  result.planning_time = to_duration_msg(this->now() - planning_started);
  error_message.clear();
  return true;
}

nav_msgs::msg::Path PathPlannerNode::to_path_msg(
    const std::vector<WorldPoint>& points,
    const std::vector<double>* yaws) const {
  nav_msgs::msg::Path path;
  path.header.frame_id = planning_frame_id_;
  path.header.stamp = this->now();

  for (size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[i].x;
    pose.pose.position.y = points[i].y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;

    double yaw = 0.0;
    if (yaws != nullptr && i < yaws->size()) {
      yaw = (*yaws)[i];
    } else if (i + 1 < points.size()) {
      yaw = std::atan2(points[i + 1].y - points[i].y,
                       points[i + 1].x - points[i].x);
    } else if (i > 0) {
      yaw = std::atan2(points[i].y - points[i - 1].y,
                       points[i].x - points[i - 1].x);
    }
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);

    path.poses.push_back(pose);
  }

  return path;
}

void PathPlannerNode::publish_plan(const nav_msgs::msg::Path& path) {
  plan_pub_->publish(path);
}

}  // namespace path_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<path_planner::PathPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
