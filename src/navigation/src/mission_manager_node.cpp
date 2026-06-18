#include "path_planner/mission_manager_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "path_planner/math_utils.hpp"
#include "path_planner/mission_io.hpp"

namespace path_planner {

namespace {

using namespace std::chrono_literals;

constexpr char kMapFrameId[] = "map";

std::string sanitize_file_stem(const std::string& input) {
  std::string output = input;
  std::transform(output.begin(), output.end(), output.begin(), [](unsigned char c) {
    if (std::isalnum(c) != 0 || c == '_' || c == '-') {
      return static_cast<char>(c);
    }
    return '_';
  });
  output.erase(
      std::remove_if(output.begin(), output.end(),
                     [](unsigned char c) { return std::isspace(c) != 0; }),
      output.end());
  if (output.empty()) {
    output = "mission";
  }
  return output;
}

geometry_msgs::msg::PoseStamped to_pose_stamped(const MissionPose& pose,
                                                const rclcpp::Time& stamp,
                                                const std::string& frame_id) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.pose.position.x = pose.x;
  msg.pose.position.y = pose.y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = std::sin(pose.yaw * 0.5);
  msg.pose.orientation.w = std::cos(pose.yaw * 0.5);
  return msg;
}

nav_msgs::msg::Path build_path_from_points(
    const std::vector<MissionPose>& points, const rclcpp::Time& stamp,
    const std::string& frame_id) {
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  for (const auto& point : points) {
    path.poses.push_back(to_pose_stamped(point, stamp, frame_id));
  }
  return path;
}

std::string json_escape(const std::string& text) {
  std::ostringstream oss;
  for (const char c : text) {
    switch (c) {
      case '\\':
        oss << "\\\\";
        break;
      case '"':
        oss << "\\\"";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      case '\b':
        oss << "\\b";
        break;
      case '\f':
        oss << "\\f";
        break;
      default:
        // 过滤 ASCII 控制字符
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          oss << buf;
        } else {
          oss << c;
        }
        break;
    }
  }
  return oss.str();
}

std::string pose_json(const MissionPose& pose) {
  std::ostringstream oss;
  oss << "{\"x\":" << pose.x << ",\"y\":" << pose.y << ",\"yaw\":" << pose.yaw << "}";
  return oss.str();
}

std::string segment_name_for_index(std::size_t index) {
  std::ostringstream oss;
  oss << "seg_" << std::setw(3) << std::setfill('0') << (index + 1);
  return oss.str();
}


}  // namespace

MissionManagerNode::MissionManagerNode() : Node("mission_manager_node") {
  declare_and_load_parameters();
  mission_.frame_id = planning_frame_id();
  mission_.planning_mode = planning_mode_;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&MissionManagerNode::odom_callback, this, std::placeholders::_1));
  rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
  latched_qos.transient_local();
  latched_qos.reliable();
  status_pub_ =
      this->create_publisher<std_msgs::msg::String>("/mission_manager/status", latched_qos);
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/mission_markers", latched_qos);
  preview_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/mission_preview_path", rclcpp::QoS(1).transient_local().reliable());
  record_start_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/record_start_pose",
      std::bind(&MissionManagerNode::record_start_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  record_waypoint_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/record_waypoint",
      std::bind(&MissionManagerNode::record_waypoint_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  record_goal_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/record_goal_pose",
      std::bind(&MissionManagerNode::record_goal_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  undo_waypoint_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/undo_waypoint",
      std::bind(&MissionManagerNode::undo_waypoint_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  clear_draft_segment_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/clear_draft_segment",
      std::bind(&MissionManagerNode::clear_draft_segment_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  append_draft_segment_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/append_draft_segment",
      std::bind(&MissionManagerNode::append_draft_segment_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  remove_last_segment_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/remove_last_segment",
      std::bind(&MissionManagerNode::remove_last_segment_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  clear_loaded_mission_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/clear_loaded_mission",
      std::bind(&MissionManagerNode::clear_loaded_mission_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  save_mission_srv_ = this->create_service<navigation::srv::SaveMission>(
      "/save_mission",
      std::bind(&MissionManagerNode::save_mission_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  load_mission_srv_ = this->create_service<navigation::srv::LoadMission>(
      "/load_mission",
      std::bind(&MissionManagerNode::load_mission_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::declare_and_load_parameters() {
  std::string default_missions_dir;
  try {
    const auto package_share =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("navigation"));
    const auto installed_missions_dir = package_share / "missions";
    const auto workspace_root =
        package_share.parent_path().parent_path().parent_path().parent_path();
    const std::vector<std::filesystem::path> mission_dir_candidates {
        workspace_root / "navigation" / "missions",
        workspace_root / "src" / "navigation" / "missions",
        installed_missions_dir,
    };
    for (const auto& candidate : mission_dir_candidates) {
      if (std::filesystem::exists(candidate)) {
        default_missions_dir = candidate.string();
        break;
      }
    }
    if (default_missions_dir.empty()) {
      default_missions_dir = installed_missions_dir.string();
    }
  } catch (const std::exception&) {
    default_missions_dir = "missions";
  }

  this->declare_parameter("missions_dir", default_missions_dir);
  this->declare_parameter("odom_topic", "/Odometry");
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("planning_mode", "straight_line");
  this->declare_parameter("pose_avg_window_size", 20);

  missions_dir_ = this->get_parameter("missions_dir").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  planning_mode_ = this->get_parameter("planning_mode").as_string();
  pose_avg_window_size_ = static_cast<std::size_t>(
      std::max(1, static_cast<int>(this->get_parameter("pose_avg_window_size").as_int())));
}

void MissionManagerNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::string error_message;
  bool success = false;
  const MissionPose map_pose = pose_from_msg_in_map(*msg, success, error_message);
  if (!success) {
    RCLCPP_WARN(this->get_logger(), "%s", error_message.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(pose_mutex_);
  current_pose_ = map_pose;
  odom_received_ = true;

  // 维护打点均值滤波的滑动窗口
  pose_buffer_.push_back(map_pose);
  if (pose_buffer_.size() > pose_avg_window_size_) {
    pose_buffer_.pop_front();
  }
}

void MissionManagerNode::record_start_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  std::string error_message;
  if (!capture_current_pose(draft_start_, error_message)) {
    response->success = false;
    response->message = error_message;
  } else {
    draft_has_start_ = true;
    last_result_message_ = "已采集当前段起点";
    response->success = true;
    response->message = last_result_message_;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::record_waypoint_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  MissionPose waypoint;
  std::string error_message;
  if (!capture_current_pose(waypoint, error_message)) {
    response->success = false;
    response->message = error_message;
  } else {
    draft_waypoints_.push_back(waypoint);
    last_result_message_ = "已采集当前段途径点";
    response->success = true;
    response->message = last_result_message_;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::record_goal_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  std::string error_message;
  if (!capture_current_pose(draft_goal_, error_message)) {
    response->success = false;
    response->message = error_message;
  } else {
    draft_has_goal_ = true;
    last_result_message_ = "已采集当前段终点";
    response->success = true;
    response->message = last_result_message_;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::undo_waypoint_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  if (draft_waypoints_.empty()) {
    response->success = false;
    response->message = "当前草稿段没有可撤销的途径点";
  } else {
    draft_waypoints_.pop_back();
    last_result_message_ = "已撤销最后一个途径点";
    response->success = true;
    response->message = last_result_message_;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::clear_draft_segment_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  clear_draft_segment();
  last_result_message_ = "已清空当前草稿段";
  response->success = true;
  response->message = last_result_message_;
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::append_draft_segment_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  std::string error_message;
  response->success = append_current_draft_segment(error_message);
  response->message = response->success ? last_result_message_ : error_message;
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::remove_last_segment_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  if (mission_.segments.empty()) {
    response->success = false;
    response->message = "当前 mission 中没有已追加的 segment";
  } else {
    mission_.segments.pop_back();
    if (mission_.segments.empty()) {
      mission_.mission_name.clear();
      execution_state_ = ExecutionState::kIdle;
    }
    last_result_message_ = "已删除最后一个 segment";
    response->success = true;
    response->message = last_result_message_;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::clear_loaded_mission_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  clear_loaded_mission();
  last_result_message_ = "已清空当前 mission";
  response->success = true;
  response->message = last_result_message_;
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::save_mission_callback(
    const std::shared_ptr<navigation::srv::SaveMission::Request> request,
    std::shared_ptr<navigation::srv::SaveMission::Response> response) {
  std::string saved_path;
  std::string error_message;
  response->success =
      save_current_mission(request->mission_name, saved_path, error_message);
  response->message = response->success ? last_result_message_ : error_message;
  response->file_path = saved_path;
  publish_status();
}

void MissionManagerNode::load_mission_callback(
    const std::shared_ptr<navigation::srv::LoadMission::Request> request,
    std::shared_ptr<navigation::srv::LoadMission::Response> response) {
  MissionDefinition mission;
  std::string file_path;
  std::string yaml_text;
  std::string error_message;
  if (load_named_mission(request->mission_name, mission, file_path, yaml_text,
                         error_message)) {
    mission_ = mission;
    clear_draft_segment();
    execution_state_ = ExecutionState::kMissionLoaded;
    last_loaded_path_ = file_path;
    last_result_message_ = "已加载 mission: " + mission_.mission_name;
    response->success = true;
    response->message = last_result_message_;
    response->file_path = file_path;
    response->mission_yaml = yaml_text;
  } else {
    response->success = false;
    response->message = error_message;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

bool MissionManagerNode::capture_current_pose(MissionPose& pose_out,
                                              std::string& error_message) const {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!odom_received_) {
    error_message = "尚未收到 /Odometry 或无法换算到 map，无法采集当前位姿";
    return false;
  }
  if (pose_buffer_.empty()) {
    pose_out = current_pose_;
    return true;
  }

  // 对缓冲区内的 pose 取均值，过滤定位噪声
  const double n = static_cast<double>(pose_buffer_.size());
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_sin = 0.0;
  double sum_cos = 0.0;
  for (const auto& p : pose_buffer_) {
    sum_x += p.x;
    sum_y += p.y;
    sum_sin += std::sin(p.yaw);
    sum_cos += std::cos(p.yaw);
  }
  pose_out.x = sum_x / n;
  pose_out.y = sum_y / n;
  // 角度均值用 atan2 处理周期性
  pose_out.yaw = std::atan2(sum_sin / n, sum_cos / n);
  return true;
}

const std::vector<MissionSegment>& MissionManagerNode::display_segments() const {
  return mission_.segments;
}

MissionPose MissionManagerNode::pose_from_msg_in_map(
    const nav_msgs::msg::Odometry& msg, bool& success, std::string& error_message) const {
  success = false;
  if (msg.header.frame_id.empty()) {
    error_message = "Odometry header.frame_id 为空，无法换算到 map";
    return MissionPose {};
  }

  geometry_msgs::msg::PoseStamped source_pose;
  source_pose.header = msg.header;
  source_pose.pose = msg.pose.pose;

  try {
    const auto map_to_source =
        tf_buffer_->lookupTransform(kMapFrameId, msg.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped map_pose_msg;
    tf2::doTransform(source_pose, map_pose_msg, map_to_source);

    MissionPose pose;
    pose.x = map_pose_msg.pose.position.x;
    pose.y = map_pose_msg.pose.position.y;
    pose.yaw = tf2::getYaw(map_pose_msg.pose.orientation);
    success = true;
    return pose;
  } catch (const tf2::TransformException& ex) {
    error_message = std::string("无法将当前位姿换算到 map: ") + ex.what();
    return MissionPose {};
  }
}

std::string MissionManagerNode::planning_frame_id() const {
  if (!mission_.frame_id.empty()) {
    return mission_.frame_id;
  }
  if (!frame_id_.empty()) {
    return frame_id_;
  }
  return kMapFrameId;
}

bool MissionManagerNode::append_current_draft_segment(std::string& error_message) {
  if (!draft_has_start_) {
    error_message = "当前草稿段还没有起点";
    return false;
  }
  if (!draft_has_goal_) {
    error_message = "当前草稿段还没有终点";
    return false;
  }

  if (mission_.segments.empty()) {
    mission_.frame_id = planning_frame_id();
    mission_.planning_mode = planning_mode_;
  }

  MissionSegment segment;
  segment.segment_name = segment_name_for_index(mission_.segments.size());
  segment.start = draft_start_;
  segment.waypoints = draft_waypoints_;
  segment.goal = draft_goal_;
  mission_.segments.push_back(segment);
  if (mission_.mission_name.empty()) {
    mission_.mission_name = "unnamed_mission";
  }
  execution_state_ = ExecutionState::kMissionLoaded;
  clear_draft_segment();
  last_result_message_ = "已追加新的 segment: " + segment.segment_name;
  return true;
}

bool MissionManagerNode::save_current_mission(const std::string& mission_name,
                                              std::string& saved_path,
                                              std::string& error_message) {
  if (mission_.segments.empty()) {
    error_message = "当前 mission 为空，至少追加一个 segment 后才能保存";
    return false;
  }
  if (!draft_waypoints_.empty() || draft_has_start_ || draft_has_goal_) {
    error_message = "当前还有未追加的草稿段，请先追加或清空后再保存";
    return false;
  }

  mission_.frame_id = planning_frame_id();
  mission_.planning_mode = planning_mode_.empty() ? "straight_line" : planning_mode_;
  mission_.mission_name = mission_name.empty() ? mission_.mission_name : mission_name;
  if (mission_.mission_name.empty()) {
    error_message = "mission_name 不能为空";
    return false;
  }

  std::filesystem::create_directories(missions_dir_);
  saved_path = mission_file_path(mission_.mission_name);
  if (!save_mission_file(saved_path, mission_, error_message)) {
    return false;
  }
  last_saved_path_ = saved_path;
  last_result_message_ = "已保存 mission 到 " + saved_path;
  return true;
}

bool MissionManagerNode::load_named_mission(const std::string& mission_name,
                                            MissionDefinition& mission_out,
                                            std::string& file_path_out,
                                            std::string& yaml_out,
                                            std::string& error_message) {
  if (mission_name.empty()) {
    error_message = "mission_name 不能为空";
    return false;
  }

  file_path_out = mission_file_path(mission_name);
  if (!load_mission_file(file_path_out, mission_out, error_message)) {
    return false;
  }
  yaml_out = mission_to_yaml_string(mission_out);
  return true;
}



void MissionManagerNode::clear_draft_segment() {
  draft_has_start_ = false;
  draft_has_goal_ = false;
  draft_start_ = MissionPose {};
  draft_goal_ = MissionPose {};
  draft_waypoints_.clear();
}

void MissionManagerNode::clear_loaded_mission() {
  stop_execution("已清空任务并停止执行", ExecutionState::kIdle);
  mission_ = MissionDefinition {};
  mission_.frame_id = planning_frame_id();
  mission_.planning_mode = planning_mode_;
  clear_draft_segment();
  last_loaded_path_.clear();
  last_saved_path_.clear();
}

void MissionManagerNode::stop_execution(const std::string& final_message,
                                        ExecutionState new_state) {
  execution_state_ = new_state;
  last_result_message_ = final_message;
  publish_status();
}

void MissionManagerNode::publish_status() {
  std_msgs::msg::String msg;
  std::ostringstream oss;
  oss << "{";
  oss << "\"mission_name\":\"" << json_escape(mission_.mission_name) << "\",";
  oss << "\"frame_id\":\"" << json_escape(mission_.frame_id) << "\",";
  oss << "\"planning_mode\":\"" << json_escape(mission_.planning_mode) << "\",";
  oss << "\"planning_frame_id\":\"" << json_escape(planning_frame_id()) << "\",";
  oss << "\"segment_count\":" << mission_.segments.size() << ",";
  oss << "\"draft_has_start\":" << (draft_has_start_ ? "true" : "false") << ",";
  oss << "\"draft_has_goal\":" << (draft_has_goal_ ? "true" : "false") << ",";
  oss << "\"draft_waypoint_count\":" << draft_waypoints_.size() << ",";
  oss << "\"execution_state\":\"" << execution_state_string() << "\",";
  oss << "\"last_result_message\":\"" << json_escape(last_result_message_) << "\",";
  oss << "\"last_saved_path\":\"" << json_escape(last_saved_path_) << "\",";
  oss << "\"last_loaded_path\":\"" << json_escape(last_loaded_path_) << "\",";
  oss << "\"segments\":[";
  const auto status_segments = display_segments();
  for (std::size_t index = 0; index < status_segments.size(); ++index) {
    const auto& segment = status_segments[index];
    if (index > 0) {
      oss << ",";
    }
    oss << "{";
    oss << "\"segment_name\":\"" << json_escape(segment.segment_name) << "\",";
    oss << "\"start\":" << pose_json(segment.start) << ",";
    oss << "\"goal\":" << pose_json(segment.goal) << ",";
    oss << "\"waypoint_count\":" << segment.waypoints.size();
    oss << "}";
  }
  oss << "],";
  oss << "\"draft_start\":"
      << (draft_has_start_ ? pose_json(draft_start_) : std::string("null")) << ",";
  oss << "\"draft_goal\":"
      << (draft_has_goal_ ? pose_json(draft_goal_) : std::string("null"));
  oss << "}";
  msg.data = oss.str();
  status_pub_->publish(msg);
}

void MissionManagerNode::publish_markers() {
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;
  const auto stamp = this->now();

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = planning_frame_id();
  clear_marker.header.stamp = stamp;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  auto add_point_marker = [&](const MissionPose& pose, const std::string& ns, int r, int g,
                              int b, double scale) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_id();
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pose.x;
    marker.pose.position.y = pose.y;
    marker.pose.position.z = 0.05;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = static_cast<float>(r) / 255.0F;
    marker.color.g = static_cast<float>(g) / 255.0F;
    marker.color.b = static_cast<float>(b) / 255.0F;
    marker.color.a = 0.95F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    marker_array.markers.push_back(marker);
  };

  // 箭头 marker，用于可视化各点的 yaw 朝向
  auto add_arrow_marker = [&](const MissionPose& pose, const std::string& ns, int r, int g,
                              int b, double length) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_id();
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point start_point;
    start_point.x = pose.x;
    start_point.y = pose.y;
    start_point.z = 0.06;
    geometry_msgs::msg::Point end_point;
    end_point.x = pose.x + length * std::cos(pose.yaw);
    end_point.y = pose.y + length * std::sin(pose.yaw);
    end_point.z = 0.06;
    marker.points.push_back(start_point);
    marker.points.push_back(end_point);
    marker.scale.x = 0.04;  // 箭头杆粗细
    marker.scale.y = 0.08;  // 箭头头部宽度
    marker.scale.z = 0.0;
    marker.color.r = static_cast<float>(r) / 255.0F;
    marker.color.g = static_cast<float>(g) / 255.0F;
    marker.color.b = static_cast<float>(b) / 255.0F;
    marker.color.a = 0.95F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    marker_array.markers.push_back(marker);
  };

  auto add_line_marker = [&](const std::vector<MissionPose>& chain, const std::string& ns, int r,
                             int g, int b) {
    if (chain.size() < 2) {
      return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_id();
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = static_cast<float>(r) / 255.0F;
    marker.color.g = static_cast<float>(g) / 255.0F;
    marker.color.b = static_cast<float>(b) / 255.0F;
    marker.color.a = 0.9F;
    for (const auto& pose : chain) {
      geometry_msgs::msg::Point point;
      point.x = pose.x;
      point.y = pose.y;
      point.z = 0.02;
      marker.points.push_back(point);
    }
    marker_array.markers.push_back(marker);
  };

  const auto shown_segments = display_segments();
  for (std::size_t index = 0; index < shown_segments.size(); ++index) {
    const auto& segment = shown_segments[index];
    std::vector<MissionPose> chain;
    chain.push_back(segment.start);
    chain.insert(chain.end(), segment.waypoints.begin(), segment.waypoints.end());
    chain.push_back(segment.goal);
    add_line_marker(chain, "mission_segments", 0, 180, 255);
    add_point_marker(segment.start, "mission_start", 0, 200, 0, 0.18);
    add_arrow_marker(segment.start, "mission_start_heading", 0, 200, 0, 0.25);
    for (const auto& waypoint : segment.waypoints) {
      add_point_marker(waypoint, "mission_waypoint", 64, 160, 255, 0.14);
      add_arrow_marker(waypoint, "mission_waypoint_heading", 64, 160, 255, 0.20);
    }
    add_point_marker(segment.goal, "mission_goal", 255, 32, 32, 0.18);
    add_arrow_marker(segment.goal, "mission_goal_heading", 255, 32, 32, 0.25);
  }

  if (draft_has_start_ || draft_has_goal_ || !draft_waypoints_.empty()) {
    std::vector<MissionPose> draft_chain;
    if (draft_has_start_) {
      draft_chain.push_back(draft_start_);
    }
    draft_chain.insert(draft_chain.end(), draft_waypoints_.begin(), draft_waypoints_.end());
    if (draft_has_goal_) {
      draft_chain.push_back(draft_goal_);
    }
    add_line_marker(draft_chain, "draft_segment", 255, 200, 0);
    if (draft_has_start_) {
      add_point_marker(draft_start_, "draft_start", 255, 220, 0, 0.18);
      add_arrow_marker(draft_start_, "draft_start_heading", 255, 220, 0, 0.25);
    }
    for (const auto& waypoint : draft_waypoints_) {
      add_point_marker(waypoint, "draft_waypoint", 255, 180, 0, 0.14);
      add_arrow_marker(waypoint, "draft_waypoint_heading", 255, 180, 0, 0.20);
    }
    if (draft_has_goal_) {
      add_point_marker(draft_goal_, "draft_goal", 255, 140, 0, 0.18);
      add_arrow_marker(draft_goal_, "draft_goal_heading", 255, 140, 0, 0.25);
    }
  }

  markers_pub_->publish(marker_array);
}

void MissionManagerNode::publish_preview_path() {
  std::vector<MissionPose> chain;
  for (const auto& segment : display_segments()) {
    chain.push_back(segment.start);
    chain.insert(chain.end(), segment.waypoints.begin(), segment.waypoints.end());
    chain.push_back(segment.goal);
  }
  if (draft_has_start_) {
    chain.push_back(draft_start_);
  }
  chain.insert(chain.end(), draft_waypoints_.begin(), draft_waypoints_.end());
  if (draft_has_goal_) {
    chain.push_back(draft_goal_);
  }
  preview_path_pub_->publish(build_path_from_points(chain, this->now(), planning_frame_id()));
}

std::string MissionManagerNode::execution_state_string() const {
  switch (execution_state_) {
    case ExecutionState::kIdle:
      return "idle";
    case ExecutionState::kMissionLoaded:
      return "mission_loaded";
  }
  return "unknown";
}


std::string MissionManagerNode::mission_file_path(const std::string& mission_name) const {
  return (std::filesystem::path(missions_dir_) /
          (sanitize_file_stem(mission_name) + ".yaml"))
      .string();
}


}  // namespace path_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<path_planner::MissionManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
