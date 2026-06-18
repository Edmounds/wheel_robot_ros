#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <navigation/srv/load_mission.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
geometry_msgs::msg::PoseStamped pose_from_yaml(
  const YAML::Node & node,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = stamp;
  pose.pose.position.x = node["x"].as<double>();
  pose.pose.position.y = node["y"].as<double>();
  pose.pose.position.z = 0.0;

  const double yaw = node["yaw"].as<double>();
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

std::vector<geometry_msgs::msg::PoseStamped> route_from_mission_yaml(
  const std::string & mission_yaml,
  const rclcpp::Time & stamp)
{
  const YAML::Node root = YAML::Load(mission_yaml);
  if (!root || !root.IsMap()) {
    throw std::runtime_error("mission YAML root is not a map");
  }
  const std::string frame_id = root["frame_id"] ? root["frame_id"].as<std::string>() : "map";
  const YAML::Node segments = root["segments"];
  if (!segments || !segments.IsSequence() || segments.size() == 0) {
    throw std::runtime_error("mission YAML has no executable segments");
  }

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (const auto & segment : segments) {
    if (!segment || !segment.IsMap()) {
      throw std::runtime_error("mission segment is not a map");
    }
    const YAML::Node waypoints = segment["waypoints"];
    if (waypoints) {
      if (!waypoints.IsSequence()) {
        throw std::runtime_error("mission segment waypoints is not a sequence");
      }
      for (const auto & waypoint : waypoints) {
        poses.push_back(pose_from_yaml(waypoint, frame_id, stamp));
      }
    }
    if (!segment["goal"]) {
      throw std::runtime_error("mission segment is missing goal");
    }
    poses.push_back(pose_from_yaml(segment["goal"], frame_id, stamp));
  }
  if (poses.empty()) {
    throw std::runtime_error("mission produced no Nav2 target poses");
  }
  return poses;
}
}  // namespace

class MissionNav2Player : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
  using LoadMission = navigation::srv::LoadMission;

  MissionNav2Player()
  : Node("mission_nav2_player")
  {
    mission_name_ = declare_parameter<std::string>("mission_name", "default");
    load_mission_service_ = declare_parameter<std::string>("load_mission_service", "/load_mission");
    navigate_action_name_ = declare_parameter<std::string>(
      "navigate_through_poses_action_name", "/navigate_through_poses");
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 5.0);
    action_timeout_sec_ = declare_parameter<double>("action_timeout_sec", 5.0);

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    load_mission_client_ = create_client<LoadMission>(load_mission_service_, rmw_qos_profile_services_default,
        callback_group_);
    nav_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      navigate_action_name_,
      callback_group_);
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/start_navigation_mission",
      std::bind(&MissionNav2Player::handle_start, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/stop_navigation_mission",
      std::bind(&MissionNav2Player::handle_stop, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    RCLCPP_INFO(
      get_logger(), "Mission Nav2 player ready: load_service=%s action=%s",
      load_mission_service_.c_str(), navigate_action_name_.c_str());
  }

private:
  void handle_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    try {
      mission_name_ = get_parameter("mission_name").as_string();
      if (mission_name_.empty()) {
        throw std::runtime_error("mission_name parameter is empty");
      }
      cancel_current_goal();

      if (!load_mission_client_->wait_for_service(std::chrono::duration<double>(service_timeout_sec_))) {
        throw std::runtime_error("LoadMission service is not available: " + load_mission_service_);
      }
      auto load_request = std::make_shared<LoadMission::Request>();
      load_request->mission_name = mission_name_;
      auto load_future = load_mission_client_->async_send_request(load_request);
      if (load_future.wait_for(std::chrono::duration<double>(service_timeout_sec_)) !=
        std::future_status::ready)
      {
        throw std::runtime_error("LoadMission service call timed out");
      }
      const auto load_response = load_future.get();
      if (!load_response->success) {
        throw std::runtime_error(load_response->message);
      }

      auto poses = route_from_mission_yaml(load_response->mission_yaml, now());
      if (!nav_client_->wait_for_action_server(std::chrono::duration<double>(action_timeout_sec_))) {
        throw std::runtime_error("NavigateThroughPoses action server is not available");
      }

      NavigateThroughPoses::Goal goal;
      goal.poses = poses;

      auto send_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
      send_options.goal_response_callback =
        [this](std::shared_ptr<GoalHandleNavigateThroughPoses> goal_handle) {
          if (!goal_handle) {
            RCLCPP_WARN(get_logger(), "Mission navigation goal was rejected");
            return;
          }
          std::lock_guard<std::mutex> lock(goal_mutex_);
          current_goal_ = goal_handle;
          RCLCPP_INFO(get_logger(), "Mission navigation goal accepted");
        };
      send_options.result_callback =
        [this](const GoalHandleNavigateThroughPoses::WrappedResult & result) {
          {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            if (current_goal_ && current_goal_->get_goal_id() == result.goal_id) {
              current_goal_.reset();
            }
          }
          RCLCPP_INFO(get_logger(), "Mission navigation finished with result code %d", static_cast<int>(result.code));
        };

      auto goal_future = nav_client_->async_send_goal(goal, send_options);
      if (goal_future.wait_for(std::chrono::duration<double>(action_timeout_sec_)) !=
        std::future_status::ready)
      {
        throw std::runtime_error("timed out while sending NavigateThroughPoses goal");
      }
      const auto goal_handle = goal_future.get();
      if (!goal_handle) {
        throw std::runtime_error("NavigateThroughPoses goal was rejected");
      }
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_ = goal_handle;
      }

      response->success = true;
      response->message = "started mission: " + mission_name_;
    } catch (const std::exception & ex) {
      response->success = false;
      response->message = ex.what();
    }
  }

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    cancel_current_goal();
    response->success = true;
    response->message = "mission navigation stop requested";
  }

  void cancel_current_goal()
  {
    std::shared_ptr<GoalHandleNavigateThroughPoses> goal_handle;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal_handle = current_goal_;
      current_goal_.reset();
    }
    if (!goal_handle) {
      return;
    }
    try {
      nav_client_->async_cancel_goal(goal_handle);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "Failed to cancel mission navigation goal: %s", ex.what());
    }
  }

  std::string mission_name_;
  std::string load_mission_service_;
  std::string navigate_action_name_;
  double service_timeout_sec_;
  double action_timeout_sec_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<LoadMission>::SharedPtr load_mission_client_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandleNavigateThroughPoses> current_goal_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionNav2Player>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
