#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <navigation/srv/save_mission.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "voice_command_bridge/srv/dispatch_voice_command.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::string expand_user_path(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }

  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    return path;
  }

  if (path.size() == 1) {
    return std::string(home);
  }
  if (path[1] == '/') {
    return std::string(home) + path.substr(1);
  }
  return path;
}

double yaml_number(const YAML::Node & node, const std::string & key)
{
  if (!node[key] || !node[key].IsScalar()) {
    throw std::runtime_error("missing numeric argument: " + key);
  }
  return node[key].as<double>();
}

std::string yaml_string(const YAML::Node & node, const std::string & key)
{
  if (!node[key] || !node[key].IsScalar()) {
    throw std::runtime_error("missing string argument: " + key);
  }
  return node[key].as<std::string>();
}

}  // namespace

class VoiceCommandBridge : public rclcpp::Node
{
public:
  using DispatchVoiceCommand = voice_command_bridge::srv::DispatchVoiceCommand;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using SetParameters = rcl_interfaces::srv::SetParameters;
  using SaveMission = navigation::srv::SaveMission;
  using Trigger = std_srvs::srv::Trigger;

  VoiceCommandBridge()
  : Node("voice_command_bridge_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    service_name_ = declare_parameter<std::string>("service_name", "/voice_command/dispatch");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    navigate_action_name_ = declare_parameter<std::string>(
      "navigate_action_name", "/navigate_to_pose");
    waypoints_file_ = expand_user_path(
      declare_parameter<std::string>(
        "waypoints_file", "~/.ros/voice_command_bridge/waypoints.yaml"));
    linear_speed_mps_ = declare_parameter<double>("linear_speed_mps", 0.15);
    angular_speed_radps_ = declare_parameter<double>("angular_speed_radps", 0.45);
    cmd_vel_period_ms_ = declare_parameter<int>("cmd_vel_period_ms", 100);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 1.0);
    nav_server_timeout_sec_ = declare_parameter<double>("nav_server_timeout_sec", 2.0);
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 5.0);
    save_map_service_name_ = declare_parameter<std::string>("save_map_service_name", "/cartographer/save_map");
    cartographer_save_map_node_name_ = declare_parameter<std::string>(
      "cartographer_save_map_node_name", "/cartographer_save_map");
    record_start_service_name_ = declare_parameter<std::string>(
      "record_start_service_name", "/record_start_pose");
    record_waypoint_service_name_ = declare_parameter<std::string>(
      "record_waypoint_service_name", "/record_waypoint");
    record_goal_service_name_ = declare_parameter<std::string>(
      "record_goal_service_name", "/record_goal_pose");
    append_segment_service_name_ = declare_parameter<std::string>(
      "append_segment_service_name", "/append_draft_segment");
    save_mission_service_name_ = declare_parameter<std::string>(
      "save_mission_service_name", "/save_mission");
    start_mission_service_name_ = declare_parameter<std::string>(
      "start_mission_service_name", "/start_navigation_mission");
    stop_mission_service_name_ = declare_parameter<std::string>(
      "stop_mission_service_name", "/stop_navigation_mission");
    mission_player_node_name_ = declare_parameter<std::string>(
      "mission_player_node_name", "/mission_nav2_player");

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      navigate_action_name_,
      callback_group_);
    save_map_client_ = create_client<Trigger>(
      save_map_service_name_, rmw_qos_profile_services_default, callback_group_);
    record_start_client_ = create_client<Trigger>(
      record_start_service_name_, rmw_qos_profile_services_default, callback_group_);
    record_waypoint_client_ = create_client<Trigger>(
      record_waypoint_service_name_, rmw_qos_profile_services_default, callback_group_);
    record_goal_client_ = create_client<Trigger>(
      record_goal_service_name_, rmw_qos_profile_services_default, callback_group_);
    append_segment_client_ = create_client<Trigger>(
      append_segment_service_name_, rmw_qos_profile_services_default, callback_group_);
    start_mission_client_ = create_client<Trigger>(
      start_mission_service_name_, rmw_qos_profile_services_default, callback_group_);
    stop_mission_client_ = create_client<Trigger>(
      stop_mission_service_name_, rmw_qos_profile_services_default, callback_group_);
    save_mission_client_ = create_client<SaveMission>(
      save_mission_service_name_, rmw_qos_profile_services_default, callback_group_);
    cartographer_params_client_ = create_client<SetParameters>(
      cartographer_save_map_node_name_ + "/set_parameters",
      rmw_qos_profile_services_default,
      callback_group_);
    mission_player_params_client_ = create_client<SetParameters>(
      mission_player_node_name_ + "/set_parameters",
      rmw_qos_profile_services_default,
      callback_group_);
    service_ = create_service<DispatchVoiceCommand>(
      service_name_,
      std::bind(
        &VoiceCommandBridge::handle_dispatch, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    RCLCPP_INFO(
      get_logger(), "Voice command bridge ready: service=%s cmd_vel=%s waypoints=%s",
      service_name_.c_str(), cmd_vel_topic_.c_str(), waypoints_file_.c_str());
  }

private:
  void handle_dispatch(
    const std::shared_ptr<DispatchVoiceCommand::Request> request,
    std::shared_ptr<DispatchVoiceCommand::Response> response)
  {
    try {
      const YAML::Node commands = YAML::Load(request->commands_json);
      if (!commands || !commands.IsSequence()) {
        set_response(response, false, "rejected", "", "commands_json must be a JSON array");
        return;
      }

      if (commands.size() == 0) {
        set_response(response, false, "ignored", "", "empty command array");
        return;
      }

      if (commands.size() > 1) {
        set_response(response, false, "rejected", "", "multiple commands are not supported yet");
        return;
      }

      const YAML::Node command = commands[0];
      if (!command["name"] || !command["name"].IsScalar()) {
        set_response(response, false, "rejected", "", "command name is missing");
        return;
      }

      const std::string name = command["name"].as<std::string>();
      const YAML::Node args =
        command["arguments"] ? command["arguments"] : YAML::Node(YAML::NodeType::Map);

      response->command_name = name;
      dispatch_command(name, args, response);
    } catch (const YAML::Exception & ex) {
      set_response(response, false, "rejected", "", std::string("invalid JSON/YAML: ") + ex.what());
    } catch (const std::exception & ex) {
      set_response(response, false, "failed", response->command_name, ex.what());
    }
  }

  void dispatch_command(
    const std::string & name,
    const YAML::Node & args,
    const std::shared_ptr<DispatchVoiceCommand::Response> & response)
  {
    if (name == "move_forward") {
      start_linear_move(yaml_number(args, "distance"));
      set_response(response, true, "accepted", name, "forward move started");
      return;
    }

    if (name == "move_backward") {
      start_linear_move(-yaml_number(args, "distance"));
      set_response(response, true, "accepted", name, "backward move started");
      return;
    }

    if (name == "turn") {
      start_turn(yaml_number(args, "angle"));
      set_response(response, true, "accepted", name, "turn started");
      return;
    }

    if (name == "stop") {
      stop_all();
      set_response(response, true, "done", name, "stop requested");
      return;
    }

    if (name == "set_waypoint") {
      const std::string waypoint_name = yaml_string(args, "name");
      set_waypoint(waypoint_name);
      set_response(response, true, "done", name, "waypoint saved: " + waypoint_name);
      return;
    }

    if (name == "goto_waypoint") {
      const std::string waypoint_name = yaml_string(args, "name");
      goto_waypoint(waypoint_name);
      set_response(response, true, "accepted", name, "navigation goal sent: " + waypoint_name);
      return;
    }

    if (name == "save_map") {
      const std::string map_name = yaml_string(args, "name");
      set_remote_string_parameter(cartographer_params_client_, "map_name", map_name);
      const std::string message = call_trigger(save_map_client_);
      set_response(response, true, "done", name, message);
      return;
    }

    if (name == "record_start_pose") {
      const std::string message = call_trigger(record_start_client_);
      set_response(response, true, "done", name, message);
      return;
    }

    if (name == "record_waypoint") {
      const std::string message = call_trigger(record_waypoint_client_);
      set_response(response, true, "done", name, message);
      return;
    }

    if (name == "record_goal_pose") {
      const std::string message = call_trigger(record_goal_client_);
      set_response(response, true, "done", name, message);
      return;
    }

    if (name == "append_segment") {
      const std::string message = call_trigger(append_segment_client_);
      set_response(response, true, "done", name, message);
      return;
    }

    if (name == "save_mission") {
      const std::string mission_name = yaml_string(args, "name");
      const std::string message = call_save_mission(mission_name);
      set_response(response, true, "done", name, message);
      return;
    }

    if (name == "start_mission") {
      const std::string mission_name = yaml_string(args, "name");
      set_remote_string_parameter(mission_player_params_client_, "mission_name", mission_name);
      const std::string message = call_trigger(start_mission_client_);
      set_response(response, true, "accepted", name, message);
      return;
    }

    if (name == "jump") {
      set_response(response, false, "unsupported", name, "jump is recognized but not implemented");
      return;
    }

    set_response(response, false, "rejected", name, "unknown command");
  }

  void start_linear_move(double distance_m)
  {
    const double speed = std::abs(linear_speed_mps_);
    if (speed <= 0.0) {
      throw std::runtime_error("linear_speed_mps must be positive");
    }

    const double direction = distance_m >= 0.0 ? 1.0 : -1.0;
    const auto duration = std::chrono::duration<double>(std::abs(distance_m) / speed);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = direction * speed;
    start_open_loop_motion(twist, duration);
  }

  void start_turn(double angle_deg)
  {
    const double speed = std::abs(angular_speed_radps_);
    if (speed <= 0.0) {
      throw std::runtime_error("angular_speed_radps must be positive");
    }

    const double angle_rad = angle_deg * kPi / 180.0;
    const double direction = angle_rad >= 0.0 ? 1.0 : -1.0;
    const auto duration = std::chrono::duration<double>(std::abs(angle_rad) / speed);

    geometry_msgs::msg::Twist twist;
    twist.angular.z = direction * speed;
    start_open_loop_motion(twist, duration);
  }

  void start_open_loop_motion(
    const geometry_msgs::msg::Twist & twist,
    const std::chrono::duration<double> & duration)
  {
    cancel_open_loop_motion(false);

    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      active_twist_ = twist;
      motion_end_time_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(duration);
      motion_timer_ = create_wall_timer(
        std::chrono::milliseconds(std::max(10, cmd_vel_period_ms_)),
        std::bind(&VoiceCommandBridge::on_motion_timer, this),
        callback_group_);
    }
    cmd_vel_pub_->publish(twist);
  }

  void on_motion_timer()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (!motion_timer_) {
      return;
    }

    if (std::chrono::steady_clock::now() >= motion_end_time_) {
      motion_timer_->cancel();
      motion_timer_.reset();
      publish_stop_locked();
      return;
    }

    cmd_vel_pub_->publish(active_twist_);
  }

  void publish_stop()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    publish_stop_locked();
  }

  void publish_stop_locked()
  {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }

  void stop_all()
  {
    cancel_open_loop_motion(true);
    cancel_navigation_goal();
    try {
      call_trigger(stop_mission_client_, false);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "Failed to stop mission navigation: %s", ex.what());
    }
  }

  void cancel_open_loop_motion(bool publish_zero)
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (motion_timer_) {
      motion_timer_->cancel();
      motion_timer_.reset();
    }
    active_twist_ = geometry_msgs::msg::Twist();
    if (publish_zero) {
      publish_stop_locked();
    }
  }

  void set_waypoint(const std::string & name)
  {
    const auto transform = tf_buffer_.lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_sec_));

    YAML::Node root = load_waypoints();
    YAML::Node waypoint;
    waypoint["frame_id"] = global_frame_;
    waypoint["x"] = transform.transform.translation.x;
    waypoint["y"] = transform.transform.translation.y;
    waypoint["z"] = transform.transform.translation.z;
    waypoint["qx"] = transform.transform.rotation.x;
    waypoint["qy"] = transform.transform.rotation.y;
    waypoint["qz"] = transform.transform.rotation.z;
    waypoint["qw"] = transform.transform.rotation.w;
    root["waypoints"][name] = waypoint;
    save_waypoints(root);
  }

  void goto_waypoint(const std::string & name)
  {
    YAML::Node root = load_waypoints();
    YAML::Node waypoint = root["waypoints"][name];
    if (!waypoint) {
      throw std::runtime_error("waypoint not found: " + name);
    }

    if (!nav_client_->wait_for_action_server(
        std::chrono::duration<double>(nav_server_timeout_sec_)))
    {
      throw std::runtime_error("NavigateToPose action server is not available");
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = waypoint["frame_id"] ? waypoint["frame_id"].as<std::string>() : global_frame_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = yaml_number(waypoint, "x");
    goal.pose.pose.position.y = yaml_number(waypoint, "y");
    goal.pose.pose.position.z = yaml_number(waypoint, "z");
    goal.pose.pose.orientation.x = yaml_number(waypoint, "qx");
    goal.pose.pose.orientation.y = yaml_number(waypoint, "qy");
    goal.pose.pose.orientation.z = yaml_number(waypoint, "qz");
    goal.pose.pose.orientation.w = yaml_number(waypoint, "qw");

    auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_options.goal_response_callback =
      [this, name](std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(get_logger(), "Navigation goal rejected for waypoint: %s", name.c_str());
          return;
        }
        {
          std::lock_guard<std::mutex> lock(nav_mutex_);
          current_nav_goal_ = goal_handle;
        }
        RCLCPP_INFO(get_logger(), "Navigation goal accepted for waypoint: %s", name.c_str());
      };
    send_options.result_callback =
      [this, name](const GoalHandleNavigateToPose::WrappedResult & result) {
        {
          std::lock_guard<std::mutex> lock(nav_mutex_);
          if (current_nav_goal_ && current_nav_goal_->get_goal_id() == result.goal_id) {
            current_nav_goal_.reset();
          }
        }
        RCLCPP_INFO(
          get_logger(), "Navigation finished for waypoint %s with result code %d",
          name.c_str(), static_cast<int>(result.code));
      };
    auto future_goal_handle = nav_client_->async_send_goal(goal, send_options);
    if (future_goal_handle.wait_for(std::chrono::duration<double>(nav_server_timeout_sec_)) !=
      std::future_status::ready)
    {
      throw std::runtime_error("timed out while sending navigation goal");
    }

    const auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
      throw std::runtime_error("navigation goal was rejected");
    }
    {
      std::lock_guard<std::mutex> lock(nav_mutex_);
      current_nav_goal_ = goal_handle;
    }
  }

  void cancel_navigation_goal()
  {
    std::shared_ptr<GoalHandleNavigateToPose> goal_handle;
    {
      std::lock_guard<std::mutex> lock(nav_mutex_);
      goal_handle = current_nav_goal_;
      current_nav_goal_.reset();
    }

    try {
      if (nav_client_->action_server_is_ready()) {
        nav_client_->async_cancel_all_goals(
          [logger = get_logger()](NavigateToPose::Impl::CancelGoalService::Response::SharedPtr) {
            RCLCPP_INFO(logger, "Navigation cancel-all request acknowledged");
          });
      } else if (goal_handle) {
        nav_client_->async_cancel_goal(
          goal_handle,
          [logger = get_logger()](NavigateToPose::Impl::CancelGoalService::Response::SharedPtr) {
            RCLCPP_INFO(logger, "Navigation cancel request acknowledged");
          });
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "Failed to cancel navigation goal: %s", ex.what());
    }
  }

  std::string call_trigger(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    bool require_success = true)
  {
    if (!client->wait_for_service(std::chrono::duration<double>(service_timeout_sec_))) {
      throw std::runtime_error(
        std::string("service is not available: ") + client->get_service_name());
    }
    const auto request = std::make_shared<Trigger::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(service_timeout_sec_)) !=
      std::future_status::ready)
    {
      throw std::runtime_error(
        std::string("service call timed out: ") + client->get_service_name());
    }
    const auto result = future.get();
    if (require_success && !result->success) {
      throw std::runtime_error(result->message);
    }
    return result->message;
  }

  std::string call_save_mission(const std::string & mission_name)
  {
    if (!save_mission_client_->wait_for_service(std::chrono::duration<double>(service_timeout_sec_))) {
      throw std::runtime_error(
        std::string("service is not available: ") + save_mission_client_->get_service_name());
    }
    auto request = std::make_shared<SaveMission::Request>();
    request->mission_name = mission_name;
    auto future = save_mission_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(service_timeout_sec_)) !=
      std::future_status::ready)
    {
      throw std::runtime_error(
        std::string("service call timed out: ") + save_mission_client_->get_service_name());
    }
    const auto result = future.get();
    if (!result->success) {
      throw std::runtime_error(result->message);
    }
    return result->message;
  }

  void set_remote_string_parameter(
    const rclcpp::Client<SetParameters>::SharedPtr & client,
    const std::string & parameter_name,
    const std::string & value)
  {
    if (!client->wait_for_service(std::chrono::duration<double>(service_timeout_sec_))) {
      throw std::runtime_error(
        std::string("parameter service is not available: ") + client->get_service_name());
    }

    auto request = std::make_shared<SetParameters::Request>();
    request->parameters.push_back(rclcpp::Parameter(parameter_name, value).to_parameter_msg());
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(service_timeout_sec_)) !=
      std::future_status::ready)
    {
      throw std::runtime_error(
        std::string("parameter service call timed out: ") + client->get_service_name());
    }
    const auto result = future.get();
    if (result->results.empty() || !result->results[0].successful) {
      const std::string reason = result->results.empty() ? "" : result->results[0].reason;
      throw std::runtime_error("failed to set remote parameter " + parameter_name + ": " + reason);
    }
  }

  YAML::Node load_waypoints() const
  {
    if (!std::filesystem::exists(waypoints_file_)) {
      YAML::Node root;
      root["waypoints"] = YAML::Node(YAML::NodeType::Map);
      return root;
    }

    YAML::Node root = YAML::LoadFile(waypoints_file_);
    if (!root["waypoints"] || !root["waypoints"].IsMap()) {
      root["waypoints"] = YAML::Node(YAML::NodeType::Map);
    }
    return root;
  }

  void save_waypoints(const YAML::Node & root) const
  {
    const std::filesystem::path path(waypoints_file_);
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream out(waypoints_file_);
    if (!out) {
      throw std::runtime_error("failed to open waypoints file for writing: " + waypoints_file_);
    }
    out << root;
  }

  void set_response(
    const std::shared_ptr<DispatchVoiceCommand::Response> & response,
    bool ok,
    const std::string & status,
    const std::string & command_name,
    const std::string & message) const
  {
    response->ok = ok;
    response->status = status;
    response->command_name = command_name;
    response->message = message;
  }

  std::string cmd_vel_topic_;
  std::string service_name_;
  std::string global_frame_;
  std::string robot_frame_;
  std::string navigate_action_name_;
  std::string waypoints_file_;
  double linear_speed_mps_;
  double angular_speed_radps_;
  int cmd_vel_period_ms_;
  double tf_timeout_sec_;
  double nav_server_timeout_sec_;
  double service_timeout_sec_;
  std::string save_map_service_name_;
  std::string cartographer_save_map_node_name_;
  std::string record_start_service_name_;
  std::string record_waypoint_service_name_;
  std::string record_goal_service_name_;
  std::string append_segment_service_name_;
  std::string save_mission_service_name_;
  std::string start_mission_service_name_;
  std::string stop_mission_service_name_;
  std::string mission_player_node_name_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<DispatchVoiceCommand>::SharedPtr service_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<Trigger>::SharedPtr save_map_client_;
  rclcpp::Client<Trigger>::SharedPtr record_start_client_;
  rclcpp::Client<Trigger>::SharedPtr record_waypoint_client_;
  rclcpp::Client<Trigger>::SharedPtr record_goal_client_;
  rclcpp::Client<Trigger>::SharedPtr append_segment_client_;
  rclcpp::Client<Trigger>::SharedPtr start_mission_client_;
  rclcpp::Client<Trigger>::SharedPtr stop_mission_client_;
  rclcpp::Client<SaveMission>::SharedPtr save_mission_client_;
  rclcpp::Client<SetParameters>::SharedPtr cartographer_params_client_;
  rclcpp::Client<SetParameters>::SharedPtr mission_player_params_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex motion_mutex_;
  geometry_msgs::msg::Twist active_twist_;
  std::chrono::steady_clock::time_point motion_end_time_;
  rclcpp::TimerBase::SharedPtr motion_timer_;
  std::mutex nav_mutex_;
  std::shared_ptr<GoalHandleNavigateToPose> current_nav_goal_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoiceCommandBridge>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
