#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class XboxSeriesTeleopNode : public rclcpp::Node
{
public:
  XboxSeriesTeleopNode()
  : Node("xbox_series_teleop_node")
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    require_enable_button_ = declare_parameter<bool>("require_enable_button", false);
    enable_button_ = declare_parameter<int>("enable_button", 5);
    turbo_button_ = declare_parameter<int>("turbo_button", 4);
    linear_axis_ = declare_parameter<int>("linear_axis", 1);
    angular_axis_ = declare_parameter<int>("angular_axis", 2);
    deadzone_ = declare_parameter<double>("deadzone", 0.20);
    normal_linear_scale_ = declare_parameter<double>("normal_linear_scale", 0.35);
    normal_angular_scale_ = declare_parameter<double>("normal_angular_scale", 0.9);
    turbo_linear_scale_ = declare_parameter<double>("turbo_linear_scale", 0.7);
    turbo_angular_scale_ = declare_parameter<double>("turbo_angular_scale", 1.6);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    joy_timeout_sec_ = declare_parameter<double>("joy_timeout_sec", 0.5);

    deadzone_ = std::clamp(deadzone_, 0.0, 0.95);
    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
    joy_timeout_sec_ = std::max(0.05, joy_timeout_sec_);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&XboxSeriesTeleopNode::joyCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&XboxSeriesTeleopNode::publishTwist, this));

    if (require_enable_button_) {
      RCLCPP_INFO(
        get_logger(),
        "Xbox Series teleop ready: joy=%s cmd_vel=%s enable=button[%d] turbo=button[%d]. "
        "Hold the enable button and move the sticks to publish velocity.",
        joy_topic_.c_str(), cmd_vel_topic_.c_str(), enable_button_, turbo_button_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Xbox Series teleop ready: joy=%s cmd_vel=%s enable=always turbo=button[%d]. "
        "Move the sticks to publish velocity.",
        joy_topic_.c_str(), cmd_vel_topic_.c_str(), turbo_button_);
    }
  }

private:
  static bool hasButton(const sensor_msgs::msg::Joy & joy, int index)
  {
    return index >= 0 && static_cast<size_t>(index) < joy.buttons.size();
  }

  static bool hasAxis(const sensor_msgs::msg::Joy & joy, int index)
  {
    return index >= 0 && static_cast<size_t>(index) < joy.axes.size();
  }

  double axisValue(const sensor_msgs::msg::Joy & joy, int index)
  {
    if (!hasAxis(joy, index)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joy axis %d is unavailable; received %zu axes", index, joy.axes.size());
      return 0.0;
    }

    const double value = joy.axes[static_cast<size_t>(index)];
    if (std::abs(value) <= deadzone_) {
      return 0.0;
    }

    const double sign = value >= 0.0 ? 1.0 : -1.0;
    return sign * (std::abs(value) - deadzone_) / (1.0 - deadzone_);
  }

  bool buttonPressed(const sensor_msgs::msg::Joy & joy, int index)
  {
    if (!hasButton(joy, index)) {
      if (index >= 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Joy button %d is unavailable; received %zu buttons", index, joy.buttons.size());
      }
      return false;
    }
    return joy.buttons[static_cast<size_t>(index)] != 0;
  }

  static std::string pressedButtons(const sensor_msgs::msg::Joy & joy)
  {
    std::string pressed;
    for (size_t i = 0; i < joy.buttons.size(); ++i) {
      if (joy.buttons[i] == 0) {
        continue;
      }
      if (!pressed.empty()) {
        pressed += ",";
      }
      pressed += std::to_string(i);
    }
    return pressed.empty() ? "none" : pressed;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const bool first_joy = !have_joy_;
    last_joy_time_ = now();
    have_joy_ = true;

    if (first_joy && require_enable_button_) {
      RCLCPP_INFO(
        get_logger(),
        "Received first Joy message: axes=%zu buttons=%zu pressed_buttons=%s. "
        "Waiting for enable button[%d].",
        msg->axes.size(), msg->buttons.size(), pressedButtons(*msg).c_str(), enable_button_);
    } else if (first_joy) {
      RCLCPP_INFO(
        get_logger(),
        "Received first Joy message: axes=%zu buttons=%zu pressed_buttons=%s. "
        "Teleop is active without an enable button.",
        msg->axes.size(), msg->buttons.size(), pressedButtons(*msg).c_str());
    }

    const bool enabled = !require_enable_button_ || buttonPressed(*msg, enable_button_);
    if (!enabled) {
      if (was_enabled_) {
        RCLCPP_INFO(get_logger(), "Enable button released; publishing stop once.");
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Teleop disabled. Hold enable button[%d] to publish /cmd_vel. pressed_buttons=%s",
          enable_button_, pressedButtons(*msg).c_str());
      }
      was_enabled_ = false;
      active_twist_ = geometry_msgs::msg::Twist();
      publish_zero_once_ = true;
      return;
    }

    if (require_enable_button_ && !was_enabled_) {
      RCLCPP_INFO(get_logger(), "Enable button pressed; teleop active.");
    }
    was_enabled_ = true;

    const bool turbo = buttonPressed(*msg, turbo_button_);
    const double linear_scale = turbo ? turbo_linear_scale_ : normal_linear_scale_;
    const double angular_scale = turbo ? turbo_angular_scale_ : normal_angular_scale_;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = axisValue(*msg, linear_axis_) * linear_scale;
    twist.angular.z = axisValue(*msg, angular_axis_) * angular_scale;
    active_twist_ = twist;
    publish_zero_once_ = false;
  }

  void publishTwist()
  {
    if (!have_joy_) {
      return;
    }

    const bool timed_out = (now() - last_joy_time_).seconds() > joy_timeout_sec_;
    if (timed_out) {
      active_twist_ = geometry_msgs::msg::Twist();
      if (!published_timeout_zero_) {
        cmd_vel_pub_->publish(active_twist_);
        published_timeout_zero_ = true;
      }
      return;
    }

    published_timeout_zero_ = false;

    if (isZero(active_twist_)) {
      if (publish_zero_once_) {
        cmd_vel_pub_->publish(active_twist_);
        publish_zero_once_ = false;
      }
      return;
    }

    cmd_vel_pub_->publish(active_twist_);
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Publishing /cmd_vel: linear.x=%.3f angular.z=%.3f",
      active_twist_.linear.x, active_twist_.angular.z);
  }

  static bool isZero(const geometry_msgs::msg::Twist & twist)
  {
    return twist.linear.x == 0.0 && twist.linear.y == 0.0 && twist.linear.z == 0.0 &&
           twist.angular.x == 0.0 && twist.angular.y == 0.0 && twist.angular.z == 0.0;
  }

  std::string joy_topic_;
  std::string cmd_vel_topic_;
  bool require_enable_button_;
  int enable_button_;
  int turbo_button_;
  int linear_axis_;
  int angular_axis_;
  double deadzone_;
  double normal_linear_scale_;
  double normal_angular_scale_;
  double turbo_linear_scale_;
  double turbo_angular_scale_;
  double publish_rate_hz_;
  double joy_timeout_sec_;

  bool have_joy_{false};
  bool was_enabled_{false};
  bool publish_zero_once_{true};
  bool published_timeout_zero_{false};
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist active_twist_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XboxSeriesTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
