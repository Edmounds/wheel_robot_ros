#ifndef PATH_PLANNER__PATH_PLANNER_NODE_HPP_
#define PATH_PLANNER__PATH_PLANNER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation/srv/plan_path.hpp"
#include "path_planner/theta_star.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace path_planner {

/// Theta* 路径规划 ROS 2 节点。
///
/// 加载静态 2D 地图并发布 /costmap，订阅里程计，提供 Service 和 RViz 2D Goal Pose
/// 两种规划触发方式，将规划结果发布到 /plan 话题用于预览。
class PathPlannerNode : public rclcpp::Node {
 public:
  using ComputePathThroughPoses =
      nav2_msgs::action::ComputePathThroughPoses;
  using GoalHandleComputePathThroughPoses =
      rclcpp_action::ServerGoalHandle<ComputePathThroughPoses>;

  explicit PathPlannerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  /// 从配置文件参数中声明并读取所有参数。
  void declare_and_load_parameters();

  /// 里程计话题回调。
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// RViz 2D Goal Pose 回调。
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /// PlanPath Service 回调。
  void plan_path_callback(
      const std::shared_ptr<navigation::srv::PlanPath::Request> request,
      std::shared_ptr<navigation::srv::PlanPath::Response> response);

  /// Nav2 ComputePathThroughPoses goal admission callback。
  rclcpp_action::GoalResponse handle_compute_path_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ComputePathThroughPoses::Goal> goal);

  /// Nav2 ComputePathThroughPoses cancel callback。
  rclcpp_action::CancelResponse handle_compute_path_cancel(
      const std::shared_ptr<GoalHandleComputePathThroughPoses> goal_handle);

  /// Nav2 ComputePathThroughPoses accepted-goal callback。
  void handle_compute_path_accepted(
      const std::shared_ptr<GoalHandleComputePathThroughPoses> goal_handle);

  /// Execute Nav2 ComputePathThroughPoses goal。
  void execute_compute_path(
      const std::shared_ptr<GoalHandleComputePathThroughPoses> goal_handle);

  /// ClearCostmapAroundRobot compatibility callback；point_nav 下为空操作。
  void clear_costmap_callback(
      const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Request> request,
      std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Response> response);

  /// 执行路径规划（含途径点分段规划拼接）。
  ///
  /// Args:
  ///   start: 起点世界坐标。
  ///   goal: 终点世界坐标。
  ///   waypoints: 途径点列表。
  ///   path_out: 输出路径消息。
  ///
  /// Returns:
  ///   规划成功返回 true。
  bool plan(const WorldPoint& start,
            const WorldPoint& goal,
            const std::vector<WorldPoint>& waypoints,
            nav_msgs::msg::Path& path_out);

  bool plan_checkpoints(const std::vector<WorldPoint>& checkpoints,
                        const std::vector<double>* checkpoint_yaws,
                        nav_msgs::msg::Path& path_out);

  bool plan_compute_path_through_poses(
      const ComputePathThroughPoses::Goal& goal,
      ComputePathThroughPoses::Result& result,
      std::string& error_message);

  /// 将世界坐标路径点转换为 nav_msgs/Path 消息。
  nav_msgs::msg::Path to_path_msg(
      const std::vector<WorldPoint>& points,
      const std::vector<double>* yaws = nullptr) const;

  /// 发布路径到 /plan 话题。
  void publish_plan(const nav_msgs::msg::Path& path);

  // 参数
  std::string map_yaml_path_;
  std::string planning_mode_;
  std::string costmap_publish_topic_;
  double inflation_radius_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string plan_topic_;
  std::string planning_frame_id_;
  std::string compute_path_action_name_;
  std::string clear_costmap_service_name_;
  int8_t lethal_cost_threshold_;
  bool allow_unknown_;
  double weight_;
  double interpolation_resolution_;
  std::size_t max_search_iterations_;

  // 状态
  std::shared_ptr<GridMap> grid_map_;
  std::mutex odom_mutex_;
  WorldPoint current_position_;
  double current_yaw_;
  bool odom_received_;

  // ROS 接口
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Service<navigation::srv::PlanPath>::SharedPtr plan_service_;
  rclcpp_action::Server<ComputePathThroughPoses>::SharedPtr compute_path_action_server_;
  rclcpp::Service<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_costmap_service_;

  // 算法
  std::unique_ptr<ThetaStar> theta_star_;
};

}  // namespace path_planner

#endif  // PATH_PLANNER__PATH_PLANNER_NODE_HPP_
