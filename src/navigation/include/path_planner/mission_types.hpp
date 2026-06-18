#ifndef PATH_PLANNER__MISSION_TYPES_HPP_
#define PATH_PLANNER__MISSION_TYPES_HPP_

#include <string>
#include <vector>

namespace path_planner {

/// 任务位姿，包含 2D 位置和航向角。
struct MissionPose {
  double x {0.0};
  double y {0.0};
  double yaw {0.0};
};

/// 任务段，由起点、途径点和终点构成的一条路径段。
struct MissionSegment {
  std::string segment_name;
  MissionPose start;
  std::vector<MissionPose> waypoints;
  MissionPose goal;
};

/// 完整的任务定义，包含多个有序的任务段以及全局配置。
struct MissionDefinition {
  std::string mission_name;
  std::string frame_id {"map"};
  std::string planning_mode {"straight_line"};
  std::vector<MissionSegment> segments;
};

}  // namespace path_planner

#endif  // PATH_PLANNER__MISSION_TYPES_HPP_
