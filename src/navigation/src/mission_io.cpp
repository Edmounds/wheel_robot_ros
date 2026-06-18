#include "path_planner/mission_io.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace path_planner {

namespace {

bool pose_from_yaml(const YAML::Node& node,
                    const std::string& path,
                    MissionPose& pose,
                    std::string& error_message) {
  if (!node || !node.IsMap()) {
    error_message = "mission YAML pose 节点缺失或不是映射: " + path;
    return false;
  }

  auto parse_field = [&](const char* field_name, double& value) {
    const std::string field_path = path + "." + field_name;
    const YAML::Node field = node[field_name];
    if (!field || !field.IsScalar()) {
      error_message = "mission YAML pose 字段缺失或不是标量: " + field_path;
      return false;
    }
    try {
      value = field.as<double>();
    } catch (const std::exception&) {
      error_message = "mission YAML pose 字段不是数值: " + field_path;
      return false;
    }
    if (!std::isfinite(value)) {
      error_message = "mission YAML pose 字段不是有限数值: " + field_path;
      return false;
    }
    return true;
  };

  return parse_field("x", pose.x) && parse_field("y", pose.y) &&
         parse_field("yaw", pose.yaw);
}

YAML::Node pose_to_yaml(const MissionPose& pose) {
  YAML::Node node;
  node["x"] = pose.x;
  node["y"] = pose.y;
  node["yaw"] = pose.yaw;
  return node;
}

std::string default_segment_name(std::size_t index) {
  std::ostringstream oss;
  oss << "seg_" << std::setw(3) << std::setfill('0') << (index + 1);
  return oss.str();
}

bool segment_from_yaml(const YAML::Node& node,
                       std::size_t index,
                       MissionSegment& segment,
                       std::string& error_message) {
  const std::string segment_path = "segments[" + std::to_string(index) + "]";
  if (!node || !node.IsMap()) {
    error_message = "mission YAML segment 节点缺失或不是映射: " + segment_path;
    return false;
  }

  segment.segment_name = node["segment_name"] ? node["segment_name"].as<std::string>()
                                               : default_segment_name(index);
  if (!pose_from_yaml(node["start"], segment_path + ".start", segment.start, error_message)) {
    return false;
  }
  if (node["waypoints"]) {
    if (!node["waypoints"].IsSequence()) {
      error_message = "mission YAML waypoints 不是序列: " + segment_path + ".waypoints";
      return false;
    }
    std::size_t waypoint_index = 0;
    for (const auto& waypoint_node : node["waypoints"]) {
      MissionPose waypoint;
      if (!pose_from_yaml(
            waypoint_node,
            segment_path + ".waypoints[" + std::to_string(waypoint_index) + "]",
            waypoint, error_message)) {
        return false;
      }
      segment.waypoints.push_back(waypoint);
      ++waypoint_index;
    }
  }
  if (!pose_from_yaml(node["goal"], segment_path + ".goal", segment.goal, error_message)) {
    return false;
  }
  return true;
}

bool parse_yaml_root(const YAML::Node& root,
                     MissionDefinition& parsed_mission,
                     std::string& error_message) {
  parsed_mission.mission_name =
      root["mission_name"] ? root["mission_name"].as<std::string>() : "unnamed_mission";
  parsed_mission.frame_id = root["frame_id"] ? root["frame_id"].as<std::string>() : "map";
  parsed_mission.planning_mode =
      root["planning_mode"] ? root["planning_mode"].as<std::string>() : "straight_line";

  if (root["segments"] && root["segments"].IsSequence()) {
    std::size_t index = 0;
    for (const auto& segment_node : root["segments"]) {
      MissionSegment segment;
      if (!segment_from_yaml(segment_node, index, segment, error_message)) {
        return false;
      }
      parsed_mission.segments.push_back(segment);
      ++index;
    }
    return true;
  }

  return true;
}

}  // namespace

bool load_mission_file(const std::string& mission_path,
                       MissionDefinition& mission,
                       std::string& error_message) {
  std::ifstream stream(mission_path);
  if (!stream.is_open()) {
    error_message = "无法打开 mission 文件: " + mission_path;
    return false;
  }

  std::stringstream buffer;
  buffer << stream.rdbuf();
  return mission_from_yaml_string(buffer.str(), mission, error_message);
}

bool mission_from_yaml_string(const std::string& mission_yaml,
                              MissionDefinition& mission,
                              std::string& error_message) {
  try {
    YAML::Node root = YAML::Load(mission_yaml);
    if (!root || !root.IsMap()) {
      error_message = "mission YAML 为空或根节点不是映射";
      return false;
    }

    MissionDefinition parsed_mission;
    if (!parse_yaml_root(root, parsed_mission, error_message)) {
      return false;
    }
    if (parsed_mission.segments.empty()) {
      error_message = "mission YAML 缺少可执行的 segments";
      return false;
    }
    if (parsed_mission.frame_id != "map") {
      error_message = "mission frame_id 必须为 map";
      return false;
    }
    if (
      parsed_mission.planning_mode != "straight_line" &&
      parsed_mission.planning_mode != "theta_star") {
      error_message = "mission planning_mode 必须为 straight_line 或 theta_star";
      return false;
    }
    mission = parsed_mission;
    return true;
  } catch (const std::exception& ex) {
    error_message = std::string("解析 mission YAML 失败: ") + ex.what();
    return false;
  }
}

bool save_mission_file(const std::string& mission_path,
                       const MissionDefinition& mission,
                       std::string& error_message) {
  std::ofstream stream(mission_path);
  if (!stream.is_open()) {
    error_message = "无法写入 mission 文件: " + mission_path;
    return false;
  }
  stream << mission_to_yaml_string(mission);
  stream.flush();
  if (!stream.good()) {
    error_message = "写入 mission 文件后状态异常: " + mission_path;
    return false;
  }
  return true;
}

std::string mission_to_yaml_string(const MissionDefinition& mission) {
  YAML::Node root;
  root["mission_name"] = mission.mission_name;
  root["frame_id"] = mission.frame_id.empty() ? "map" : mission.frame_id;
  root["planning_mode"] =
      mission.planning_mode.empty() ? "straight_line" : mission.planning_mode;
  YAML::Node segments_node(YAML::NodeType::Sequence);
  for (std::size_t index = 0; index < mission.segments.size(); ++index) {
    const MissionSegment& segment = mission.segments[index];
    YAML::Node segment_node;
    segment_node["segment_name"] =
        segment.segment_name.empty() ? default_segment_name(index) : segment.segment_name;
    segment_node["start"] = pose_to_yaml(segment.start);
    YAML::Node waypoints_node(YAML::NodeType::Sequence);
    for (const auto& waypoint : segment.waypoints) {
      waypoints_node.push_back(pose_to_yaml(waypoint));
    }
    segment_node["waypoints"] = waypoints_node;
    segment_node["goal"] = pose_to_yaml(segment.goal);
    segments_node.push_back(segment_node);
  }
  root["segments"] = segments_node;

  YAML::Emitter emitter;
  emitter.SetIndent(2);
  emitter << root;
  return std::string(emitter.c_str()) + "\n";
}

}  // namespace path_planner
