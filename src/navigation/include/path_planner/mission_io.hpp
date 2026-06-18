#ifndef PATH_PLANNER__MISSION_IO_HPP_
#define PATH_PLANNER__MISSION_IO_HPP_

#include <string>

#include "path_planner/mission_types.hpp"

namespace path_planner {

bool load_mission_file(const std::string& mission_path,
                       MissionDefinition& mission,
                       std::string& error_message);

bool mission_from_yaml_string(const std::string& mission_yaml,
                              MissionDefinition& mission,
                              std::string& error_message);

bool save_mission_file(const std::string& mission_path,
                       const MissionDefinition& mission,
                       std::string& error_message);

std::string mission_to_yaml_string(const MissionDefinition& mission);

}  // namespace path_planner

#endif  // PATH_PLANNER__MISSION_IO_HPP_
