cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  "srv/SaveMission.srv"
  "srv/LoadMission.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

add_library(theta_star SHARED
  src/theta_star.cpp
)
target_include_directories(theta_star PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(theta_star
  yaml-cpp
)
ament_target_dependencies(theta_star
  nav_msgs
  ament_index_cpp
)

add_library(mission_io SHARED
  src/mission_io.cpp
)
target_include_directories(mission_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mission_io
  yaml-cpp
)
ament_target_dependencies(mission_io
  rclcpp
)

add_executable(path_planner_node
  src/path_planner_node.cpp
)
target_include_directories(path_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(path_planner_node
  theta_star
  mission_io
)
ament_target_dependencies(path_planner_node
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

add_executable(mission_manager_node
  src/mission_manager_node.cpp
)
target_include_directories(mission_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mission_manager_node
  mission_io
)
ament_target_dependencies(mission_manager_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

add_executable(mission_nav2_player
  src/mission_nav2_player.cpp
)

ament_target_dependencies(mission_nav2_player
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
add_dependencies(path_planner_node ${PROJECT_NAME}__rosidl_generator_cpp)
add_dependencies(mission_manager_node ${PROJECT_NAME}__rosidl_generator_cpp)
add_dependencies(mission_nav2_player ${PROJECT_NAME}__rosidl_generator_cpp)
target_link_libraries(path_planner_node "${cpp_typesupport_target}")
target_link_libraries(mission_manager_node "${cpp_typesupport_target}")
target_link_libraries(mission_nav2_player
  "${cpp_typesupport_target}"
  yaml-cpp
)

install(TARGETS theta_star mission_io
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS path_planner_node mission_manager_node mission_nav2_player
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch missions rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
