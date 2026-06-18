cmake_minimum_required(VERSION 3.8)
project(xbox_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(xbox_series_teleop_node src/xbox_series_teleop_node.cpp)
ament_target_dependencies(
  xbox_series_teleop_node
  geometry_msgs
  rclcpp
  sensor_msgs
)

install(
  TARGETS xbox_series_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
