cmake_minimum_required(VERSION 3.8)
project(voice_command_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(navigation REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DispatchVoiceCommand.srv"
)

add_executable(voice_command_bridge_node
  src/voice_command_bridge_node.cpp
)

ament_target_dependencies(voice_command_bridge_node
  rclcpp
  geometry_msgs
  std_srvs
  rcl_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  navigation
  rclcpp_action
  ament_index_cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(voice_command_bridge_node
  "${cpp_typesupport_target}"
  Boost::system
  yaml-cpp
)

install(TARGETS voice_command_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
