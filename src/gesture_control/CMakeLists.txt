cmake_minimum_required(VERSION 3.8)
project(gesture_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetBodyHeight.srv"
)

ament_python_install_package(gesture_control_impl)

install(PROGRAMS
  scripts/gesture_control_node
  scripts/height_service_stub_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  assets
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
