cmake_minimum_required(VERSION 3.8)
project(sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/verify_headless_flow.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
