cmake_minimum_required(VERSION 3.8)
project(pose_to_yaml)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  PROGRAMS scripts/pose_to_yaml_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
