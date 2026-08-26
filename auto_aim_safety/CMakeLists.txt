cmake_minimum_required(VERSION 3.8)
project(auto_aim_safety)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/target_watchdog_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
