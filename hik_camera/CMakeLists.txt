cmake_minimum_required(VERSION 3.10)
project(hik_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS installation root")
find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS "${MVS_ROOT}/include"
  NO_DEFAULT_PATH
  REQUIRED
)
find_library(MVS_LIBRARY
  NAMES MvCameraControl
  PATHS "${MVS_ROOT}/lib/64"
  NO_DEFAULT_PATH
  REQUIRED
)

add_executable(hik_camera_node src/hik_camera_node.cpp)
target_include_directories(hik_camera_node PRIVATE "${MVS_INCLUDE_DIR}")
ament_target_dependencies(hik_camera_node
  camera_info_manager
  rclcpp
  sensor_msgs
)
target_link_libraries(hik_camera_node "${MVS_LIBRARY}")
set_target_properties(hik_camera_node PROPERTIES
  BUILD_RPATH "${MVS_ROOT}/lib/64"
  INSTALL_RPATH "${MVS_ROOT}/lib/64"
)

install(TARGETS hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
