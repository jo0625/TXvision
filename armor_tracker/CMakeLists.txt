cmake_minimum_required(VERSION 3.10)
project(armor_tracker)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ekf_models.cpp
  src/extended_kalman_filter.cpp
  src/tracker.cpp
  src/tracker_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorTrackerNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_time_utils test/test_time_utils.cpp)
  target_include_directories(test_time_utils PRIVATE include)

  ament_add_gtest(test_tracker test/test_tracker.cpp)
  target_link_libraries(test_tracker ${PROJECT_NAME})

  ament_add_gtest(test_motion_model_node test/test_motion_model_node.cpp
    TIMEOUT 30
    ENV ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1)
  target_link_libraries(test_motion_model_node ${PROJECT_NAME})

  ament_add_gtest(test_ekf_math test/test_ekf_math.cpp)
  target_link_libraries(test_ekf_math ${PROJECT_NAME})
endif()

#############
## Install ##
#############

ament_auto_package()
