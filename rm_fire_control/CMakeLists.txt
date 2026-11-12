cmake_minimum_required(VERSION 3.8)
project(rm_fire_control)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/fire_permission_node.py scripts/aim_run_recorder.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/fire_permission.py scripts/run_log.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_run_log test/test_run_log.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  ament_add_pytest_test(test_run_recorder_node test/test_run_recorder_node.py
    TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
    ENV ROS_DOMAIN_ID=216 ROS_LOCALHOST_ONLY=1)
  ament_add_pytest_test(test_fire_permission test/test_fire_permission.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  ament_add_pytest_test(test_fire_permission_node test/test_fire_permission_node.py
    TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
    ENV ROS_DOMAIN_ID=213 ROS_LOCALHOST_ONLY=1)
endif()
ament_package()
