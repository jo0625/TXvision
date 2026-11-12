cmake_minimum_required(VERSION 3.8)
project(rm_aimer)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/basic_aimer_node.py scripts/controller_command_preview.py
    scripts/ballistic_image_overlay.py scripts/impact_speed_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES scripts/aimer_geometry.py scripts/ballistic_solver.py scripts/controller_angles.py README.md
    scripts/aim_stability_trace.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch docs
  DESTINATION share/${PROJECT_NAME}
)

install(FILES LICENSE.sp_vision DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_impact_speed_calibration test/test_impact_speed_calibration.py
    TIMEOUT 30
    ENV ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_ballistic_image_overlay test/test_ballistic_image_overlay.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_solution_diagnostics test/test_solution_diagnostics.py
    TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
    ENV ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1)
  ament_add_pytest_test(test_ballistic_solver test/test_ballistic_solver.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_aimer_geometry test/test_aimer_geometry.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_aimer_node test/test_aimer_node.py
    TIMEOUT 30
    ENV ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_controller_angles test/test_controller_angles.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_controller_preview test/test_controller_preview.py
    TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
endif()

ament_package()
