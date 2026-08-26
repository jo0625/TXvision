cmake_minimum_required(VERSION 3.8)
project(yolo_armor_detector)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/yolo_armor_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/raw_track_latch.py scripts/track_diagnostics.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_raw_track_latch
    test/test_raw_track_latch.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(
    test_image_conversion
    test/test_image_conversion.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(
    test_candidate_config_contract
    test/test_candidate_config_contract.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(
    test_track_diagnostics
    test/test_track_diagnostics.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
