cmake_minimum_required(VERSION 3.8)
project(rm_serial_driver)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/gimbal_tf_broadcaster.py scripts/serial_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES scripts/fire_transmit_gate.py scripts/gimbal_kinematics.py scripts/protocol.py scripts/windows_serial.py
    scripts/windows_duplex.py scripts/windows_duplex_helper.cs README.md
    scripts/control_latency_trace.py scripts/control_latency_protocol.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fire_transmit_gate test/test_fire_transmit_gate.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  ament_add_pytest_test(test_fire_serial_integration test/test_fire_serial_integration.py
    TIMEOUT 45
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  ament_add_pytest_test(test_control_latency test/test_control_latency.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_control_latency_node test/test_control_latency_node.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_protocol test/test_protocol.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_windows_serial test/test_windows_serial.py
    TIMEOUT 20
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_windows_duplex test/test_windows_duplex.py
    TIMEOUT 60
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_windows_duplex_host test/test_windows_duplex_host.py
    TIMEOUT 60
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_windows_duplex_fault_recovery test/test_windows_duplex_fault_recovery.py
    TIMEOUT 60
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_windows_duplex_node test/test_windows_duplex_node.py
    TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_serial_bridge_integration test/test_serial_bridge_integration.py
    TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_gimbal_kinematics test/test_gimbal_kinematics.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_gimbal_tf_integration test/test_gimbal_tf_integration.py
    TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
endif()

ament_package()
