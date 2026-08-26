cmake_minimum_required(VERSION 3.16)
project(mvs_raw_capture_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS installation root")
find_path(
  MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS "${MVS_ROOT}/include"
  NO_DEFAULT_PATH
  REQUIRED
)
find_library(
  MVS_LIBRARY
  NAMES MvCameraControl
  PATHS "${MVS_ROOT}/lib/64"
  NO_DEFAULT_PATH
  REQUIRED
)

add_library(raw_capture_session STATIC src/session.cpp src/cli.cpp src/capture_result.cpp)
target_include_directories(raw_capture_session PUBLIC include)
target_link_libraries(raw_capture_session PUBLIC ZLIB::ZLIB)
target_compile_options(raw_capture_session PRIVATE -Wall -Wextra -Wpedantic)

add_library(raw_capture_converter STATIC src/converter.cpp)
target_include_directories(raw_capture_converter PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(raw_capture_converter PUBLIC raw_capture_session ${OpenCV_LIBS})
target_compile_options(raw_capture_converter PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mvs_raw_capture src/mvs_raw_capture.cpp)
target_include_directories(mvs_raw_capture PRIVATE "${MVS_INCLUDE_DIR}")
target_link_libraries(
  mvs_raw_capture PRIVATE raw_capture_session "${MVS_LIBRARY}" Threads::Threads
)
target_compile_options(mvs_raw_capture PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(mvs_raw_capture PROPERTIES BUILD_RPATH "${MVS_ROOT}/lib/64")

add_executable(mvs_bayer_convert src/mvs_bayer_convert.cpp)
target_link_libraries(mvs_bayer_convert PRIVATE raw_capture_converter)
target_compile_options(mvs_bayer_convert PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  add_executable(test_raw_session tests/test_raw_session.cpp)
  target_link_libraries(test_raw_session PRIVATE raw_capture_converter)
  target_compile_options(test_raw_session PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(test_capture_result tests/test_capture_result.cpp)
  target_link_libraries(test_capture_result PRIVATE raw_capture_session)
  target_compile_options(test_capture_result PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(test_diag26_decode tests/test_diag26_decode.cpp)
  target_include_directories(test_diag26_decode PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_diag26_decode PRIVATE raw_capture_session ${OpenCV_LIBS})
  target_compile_options(test_diag26_decode PRIVATE -Wall -Wextra -Wpedantic)

  add_test(NAME raw_session_unit COMMAND test_raw_session)
  add_test(NAME capture_result_unit COMMAND test_capture_result)
  add_test(NAME capture_help COMMAND mvs_raw_capture --help)
  add_test(NAME converter_help COMMAND mvs_bayer_convert --help)
  add_test(
    NAME cli_error_codes
    COMMAND
      ${CMAKE_COMMAND}
      -DCAPTURE=$<TARGET_FILE:mvs_raw_capture>
      -DCONVERTER=$<TARGET_FILE:mvs_bayer_convert>
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_cli_error_codes.cmake
  )
endif()
