execute_process(
  COMMAND "${CAPTURE}" --duration 1
  RESULT_VARIABLE capture_result
  OUTPUT_VARIABLE capture_stdout
  ERROR_VARIABLE capture_stderr
)
if(NOT capture_result EQUAL 2)
  message(FATAL_ERROR "capture CLI exit=${capture_result}, expected 2\n${capture_stdout}${capture_stderr}")
endif()

execute_process(
  COMMAND "${CAPTURE}" --duration 1e9999 --output-dir /tmp --session-name invalid_duration
  RESULT_VARIABLE capture_range_result
  OUTPUT_VARIABLE capture_range_stdout
  ERROR_VARIABLE capture_range_stderr
)
if(NOT capture_range_result EQUAL 2)
  message(FATAL_ERROR
    "capture range CLI exit=${capture_range_result}, expected 2\n"
    "${capture_range_stdout}${capture_range_stderr}"
  )
endif()

execute_process(
  COMMAND "${CONVERTER}" --session-dir missing
  RESULT_VARIABLE converter_result
  OUTPUT_VARIABLE converter_stdout
  ERROR_VARIABLE converter_stderr
)
if(NOT converter_result EQUAL 2)
  message(FATAL_ERROR
    "converter CLI exit=${converter_result}, expected 2\n${converter_stdout}${converter_stderr}"
  )
endif()
