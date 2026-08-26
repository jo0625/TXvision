#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "mvs_raw_capture/capture_result.hpp"

using namespace mvs_raw_capture;

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

template <typename Function>
void require_throws(Function function, const std::string & message)
{
  try {
    function();
  } catch (const std::exception &) {
    return;
  }
  throw std::runtime_error(message);
}

CompletionGate passing_gate()
{
  CompletionGate gate;
  gate.parameter_gate_pass = true;
  gate.duration_complete = true;
  gate.camera_released = true;
  gate.acquired_frames = 100U;
  gate.written_frames = 100U;
  gate.output_verification_pass = true;
  gate.runtime_error_free = true;
  gate.result_file_persisted = true;
  return gate;
}

}  // namespace

int main()
{
  try {
    const DiskBudget five_seconds = calculate_disk_budget(5.0);
    require(
      five_seconds.current_reference_estimated_bytes == 490000000ULL,
      "5-second reference budget mismatch");
    require(
      five_seconds.worst_case_estimated_bytes == 1944000000ULL,
      "5-second worst-case budget mismatch");
    require(
      five_seconds.required_available_bytes == 3017741824ULL, "5-second required budget mismatch");

    const DiskBudget sixty_seconds = calculate_disk_budget(60.0);
    require(
      sixty_seconds.current_reference_estimated_bytes == 5880000000ULL,
      "60-second reference budget mismatch");
    require(
      sixty_seconds.worst_case_estimated_bytes == 23328000000ULL,
      "60-second worst-case budget mismatch");
    require(
      sixty_seconds.required_available_bytes == 24401741824ULL,
      "60-second required budget mismatch");

    DiskBudgetParameters multiplication_overflow;
    multiplication_overflow.reference_bytes_per_second = 1U;
    multiplication_overflow.payload_bytes_per_frame = std::numeric_limits<std::uint64_t>::max();
    multiplication_overflow.worst_case_frames_per_second = 2U;
    multiplication_overflow.reserve_bytes = 0U;
    require_throws(
      [&]() { calculate_disk_budget(1.0, multiplication_overflow); },
      "worst-case rate multiplication overflow was accepted");

    DiskBudgetParameters estimate_overflow;
    estimate_overflow.reference_bytes_per_second = std::numeric_limits<std::uint64_t>::max();
    estimate_overflow.payload_bytes_per_frame = 1U;
    estimate_overflow.worst_case_frames_per_second = 1U;
    estimate_overflow.reserve_bytes = 0U;
    require_throws(
      [&]() { calculate_disk_budget(2.0, estimate_overflow); },
      "duration multiplication overflow was accepted");

    DiskBudgetParameters addition_overflow;
    addition_overflow.reference_bytes_per_second = 1U;
    addition_overflow.payload_bytes_per_frame =
      std::numeric_limits<std::uint64_t>::max() - kDiskReserveBytes + 1U;
    addition_overflow.worst_case_frames_per_second = 1U;
    addition_overflow.reserve_bytes = kDiskReserveBytes;
    require_throws(
      [&]() { calculate_disk_budget(1.0, addition_overflow); },
      "budget plus reserve overflow was accepted");

    CaptureResult result;
    result.session_name = std::string("session\"\\\n\t") + static_cast<char>(1);
    result.requested_duration_seconds = 5.0;
    result.started_at_utc = "2026-08-07T00:00:00Z";
    result.finished_at_utc = "2026-08-07T00:00:05Z";
    result.elapsed_seconds = 5.0;
    result.current_reference_estimated_bytes = five_seconds.current_reference_estimated_bytes;
    result.worst_case_estimated_bytes = five_seconds.worst_case_estimated_bytes;
    result.available_bytes_at_start = five_seconds.required_available_bytes;
    result.acquired_frames = 300U;
    result.written_frames = 300U;
    result.written_fps = 60.0;
    result.written_MBps = 93.312;
    result.queue_peak_depth = 4U;
    result.sensor_gap_events = 1U;
    result.missing_frames = 1U;
    result.reset_events = 1U;
    result.repeat_events = 2U;
    result.periodic_fdatasync_calls = 4U;
    result.output_verification_pass = true;
    result.camera_released = true;
    result.final_status = "complete";
    result.failure_reason = "";
    const std::string json = capture_result_json(result);
    const char * required_fields[] = {
      "schema_version",
      "tool_version",
      "session_name",
      "requested_duration_seconds",
      "started_at_utc",
      "finished_at_utc",
      "elapsed_seconds",
      "current_reference_estimated_bytes",
      "worst_case_estimated_bytes",
      "available_bytes_at_start",
      "acquired_frames",
      "written_frames",
      "written_fps",
      "written_MBps",
      "queue_peak_depth",
      "queue_overflow_count",
      "sensor_gap_events",
      "missing_frames",
      "reset_events",
      "repeat_events",
      "timeout_count",
      "sdk_error_count",
      "metadata_error_count",
      "ownership_error_count",
      "write_error_count",
      "sync_error_count",
      "close_error_count",
      "periodic_fdatasync_calls",
      "output_verification_pass",
      "camera_released",
      "final_status",
      "failure_reason",
    };
    for (const char * field : required_fields) {
      require(
        json.find(std::string("\"") + field + "\"") != std::string::npos,
        std::string("capture result field missing: ") + field);
    }
    require(
      json.find("session\\\"\\\\\\n\\t\\u0001") != std::string::npos,
      "capture result JSON string escaping failed");

    const CompletionGate good = passing_gate();
    require(can_mark_session_complete(good), "passing completion gate was rejected");
    CompletionGate failed = good;
    failed.queue_overflow_count = 1U;
    require(!can_mark_session_complete(failed), "queue overflow completion was accepted");
    failed = good;
    failed.written_frames = 99U;
    require(!can_mark_session_complete(failed), "frame count mismatch completion was accepted");
    failed = good;
    failed.camera_released = false;
    require(!can_mark_session_complete(failed), "unreleased camera completion was accepted");
    failed = good;
    failed.output_verification_pass = false;
    require(!can_mark_session_complete(failed), "unverified output completion was accepted");
    failed = good;
    failed.result_file_persisted = false;
    require(!can_mark_session_complete(failed), "missing result file completion was accepted");

    std::cout << "test_capture_result: PASS\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "test_capture_result: FAIL: " << error.what() << '\n';
    return 1;
  }
}
