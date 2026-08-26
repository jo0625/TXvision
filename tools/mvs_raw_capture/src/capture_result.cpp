#include "mvs_raw_capture/capture_result.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "mvs_raw_capture/session.hpp"

namespace mvs_raw_capture
{
namespace
{
std::uint64_t estimate_for_duration(
  double duration_seconds, std::uint64_t bytes_per_second, const char * description)
{
  if (!std::isfinite(duration_seconds) || duration_seconds <= 0.0) {
    throw std::invalid_argument("duration must be finite and greater than zero");
  }
  const long double bytes =
    static_cast<long double>(duration_seconds) * static_cast<long double>(bytes_per_second);
  if (!std::isfinite(bytes) || bytes > std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error(std::string(description) + " overflow");
  }
  return static_cast<std::uint64_t>(std::ceil(bytes));
}

void require_finite(double value, const char * field)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string("non-finite capture result field: ") + field);
  }
}

}  // namespace

std::uint64_t checked_multiply(std::uint64_t left, std::uint64_t right, const char * description)
{
  if (right != 0U && left > std::numeric_limits<std::uint64_t>::max() / right) {
    throw std::overflow_error(std::string(description) + " overflow");
  }
  return left * right;
}

std::uint64_t checked_add(std::uint64_t left, std::uint64_t right, const char * description)
{
  if (left > std::numeric_limits<std::uint64_t>::max() - right) {
    throw std::overflow_error(std::string(description) + " overflow");
  }
  return left + right;
}

DiskBudget calculate_disk_budget(double duration_seconds, const DiskBudgetParameters & parameters)
{
  const std::uint64_t worst_case_bytes_per_second = checked_multiply(
    parameters.payload_bytes_per_frame, parameters.worst_case_frames_per_second,
    "worst-case bytes per second");
  DiskBudget budget;
  budget.current_reference_estimated_bytes = estimate_for_duration(
    duration_seconds, parameters.reference_bytes_per_second, "reference data estimate");
  budget.worst_case_estimated_bytes = estimate_for_duration(
    duration_seconds, worst_case_bytes_per_second, "worst-case data estimate");
  budget.required_available_bytes = checked_add(
    budget.worst_case_estimated_bytes, parameters.reserve_bytes,
    "worst-case data estimate plus disk reserve");
  return budget;
}

std::string capture_result_json(const CaptureResult & result)
{
  require_finite(result.requested_duration_seconds, "requested_duration_seconds");
  require_finite(result.elapsed_seconds, "elapsed_seconds");
  require_finite(result.written_fps, "written_fps");
  require_finite(result.written_MBps, "written_MBps");
  std::ostringstream output;
  output << std::setprecision(17) << "{\n"
         << "  \"schema_version\": " << result.schema_version << ",\n"
         << "  \"tool_version\": \"" << json_escape(result.tool_version) << "\",\n"
         << "  \"session_name\": \"" << json_escape(result.session_name) << "\",\n"
         << "  \"requested_duration_seconds\": " << result.requested_duration_seconds << ",\n"
         << "  \"started_at_utc\": \"" << json_escape(result.started_at_utc) << "\",\n"
         << "  \"finished_at_utc\": \"" << json_escape(result.finished_at_utc) << "\",\n"
         << "  \"elapsed_seconds\": " << result.elapsed_seconds << ",\n"
         << "  \"current_reference_estimated_bytes\": " << result.current_reference_estimated_bytes
         << ",\n"
         << "  \"worst_case_estimated_bytes\": " << result.worst_case_estimated_bytes << ",\n"
         << "  \"available_bytes_at_start\": " << result.available_bytes_at_start << ",\n"
         << "  \"acquired_frames\": " << result.acquired_frames << ",\n"
         << "  \"written_frames\": " << result.written_frames << ",\n"
         << "  \"written_fps\": " << result.written_fps << ",\n"
         << "  \"written_MBps\": " << result.written_MBps << ",\n"
         << "  \"queue_peak_depth\": " << result.queue_peak_depth << ",\n"
         << "  \"queue_overflow_count\": " << result.queue_overflow_count << ",\n"
         << "  \"sensor_gap_events\": " << result.sensor_gap_events << ",\n"
         << "  \"missing_frames\": " << result.missing_frames << ",\n"
         << "  \"reset_events\": " << result.reset_events << ",\n"
         << "  \"repeat_events\": " << result.repeat_events << ",\n"
         << "  \"timeout_count\": " << result.timeout_count << ",\n"
         << "  \"sdk_error_count\": " << result.sdk_error_count << ",\n"
         << "  \"metadata_error_count\": " << result.metadata_error_count << ",\n"
         << "  \"ownership_error_count\": " << result.ownership_error_count << ",\n"
         << "  \"write_error_count\": " << result.write_error_count << ",\n"
         << "  \"sync_error_count\": " << result.sync_error_count << ",\n"
         << "  \"close_error_count\": " << result.close_error_count << ",\n"
         << "  \"periodic_fdatasync_calls\": " << result.periodic_fdatasync_calls << ",\n"
         << "  \"output_verification_pass\": "
         << (result.output_verification_pass ? "true" : "false") << ",\n"
         << "  \"camera_released\": " << (result.camera_released ? "true" : "false") << ",\n"
         << "  \"final_status\": \"" << json_escape(result.final_status) << "\",\n"
         << "  \"failure_reason\": \"" << json_escape(result.failure_reason) << "\"\n"
         << "}\n";
  return output.str();
}

bool can_mark_session_complete(const CompletionGate & gate) noexcept
{
  return gate.parameter_gate_pass && gate.duration_complete && !gate.stopped_by_signal &&
         gate.camera_released && gate.acquired_frames == gate.written_frames &&
         gate.queue_overflow_count == 0U && gate.sdk_error_count == 0U &&
         gate.metadata_error_count == 0U && gate.ownership_error_count == 0U &&
         gate.write_error_count == 0U && gate.sync_error_count == 0U &&
         gate.close_error_count == 0U && gate.output_verification_pass && gate.runtime_error_free &&
         gate.result_file_persisted;
}

std::string completion_failure_reason(const CompletionGate & gate)
{
  if (!gate.parameter_gate_pass) return "camera_parameter_gate_failed";
  if (!gate.duration_complete) return "requested_duration_not_completed";
  if (gate.stopped_by_signal) return "signal_received";
  if (!gate.camera_released) return "camera_release_failed";
  if (gate.acquired_frames != gate.written_frames) return "acquired_written_count_mismatch";
  if (gate.queue_overflow_count != 0U) return "queue_overflow";
  if (gate.sdk_error_count != 0U) return "sdk_error";
  if (gate.metadata_error_count != 0U) return "frame_metadata_error";
  if (gate.ownership_error_count != 0U) return "buffer_ownership_error";
  if (gate.write_error_count != 0U) return "data_or_metadata_write_error";
  if (gate.sync_error_count != 0U) return "data_or_metadata_sync_error";
  if (gate.close_error_count != 0U) return "data_or_metadata_close_error";
  if (!gate.output_verification_pass) return "output_verification_failed";
  if (!gate.runtime_error_free) return "runtime_error";
  if (!gate.result_file_persisted) return "capture_result_not_persisted";
  return "";
}

}  // namespace mvs_raw_capture
