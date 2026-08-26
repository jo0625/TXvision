#pragma once

#include <cstdint>
#include <string>

namespace mvs_raw_capture
{

constexpr std::uint32_t kCaptureResultSchemaVersion = 1U;
constexpr const char * kToolVersion = "1.0.0";
constexpr std::uint64_t kReferenceBytesPerSecond = 98000000ULL;
constexpr std::uint64_t kWorstCaseFramesPerSecond = 250ULL;
constexpr std::uint64_t kDiskReserveBytes = 1ULL * 1024ULL * 1024ULL * 1024ULL;

struct DiskBudgetParameters
{
  std::uint64_t reference_bytes_per_second = kReferenceBytesPerSecond;
  std::uint64_t payload_bytes_per_frame = 1555200ULL;
  std::uint64_t worst_case_frames_per_second = kWorstCaseFramesPerSecond;
  std::uint64_t reserve_bytes = kDiskReserveBytes;
};

struct DiskBudget
{
  std::uint64_t current_reference_estimated_bytes = 0;
  std::uint64_t worst_case_estimated_bytes = 0;
  std::uint64_t required_available_bytes = 0;
};

std::uint64_t checked_multiply(std::uint64_t left, std::uint64_t right, const char * description);
std::uint64_t checked_add(std::uint64_t left, std::uint64_t right, const char * description);
DiskBudget calculate_disk_budget(
  double duration_seconds, const DiskBudgetParameters & parameters = {});

struct CaptureResult
{
  std::uint32_t schema_version = kCaptureResultSchemaVersion;
  std::string tool_version = kToolVersion;
  std::string session_name;
  double requested_duration_seconds = 0.0;
  std::string started_at_utc;
  std::string finished_at_utc;
  double elapsed_seconds = 0.0;
  std::uint64_t current_reference_estimated_bytes = 0;
  std::uint64_t worst_case_estimated_bytes = 0;
  std::uint64_t available_bytes_at_start = 0;
  std::uint64_t acquired_frames = 0;
  std::uint64_t written_frames = 0;
  double written_fps = 0.0;
  double written_MBps = 0.0;
  std::uint64_t queue_peak_depth = 0;
  std::uint64_t queue_overflow_count = 0;
  std::uint64_t sensor_gap_events = 0;
  std::uint64_t missing_frames = 0;
  std::uint64_t reset_events = 0;
  std::uint64_t repeat_events = 0;
  std::uint64_t timeout_count = 0;
  std::uint64_t sdk_error_count = 0;
  std::uint64_t metadata_error_count = 0;
  std::uint64_t ownership_error_count = 0;
  std::uint64_t write_error_count = 0;
  std::uint64_t sync_error_count = 0;
  std::uint64_t close_error_count = 0;
  std::uint64_t periodic_fdatasync_calls = 0;
  bool output_verification_pass = false;
  bool camera_released = false;
  std::string final_status = "incomplete";
  std::string failure_reason = "capture_not_completed";
};

std::string capture_result_json(const CaptureResult & result);

struct CompletionGate
{
  bool parameter_gate_pass = false;
  bool duration_complete = false;
  bool stopped_by_signal = false;
  bool camera_released = false;
  std::uint64_t acquired_frames = 0;
  std::uint64_t written_frames = 0;
  std::uint64_t queue_overflow_count = 0;
  std::uint64_t sdk_error_count = 0;
  std::uint64_t metadata_error_count = 0;
  std::uint64_t ownership_error_count = 0;
  std::uint64_t write_error_count = 0;
  std::uint64_t sync_error_count = 0;
  std::uint64_t close_error_count = 0;
  bool output_verification_pass = false;
  bool runtime_error_free = false;
  bool result_file_persisted = false;
};

bool can_mark_session_complete(const CompletionGate & gate) noexcept;
std::string completion_failure_reason(const CompletionGate & gate);

}  // namespace mvs_raw_capture
