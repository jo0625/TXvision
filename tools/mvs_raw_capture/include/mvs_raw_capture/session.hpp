#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace mvs_raw_capture
{
namespace fs = std::filesystem;

constexpr std::uint32_t kSchemaVersion = 2U;
constexpr std::uint32_t kBaselineWidth = 1440U;
constexpr std::uint32_t kBaselineHeight = 1080U;
constexpr std::uint32_t kBaselinePixelFormat = 17301513U;
constexpr std::uint64_t kBaselinePayloadSize = 1555200U;
constexpr std::size_t kQueueCapacity = 32U;
constexpr std::uint64_t kSyncIntervalFrames = 64U;

enum class CounterSource {
  not_available,
  frame_counter,
  frame_number,
};

enum class CounterEvent {
  none,
  gap,
  reset,
  repeat,
};

const char * to_string(CounterSource value) noexcept;
const char * to_string(CounterEvent value) noexcept;
CounterSource parse_counter_source(const std::string & value);
CounterEvent parse_counter_event(const std::string & value);

struct CounterObservation
{
  CounterEvent event = CounterEvent::none;
  std::uint32_t previous_sensor_frame_id = 0;
  std::uint64_t missing_count = 0;
};

class SensorCounterTracker
{
public:
  CounterObservation observe(std::uint32_t sensor_frame_id) noexcept;
  std::uint64_t gap_events() const noexcept { return gap_events_; }
  std::uint64_t missing_frames() const noexcept { return missing_frames_; }
  std::uint64_t reset_events() const noexcept { return reset_events_; }
  std::uint64_t repeat_events() const noexcept { return repeat_events_; }

private:
  bool have_previous_ = false;
  std::uint32_t previous_ = 0;
  std::uint64_t gap_events_ = 0;
  std::uint64_t missing_frames_ = 0;
  std::uint64_t reset_events_ = 0;
  std::uint64_t repeat_events_ = 0;
};

struct FrameRecord
{
  std::uint64_t acquired_index = 0;
  std::uint32_t sensor_frame_id = 0;
  CounterSource counter_source = CounterSource::not_available;
  std::uint64_t sensor_timestamp = 0;
  std::uint64_t host_receive_timestamp_ns = 0;
  std::uint64_t file_byte_offset = 0;
  std::uint64_t payload_size = 0;
  std::uint32_t crc32 = 0;
  CounterEvent counter_event = CounterEvent::none;
  std::uint32_t previous_sensor_frame_id = 0;
  std::uint64_t missing_count = 0;
};

const char * frame_csv_header() noexcept;
std::string frame_record_csv(const FrameRecord & record);
FrameRecord parse_frame_record_csv(const std::string & line);

struct SessionManifest
{
  std::string status = "incomplete";
  std::string session_name;
  std::string created_at_utc;
  std::string completed_at_utc;
  std::string incomplete_reason = "capture_not_completed";
  double requested_duration_seconds = 0.0;
  std::uint64_t current_reference_estimated_bytes = 0;
  std::uint64_t worst_case_estimated_bytes = 0;
  std::uint32_t width = kBaselineWidth;
  std::uint32_t height = kBaselineHeight;
  std::uint32_t pixel_format = kBaselinePixelFormat;
  std::uint64_t payload_size = kBaselinePayloadSize;
  std::uint64_t acquired_frames = 0;
  std::uint64_t written_frames = 0;
  std::uint64_t queue_peak_depth = 0;
  std::uint64_t queue_overflow_count = 0;
  std::uint64_t sensor_gap_events = 0;
  std::uint64_t sensor_missing_frames = 0;
  std::uint64_t sensor_reset_events = 0;
  std::uint64_t sensor_repeat_events = 0;
  std::string data_file = "frames.bayer";
  std::string frames_csv_file = "frames.csv";
  std::string counter_events_file = "counter_events.csv";
  std::string capture_result_file = "capture_result.json";
};

struct SessionPaths
{
  fs::path directory;
  fs::path manifest;
  fs::path capture_result;
  fs::path data;
  fs::path frames_csv;
  fs::path counter_events_csv;
  fs::path initial_parameters;
  fs::path applied_parameters;
};

struct LoadedSession
{
  SessionManifest manifest;
  SessionPaths paths;
  std::vector<FrameRecord> records;
};

std::string utc_timestamp();
std::string json_escape(const std::string & value);
std::string manifest_json(const SessionManifest & manifest);
SessionManifest parse_manifest_json(const std::string & json);
void write_text_atomic_synced(const fs::path & path, const std::string & content);
std::string read_text_file(const fs::path & path);
SessionPaths make_session_paths(const fs::path & directory);
SessionPaths create_session_directory(
  const fs::path & output_dir, const std::string & session_name);
LoadedSession load_session(const fs::path & session_dir, bool allow_incomplete);
std::vector<FrameRecord> read_frame_records(const fs::path & csv_path);
void validate_frame_records(
  const std::vector<FrameRecord> & records, std::uint64_t payload_size, bool require_from_zero);
std::vector<unsigned char> read_frame_at(
  const fs::path & data_path, const FrameRecord & record, bool verify_crc);
std::uint32_t crc32_bytes(const unsigned char * data, std::size_t size) noexcept;
std::uint64_t available_bytes(const fs::path & path);
void validate_session_name(const std::string & value);

}  // namespace mvs_raw_capture
