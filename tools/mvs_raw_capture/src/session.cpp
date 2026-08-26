#include "mvs_raw_capture/session.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>
#include <zlib.h>

#include <array>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include "mvs_raw_capture/capture_result.hpp"

namespace mvs_raw_capture
{
namespace
{
constexpr const char * kFrameCsvHeader =
  "acquired_index,sensor_frame_id,sensor_counter_source,sensor_timestamp,"
  "host_receive_timestamp_ns,file_byte_offset,payload_size,crc32,counter_event,"
  "previous_sensor_frame_id,missing_count";

std::string json_escape_impl(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<unsigned int>(character) << std::dec;
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  return output.str();
}

void write_all(int fd, const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  std::size_t offset = 0;
  while (offset < size) {
    const ssize_t count = write(fd, bytes + offset, size - offset);
    if (count > 0) {
      offset += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) continue;
    throw std::runtime_error("write failed: " + std::string(std::strerror(errno)));
  }
}

void fdatasync_checked(int fd)
{
  int result = 0;
  do {
    result = fdatasync(fd);
  } while (result != 0 && errno == EINTR);
  if (result != 0) {
    throw std::runtime_error("fdatasync failed: " + std::string(std::strerror(errno)));
  }
}

void close_checked(int fd)
{
  int result = 0;
  do {
    result = close(fd);
  } while (result != 0 && errno == EINTR);
  if (result != 0) throw std::runtime_error("close failed: " + std::string(std::strerror(errno)));
}

void sync_directory(const fs::path & directory)
{
  const int fd = open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error("open directory failed: " + std::string(std::strerror(errno)));
  }
  int result = 0;
  do {
    result = fsync(fd);
  } while (result != 0 && errno == EINTR);
  const int saved_errno = errno;
  close(fd);
  if (result != 0) {
    throw std::runtime_error("directory fsync failed: " + std::string(std::strerror(saved_errno)));
  }
}

template <typename T>
T parse_unsigned(const std::string & value, const char * field)
{
  T parsed{};
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
    throw std::runtime_error(std::string("invalid unsigned field: ") + field);
  }
  return parsed;
}

std::vector<std::string> split_csv(const std::string & line)
{
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = line.find(',', start);
    fields.push_back(line.substr(start, comma == std::string::npos ? comma : comma - start));
    if (comma == std::string::npos) break;
    start = comma + 1U;
  }
  return fields;
}

std::string extract_string(const std::string & json, const std::string & key)
{
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  if (!std::regex_search(json, match, pattern)) {
    throw std::runtime_error("manifest key missing: " + key);
  }
  return match[1].str();
}

std::uint64_t extract_uint(const std::string & json, const std::string & key)
{
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*([0-9]+)");
  std::smatch match;
  if (!std::regex_search(json, match, pattern)) {
    throw std::runtime_error("manifest key missing: " + key);
  }
  return parse_unsigned<std::uint64_t>(match[1].str(), key.c_str());
}

double extract_double(const std::string & json, const std::string & key)
{
  const std::regex pattern(
    "\\\"" + key + "\\\"\\s*:\\s*([-+]?[0-9]+(?:\\.[0-9]+)?(?:[eE][-+]?[0-9]+)?)");
  std::smatch match;
  if (!std::regex_search(json, match, pattern)) {
    throw std::runtime_error("manifest key missing: " + key);
  }
  std::size_t consumed = 0;
  const double value = std::stod(match[1].str(), &consumed);
  if (consumed != match[1].str().size() || !std::isfinite(value)) {
    throw std::runtime_error("invalid manifest number: " + key);
  }
  return value;
}

}  // namespace

std::string json_escape(const std::string & value) { return json_escape_impl(value); }

const char * to_string(CounterSource value) noexcept
{
  switch (value) {
    case CounterSource::frame_counter:
      return "nFrameCounter";
    case CounterSource::frame_number:
      return "nFrameNum";
    case CounterSource::not_available:
      return "NOT_AVAILABLE";
  }
  return "NOT_AVAILABLE";
}

const char * to_string(CounterEvent value) noexcept
{
  switch (value) {
    case CounterEvent::none:
      return "NONE";
    case CounterEvent::gap:
      return "GAP";
    case CounterEvent::reset:
      return "RESET";
    case CounterEvent::repeat:
      return "REPEAT";
  }
  return "NONE";
}

CounterSource parse_counter_source(const std::string & value)
{
  if (value == "nFrameCounter") return CounterSource::frame_counter;
  if (value == "nFrameNum") return CounterSource::frame_number;
  if (value == "NOT_AVAILABLE") return CounterSource::not_available;
  throw std::runtime_error("invalid counter source: " + value);
}

CounterEvent parse_counter_event(const std::string & value)
{
  if (value == "NONE") return CounterEvent::none;
  if (value == "GAP") return CounterEvent::gap;
  if (value == "RESET") return CounterEvent::reset;
  if (value == "REPEAT") return CounterEvent::repeat;
  throw std::runtime_error("invalid counter event: " + value);
}

CounterObservation SensorCounterTracker::observe(std::uint32_t sensor_frame_id) noexcept
{
  CounterObservation observation;
  if (!have_previous_) {
    previous_ = sensor_frame_id;
    have_previous_ = true;
    return observation;
  }
  observation.previous_sensor_frame_id = previous_;
  if (sensor_frame_id > previous_) {
    const std::uint32_t difference = sensor_frame_id - previous_;
    if (difference > 1U) {
      observation.event = CounterEvent::gap;
      observation.missing_count = difference - 1U;
      ++gap_events_;
      missing_frames_ += observation.missing_count;
    }
  } else if (sensor_frame_id < previous_) {
    observation.event = CounterEvent::reset;
    ++reset_events_;
  } else {
    observation.event = CounterEvent::repeat;
    ++repeat_events_;
  }
  previous_ = sensor_frame_id;
  return observation;
}

const char * frame_csv_header() noexcept { return kFrameCsvHeader; }

std::string frame_record_csv(const FrameRecord & record)
{
  std::ostringstream output;
  output << record.acquired_index << ',' << record.sensor_frame_id << ','
         << to_string(record.counter_source) << ',' << record.sensor_timestamp << ','
         << record.host_receive_timestamp_ns << ',' << record.file_byte_offset << ','
         << record.payload_size << ',' << record.crc32 << ',' << to_string(record.counter_event)
         << ',' << record.previous_sensor_frame_id << ',' << record.missing_count;
  return output.str();
}

FrameRecord parse_frame_record_csv(const std::string & line)
{
  const std::vector<std::string> fields = split_csv(line);
  if (fields.size() != 11U) throw std::runtime_error("frame CSV field count mismatch");
  FrameRecord record;
  record.acquired_index = parse_unsigned<std::uint64_t>(fields[0], "acquired_index");
  record.sensor_frame_id = parse_unsigned<std::uint32_t>(fields[1], "sensor_frame_id");
  record.counter_source = parse_counter_source(fields[2]);
  record.sensor_timestamp = parse_unsigned<std::uint64_t>(fields[3], "sensor_timestamp");
  record.host_receive_timestamp_ns =
    parse_unsigned<std::uint64_t>(fields[4], "host_receive_timestamp_ns");
  record.file_byte_offset = parse_unsigned<std::uint64_t>(fields[5], "file_byte_offset");
  record.payload_size = parse_unsigned<std::uint64_t>(fields[6], "payload_size");
  record.crc32 = parse_unsigned<std::uint32_t>(fields[7], "crc32");
  record.counter_event = parse_counter_event(fields[8]);
  record.previous_sensor_frame_id =
    parse_unsigned<std::uint32_t>(fields[9], "previous_sensor_frame_id");
  record.missing_count = parse_unsigned<std::uint64_t>(fields[10], "missing_count");
  return record;
}

std::string utc_timestamp()
{
  const std::time_t now = std::time(nullptr);
  struct tm value
  {
  };
  if (gmtime_r(&now, &value) == nullptr) throw std::runtime_error("gmtime_r failed");
  std::array<char, 32> text{};
  if (std::strftime(text.data(), text.size(), "%Y-%m-%dT%H:%M:%SZ", &value) == 0U) {
    throw std::runtime_error("strftime failed");
  }
  return text.data();
}

std::string manifest_json(const SessionManifest & value)
{
  std::ostringstream output;
  output << std::setprecision(17) << "{\n"
         << "  \"schema_version\": " << kSchemaVersion << ",\n"
         << "  \"status\": \"" << json_escape(value.status) << "\",\n"
         << "  \"session_name\": \"" << json_escape(value.session_name) << "\",\n"
         << "  \"created_at_utc\": \"" << json_escape(value.created_at_utc) << "\",\n"
         << "  \"completed_at_utc\": \"" << json_escape(value.completed_at_utc) << "\",\n"
         << "  \"incomplete_reason\": \"" << json_escape(value.incomplete_reason) << "\",\n"
         << "  \"requested_duration_seconds\": " << value.requested_duration_seconds << ",\n"
         << "  \"current_reference_estimated_bytes\": " << value.current_reference_estimated_bytes
         << ",\n"
         << "  \"worst_case_estimated_bytes\": " << value.worst_case_estimated_bytes << ",\n"
         << "  \"width\": " << value.width << ",\n"
         << "  \"height\": " << value.height << ",\n"
         << "  \"pixel_format\": " << value.pixel_format << ",\n"
         << "  \"payload_size\": " << value.payload_size << ",\n"
         << "  \"acquired_frames\": " << value.acquired_frames << ",\n"
         << "  \"written_frames\": " << value.written_frames << ",\n"
         << "  \"queue_peak_depth\": " << value.queue_peak_depth << ",\n"
         << "  \"queue_overflow_count\": " << value.queue_overflow_count << ",\n"
         << "  \"sensor_gap_events\": " << value.sensor_gap_events << ",\n"
         << "  \"sensor_missing_frames\": " << value.sensor_missing_frames << ",\n"
         << "  \"sensor_reset_events\": " << value.sensor_reset_events << ",\n"
         << "  \"sensor_repeat_events\": " << value.sensor_repeat_events << ",\n"
         << "  \"data_file\": \"" << json_escape(value.data_file) << "\",\n"
         << "  \"frames_csv_file\": \"" << json_escape(value.frames_csv_file) << "\",\n"
         << "  \"counter_events_file\": \"" << json_escape(value.counter_events_file) << "\",\n"
         << "  \"capture_result_file\": \"" << json_escape(value.capture_result_file) << "\"\n}\n";
  return output.str();
}

SessionManifest parse_manifest_json(const std::string & json)
{
  if (extract_uint(json, "schema_version") != kSchemaVersion) {
    throw std::runtime_error("unsupported session schema_version");
  }
  SessionManifest value;
  value.status = extract_string(json, "status");
  value.session_name = extract_string(json, "session_name");
  value.created_at_utc = extract_string(json, "created_at_utc");
  value.completed_at_utc = extract_string(json, "completed_at_utc");
  value.incomplete_reason = extract_string(json, "incomplete_reason");
  value.requested_duration_seconds = extract_double(json, "requested_duration_seconds");
  value.current_reference_estimated_bytes = extract_uint(json, "current_reference_estimated_bytes");
  value.worst_case_estimated_bytes = extract_uint(json, "worst_case_estimated_bytes");
  value.width = static_cast<std::uint32_t>(extract_uint(json, "width"));
  value.height = static_cast<std::uint32_t>(extract_uint(json, "height"));
  value.pixel_format = static_cast<std::uint32_t>(extract_uint(json, "pixel_format"));
  value.payload_size = extract_uint(json, "payload_size");
  value.acquired_frames = extract_uint(json, "acquired_frames");
  value.written_frames = extract_uint(json, "written_frames");
  value.queue_peak_depth = extract_uint(json, "queue_peak_depth");
  value.queue_overflow_count = extract_uint(json, "queue_overflow_count");
  value.sensor_gap_events = extract_uint(json, "sensor_gap_events");
  value.sensor_missing_frames = extract_uint(json, "sensor_missing_frames");
  value.sensor_reset_events = extract_uint(json, "sensor_reset_events");
  value.sensor_repeat_events = extract_uint(json, "sensor_repeat_events");
  value.data_file = extract_string(json, "data_file");
  value.frames_csv_file = extract_string(json, "frames_csv_file");
  value.counter_events_file = extract_string(json, "counter_events_file");
  value.capture_result_file = extract_string(json, "capture_result_file");
  return value;
}

void write_text_atomic_synced(const fs::path & path, const std::string & content)
{
  const fs::path temporary = path.string() + ".tmp." + std::to_string(getpid());
  int fd = open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
  if (fd < 0) {
    throw std::runtime_error("create temporary file failed: " + std::string(std::strerror(errno)));
  }
  try {
    write_all(fd, content.data(), content.size());
    fdatasync_checked(fd);
    close_checked(fd);
    fd = -1;
    fs::rename(temporary, path);
    sync_directory(path.parent_path());
  } catch (...) {
    if (fd >= 0) close(fd);
    unlink(temporary.c_str());
    throw;
  }
}

std::string read_text_file(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open " + path.string());
  std::ostringstream output;
  output << input.rdbuf();
  if (input.bad()) throw std::runtime_error("cannot read " + path.string());
  return output.str();
}

SessionPaths make_session_paths(const fs::path & directory)
{
  return {
    directory,
    directory / "session.json",
    directory / "capture_result.json",
    directory / "frames.bayer",
    directory / "frames.csv",
    directory / "counter_events.csv",
    directory / "initial_camera_parameters.json",
    directory / "applied_camera_parameters.json"};
}

void validate_session_name(const std::string & value)
{
  if (value.empty() || value.size() > 128U || value == "." || value == "..") {
    throw std::invalid_argument("invalid session name");
  }
  for (const unsigned char character : value) {
    if (!(std::isalnum(character) != 0 || character == '-' || character == '_' ||
          character == '.')) {
      throw std::invalid_argument(
        "session name may contain only letters, digits, '.', '-' and '_'");
    }
  }
}

SessionPaths create_session_directory(const fs::path & output_dir, const std::string & session_name)
{
  validate_session_name(session_name);
  if (output_dir.empty()) throw std::invalid_argument("output directory is empty");
  std::error_code error;
  fs::create_directories(output_dir, error);
  if (error) throw std::runtime_error("cannot create output directory: " + error.message());
  const fs::path directory = output_dir / session_name;
  if (!fs::create_directory(directory, error)) {
    if (error) throw std::runtime_error("cannot create session directory: " + error.message());
    throw std::runtime_error("session directory already exists: " + directory.string());
  }
  sync_directory(output_dir);
  return make_session_paths(directory);
}

std::vector<FrameRecord> read_frame_records(const fs::path & csv_path)
{
  std::ifstream input(csv_path);
  if (!input) throw std::runtime_error("cannot open frame CSV: " + csv_path.string());
  std::string line;
  if (!std::getline(input, line) || line != frame_csv_header()) {
    throw std::runtime_error("frame CSV header mismatch");
  }
  std::vector<FrameRecord> records;
  while (std::getline(input, line)) {
    if (line.empty()) throw std::runtime_error("empty frame CSV row");
    records.push_back(parse_frame_record_csv(line));
  }
  if (!input.eof()) throw std::runtime_error("frame CSV read failed");
  return records;
}

void validate_frame_records(
  const std::vector<FrameRecord> & records, std::uint64_t payload_size, bool require_from_zero)
{
  for (std::size_t index = 0; index < records.size(); ++index) {
    const FrameRecord & record = records[index];
    const std::uint64_t expected_index =
      require_from_zero ? index : records.front().acquired_index + index;
    if (record.acquired_index != expected_index) {
      throw std::runtime_error("acquired_index is not contiguous");
    }
    if (record.file_byte_offset != static_cast<std::uint64_t>(index) * payload_size) {
      throw std::runtime_error("frame byte offset is not contiguous");
    }
    if (record.payload_size != payload_size) {
      throw std::runtime_error("frame payload size mismatch");
    }
  }
}

LoadedSession load_session(const fs::path & session_dir, bool allow_incomplete)
{
  LoadedSession loaded;
  loaded.paths = make_session_paths(session_dir);
  loaded.manifest = parse_manifest_json(read_text_file(loaded.paths.manifest));
  if (loaded.manifest.status != "complete" && !allow_incomplete) {
    throw std::runtime_error("session is incomplete; use --allow-incomplete to override");
  }
  loaded.paths.data = session_dir / loaded.manifest.data_file;
  loaded.paths.frames_csv = session_dir / loaded.manifest.frames_csv_file;
  loaded.paths.counter_events_csv = session_dir / loaded.manifest.counter_events_file;
  loaded.paths.capture_result = session_dir / loaded.manifest.capture_result_file;
  loaded.records = read_frame_records(loaded.paths.frames_csv);
  validate_frame_records(loaded.records, loaded.manifest.payload_size, true);
  if (
    loaded.manifest.status == "complete" &&
    loaded.records.size() != loaded.manifest.written_frames) {
    throw std::runtime_error("complete manifest frame count differs from CSV");
  }
  if (loaded.manifest.status == "complete" && !fs::is_regular_file(loaded.paths.capture_result)) {
    throw std::runtime_error("complete session capture result is missing");
  }
  const std::uint64_t expected_size =
    static_cast<std::uint64_t>(loaded.records.size()) * loaded.manifest.payload_size;
  const std::uint64_t actual_size = fs::file_size(loaded.paths.data);
  if (
    (loaded.manifest.status == "complete" && actual_size != expected_size) ||
    (loaded.manifest.status != "complete" && actual_size < expected_size)) {
    throw std::runtime_error("Bayer file size does not match frame records");
  }
  return loaded;
}

std::vector<unsigned char> read_frame_at(
  const fs::path & data_path, const FrameRecord & record, bool verify_crc)
{
  const int fd = open(data_path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0)
    throw std::runtime_error("cannot open Bayer data: " + std::string(std::strerror(errno)));
  std::vector<unsigned char> data(record.payload_size);
  std::size_t offset = 0;
  try {
    while (offset < data.size()) {
      const ssize_t count = pread(
        fd, data.data() + offset, data.size() - offset,
        static_cast<off_t>(record.file_byte_offset + offset));
      if (count > 0) {
        offset += static_cast<std::size_t>(count);
        continue;
      }
      if (count < 0 && errno == EINTR) continue;
      throw std::runtime_error(
        count == 0 ? "unexpected end of Bayer data"
                   : "pread failed: " + std::string(std::strerror(errno)));
    }
    close_checked(fd);
  } catch (...) {
    close(fd);
    throw;
  }
  if (verify_crc && crc32_bytes(data.data(), data.size()) != record.crc32) {
    throw std::runtime_error(
      "frame CRC32 mismatch at acquired_index=" + std::to_string(record.acquired_index));
  }
  return data;
}

std::uint32_t crc32_bytes(const unsigned char * data, std::size_t size) noexcept
{
  return static_cast<std::uint32_t>(
    crc32(0L, reinterpret_cast<const Bytef *>(data), static_cast<uInt>(size)));
}

std::uint64_t available_bytes(const fs::path & path)
{
  struct statvfs value
  {
  };
  if (statvfs(path.c_str(), &value) != 0) {
    throw std::runtime_error("statvfs failed: " + std::string(std::strerror(errno)));
  }
  return checked_multiply(
    static_cast<std::uint64_t>(value.f_bavail), static_cast<std::uint64_t>(value.f_frsize),
    "available filesystem bytes");
}

}  // namespace mvs_raw_capture
