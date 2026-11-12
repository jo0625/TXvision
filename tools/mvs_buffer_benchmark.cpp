#include <MvCameraControl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/types.h>
#include <unistd.h>

namespace fs = std::filesystem;
using SteadyClock = std::chrono::steady_clock;
using SystemClock = std::chrono::system_clock;

namespace
{
std::atomic_bool g_stop{false};

void signal_handler(int)
{
  g_stop.store(true);
}

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    SteadyClock::now().time_since_epoch()).count();
}

int64_t realtime_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    SystemClock::now().time_since_epoch()).count();
}

std::string hex_error(int code)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::setw(8) << std::setfill('0')
         << static_cast<uint32_t>(code);
  return stream.str();
}

void require_ok(int result, const std::string & operation)
{
  if (result != MV_OK) {
    throw std::runtime_error(operation + " failed: " + hex_error(result));
  }
}

std::string bounded_string(const unsigned char * value, size_t capacity)
{
  return std::string(reinterpret_cast<const char *>(value),
    strnlen(reinterpret_cast<const char *>(value), capacity));
}

std::string csv_escape(const std::string & value)
{
  if (value.find_first_of(",\"\n\r") == std::string::npos) {
    return value;
  }
  std::string escaped = "\"";
  for (const char character : value) {
    if (character == '\"') {
      escaped += "\"\"";
    } else {
      escaped += character;
    }
  }
  escaped += '\"';
  return escaped;
}

uint64_t resident_bytes()
{
  std::ifstream stream("/proc/self/statm");
  uint64_t total_pages = 0;
  uint64_t resident_pages = 0;
  if (!(stream >> total_pages >> resident_pages)) {
    return 0;
  }
  return resident_pages * static_cast<uint64_t>(::sysconf(_SC_PAGESIZE));
}

struct Options
{
  std::string serial{"DA8541046"};
  fs::path output_dir;
  std::vector<unsigned int> nodes{2, 4, 8, 16, 32, 64};
  double continuous_seconds{60.0};
  double load_seconds{60.0};
  double warmup_seconds{2.0};
  double recovery_seconds{1.0};
  double frame_rate{100.0};
  double exposure_us{3000.0};
  double gain_db{12.0};
  unsigned int width{1440};
  unsigned int height{1080};
  unsigned int offset_x{0};
  unsigned int offset_y{0};
  unsigned int timeout_ms{1000};
  std::vector<unsigned int> pause_ms{20, 50, 100, 250};
  unsigned int pause_repeats{3};
  bool allow_short{false};
};

std::vector<unsigned int> parse_uint_list(const std::string & text)
{
  std::vector<unsigned int> values;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ',')) {
    if (item.empty()) {
      throw std::invalid_argument("empty value in list: " + text);
    }
    values.push_back(static_cast<unsigned int>(std::stoul(item)));
  }
  if (values.empty()) {
    throw std::invalid_argument("list must not be empty");
  }
  return values;
}

void usage(const char * program)
{
  std::cout
    << "Usage: " << program << " --output DIR [options]\n"
    << "  --serial SN                 default DA8541046\n"
    << "  --nodes 2,4,8,16,32,64\n"
    << "  --continuous-s 60 --load-s 60 --warmup-s 2 --recovery-s 1\n"
    << "  --pause-ms 20,50,100,250 --pause-repeats 3\n"
    << "  --fps 100 --exposure-us 3000 --gain-db 12\n"
    << "  --width 1440 --height 1080 --offset-x 0 --offset-y 0\n"
    << "  --timeout-ms 1000\n"
    << "  --allow-short               allow non-conforming runs under 60 s\n";
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  auto require_value = [&](int & index) -> std::string {
      if (++index >= argc) {
        throw std::invalid_argument(std::string("missing value after ") + argv[index - 1]);
      }
      return argv[index];
    };

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      usage(argv[0]);
      std::exit(0);
    } else if (argument == "--output") {
      options.output_dir = require_value(index);
    } else if (argument == "--serial") {
      options.serial = require_value(index);
    } else if (argument == "--nodes") {
      options.nodes = parse_uint_list(require_value(index));
    } else if (argument == "--pause-ms") {
      options.pause_ms = parse_uint_list(require_value(index));
    } else if (argument == "--continuous-s") {
      options.continuous_seconds = std::stod(require_value(index));
    } else if (argument == "--load-s") {
      options.load_seconds = std::stod(require_value(index));
    } else if (argument == "--warmup-s") {
      options.warmup_seconds = std::stod(require_value(index));
    } else if (argument == "--recovery-s") {
      options.recovery_seconds = std::stod(require_value(index));
    } else if (argument == "--fps") {
      options.frame_rate = std::stod(require_value(index));
    } else if (argument == "--exposure-us") {
      options.exposure_us = std::stod(require_value(index));
    } else if (argument == "--gain-db") {
      options.gain_db = std::stod(require_value(index));
    } else if (argument == "--width") {
      options.width = static_cast<unsigned int>(std::stoul(require_value(index)));
    } else if (argument == "--height") {
      options.height = static_cast<unsigned int>(std::stoul(require_value(index)));
    } else if (argument == "--offset-x") {
      options.offset_x = static_cast<unsigned int>(std::stoul(require_value(index)));
    } else if (argument == "--offset-y") {
      options.offset_y = static_cast<unsigned int>(std::stoul(require_value(index)));
    } else if (argument == "--timeout-ms") {
      options.timeout_ms = static_cast<unsigned int>(std::stoul(require_value(index)));
    } else if (argument == "--pause-repeats") {
      options.pause_repeats = static_cast<unsigned int>(std::stoul(require_value(index)));
    } else if (argument == "--allow-short") {
      options.allow_short = true;
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }

  if (options.output_dir.empty()) {
    throw std::invalid_argument("--output is required");
  }
  if (!options.allow_short &&
    (options.continuous_seconds < 60.0 || options.load_seconds < 60.0))
  {
    throw std::invalid_argument(
      "continuous and load phases must each be at least 60 seconds; use --allow-short only for validation");
  }
  if (options.pause_repeats < 3 && !options.allow_short) {
    throw std::invalid_argument("pause repetitions must be at least 3");
  }
  for (const auto node_count : options.nodes) {
    if (node_count < 1) {
      throw std::invalid_argument("node counts must be positive");
    }
  }
  return options;
}

struct CameraSettings
{
  uint64_t width{};
  uint64_t height{};
  uint64_t offset_x{};
  uint64_t offset_y{};
  uint64_t pixel_format{};
  uint64_t trigger_mode{};
  bool frame_rate_enable{};
  float frame_rate{};
  uint64_t exposure_auto{};
  float exposure_us{};
  uint64_t gain_auto{};
  float gain_db{};
};

uint64_t get_int(void * handle, const char * name)
{
  MVCC_INTVALUE_EX value{};
  require_ok(MV_CC_GetIntValueEx(handle, name, &value), std::string("get ") + name);
  return static_cast<uint64_t>(value.nCurValue);
}

uint64_t get_enum(void * handle, const char * name)
{
  MVCC_ENUMVALUE value{};
  require_ok(MV_CC_GetEnumValue(handle, name, &value), std::string("get ") + name);
  return value.nCurValue;
}

float get_float(void * handle, const char * name)
{
  MVCC_FLOATVALUE value{};
  require_ok(MV_CC_GetFloatValue(handle, name, &value), std::string("get ") + name);
  return value.fCurValue;
}

bool get_bool(void * handle, const char * name)
{
  bool value = false;
  require_ok(MV_CC_GetBoolValue(handle, name, &value), std::string("get ") + name);
  return value;
}

CameraSettings read_settings(void * handle)
{
  CameraSettings settings;
  settings.width = get_int(handle, "Width");
  settings.height = get_int(handle, "Height");
  settings.offset_x = get_int(handle, "OffsetX");
  settings.offset_y = get_int(handle, "OffsetY");
  settings.pixel_format = get_enum(handle, "PixelFormat");
  settings.trigger_mode = get_enum(handle, "TriggerMode");
  settings.frame_rate_enable = get_bool(handle, "AcquisitionFrameRateEnable");
  settings.frame_rate = get_float(handle, "AcquisitionFrameRate");
  settings.exposure_auto = get_enum(handle, "ExposureAuto");
  settings.exposure_us = get_float(handle, "ExposureTime");
  settings.gain_auto = get_enum(handle, "GainAuto");
  settings.gain_db = get_float(handle, "Gain");
  return settings;
}

void set_roi(void * handle, uint64_t width, uint64_t height, uint64_t offset_x, uint64_t offset_y)
{
  require_ok(MV_CC_SetIntValueEx(handle, "OffsetX", 0), "set OffsetX=0");
  require_ok(MV_CC_SetIntValueEx(handle, "OffsetY", 0), "set OffsetY=0");
  require_ok(MV_CC_SetIntValueEx(handle, "Width", static_cast<int64_t>(width)), "set Width");
  require_ok(MV_CC_SetIntValueEx(handle, "Height", static_cast<int64_t>(height)), "set Height");
  require_ok(MV_CC_SetIntValueEx(handle, "OffsetX", static_cast<int64_t>(offset_x)), "set OffsetX");
  require_ok(MV_CC_SetIntValueEx(handle, "OffsetY", static_cast<int64_t>(offset_y)), "set OffsetY");
}

void apply_fixed_settings(void * handle, const Options & options)
{
  require_ok(MV_CC_SetEnumValue(handle, "TriggerMode", 0), "set TriggerMode=Off");
  set_roi(handle, options.width, options.height, options.offset_x, options.offset_y);
  require_ok(
    MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_BayerRG8),
    "set PixelFormat=BayerRG8");
  require_ok(
    MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true),
    "enable AcquisitionFrameRate");
  require_ok(
    MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", static_cast<float>(options.frame_rate)),
    "set AcquisitionFrameRate");
  require_ok(MV_CC_SetEnumValue(handle, "ExposureAuto", 0), "set ExposureAuto=Off");
  require_ok(
    MV_CC_SetFloatValue(handle, "ExposureTime", static_cast<float>(options.exposure_us)),
    "set ExposureTime");
  require_ok(MV_CC_SetEnumValue(handle, "GainAuto", 0), "set GainAuto=Off");
  require_ok(
    MV_CC_SetFloatValue(handle, "Gain", static_cast<float>(options.gain_db)), "set Gain");
}

void restore_settings(void * handle, const CameraSettings & settings)
{
  std::vector<std::string> failures;
  const auto attempt = [&](int code, const char * operation) {
      if (code != MV_OK) failures.push_back(std::string(operation) + ": " + hex_error(code));
    };
  // Restoring the ROI with zero offsets first avoids invalid intermediate geometry.
  try {
    set_roi(handle, settings.width, settings.height, settings.offset_x, settings.offset_y);
  } catch (const std::exception & error) {
    failures.push_back(error.what());
  }
  attempt(
    MV_CC_SetEnumValue(handle, "PixelFormat", static_cast<unsigned int>(settings.pixel_format)),
    "restore PixelFormat");
  attempt(MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true), "enable frame rate restore");
  attempt(MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", settings.frame_rate),
    "restore AcquisitionFrameRate");
  attempt(MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", settings.frame_rate_enable),
    "restore AcquisitionFrameRateEnable");
  attempt(MV_CC_SetEnumValue(handle, "ExposureAuto", 0), "disable ExposureAuto for restore");
  attempt(MV_CC_SetFloatValue(handle, "ExposureTime", settings.exposure_us), "restore ExposureTime");
  attempt(
    MV_CC_SetEnumValue(handle, "ExposureAuto", static_cast<unsigned int>(settings.exposure_auto)),
    "restore ExposureAuto");
  attempt(MV_CC_SetEnumValue(handle, "GainAuto", 0), "disable GainAuto for restore");
  attempt(MV_CC_SetFloatValue(handle, "Gain", settings.gain_db), "restore Gain");
  attempt(
    MV_CC_SetEnumValue(handle, "GainAuto", static_cast<unsigned int>(settings.gain_auto)),
    "restore GainAuto");
  attempt(
    MV_CC_SetEnumValue(handle, "TriggerMode", static_cast<unsigned int>(settings.trigger_mode)),
    "restore TriggerMode");
  if (!failures.empty()) {
    std::string message;
    for (const auto & failure : failures) message += failure + "; ";
    throw std::runtime_error(message);
  }
}

struct CameraSession
{
  void * handle{nullptr};
  bool sdk_initialized{false};
  bool open{false};
  bool grabbing{false};
  bool have_saved_settings{false};
  fs::path restore_status_path;
  CameraSettings saved_settings{};

  ~CameraSession()
  {
    try {
      if (handle != nullptr && grabbing) {
        MV_CC_StopGrabbing(handle);
        grabbing = false;
      }
      if (handle != nullptr && open && have_saved_settings) {
        restore_settings(handle, saved_settings);
        const auto restored = read_settings(handle);
        const auto & saved = saved_settings;
        if (restored.width != saved.width || restored.height != saved.height ||
          restored.offset_x != saved.offset_x || restored.offset_y != saved.offset_y ||
          restored.pixel_format != saved.pixel_format || restored.trigger_mode != saved.trigger_mode ||
          restored.frame_rate_enable != saved.frame_rate_enable ||
          restored.exposure_auto != saved.exposure_auto || restored.gain_auto != saved.gain_auto ||
          std::abs(restored.frame_rate - saved.frame_rate) > .01F ||
          (saved.exposure_auto == 0 && std::abs(restored.exposure_us - saved.exposure_us) > .1F) ||
          (saved.gain_auto == 0 && std::abs(restored.gain_db - saved.gain_db) > .01F))
        {
          throw std::runtime_error("restored settings readback differs from original");
        }
        std::ofstream(restore_status_path) << "restoration_verified\n";
        std::cerr << "Camera settings restored.\n";
      }
    } catch (const std::exception & error) {
      std::ofstream(restore_status_path) << "restore_failed: " << error.what() << '\n';
      std::cerr << "WARNING: restoring camera settings failed: " << error.what() << '\n';
    }
    if (handle != nullptr && open) {
      MV_CC_CloseDevice(handle);
    }
    if (handle != nullptr) {
      MV_CC_DestroyHandle(handle);
    }
    if (sdk_initialized) {
      MV_CC_Finalize();
    }
  }
};

struct EventRow
{
  unsigned int nodes{};
  std::string phase;
  unsigned int pause_ms{};
  unsigned int repeat{};
  std::string event;
  int64_t steady_time_ns{};
  int64_t realtime_time_ns{};
  int code{};
  std::string detail;
};

struct FrameRow
{
  unsigned int nodes{};
  std::string phase;
  unsigned int pause_ms{};
  unsigned int repeat{};
  uint64_t sequence{};
  int64_t phase_elapsed_ns{};
  int64_t get_started_ns{};
  int64_t app_get_ns{};
  int64_t app_realtime_ns{};
  uint32_t frame_number{};
  uint64_t device_timestamp_raw{};
  int64_t sdk_host_timestamp_raw{};
  uint64_t frame_bytes{};
  unsigned int width{};
  unsigned int height{};
  uint32_t pixel_type{};
  unsigned int sdk_queue_after_get{};
  int queue_query_code{};
  uint64_t frame_gap_before{};
  bool duplicate_or_reordered{};
  int64_t process_ns{};
  int process_code{};
  int free_code{};
  uint64_t rss_bytes{};
  uint64_t checksum{};
};

struct CalibrationRow
{
  unsigned int nodes{};
  std::string position;
  unsigned int sample{};
  int64_t host_before_ns{};
  int64_t host_after_ns{};
  uint64_t device_timestamp_ns{};
  uint64_t timestamp_increment_ns{};
};

struct RuntimeRow
{
  unsigned int nodes{};
  uint64_t rss_before_start{};
  uint64_t rss_after_start{};
  uint64_t rss_after_stop{};
  uint64_t usb_received_before{};
  uint64_t usb_received_after{};
  uint64_t usb_errors_before{};
  uint64_t usb_errors_after{};
};

struct UsbStats
{
  uint64_t received_frames{};
  uint64_t error_frames{};
  int code{};
};

UsbStats get_usb_stats(void * handle)
{
  MV_MATCH_INFO_USB_DETECT usb{};
  MV_ALL_MATCH_INFO request{};
  request.nType = MV_MATCH_TYPE_USB_DETECT;
  request.pInfo = &usb;
  request.nInfoSize = sizeof(usb);
  const int result = MV_CC_GetAllMatchInfo(handle, &request);
  return {usb.nReceivedFrameCount, usb.nErrorFrameCount, result};
}

uint64_t get_timestamp_increment(void * handle)
{
  try {
    return get_int(handle, "DeviceTimestampIncrement");
  } catch (const std::exception &) {
    // This camera family documents frame timestamps in nanoseconds if the feature is absent.
    return 1;
  }
}

void collect_calibration(
  void * handle, unsigned int nodes, const std::string & position,
  std::vector<CalibrationRow> & output)
{
  const uint64_t increment_ns = get_timestamp_increment(handle);
  for (unsigned int sample = 0; sample < 20 && !g_stop.load(); ++sample) {
    const int64_t before = steady_ns();
    const int command_result = MV_CC_SetCommandValue(handle, "DeviceTimestampLatch");
    MVCC_INTVALUE_EX timestamp{};
    const int read_result = command_result == MV_OK ?
      MV_CC_GetIntValueEx(handle, "DeviceTimestamp", &timestamp) : command_result;
    const int64_t after = steady_ns();
    if (command_result != MV_OK || read_result != MV_OK) {
      throw std::runtime_error(
        "device timestamp calibration failed: command=" + hex_error(command_result) +
        " read=" + hex_error(read_result));
    }
    output.push_back({
      nodes, position, sample, before, after,
      static_cast<uint64_t>(timestamp.nCurValue), increment_ns});
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void write_events(const fs::path & path, const std::vector<EventRow> & rows)
{
  std::ofstream stream(path);
  stream << "node_count,phase,pause_ms,repeat,event,steady_ns,realtime_ns,code,detail\n";
  for (const auto & row : rows) {
    stream << row.nodes << ',' << csv_escape(row.phase) << ',' << row.pause_ms << ','
           << row.repeat << ',' << csv_escape(row.event) << ',' << row.steady_time_ns << ','
           << row.realtime_time_ns << ',' << row.code << ',' << csv_escape(row.detail) << '\n';
  }
}

void write_calibrations(const fs::path & path, const std::vector<CalibrationRow> & rows)
{
  std::ofstream stream(path);
  stream << "node_count,position,sample,host_before_ns,host_after_ns,device_timestamp_ns,"
            "timestamp_increment_ns\n";
  for (const auto & row : rows) {
    stream << row.nodes << ',' << row.position << ',' << row.sample << ',' << row.host_before_ns
           << ',' << row.host_after_ns << ',' << row.device_timestamp_ns << ','
           << row.timestamp_increment_ns << '\n';
  }
}

void write_runtime(const fs::path & path, const std::vector<RuntimeRow> & rows)
{
  std::ofstream stream(path);
  stream << "node_count,rss_before_start_bytes,rss_after_start_bytes,rss_after_stop_bytes,"
            "usb_received_before,usb_received_after,usb_errors_before,usb_errors_after\n";
  for (const auto & row : rows) {
    stream << row.nodes << ',' << row.rss_before_start << ',' << row.rss_after_start << ','
           << row.rss_after_stop << ',' << row.usb_received_before << ',' << row.usb_received_after
           << ',' << row.usb_errors_before << ',' << row.usb_errors_after << '\n';
  }
}

void write_frames(const fs::path & path, const std::vector<FrameRow> & rows)
{
  std::ofstream stream(path);
  stream << "node_count,phase,pause_ms,repeat,sequence,phase_elapsed_ns,get_started_ns,app_get_ns,"
            "app_realtime_ns,frame_number,device_timestamp_raw,sdk_host_timestamp_raw,frame_bytes,"
            "width,height,pixel_type,sdk_queue_after_get,queue_query_code,frame_gap_before,"
            "duplicate_or_reordered,process_ns,"
            "process_code,free_code,rss_bytes,checksum\n";
  for (const auto & row : rows) {
    stream << row.nodes << ',' << csv_escape(row.phase) << ',' << row.pause_ms << ',' << row.repeat
           << ',' << row.sequence << ',' << row.phase_elapsed_ns << ',' << row.get_started_ns << ','
           << row.app_get_ns << ',' << row.app_realtime_ns << ',' << row.frame_number << ','
           << row.device_timestamp_raw << ',' << row.sdk_host_timestamp_raw << ',' << row.frame_bytes
           << ',' << row.width << ',' << row.height << ',' << row.pixel_type << ','
           << row.sdk_queue_after_get << ',' << row.queue_query_code << ',' << row.frame_gap_before
           << ',' << static_cast<unsigned int>(row.duplicate_or_reordered) << ',' << row.process_ns
           << ',' << row.process_code << ',' << row.free_code << ',' << row.rss_bytes << ','
           << row.checksum << '\n';
  }
}

template<typename ValueT>
void write_key_value(std::ofstream & stream, const std::string & key, const ValueT & value)
{
  stream << csv_escape(key) << ',' << csv_escape([&] {
      std::ostringstream text;
      text << std::setprecision(12) << value;
      return text.str();
    }()) << '\n';
}

void write_settings(
  std::ofstream & stream, const std::string & prefix, const CameraSettings & settings)
{
  write_key_value(stream, prefix + "width", settings.width);
  write_key_value(stream, prefix + "height", settings.height);
  write_key_value(stream, prefix + "offset_x", settings.offset_x);
  write_key_value(stream, prefix + "offset_y", settings.offset_y);
  write_key_value(stream, prefix + "pixel_format", settings.pixel_format);
  write_key_value(stream, prefix + "trigger_mode", settings.trigger_mode);
  write_key_value(stream, prefix + "frame_rate_enable", settings.frame_rate_enable ? 1 : 0);
  write_key_value(stream, prefix + "frame_rate", settings.frame_rate);
  write_key_value(stream, prefix + "exposure_auto", settings.exposure_auto);
  write_key_value(stream, prefix + "exposure_us", settings.exposure_us);
  write_key_value(stream, prefix + "gain_auto", settings.gain_auto);
  write_key_value(stream, prefix + "gain_db", settings.gain_db);
}

class Grabber
{
public:
  Grabber(void * handle, const Options & options, unsigned int nodes,
    std::vector<FrameRow> & frames, std::vector<EventRow> & events)
  : handle_(handle), options_(options), nodes_(nodes), frames_(frames), events_(events)
  {
    const size_t expected = static_cast<size_t>(
      (options.continuous_seconds + options.load_seconds +
      options.pause_ms.size() * options.pause_repeats * options.recovery_seconds + 10.0) *
      options.frame_rate * 1.5);
    frames_.reserve(expected);
  }

  void phase(
    const std::string & name, double duration_seconds, bool simulated_load,
    unsigned int pause_ms = 0, unsigned int repeat = 0)
  {
    phase_name_ = name;
    pause_ms_ = pause_ms;
    repeat_ = repeat;
    phase_started_ns_ = steady_ns();
    events_.push_back({
      nodes_, phase_name_, pause_ms_, repeat_, "phase_start", phase_started_ns_, realtime_ns(),
      MV_OK, "timed phase started"});
    const int64_t deadline = phase_started_ns_ + static_cast<int64_t>(duration_seconds * 1e9);
    while (!g_stop.load() && steady_ns() < deadline) {
      grab_one(simulated_load);
    }
    events_.push_back({
      nodes_, phase_name_, pause_ms_, repeat_, "phase_end", steady_ns(), realtime_ns(), MV_OK,
      "timed phase ended"});
  }

  void pause_and_recover(unsigned int pause_ms, unsigned int repeat)
  {
    phase_name_ = "pause_" + std::to_string(pause_ms) + "ms_r" + std::to_string(repeat);
    pause_ms_ = pause_ms;
    repeat_ = repeat;
    const int64_t before = steady_ns();
    events_.push_back({
      nodes_, phase_name_, pause_ms, repeat, "pause_start", before, realtime_ns(), MV_OK,
      "No GetImageBuffer and no ClearImageBuffer during the pause"});
    std::this_thread::sleep_for(std::chrono::milliseconds(pause_ms));
    const int64_t after = steady_ns();
    events_.push_back({
      nodes_, phase_name_, pause_ms, repeat, "pause_end", after, realtime_ns(), MV_OK,
      "actual_pause_ns=" + std::to_string(after - before)});
    phase_started_ns_ = after;
    const int64_t deadline = after + static_cast<int64_t>(options_.recovery_seconds * 1e9);
    while (!g_stop.load() && steady_ns() < deadline) {
      grab_one(false);
    }
  }

private:
  void grab_one(bool simulated_load)
  {
    MV_FRAME_OUT frame{};
    const int64_t get_started = steady_ns();
    const int result = MV_CC_GetImageBuffer(handle_, &frame, options_.timeout_ms);
    const int64_t received = steady_ns();
    const int64_t wall = realtime_ns();
    if (result != MV_OK) {
      events_.push_back({
        nodes_, phase_name_, pause_ms_, repeat_,
        static_cast<uint32_t>(result) == MV_E_NODATA ? "get_timeout" : "get_error",
        received, wall, result, hex_error(result)});
      return;
    }

    // Release the borrowed image even if allocation or processing throws.
    struct BorrowedImageGuard {
      void * handle;
      MV_FRAME_OUT * frame;
      bool active{true};
      ~BorrowedImageGuard() { if (active) MV_CC_FreeImageBuffer(handle, frame); }
    } image_guard{handle_, &frame};

    unsigned int queue_depth = 0;
    const int queue_result = MV_CC_GetValidImageNum(handle_, &queue_depth);

    uint64_t gap = 0;
    bool duplicate = false;
    const uint32_t current_frame = frame.stFrameInfo.nFrameNum;
    if (have_previous_frame_) {
      const uint32_t delta = current_frame - previous_frame_;
      if (delta > 1U && delta < 0x80000000U) {
        gap = static_cast<uint64_t>(delta - 1U);
      } else if (delta == 0U || delta >= 0x80000000U) {
        duplicate = true;
      }
    }
    previous_frame_ = current_frame;
    have_previous_frame_ = true;

    const int64_t process_started = steady_ns();
    int process_result = MV_OK;
    uint64_t checksum = 0;
    if (simulated_load) {
      const size_t bgr_size = static_cast<size_t>(frame.stFrameInfo.nExtendWidth) *
        frame.stFrameInfo.nExtendHeight * 3U;
      bgr_buffer_.resize(bgr_size);
      message_copy_.resize(bgr_size);
      MV_CC_PIXEL_CONVERT_PARAM_EX conversion{};
      conversion.nWidth = frame.stFrameInfo.nExtendWidth;
      conversion.nHeight = frame.stFrameInfo.nExtendHeight;
      conversion.enSrcPixelType = frame.stFrameInfo.enPixelType;
      conversion.pSrcData = frame.pBufAddr;
      conversion.nSrcDataLen = frame.stFrameInfo.nFrameLenEx;
      conversion.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
      conversion.pDstBuffer = bgr_buffer_.data();
      conversion.nDstBufferSize = bgr_buffer_.size();
      process_result = MV_CC_ConvertPixelTypeEx(handle_, &conversion);
      if (process_result == MV_OK) {
        std::copy_n(bgr_buffer_.begin(), conversion.nDstLen, message_copy_.begin());
        // Touch the copied data so the compiler and allocator cannot elide the simulated publish copy.
        for (size_t offset = 0; offset < conversion.nDstLen; offset += 4096) {
          checksum = (checksum * 1315423911ULL) ^ message_copy_[offset];
        }
      }
    }
    const int64_t process_finished = steady_ns();

    FrameRow row{
      nodes_, phase_name_, pause_ms_, repeat_, sequence_++, received - phase_started_ns_,
      get_started, received, wall, current_frame,
      (static_cast<uint64_t>(frame.stFrameInfo.nDevTimeStampHigh) << 32U) |
      frame.stFrameInfo.nDevTimeStampLow,
      frame.stFrameInfo.nHostTimeStamp, frame.stFrameInfo.nFrameLenEx,
      frame.stFrameInfo.nExtendWidth, frame.stFrameInfo.nExtendHeight,
      static_cast<uint32_t>(frame.stFrameInfo.enPixelType), queue_depth, queue_result,
      gap, duplicate, process_finished - process_started, process_result, MV_OK, 0, checksum};
    const int free_result = MV_CC_FreeImageBuffer(handle_, &frame);
    image_guard.active = false;
    row.free_code = free_result;
    row.rss_bytes = sample_rss(received);
    frames_.push_back(std::move(row));

    if (process_result != MV_OK) {
      events_.push_back({nodes_, phase_name_, pause_ms_, repeat_, "processing_error",
        process_finished, realtime_ns(), process_result, hex_error(process_result)});
    }
    if (queue_result != MV_OK) {
      events_.push_back({nodes_, phase_name_, pause_ms_, repeat_, "queue_query_error",
        received, wall, queue_result, hex_error(queue_result)});
    }
    if (free_result != MV_OK) {
      events_.push_back({nodes_, phase_name_, pause_ms_, repeat_, "free_error",
        steady_ns(), realtime_ns(), free_result, hex_error(free_result)});
    }
  }

  uint64_t sample_rss(int64_t now_ns)
  {
    if (last_rss_sample_ns_ == 0 || now_ns - last_rss_sample_ns_ >= 100000000LL) {
      last_rss_ = resident_bytes();
      last_rss_sample_ns_ = now_ns;
    }
    return last_rss_;
  }

  void * handle_;
  const Options & options_;
  unsigned int nodes_;
  std::vector<FrameRow> & frames_;
  std::vector<EventRow> & events_;
  std::string phase_name_;
  unsigned int pause_ms_{0};
  unsigned int repeat_{0};
  int64_t phase_started_ns_{0};
  uint64_t sequence_{0};
  uint32_t previous_frame_{0};
  bool have_previous_frame_{false};
  int64_t last_rss_sample_ns_{0};
  uint64_t last_rss_{0};
  std::vector<unsigned char> bgr_buffer_;
  std::vector<unsigned char> message_copy_;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  try {
    const Options options = parse_options(argc, argv);
    fs::create_directories(options.output_dir);

    CameraSession camera;
    camera.restore_status_path = options.output_dir / "restoration_status.txt";
    require_ok(MV_CC_Initialize(), "MV_CC_Initialize");
    camera.sdk_initialized = true;

    MV_CC_DEVICE_INFO_LIST list{};
    require_ok(MV_CC_EnumDevices(MV_USB_DEVICE, &list), "MV_CC_EnumDevices");
    MV_CC_DEVICE_INFO * selected = nullptr;
    std::string model;
    std::string serial;
    for (unsigned int index = 0; index < list.nDeviceNum; ++index) {
      auto * info = list.pDeviceInfo[index];
      if (info == nullptr || info->nTLayerType != MV_USB_DEVICE) {
        continue;
      }
      const std::string candidate_serial = bounded_string(
        info->SpecialInfo.stUsb3VInfo.chSerialNumber,
        sizeof(info->SpecialInfo.stUsb3VInfo.chSerialNumber));
      if (options.serial.empty() || candidate_serial == options.serial) {
        selected = info;
        serial = candidate_serial;
        model = bounded_string(
          info->SpecialInfo.stUsb3VInfo.chModelName,
          sizeof(info->SpecialInfo.stUsb3VInfo.chModelName));
        break;
      }
    }
    if (selected == nullptr) {
      throw std::runtime_error("requested USB3 Vision camera not found: " + options.serial);
    }
    if (model != "MV-CS016-10UC") {
      throw std::runtime_error("selected camera model is " + model + ", expected MV-CS016-10UC");
    }

    require_ok(MV_CC_CreateHandle(&camera.handle, selected), "MV_CC_CreateHandle");
    require_ok(MV_CC_OpenDevice(camera.handle), "MV_CC_OpenDevice");
    camera.open = true;
    camera.saved_settings = read_settings(camera.handle);
    camera.have_saved_settings = true;
    apply_fixed_settings(camera.handle, options);
    require_ok(MV_CC_SetBayerCvtQuality(camera.handle, 1), "set Bayer conversion quality=1");
    const CameraSettings actual_settings = read_settings(camera.handle);
    const uint64_t payload_size = get_int(camera.handle, "PayloadSize");

    std::ofstream config(options.output_dir / "configuration.csv");
    config << "key,value\n";
    write_key_value(config, "model", model);
    write_key_value(config, "serial", serial);
    write_key_value(config, "sdk_version", MV_CC_GetSDKVersion());
    write_key_value(config, "grab_strategy", "OneByOne");
    write_key_value(config, "nodes", [&] {
        std::ostringstream value;
        for (size_t index = 0; index < options.nodes.size(); ++index) {
          if (index) value << ';';
          value << options.nodes[index];
        }
        return value.str();
      }());
    write_key_value(config, "continuous_seconds", options.continuous_seconds);
    write_key_value(config, "load_seconds", options.load_seconds);
    write_key_value(config, "warmup_seconds", options.warmup_seconds);
    write_key_value(config, "recovery_seconds", options.recovery_seconds);
    write_key_value(config, "pause_repeats", options.pause_repeats);
    write_key_value(config, "pause_ms", [&] {
        std::ostringstream value;
        for (size_t index = 0; index < options.pause_ms.size(); ++index) {
          if (index) value << ';';
          value << options.pause_ms[index];
        }
        return value.str();
      }());
    write_key_value(config, "timeout_ms", options.timeout_ms);
    write_key_value(config, "payload_size_bytes", payload_size);
    write_key_value(config, "load_kind", "simulated_existing_camera_node_bayer_to_bgr_and_copy");
    write_key_value(config, "timed_loop_disk_io", "none");
    write_key_value(config, "sdk_host_timestamp_precision_note", "milliseconds_do_not_interpret_sub_ms");
    write_settings(config, "original_", camera.saved_settings);
    write_settings(config, "actual_", actual_settings);
    config.close();

    std::vector<EventRow> events;
    std::vector<CalibrationRow> calibrations;
    std::vector<RuntimeRow> runtimes;

    for (const auto node_count : options.nodes) {
      if (g_stop.load()) {
        break;
      }
      std::cout << "Starting node_count=" << node_count << std::endl;
      collect_calibration(camera.handle, node_count, "before", calibrations);
      require_ok(MV_CC_SetImageNodeNum(camera.handle, node_count), "MV_CC_SetImageNodeNum");
      require_ok(
        MV_CC_SetGrabStrategy(camera.handle, MV_GrabStrategy_OneByOne),
        "MV_CC_SetGrabStrategy(OneByOne)");

      RuntimeRow runtime{};
      runtime.nodes = node_count;
      runtime.rss_before_start = resident_bytes();
      const UsbStats usb_before = get_usb_stats(camera.handle);
      runtime.usb_received_before = usb_before.received_frames;
      runtime.usb_errors_before = usb_before.error_frames;
      require_ok(MV_CC_StartGrabbing(camera.handle), "MV_CC_StartGrabbing");
      camera.grabbing = true;
      runtime.rss_after_start = resident_bytes();

      std::vector<FrameRow> frames;
      Grabber grabber(camera.handle, options, node_count, frames, events);
      grabber.phase("warmup_excluded", options.warmup_seconds, false);
      grabber.phase("continuous", options.continuous_seconds, false);
      grabber.phase("load_warmup_excluded", options.warmup_seconds, true);
      grabber.phase("simulated_load", options.load_seconds, true);
      grabber.phase("pause_baseline_excluded", options.warmup_seconds, false);
      for (const auto pause_ms : options.pause_ms) {
        for (unsigned int repeat = 1; repeat <= options.pause_repeats && !g_stop.load(); ++repeat) {
          grabber.pause_and_recover(pause_ms, repeat);
        }
      }

      require_ok(MV_CC_StopGrabbing(camera.handle), "MV_CC_StopGrabbing");
      camera.grabbing = false;
      runtime.rss_after_stop = resident_bytes();
      const UsbStats usb_after = get_usb_stats(camera.handle);
      runtime.usb_received_after = usb_after.received_frames;
      runtime.usb_errors_after = usb_after.error_frames;
      runtimes.push_back(runtime);
      collect_calibration(camera.handle, node_count, "after", calibrations);

      std::ostringstream filename;
      filename << "raw_frames_nodes_" << std::setw(2) << std::setfill('0') << node_count << ".csv";
      write_frames(options.output_dir / filename.str(), frames);
      std::cout << "Completed node_count=" << node_count << " frames=" << frames.size() << std::endl;
    }

    write_events(options.output_dir / "events.csv", events);
    write_calibrations(options.output_dir / "clock_calibration.csv", calibrations);
    write_runtime(options.output_dir / "runtime_memory_usb.csv", runtimes);
    if (g_stop.load()) {
      throw std::runtime_error("benchmark interrupted; partial raw data retained");
    }
    std::cout << "RAW_COMPLETE " << options.output_dir << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }
}
