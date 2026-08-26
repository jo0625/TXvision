#include <MvCameraControl.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "mvs_raw_capture/capture_result.hpp"
#include "mvs_raw_capture/cli.hpp"
#include "mvs_raw_capture/session.hpp"

namespace fs = std::filesystem;
using mvs_raw_capture::CaptureResult;
using mvs_raw_capture::CompletionGate;
using mvs_raw_capture::CounterEvent;
using mvs_raw_capture::CounterObservation;
using mvs_raw_capture::CounterSource;
using mvs_raw_capture::DiskBudget;
using mvs_raw_capture::FrameRecord;
using mvs_raw_capture::SensorCounterTracker;
using mvs_raw_capture::SessionManifest;
using mvs_raw_capture::SessionPaths;

namespace
{
constexpr const char * kExpectedSerial = "DA8541046";
constexpr const char * kExpectedModel = "MV-CS016-10UC";
constexpr float kFrameRate = 250.0F;
constexpr float kExposureTime = 3000.0F;
constexpr float kGain = 12.4932F;
constexpr unsigned int kAcquisitionMode = 2U;
constexpr unsigned int kTransferSize = 1048576U;
constexpr unsigned int kTransferWays = 8U;
constexpr unsigned int kUsbfsMemoryMb = 16U;
constexpr const char * kCounterEventsHeader =
  "acquired_index,event,previous_sensor_frame_id,current_sensor_frame_id,missing_count,"
  "host_receive_timestamp_ns\n";

volatile sig_atomic_t gStopRequested = 0;

void signal_handler(int) { gStopRequested = 1; }

std::string bounded_string(const unsigned char * data, std::size_t capacity)
{
  const auto * chars = reinterpret_cast<const char *>(data);
  return std::string(chars, strnlen(chars, capacity));
}

std::string sdk_error_hex(int code)
{
  std::ostringstream output;
  output << "0x" << std::hex << std::setw(8) << std::setfill('0')
         << static_cast<std::uint32_t>(code);
  return output.str();
}

void append_error(std::string & destination, const std::string & error)
{
  if (!destination.empty()) destination += "; ";
  destination += error;
}

void require_sdk(int result, const std::string & operation)
{
  if (result != MV_OK) throw std::runtime_error(operation + " failed: " + sdk_error_hex(result));
}

std::uint64_t monotonic_ns() noexcept
{
  struct timespec value
  {
  };
  if (clock_gettime(CLOCK_MONOTONIC_RAW, &value) != 0) return 0U;
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL + value.tv_nsec;
}

unsigned int read_usbfs_memory_mb()
{
  std::ifstream input("/sys/module/usbcore/parameters/usbfs_memory_mb");
  unsigned int value = 0;
  if (!(input >> value)) throw std::runtime_error("cannot read usbfs_memory_mb");
  return value;
}

bool write_all_noexcept(int fd, const void * data, std::size_t size, std::string & error) noexcept
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
    append_error(error, "write failed: " + std::string(std::strerror(errno)));
    return false;
  }
  return true;
}

bool fdatasync_noexcept(int fd, std::string & error) noexcept
{
  int result = 0;
  do {
    result = fdatasync(fd);
  } while (result != 0 && errno == EINTR);
  if (result != 0) {
    append_error(error, "fdatasync failed: " + std::string(std::strerror(errno)));
    return false;
  }
  return true;
}

class UniqueFd
{
public:
  explicit UniqueFd(int fd = -1) : fd_(fd) {}
  UniqueFd(const UniqueFd &) = delete;
  UniqueFd & operator=(const UniqueFd &) = delete;
  ~UniqueFd()
  {
    if (fd_ >= 0) close(fd_);
  }
  int get() const noexcept { return fd_; }
  bool close_checked(std::string & error) noexcept
  {
    if (fd_ < 0) return true;
    const int fd = fd_;
    fd_ = -1;
    if (close(fd) != 0) {
      append_error(error, "close failed: " + std::string(std::strerror(errno)));
      return false;
    }
    return true;
  }

private:
  int fd_ = -1;
};

class SdkResources
{
public:
  SdkResources() = default;
  SdkResources(const SdkResources &) = delete;
  SdkResources & operator=(const SdkResources &) = delete;
  ~SdkResources() { cleanup(); }

  bool stop_grabbing(std::string & error) noexcept
  {
    if (!grabbing_) return true;
    const int result = MV_CC_StopGrabbing(handle_);
    grabbing_ = false;
    if (result != MV_OK) {
      append_error(error, "MV_CC_StopGrabbing failed: " + sdk_error_hex(result));
      return false;
    }
    return true;
  }

  bool cleanup() noexcept
  {
    if (cleaned_) return cleanup_ok_;
    cleaned_ = true;
    std::string ignored;
    if (!stop_grabbing(ignored)) cleanup_ok_ = false;
    if (device_open_) {
      if (MV_CC_CloseDevice(handle_) != MV_OK) cleanup_ok_ = false;
      device_open_ = false;
    }
    if (handle_ != nullptr) {
      if (MV_CC_DestroyHandle(handle_) != MV_OK) cleanup_ok_ = false;
      handle_ = nullptr;
    }
    if (sdk_initialized_) {
      if (MV_CC_Finalize() != MV_OK) cleanup_ok_ = false;
      sdk_initialized_ = false;
    }
    return cleanup_ok_;
  }

  bool sdk_initialized_ = false;
  void * handle_ = nullptr;
  bool device_open_ = false;
  bool grabbing_ = false;

private:
  bool cleaned_ = false;
  bool cleanup_ok_ = true;
};

template <typename T>
struct Reading
{
  bool ok = false;
  int error = 0;
  T value{};
};

using IntReading = Reading<std::int64_t>;
using EnumReading = Reading<unsigned int>;
using FloatReading = Reading<float>;
using BoolReading = Reading<bool>;
using UIntReading = Reading<unsigned int>;

struct ParameterSnapshot
{
  bool identity_ok = false;
  int identity_error = 0;
  std::string serial;
  std::string model;
  IntReading width;
  IntReading height;
  EnumReading pixel_format;
  BoolReading frame_rate_enable;
  FloatReading frame_rate;
  EnumReading exposure_auto;
  FloatReading exposure_time;
  EnumReading gain_auto;
  FloatReading gain;
  EnumReading trigger_mode;
  EnumReading acquisition_mode;
  FloatReading resulting_frame_rate;
  IntReading payload_size;
  UIntReading transfer_size;
  UIntReading transfer_ways;
  UIntReading usbfs_memory_mb;
};

IntReading get_int(void * handle, const char * name)
{
  MVCC_INTVALUE_EX value{};
  const int result = MV_CC_GetIntValueEx(handle, name, &value);
  return {result == MV_OK, result, result == MV_OK ? value.nCurValue : 0};
}

EnumReading get_enum(void * handle, const char * name)
{
  MVCC_ENUMVALUE value{};
  const int result = MV_CC_GetEnumValue(handle, name, &value);
  return {result == MV_OK, result, result == MV_OK ? value.nCurValue : 0U};
}

FloatReading get_float(void * handle, const char * name)
{
  MVCC_FLOATVALUE value{};
  const int result = MV_CC_GetFloatValue(handle, name, &value);
  return {result == MV_OK, result, result == MV_OK ? value.fCurValue : 0.0F};
}

BoolReading get_bool(void * handle, const char * name)
{
  bool value = false;
  const int result = MV_CC_GetBoolValue(handle, name, &value);
  return {result == MV_OK, result, result == MV_OK && value};
}

ParameterSnapshot read_snapshot(void * handle)
{
  ParameterSnapshot snapshot;
  MV_CC_DEVICE_INFO device{};
  snapshot.identity_error = MV_CC_GetDeviceInfo(handle, &device);
  snapshot.identity_ok = snapshot.identity_error == MV_OK && device.nTLayerType == MV_USB_DEVICE;
  if (snapshot.identity_ok) {
    snapshot.serial = bounded_string(
      device.SpecialInfo.stUsb3VInfo.chSerialNumber,
      sizeof(device.SpecialInfo.stUsb3VInfo.chSerialNumber));
    snapshot.model = bounded_string(
      device.SpecialInfo.stUsb3VInfo.chModelName,
      sizeof(device.SpecialInfo.stUsb3VInfo.chModelName));
  }
  snapshot.width = get_int(handle, "Width");
  snapshot.height = get_int(handle, "Height");
  snapshot.pixel_format = get_enum(handle, "PixelFormat");
  snapshot.frame_rate_enable = get_bool(handle, "AcquisitionFrameRateEnable");
  snapshot.frame_rate = get_float(handle, "AcquisitionFrameRate");
  snapshot.exposure_auto = get_enum(handle, "ExposureAuto");
  snapshot.exposure_time = get_float(handle, "ExposureTime");
  snapshot.gain_auto = get_enum(handle, "GainAuto");
  snapshot.gain = get_float(handle, "Gain");
  snapshot.trigger_mode = get_enum(handle, "TriggerMode");
  snapshot.acquisition_mode = get_enum(handle, "AcquisitionMode");
  snapshot.resulting_frame_rate = get_float(handle, "ResultingFrameRate");
  snapshot.payload_size = get_int(handle, "PayloadSize");
  unsigned int transfer_size = 0;
  int result = MV_USB_GetTransferSize(handle, &transfer_size);
  snapshot.transfer_size = {result == MV_OK, result, result == MV_OK ? transfer_size : 0U};
  unsigned int transfer_ways = 0;
  result = MV_USB_GetTransferWays(handle, &transfer_ways);
  snapshot.transfer_ways = {result == MV_OK, result, result == MV_OK ? transfer_ways : 0U};
  try {
    snapshot.usbfs_memory_mb = {true, 0, read_usbfs_memory_mb()};
  } catch (...) {
    snapshot.usbfs_memory_mb = {false, EIO, 0U};
  }
  return snapshot;
}

struct BaselineResult
{
  std::uint64_t successful_set_count = 0;
  bool pass = false;
  std::string failed_operation;
  int error = 0;
};

bool set_result(BaselineResult & result, int code, const char * operation)
{
  if (code == MV_OK) {
    ++result.successful_set_count;
    return true;
  }
  result.failed_operation = operation;
  result.error = code;
  return false;
}

BaselineResult apply_baseline(void * handle)
{
  BaselineResult result;
  if (!set_result(result, MV_CC_SetEnumValue(handle, "TriggerMode", 0U), "TriggerMode"))
    return result;
  if (!set_result(result, MV_CC_SetEnumValue(handle, "AcquisitionMode", 2U), "AcquisitionMode"))
    return result;
  if (!set_result(
        result, MV_CC_SetIntValueEx(handle, "Width", mvs_raw_capture::kBaselineWidth), "Width"))
    return result;
  if (!set_result(
        result, MV_CC_SetIntValueEx(handle, "Height", mvs_raw_capture::kBaselineHeight), "Height"))
    return result;
  if (!set_result(
        result, MV_CC_SetEnumValue(handle, "PixelFormat", mvs_raw_capture::kBaselinePixelFormat),
        "PixelFormat"))
    return result;
  if (!set_result(
        result, MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true),
        "AcquisitionFrameRateEnable"))
    return result;
  if (!set_result(
        result, MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", kFrameRate),
        "AcquisitionFrameRate"))
    return result;
  if (!set_result(result, MV_CC_SetEnumValue(handle, "ExposureAuto", 0U), "ExposureAuto"))
    return result;
  if (!set_result(
        result, MV_CC_SetFloatValue(handle, "ExposureTime", kExposureTime), "ExposureTime"))
    return result;
  if (!set_result(result, MV_CC_SetEnumValue(handle, "GainAuto", 0U), "GainAuto")) return result;
  if (!set_result(result, MV_CC_SetFloatValue(handle, "Gain", kGain), "Gain")) return result;
  result.pass = result.successful_set_count == 11U;
  return result;
}

bool snapshot_readable(const ParameterSnapshot & value)
{
  return value.identity_ok && value.width.ok && value.height.ok && value.pixel_format.ok &&
         value.frame_rate_enable.ok && value.frame_rate.ok && value.exposure_auto.ok &&
         value.exposure_time.ok && value.gain_auto.ok && value.gain.ok && value.trigger_mode.ok &&
         value.acquisition_mode.ok && value.resulting_frame_rate.ok && value.payload_size.ok &&
         value.transfer_size.ok && value.transfer_ways.ok && value.usbfs_memory_mb.ok;
}

bool snapshot_matches_baseline(const ParameterSnapshot & value)
{
  return snapshot_readable(value) && value.serial == kExpectedSerial &&
         value.model == kExpectedModel && value.width.value == mvs_raw_capture::kBaselineWidth &&
         value.height.value == mvs_raw_capture::kBaselineHeight &&
         value.pixel_format.value == mvs_raw_capture::kBaselinePixelFormat &&
         value.frame_rate_enable.value && std::abs(value.frame_rate.value - kFrameRate) <= 0.01F &&
         value.exposure_auto.value == 0U &&
         std::abs(value.exposure_time.value - kExposureTime) <= 0.1F &&
         value.gain_auto.value == 0U && std::abs(value.gain.value - kGain) <= 0.05F &&
         value.trigger_mode.value == 0U && value.acquisition_mode.value == kAcquisitionMode &&
         std::isfinite(value.resulting_frame_rate.value) &&
         value.resulting_frame_rate.value > 0.0F &&
         value.payload_size.value ==
           static_cast<std::int64_t>(mvs_raw_capture::kBaselinePayloadSize) &&
         value.transfer_size.value == kTransferSize && value.transfer_ways.value == kTransferWays &&
         value.usbfs_memory_mb.value == kUsbfsMemoryMb;
}

template <typename T>
std::string reading_json(const Reading<T> & reading)
{
  std::ostringstream output;
  if (reading.ok) {
    output << "{\"status\":\"OK\",\"value\":" << reading.value << '}';
  } else {
    output << "{\"status\":\"NOT_AVAILABLE\",\"error_decimal\":" << reading.error
           << ",\"error_hex\":\"" << sdk_error_hex(reading.error) << "\"}";
  }
  return output.str();
}

std::string bool_reading_json(const BoolReading & reading)
{
  if (!reading.ok) return reading_json(reading);
  return std::string("{\"status\":\"OK\",\"value\":") + (reading.value ? "true}" : "false}");
}

std::string float_reading_json(const FloatReading & reading)
{
  if (!reading.ok) return reading_json(reading);
  if (!std::isfinite(reading.value)) return "{\"status\":\"NON_FINITE\",\"value\":null}";
  std::ostringstream output;
  output << std::setprecision(9) << "{\"status\":\"OK\",\"value\":" << reading.value << '}';
  return output.str();
}

std::string snapshot_json(
  const char * phase, const ParameterSnapshot & value, const BaselineResult * application)
{
  std::ostringstream output;
  output << std::setprecision(9) << "{\n  \"phase\": \"" << mvs_raw_capture::json_escape(phase)
         << "\",\n"
         << "  \"read_status\": \"" << (snapshot_readable(value) ? "COMPLETE" : "PARTIAL")
         << "\",\n  \"baseline_match\": " << (snapshot_matches_baseline(value) ? "true" : "false")
         << ",\n"
         << "  \"identity\": {\"status\":\"" << (value.identity_ok ? "OK" : "NOT_AVAILABLE")
         << "\",\"serial\":\"" << mvs_raw_capture::json_escape(value.serial) << "\",\"model\":\""
         << mvs_raw_capture::json_escape(value.model)
         << "\",\"error_decimal\":" << value.identity_error << "},\n";
  if (application != nullptr) {
    output << "  \"application\": {\"status\":\"" << (application->pass ? "PASS" : "FAIL")
           << "\",\"successful_set_count\":" << application->successful_set_count;
    if (!application->failed_operation.empty()) {
      output << ",\"failed_operation\":\""
             << mvs_raw_capture::json_escape(application->failed_operation)
             << "\",\"error_decimal\":" << application->error << ",\"error_hex\":\""
             << sdk_error_hex(application->error) << '"';
    }
    output << "},\n";
  }
  output << "  \"parameters\": {\n"
         << "    \"Width\": " << reading_json(value.width) << ",\n"
         << "    \"Height\": " << reading_json(value.height) << ",\n"
         << "    \"PixelFormat\": " << reading_json(value.pixel_format) << ",\n"
         << "    \"AcquisitionFrameRateEnable\": " << bool_reading_json(value.frame_rate_enable)
         << ",\n    \"AcquisitionFrameRate\": " << float_reading_json(value.frame_rate) << ",\n"
         << "    \"ExposureAuto\": " << reading_json(value.exposure_auto) << ",\n"
         << "    \"ExposureTime\": " << float_reading_json(value.exposure_time) << ",\n"
         << "    \"GainAuto\": " << reading_json(value.gain_auto) << ",\n"
         << "    \"Gain\": " << float_reading_json(value.gain) << ",\n"
         << "    \"TriggerMode\": " << reading_json(value.trigger_mode) << ",\n"
         << "    \"AcquisitionMode\": " << reading_json(value.acquisition_mode) << ",\n"
         << "    \"ResultingFrameRate\": " << float_reading_json(value.resulting_frame_rate)
         << ",\n"
         << "    \"PayloadSize\": " << reading_json(value.payload_size) << ",\n"
         << "    \"TransferSize\": " << reading_json(value.transfer_size) << ",\n"
         << "    \"TransferWays\": " << reading_json(value.transfer_ways) << ",\n"
         << "    \"usbfs_memory_mb\": " << reading_json(value.usbfs_memory_mb) << "\n  }\n}\n";
  return output.str();
}

class FixedBufferPool
{
public:
  FixedBufferPool(std::size_t count, std::size_t bytes)
  : buffers_(count, std::vector<unsigned char>(bytes))
  {
    for (std::size_t index = 0; index < count; ++index) free_.push_back(index);
  }
  bool empty() const noexcept { return free_.empty(); }
  std::size_t take()
  {
    const std::size_t value = free_.back();
    free_.pop_back();
    return value;
  }
  void give(std::size_t value) { free_.push_back(value); }
  unsigned char * data(std::size_t value) { return buffers_[value].data(); }

private:
  std::vector<std::vector<unsigned char>> buffers_;
  std::deque<std::size_t> free_;
};

struct QueueItem
{
  std::size_t slot = 0;
  FrameRecord record;
};

struct CaptureMetrics
{
  std::uint64_t acquired_frames = 0;
  std::uint64_t written_frames = 0;
  std::uint64_t written_bytes = 0;
  std::uint64_t queue_peak_depth = 0;
  std::uint64_t queue_overflow_count = 0;
  std::uint64_t timeout_count = 0;
  std::uint64_t sdk_error_count = 0;
  std::uint64_t metadata_error_count = 0;
  std::uint64_t ownership_error_count = 0;
  std::uint64_t free_calls = 0;
  std::uint64_t successful_free_calls = 0;
  std::uint64_t write_error_count = 0;
  std::uint64_t sync_error_count = 0;
  std::uint64_t close_error_count = 0;
  std::uint64_t periodic_sync_calls = 0;
  std::uint64_t gap_events = 0;
  std::uint64_t missing_frames = 0;
  std::uint64_t reset_events = 0;
  std::uint64_t repeat_events = 0;
  double elapsed_seconds = 0.0;
  bool stopped_by_signal = false;
  bool output_verification_pass = false;
  std::string error;
};

CaptureMetrics capture_frames(
  SdkResources & resources, const SessionPaths & paths, double duration_seconds)
{
  CaptureMetrics metrics;
  UniqueFd data_fd(open(paths.data.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644));
  UniqueFd csv_fd(open(paths.frames_csv.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644));
  UniqueFd event_fd(
    open(paths.counter_events_csv.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644));
  if (data_fd.get() < 0 || csv_fd.get() < 0 || event_fd.get() < 0) {
    throw std::runtime_error("cannot create session data files");
  }
  std::string writer_error;
  const std::string csv_header = std::string(mvs_raw_capture::frame_csv_header()) + '\n';
  if (
    !write_all_noexcept(csv_fd.get(), csv_header.data(), csv_header.size(), writer_error) ||
    !write_all_noexcept(
      event_fd.get(), kCounterEventsHeader, std::strlen(kCounterEventsHeader), writer_error)) {
    throw std::runtime_error(writer_error);
  }

  FixedBufferPool pool(mvs_raw_capture::kQueueCapacity + 2U, mvs_raw_capture::kBaselinePayloadSize);
  std::mutex mutex;
  std::condition_variable condition;
  std::deque<QueueItem> queue;
  std::atomic_bool producer_done{false};
  std::atomic_bool writer_failed{false};
  std::atomic<std::uint64_t> written_frames{0};
  std::atomic<std::uint64_t> written_bytes{0};
  std::atomic<std::uint64_t> periodic_sync_calls{0};
  std::atomic<std::uint64_t> write_errors{0};
  std::atomic<std::uint64_t> sync_errors{0};

  require_sdk(MV_CC_StartGrabbing(resources.handle_), "MV_CC_StartGrabbing");
  resources.grabbing_ = true;

  std::thread writer([&]() {
    try {
      while (true) {
        QueueItem item;
        {
          std::unique_lock<std::mutex> lock(mutex);
          condition.wait(lock, [&]() { return producer_done.load() || !queue.empty(); });
          if (queue.empty() && producer_done.load()) break;
          item = queue.front();
          queue.pop_front();
        }
        if (writer_failed.load()) {
          std::lock_guard<std::mutex> lock(mutex);
          pool.give(item.slot);
          continue;
        }
        const unsigned char * data = pool.data(item.slot);
        item.record.crc32 =
          mvs_raw_capture::crc32_bytes(data, static_cast<std::size_t>(item.record.payload_size));
        std::string local_error;
        bool ok = write_all_noexcept(
          data_fd.get(), data, static_cast<std::size_t>(item.record.payload_size), local_error);
        const std::string frame_line = mvs_raw_capture::frame_record_csv(item.record) + '\n';
        if (ok) {
          ok = write_all_noexcept(csv_fd.get(), frame_line.data(), frame_line.size(), local_error);
        }
        if (ok && item.record.counter_event != CounterEvent::none) {
          std::ostringstream event_line;
          event_line << item.record.acquired_index << ','
                     << mvs_raw_capture::to_string(item.record.counter_event) << ','
                     << item.record.previous_sensor_frame_id << ',' << item.record.sensor_frame_id
                     << ',' << item.record.missing_count << ','
                     << item.record.host_receive_timestamp_ns << '\n';
          const std::string text = event_line.str();
          ok = write_all_noexcept(event_fd.get(), text.data(), text.size(), local_error);
        }
        if (ok) {
          written_frames.fetch_add(1);
          written_bytes.fetch_add(item.record.payload_size);
          if (written_frames.load() % mvs_raw_capture::kSyncIntervalFrames == 0U) {
            periodic_sync_calls.fetch_add(1);
            if (!fdatasync_noexcept(data_fd.get(), local_error)) {
              sync_errors.fetch_add(1);
              ok = false;
            }
          }
        }
        {
          std::lock_guard<std::mutex> lock(mutex);
          pool.give(item.slot);
        }
        if (!ok) {
          write_errors.fetch_add(1);
          append_error(writer_error, local_error);
          writer_failed.store(true);
        }
      }
    } catch (const std::exception & error) {
      append_error(writer_error, error.what());
      write_errors.fetch_add(1);
      writer_failed.store(true);
    }
    std::string sync_error;
    if (
      !fdatasync_noexcept(data_fd.get(), sync_error) ||
      !fdatasync_noexcept(csv_fd.get(), sync_error) ||
      !fdatasync_noexcept(event_fd.get(), sync_error)) {
      sync_errors.fetch_add(1);
      append_error(writer_error, sync_error);
      writer_failed.store(true);
    }
  });

  SensorCounterTracker counter_tracker;
  CounterSource counter_source = CounterSource::not_available;
  const auto started = std::chrono::steady_clock::now();
  try {
    while (gStopRequested == 0 && !writer_failed.load() &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() <
             duration_seconds) {
      MV_FRAME_OUT frame{};
      const int result = MV_CC_GetImageBuffer(resources.handle_, &frame, 1000U);
      if (result == MV_OK) {
        const std::uint64_t host_receive_timestamp_ns = monotonic_ns();
        const std::uint64_t acquired_index = metrics.acquired_frames++;
        if (counter_source == CounterSource::not_available) {
          if (frame.stFrameInfo.nFrameCounter != 0U) {
            counter_source = CounterSource::frame_counter;
          } else if (frame.stFrameInfo.nFrameNum != 0U) {
            counter_source = CounterSource::frame_number;
          }
        }
        const std::uint32_t sensor_frame_id =
          counter_source == CounterSource::frame_counter  ? frame.stFrameInfo.nFrameCounter
          : counter_source == CounterSource::frame_number ? frame.stFrameInfo.nFrameNum
                                                          : 0U;
        CounterObservation observation;
        if (counter_source != CounterSource::not_available) {
          observation = counter_tracker.observe(sensor_frame_id);
        }
        const unsigned int width = frame.stFrameInfo.nExtendWidth != 0U
                                     ? frame.stFrameInfo.nExtendWidth
                                     : frame.stFrameInfo.nWidth;
        const unsigned int height = frame.stFrameInfo.nExtendHeight != 0U
                                      ? frame.stFrameInfo.nExtendHeight
                                      : frame.stFrameInfo.nHeight;
        const std::uint64_t length = frame.stFrameInfo.nFrameLenEx != 0U
                                       ? frame.stFrameInfo.nFrameLenEx
                                       : frame.stFrameInfo.nFrameLen;
        const bool metadata_ok =
          width == mvs_raw_capture::kBaselineWidth && height == mvs_raw_capture::kBaselineHeight &&
          static_cast<unsigned int>(frame.stFrameInfo.enPixelType) ==
            mvs_raw_capture::kBaselinePixelFormat &&
          length == mvs_raw_capture::kBaselinePayloadSize && frame.pBufAddr != nullptr;
        std::size_t slot = std::numeric_limits<std::size_t>::max();
        if (metadata_ok) {
          std::lock_guard<std::mutex> lock(mutex);
          if (queue.size() >= mvs_raw_capture::kQueueCapacity || pool.empty()) {
            ++metrics.queue_overflow_count;
          } else {
            slot = pool.take();
          }
        } else {
          ++metrics.metadata_error_count;
        }
        if (slot != std::numeric_limits<std::size_t>::max()) {
          std::memcpy(pool.data(slot), frame.pBufAddr, mvs_raw_capture::kBaselinePayloadSize);
        }

        FrameRecord record;
        record.acquired_index = acquired_index;
        record.sensor_frame_id = sensor_frame_id;
        record.counter_source = counter_source;
        record.sensor_timestamp =
          (static_cast<std::uint64_t>(frame.stFrameInfo.nDevTimeStampHigh) << 32U) |
          frame.stFrameInfo.nDevTimeStampLow;
        record.host_receive_timestamp_ns = host_receive_timestamp_ns;
        record.file_byte_offset = acquired_index * mvs_raw_capture::kBaselinePayloadSize;
        record.payload_size = mvs_raw_capture::kBaselinePayloadSize;
        record.counter_event = observation.event;
        record.previous_sensor_frame_id = observation.previous_sensor_frame_id;
        record.missing_count = observation.missing_count;

        ++metrics.free_calls;
        const int free_result = MV_CC_FreeImageBuffer(resources.handle_, &frame);
        if (free_result != MV_OK) {
          ++metrics.sdk_error_count;
          ++metrics.ownership_error_count;
          append_error(
            metrics.error, "MV_CC_FreeImageBuffer failed: " + sdk_error_hex(free_result));
          if (slot != std::numeric_limits<std::size_t>::max()) {
            std::lock_guard<std::mutex> lock(mutex);
            pool.give(slot);
          }
          break;
        }
        ++metrics.successful_free_calls;
        if (slot != std::numeric_limits<std::size_t>::max()) {
          {
            std::lock_guard<std::mutex> lock(mutex);
            queue.push_back({slot, record});
            metrics.queue_peak_depth =
              std::max<std::uint64_t>(metrics.queue_peak_depth, queue.size());
          }
          condition.notify_one();
        } else {
          break;
        }
      } else if (static_cast<std::uint32_t>(result) == static_cast<std::uint32_t>(MV_E_NODATA)) {
        ++metrics.timeout_count;
      } else {
        ++metrics.sdk_error_count;
        append_error(metrics.error, "MV_CC_GetImageBuffer failed: " + sdk_error_hex(result));
        break;
      }
    }
  } catch (const std::exception & error) {
    append_error(metrics.error, std::string("capture producer failed: ") + error.what());
    ++metrics.ownership_error_count;
  }
  metrics.elapsed_seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  metrics.stopped_by_signal = gStopRequested != 0;
  if (!resources.stop_grabbing(metrics.error)) ++metrics.sdk_error_count;
  producer_done.store(true);
  condition.notify_all();
  writer.join();

  metrics.written_frames = written_frames.load();
  metrics.written_bytes = written_bytes.load();
  metrics.periodic_sync_calls = periodic_sync_calls.load();
  metrics.write_error_count = write_errors.load();
  metrics.sync_error_count = sync_errors.load();
  metrics.gap_events = counter_tracker.gap_events();
  metrics.missing_frames = counter_tracker.missing_frames();
  metrics.reset_events = counter_tracker.reset_events();
  metrics.repeat_events = counter_tracker.repeat_events();
  if (
    metrics.free_calls != metrics.acquired_frames ||
    metrics.successful_free_calls != metrics.acquired_frames) {
    ++metrics.ownership_error_count;
    append_error(metrics.error, "SDK buffer ownership accounting mismatch");
  }
  if (!writer_error.empty()) append_error(metrics.error, writer_error);
  if (!data_fd.close_checked(metrics.error)) ++metrics.close_error_count;
  if (!csv_fd.close_checked(metrics.error)) ++metrics.close_error_count;
  if (!event_fd.close_checked(metrics.error)) ++metrics.close_error_count;
  return metrics;
}

int run_capture(const mvs_raw_capture::CaptureOptions & options)
{
  std::error_code directory_error;
  fs::create_directories(options.output_dir, directory_error);
  if (directory_error) {
    throw std::runtime_error("cannot create output directory: " + directory_error.message());
  }
  const DiskBudget budget = mvs_raw_capture::calculate_disk_budget(options.duration_seconds);
  const std::uint64_t available = mvs_raw_capture::available_bytes(options.output_dir);
  std::cout << std::fixed << std::setprecision(3)
            << "current_reference_estimated_GB=" << budget.current_reference_estimated_bytes / 1.0e9
            << " worst_case_estimated_GB=" << budget.worst_case_estimated_bytes / 1.0e9
            << " required_available_GB=" << budget.required_available_bytes / 1.0e9
            << " available_GB=" << available / 1.0e9 << '\n';
  if (available < budget.required_available_bytes) {
    throw std::runtime_error("insufficient disk space for worst-case capture budget");
  }

  const SessionPaths paths =
    mvs_raw_capture::create_session_directory(options.output_dir, options.session_name);
  SessionManifest manifest;
  manifest.session_name = options.session_name;
  manifest.created_at_utc = mvs_raw_capture::utc_timestamp();
  manifest.requested_duration_seconds = options.duration_seconds;
  manifest.current_reference_estimated_bytes = budget.current_reference_estimated_bytes;
  manifest.worst_case_estimated_bytes = budget.worst_case_estimated_bytes;
  mvs_raw_capture::write_text_atomic_synced(
    paths.manifest, mvs_raw_capture::manifest_json(manifest));

  CaptureMetrics metrics;
  SdkResources resources;
  bool parameter_gate_pass = false;
  std::string fatal_error;
  try {
    require_sdk(MV_CC_Initialize(), "MV_CC_Initialize");
    resources.sdk_initialized_ = true;
    MV_CC_DEVICE_INFO_LIST devices{};
    require_sdk(MV_CC_EnumDevices(MV_USB_DEVICE, &devices), "MV_CC_EnumDevices");
    MV_CC_DEVICE_INFO * selected = nullptr;
    for (unsigned int index = 0; index < devices.nDeviceNum; ++index) {
      MV_CC_DEVICE_INFO * candidate = devices.pDeviceInfo[index];
      if (candidate == nullptr || candidate->nTLayerType != MV_USB_DEVICE) continue;
      const std::string serial = bounded_string(
        candidate->SpecialInfo.stUsb3VInfo.chSerialNumber,
        sizeof(candidate->SpecialInfo.stUsb3VInfo.chSerialNumber));
      if (serial == kExpectedSerial) {
        selected = candidate;
        break;
      }
    }
    if (selected == nullptr) throw std::runtime_error("target camera not found");
    const std::string model = bounded_string(
      selected->SpecialInfo.stUsb3VInfo.chModelName,
      sizeof(selected->SpecialInfo.stUsb3VInfo.chModelName));
    if (model != kExpectedModel) throw std::runtime_error("target camera model mismatch");
    require_sdk(MV_CC_CreateHandle(&resources.handle_, selected), "MV_CC_CreateHandle");
    require_sdk(MV_CC_OpenDevice(resources.handle_), "MV_CC_OpenDevice");
    resources.device_open_ = true;

    const ParameterSnapshot initial = read_snapshot(resources.handle_);
    mvs_raw_capture::write_text_atomic_synced(
      paths.initial_parameters, snapshot_json("initial_before_apply", initial, nullptr));
    const BaselineResult application = apply_baseline(resources.handle_);
    const ParameterSnapshot applied = read_snapshot(resources.handle_);
    mvs_raw_capture::write_text_atomic_synced(
      paths.applied_parameters, snapshot_json("applied_readback", applied, &application));
    parameter_gate_pass = application.pass && snapshot_matches_baseline(applied);
    if (!parameter_gate_pass)
      throw std::runtime_error("camera baseline application/readback failed");
    metrics = capture_frames(resources, paths, options.duration_seconds);
  } catch (const std::exception & error) {
    append_error(fatal_error, error.what());
  }

  const bool camera_released = resources.cleanup();
  if (!fatal_error.empty()) append_error(metrics.error, fatal_error);
  manifest.acquired_frames = metrics.acquired_frames;
  manifest.written_frames = metrics.written_frames;
  manifest.queue_peak_depth = metrics.queue_peak_depth;
  manifest.queue_overflow_count = metrics.queue_overflow_count;
  manifest.sensor_gap_events = metrics.gap_events;
  manifest.sensor_missing_frames = metrics.missing_frames;
  manifest.sensor_reset_events = metrics.reset_events;
  manifest.sensor_repeat_events = metrics.repeat_events;

  if (metrics.written_frames > 0U) {
    try {
      const std::vector<FrameRecord> records =
        mvs_raw_capture::read_frame_records(paths.frames_csv);
      mvs_raw_capture::validate_frame_records(records, mvs_raw_capture::kBaselinePayloadSize, true);
      metrics.output_verification_pass =
        records.size() == metrics.written_frames &&
        fs::file_size(paths.data) == metrics.written_frames * mvs_raw_capture::kBaselinePayloadSize;
    } catch (const std::exception & error) {
      append_error(metrics.error, std::string("output verification failed: ") + error.what());
    }
  }

  const bool duration_complete = metrics.elapsed_seconds >= options.duration_seconds - 0.25;
  CompletionGate gate;
  gate.parameter_gate_pass = parameter_gate_pass;
  gate.duration_complete = duration_complete;
  gate.stopped_by_signal = metrics.stopped_by_signal;
  gate.camera_released = camera_released;
  gate.acquired_frames = metrics.acquired_frames;
  gate.written_frames = metrics.written_frames;
  gate.queue_overflow_count = metrics.queue_overflow_count;
  gate.sdk_error_count = metrics.sdk_error_count;
  gate.metadata_error_count = metrics.metadata_error_count;
  gate.ownership_error_count = metrics.ownership_error_count;
  gate.write_error_count = metrics.write_error_count;
  gate.sync_error_count = metrics.sync_error_count;
  gate.close_error_count = metrics.close_error_count;
  gate.output_verification_pass = metrics.output_verification_pass;
  gate.runtime_error_free = metrics.error.empty();

  CaptureResult result;
  result.session_name = options.session_name;
  result.requested_duration_seconds = options.duration_seconds;
  result.started_at_utc = manifest.created_at_utc;
  result.finished_at_utc = mvs_raw_capture::utc_timestamp();
  result.elapsed_seconds = metrics.elapsed_seconds;
  result.current_reference_estimated_bytes = budget.current_reference_estimated_bytes;
  result.worst_case_estimated_bytes = budget.worst_case_estimated_bytes;
  result.available_bytes_at_start = available;
  result.acquired_frames = metrics.acquired_frames;
  result.written_frames = metrics.written_frames;
  result.written_fps = metrics.elapsed_seconds > 0.0
                         ? static_cast<double>(metrics.written_frames) / metrics.elapsed_seconds
                         : 0.0;
  result.written_MBps = result.written_fps * mvs_raw_capture::kBaselinePayloadSize / 1.0e6;
  result.queue_peak_depth = metrics.queue_peak_depth;
  result.queue_overflow_count = metrics.queue_overflow_count;
  result.sensor_gap_events = metrics.gap_events;
  result.missing_frames = metrics.missing_frames;
  result.reset_events = metrics.reset_events;
  result.repeat_events = metrics.repeat_events;
  result.timeout_count = metrics.timeout_count;
  result.sdk_error_count = metrics.sdk_error_count;
  result.metadata_error_count = metrics.metadata_error_count;
  result.ownership_error_count = metrics.ownership_error_count;
  result.write_error_count = metrics.write_error_count;
  result.sync_error_count = metrics.sync_error_count;
  result.close_error_count = metrics.close_error_count;
  result.periodic_fdatasync_calls = metrics.periodic_sync_calls;
  result.output_verification_pass = metrics.output_verification_pass;
  result.camera_released = camera_released;

  gate.result_file_persisted = true;
  const bool capture_complete_candidate = mvs_raw_capture::can_mark_session_complete(gate);
  result.final_status = capture_complete_candidate ? "complete" : "incomplete";
  result.failure_reason =
    capture_complete_candidate
      ? ""
      : (!metrics.error.empty() ? metrics.error : mvs_raw_capture::completion_failure_reason(gate));

  bool result_file_persisted = false;
  std::string result_write_error;
  try {
    mvs_raw_capture::write_text_atomic_synced(
      paths.capture_result, mvs_raw_capture::capture_result_json(result));
    result_file_persisted = true;
  } catch (const std::exception & error) {
    result_write_error = error.what();
    result.final_status = "incomplete";
    result.failure_reason = "capture_result_write_failed: " + result_write_error;
  }

  gate.result_file_persisted = result_file_persisted;
  bool complete = mvs_raw_capture::can_mark_session_complete(gate);
  if (complete) {
    manifest.status = "complete";
    manifest.completed_at_utc = result.finished_at_utc;
    manifest.incomplete_reason.clear();
  } else {
    manifest.status = "incomplete";
    manifest.incomplete_reason = result.failure_reason;
  }

  try {
    mvs_raw_capture::write_text_atomic_synced(
      paths.manifest, mvs_raw_capture::manifest_json(manifest));
  } catch (const std::exception & error) {
    complete = false;
    result.final_status = "incomplete";
    result.failure_reason = std::string("session_manifest_update_failed: ") + error.what();
    if (result_file_persisted) {
      try {
        mvs_raw_capture::write_text_atomic_synced(
          paths.capture_result, mvs_raw_capture::capture_result_json(result));
      } catch (...) {
      }
    }
  }
  std::cout << mvs_raw_capture::capture_result_json(result);
  return complete ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> arguments;
  for (int index = 1; index < argc; ++index) arguments.emplace_back(argv[index]);
  try {
    const mvs_raw_capture::CaptureOptions options =
      mvs_raw_capture::parse_capture_options(arguments);
    if (options.help) {
      std::cout << mvs_raw_capture::capture_usage();
      return 0;
    }
    struct sigaction action
    {
    };
    action.sa_handler = signal_handler;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, nullptr) != 0 || sigaction(SIGTERM, &action, nullptr) != 0) {
      throw std::runtime_error("cannot install signal handlers");
    }
    return run_capture(options);
  } catch (const std::invalid_argument & error) {
    std::cerr << "CLI_ERROR: " << error.what() << '\n' << mvs_raw_capture::capture_usage();
    return 2;
  } catch (const std::exception & error) {
    std::cerr << "CAPTURE_SETUP_ERROR: " << error.what() << '\n';
    return 1;
  }
}
