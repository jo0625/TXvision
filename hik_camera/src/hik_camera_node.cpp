#include <MvCameraControl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  HikCameraNode()
  : Node("hik_camera")
  {
    serial_number_ = declare_parameter("serial_number", "DA8541046");
    frame_id_ = declare_parameter("frame_id", "camera_optical_frame");
    image_topic_ = declare_parameter("image_topic", "/image_raw");
    camera_info_topic_ = declare_parameter("camera_info_topic", "/camera_info");
    frame_rate_ = declare_parameter("frame_rate", 60.0);
    exposure_time_us_ = declare_parameter("exposure_time_us", 5000.0);
    gain_db_ = declare_parameter("gain_db", 8.0);
    grab_timeout_ms_ = declare_parameter("grab_timeout_ms", 1000);
    publish_bayer_ = declare_parameter("publish_bayer", false);
    publish_camera_info_ = declare_parameter("publish_camera_info", false);
    diagnostics_enabled_ = declare_parameter("diagnostics_enabled", false);
    stability_trace_path_ = declare_parameter("stability_trace_path", "");
    if (!stability_trace_path_.empty()) {
      if (std::ifstream(stability_trace_path_).good()) {
        throw std::runtime_error("stability trace already exists");
      }
      stability_rows_.reserve(20000);
    }
    camera_name_ = declare_parameter("camera_name", "hikrobot_mv_cs016_10uc");
    camera_info_url_ = declare_parameter("camera_info_url", "");

    if (frame_rate_ <= 0.0 || exposure_time_us_ <= 0.0 || grab_timeout_ms_ <= 0) {
      throw std::invalid_argument("frame_rate, exposure_time_us and grab_timeout_ms must be positive");
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, camera_name_, camera_info_url_);

    try {
      initialize_camera();
      running_.store(true);
      grab_thread_ = std::thread(&HikCameraNode::grab_loop, this);
    } catch (...) {
      shutdown_camera();
      throw;
    }
  }

  ~HikCameraNode() override
  {
    running_.store(false);
    if (grab_thread_.joinable()) {
      grab_thread_.join();
    }
    if (!stability_trace_path_.empty() && device_open_) {
      record_configuration("stop");
    }
    shutdown_camera();
    save_stability();
  }

private:
  static std::string bounded_string(const unsigned char * data, size_t capacity)
  {
    const auto * chars = reinterpret_cast<const char *>(data);
    return std::string(chars, strnlen(chars, capacity));
  }

  static std::string error_hex(int code)
  {
    char buffer[32]{};
    std::snprintf(buffer, sizeof(buffer), "0x%08x", static_cast<unsigned int>(code));
    return buffer;
  }

  void require_ok(int result, const std::string & operation)
  {
    if (result != MV_OK) {
      throw std::runtime_error(operation + " failed: " + error_hex(result));
    }
  }

  void warn_if_failed(int result, const std::string & operation)
  {
    if (result != MV_OK) {
      RCLCPP_WARN(get_logger(), "%s failed: %s", operation.c_str(), error_hex(result).c_str());
    }
  }

  void initialize_camera()
  {
    require_ok(MV_CC_Initialize(), "MV_CC_Initialize");
    sdk_initialized_ = true;

    MV_CC_DEVICE_INFO_LIST device_list{};
    require_ok(MV_CC_EnumDevices(MV_USB_DEVICE, &device_list), "MV_CC_EnumDevices");
    if (device_list.nDeviceNum == 0) {
      throw std::runtime_error("no Hikrobot USB3 Vision camera found");
    }

    MV_CC_DEVICE_INFO * selected = nullptr;
    std::string selected_model;
    std::string selected_serial;
    for (unsigned int index = 0; index < device_list.nDeviceNum; ++index) {
      auto * info = device_list.pDeviceInfo[index];
      if (info == nullptr || info->nTLayerType != MV_USB_DEVICE) {
        continue;
      }
      const auto serial = bounded_string(
        info->SpecialInfo.stUsb3VInfo.chSerialNumber,
        sizeof(info->SpecialInfo.stUsb3VInfo.chSerialNumber));
      const auto model = bounded_string(
        info->SpecialInfo.stUsb3VInfo.chModelName,
        sizeof(info->SpecialInfo.stUsb3VInfo.chModelName));
      RCLCPP_INFO(
        get_logger(), "Found camera: model=%s serial=%s", model.c_str(), serial.c_str());
      if (serial_number_.empty() || serial == serial_number_) {
        selected = info;
        selected_model = model;
        selected_serial = serial;
        break;
      }
    }

    if (selected == nullptr) {
      throw std::runtime_error("requested camera serial not found: " + serial_number_);
    }

    require_ok(MV_CC_CreateHandle(&handle_, selected), "MV_CC_CreateHandle");
    require_ok(MV_CC_OpenDevice(handle_), "MV_CC_OpenDevice");
    device_open_ = true;

    require_ok(MV_CC_SetEnumValue(handle_, "TriggerMode", 0), "set TriggerMode=Off");
    warn_if_failed(
      MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_BayerRG8),
      "set PixelFormat=BayerRG8");
    warn_if_failed(
      MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true),
      "enable AcquisitionFrameRate");
    warn_if_failed(
      MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(frame_rate_)),
      "set AcquisitionFrameRate");
    warn_if_failed(MV_CC_SetEnumValue(handle_, "ExposureAuto", 0), "set ExposureAuto=Off");
    warn_if_failed(
      MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(exposure_time_us_)),
      "set ExposureTime");
    warn_if_failed(MV_CC_SetEnumValue(handle_, "GainAuto", 0), "set GainAuto=Off");
    warn_if_failed(
      MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_db_)), "set Gain");
    warn_if_failed(MV_CC_SetBayerCvtQuality(handle_, 1), "set Bayer conversion quality");

    require_ok(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");
    grabbing_ = true;
    if (!stability_trace_path_.empty()) {
      record_configuration("start");
    }
    RCLCPP_INFO(
      get_logger(),
      "Camera opened: model=%s serial=%s frame_rate=%.1f exposure=%.1fus gain=%.1fdB",
      selected_model.c_str(), selected_serial.c_str(), frame_rate_, exposure_time_us_, gain_db_);
  }

  void grab_loop()
  {
    uint64_t frame_count = 0;
    uint64_t camera_frame_count = 0;
    uint64_t camera_frame_gaps = 0;
    uint64_t counter_anomalies = 0;
    uint32_t previous_frame_number = 0;
    bool have_previous_frame_number = false;
    double image_build_ms_sum = 0.0;
    double image_build_ms_max = 0.0;
    double publish_ms_sum = 0.0;
    double publish_ms_max = 0.0;
    auto report_started = std::chrono::steady_clock::now();

    while (running_.load() && rclcpp::ok()) {
      MV_FRAME_OUT frame{};
      const int result = MV_CC_GetImageBuffer(handle_, &frame, grab_timeout_ms_);
      const auto received_at = std::chrono::steady_clock::now();
      if (result != MV_OK) {
        if (!stability_trace_path_.empty()) {
          ++stability_grab_errors_;
        }
        if (running_.load()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "GetImageBuffer failed: %s",
            error_hex(result).c_str());
        }
        continue;
      }

      if (diagnostics_enabled_) {
        ++camera_frame_count;
        const uint32_t frame_number = frame.stFrameInfo.nFrameNum;
        if (have_previous_frame_number) {
          const uint32_t delta = frame_number - previous_frame_number;
          if (delta > 1U && delta < 0x80000000U) {
            camera_frame_gaps += delta - 1U;
          } else if (delta == 0U || delta >= 0x80000000U) {
            ++counter_anomalies;
          }
        }
        previous_frame_number = frame_number;
        have_previous_frame_number = true;
      }

      const unsigned int width = frame.stFrameInfo.nExtendWidth;
      const unsigned int height = frame.stFrameInfo.nExtendHeight;
      const auto stamp = now();
      auto image = std::make_unique<sensor_msgs::msg::Image>();
      image->header.stamp = stamp;
      image->header.frame_id = frame_id_;
      image->height = height;
      image->width = width;
      image->is_bigendian = false;

      int64_t publish_begin_ns = 0, publish_end_ns = 0;
      bool image_ready = false;
      if (publish_bayer_) {
        const size_t required_size = static_cast<size_t>(width) * height;
        if (
          frame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8 &&
          frame.stFrameInfo.nFrameLenEx >= required_size)
        {
          image->encoding = "bayer_rggb8";
          image->step = width;
          image->data.assign(frame.pBufAddr, frame.pBufAddr + required_size);
          image_ready = true;
        } else {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cannot publish BayerRG8 frame: pixel_type=%u frame_length=%zu required=%zu",
            static_cast<unsigned int>(frame.stFrameInfo.enPixelType),
            static_cast<size_t>(frame.stFrameInfo.nFrameLenEx), required_size);
        }
      } else {
        const size_t required_size = static_cast<size_t>(width) * height * 3U;
        if (bgr_buffer_.size() != required_size) {
          bgr_buffer_.resize(required_size);
        }

        MV_CC_PIXEL_CONVERT_PARAM_EX convert{};
        convert.nWidth = width;
        convert.nHeight = height;
        convert.enSrcPixelType = frame.stFrameInfo.enPixelType;
        convert.pSrcData = frame.pBufAddr;
        convert.nSrcDataLen = frame.stFrameInfo.nFrameLenEx;
        convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        convert.pDstBuffer = bgr_buffer_.data();
        convert.nDstBufferSize = static_cast<unsigned int>(bgr_buffer_.size());
        const int convert_result = MV_CC_ConvertPixelTypeEx(handle_, &convert);
        if (convert_result == MV_OK) {
          image->encoding = "bgr8";
          image->step = width * 3U;
          image->data.assign(bgr_buffer_.begin(), bgr_buffer_.begin() + convert.nDstLen);
          image_ready = true;
        } else {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000, "Bayer-to-BGR conversion failed: %s",
            error_hex(convert_result).c_str());
        }
      }

      if (image_ready) {
        const auto publish_started = std::chrono::steady_clock::now();
        publish_begin_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(publish_started.time_since_epoch()).count();
        image_pub_->publish(std::move(image));
        const auto publish_finished = std::chrono::steady_clock::now();
        publish_end_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(publish_finished.time_since_epoch()).count();
        if (diagnostics_enabled_) {
          const double image_build_ms =
            std::chrono::duration<double, std::milli>(publish_started - received_at).count();
          const double publish_ms =
            std::chrono::duration<double, std::milli>(publish_finished - publish_started).count();
          image_build_ms_sum += image_build_ms;
          image_build_ms_max = std::max(image_build_ms_max, image_build_ms);
          publish_ms_sum += publish_ms;
          publish_ms_max = std::max(publish_ms_max, publish_ms);
        }

        if (publish_camera_info_) {
          auto info = camera_info_manager_->getCameraInfo();
          info.header.stamp = stamp;
          info.header.frame_id = frame_id_;
          info.width = width;
          info.height = height;
          camera_info_pub_->publish(info);
        }
        ++frame_count;
      }

      const auto stability_number = frame.stFrameInfo.nFrameNum;
      const auto stability_pixel_format = frame.stFrameInfo.enPixelType;
      const uint64_t device_ticks = (static_cast<uint64_t>(frame.stFrameInfo.nDevTimeStampHigh) << 32) | frame.stFrameInfo.nDevTimeStampLow;
      const int64_t sdk_host_timestamp = frame.stFrameInfo.nHostTimeStamp;
      const int free_result = MV_CC_FreeImageBuffer(handle_, &frame);
      if (!stability_trace_path_.empty()) {
        if (stability_rows_.size() < 20000) {
          stability_rows_.push_back({
            std::chrono::duration_cast<std::chrono::nanoseconds>(received_at.time_since_epoch()).count(),
            stamp.nanoseconds(), stability_number, width, height,
            static_cast<uint32_t>(stability_pixel_format), image_ready, free_result,
            publish_begin_ns, publish_end_ns, device_ticks, sdk_host_timestamp});
        } else {
          ++stability_dropped_;
        }
      }
      if (free_result != MV_OK) {
        RCLCPP_ERROR(
          get_logger(), "FreeImageBuffer failed: %s", error_hex(free_result).c_str());
      }

      const auto current = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(current - report_started).count();
      if (elapsed >= 5.0) {
        RCLCPP_INFO(
          get_logger(), "Publishing %.1f FPS at %ux%u %s",
          static_cast<double>(frame_count) / elapsed, width, height,
          publish_bayer_ ? "bayer_rggb8" : "bgr8");
        if (diagnostics_enabled_) {
          const double divisor = frame_count == 0 ? 1.0 : static_cast<double>(frame_count);
          RCLCPP_INFO(
            get_logger(),
            "Camera diagnostics: received=%lu published=%lu frame_gaps=%lu "
            "counter_anomalies=%lu image_build_ms=%.2f/%.2f publish_ms=%.2f/%.2f "
            "(mean/max)",
            camera_frame_count, frame_count, camera_frame_gaps, counter_anomalies,
            image_build_ms_sum / divisor, image_build_ms_max,
            publish_ms_sum / divisor, publish_ms_max);
          camera_frame_count = 0;
          camera_frame_gaps = 0;
          counter_anomalies = 0;
          image_build_ms_sum = 0.0;
          image_build_ms_max = 0.0;
          publish_ms_sum = 0.0;
          publish_ms_max = 0.0;
        }
        frame_count = 0;
        report_started = current;
      }
    }
  }

  void shutdown_camera()
  {
    if (handle_ != nullptr && grabbing_) {
      MV_CC_StopGrabbing(handle_);
      grabbing_ = false;
    }
    if (handle_ != nullptr && device_open_) {
      MV_CC_CloseDevice(handle_);
      device_open_ = false;
    }
    if (handle_ != nullptr) {
      MV_CC_DestroyHandle(handle_);
      handle_ = nullptr;
    }
    if (sdk_initialized_) {
      MV_CC_Finalize();
      sdk_initialized_ = false;
    }
  }

  void record_configuration(const std::string & phase)
  {
    for (const auto * name : {"Width", "Height", "OffsetX", "OffsetY"}) {
      MVCC_INTVALUE_EX value{};
      const int status = MV_CC_GetIntValueEx(handle_, name, &value);
      stability_config_.push_back(phase + "," + name + "," + std::to_string(status) +
        "," + (status == MV_OK ? std::to_string(value.nCurValue) : ""));
    }
    for (const auto * name : {"ExposureTime", "Gain", "AcquisitionFrameRate", "ResultingFrameRate"}) {
      MVCC_FLOATVALUE value{};
      const int status = MV_CC_GetFloatValue(handle_, name, &value);
      stability_config_.push_back(phase + "," + name + "," + std::to_string(status) +
        "," + (status == MV_OK ? std::to_string(value.fCurValue) : ""));
    }
    for (const auto * name : {"PixelFormat", "ExposureAuto", "GainAuto", "TriggerMode"}) {
      MVCC_ENUMVALUE value{};
      const int status = MV_CC_GetEnumValue(handle_, name, &value);
      stability_config_.push_back(phase + "," + name + "," + std::to_string(status) +
        "," + (status == MV_OK ? std::to_string(value.nCurValue) : ""));
    }
    bool enabled{};
    const int status = MV_CC_GetBoolValue(handle_, "AcquisitionFrameRateEnable", &enabled);
    stability_config_.push_back(phase + ",AcquisitionFrameRateEnable," + std::to_string(status) +
      "," + (status == MV_OK ? std::to_string(enabled) : ""));
  }

  void save_stability()
  {
    if (stability_trace_path_.empty()) {
      return;
    }
    std::ofstream stream(stability_trace_path_);
    stream << "host_ns,source_stamp_ns,sdk_frame_number,width,height,pixel_format,published,free_result,publish_begin_ns,publish_end_ns,device_ticks_raw,sdk_host_timestamp_raw\n";
    for (const auto & row : stability_rows_) {
      stream << row.host_ns << ',' << row.stamp_ns << ',' << row.number << ',' << row.width << ',' <<
        row.height << ',' << row.pixel_format << ',' << row.published << ',' << row.free_result << ',' <<
        row.publish_begin_ns << ',' << row.publish_end_ns << ',' << row.device_ticks << ',' << row.sdk_host_timestamp << '\n';
    }
    std::ofstream config(stability_trace_path_ + ".configuration.csv");
    config << "phase,node,status,value\n";
    for (const auto & line : stability_config_) {
      config << line << '\n';
    }
    config << "stop,log_dropped,0," << stability_dropped_ << '\n' <<
      "stop,grab_errors,0," << stability_grab_errors_ << '\n';
    if (!stream.good() || !config.good()) {
      RCLCPP_ERROR(get_logger(), "Failed to save camera stability trace");
    }
  }

  struct StabilityRow
  {
    int64_t host_ns, stamp_ns;
    uint32_t number, width, height, pixel_format;
    bool published;
    int free_result;
    int64_t publish_begin_ns, publish_end_ns;
    uint64_t device_ticks;
    int64_t sdk_host_timestamp;
  };
  std::string stability_trace_path_;
  std::vector<StabilityRow> stability_rows_;
  std::vector<std::string> stability_config_;
  uint64_t stability_dropped_{0}, stability_grab_errors_{0};

  std::string serial_number_;
  std::string frame_id_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string camera_name_;
  std::string camera_info_url_;
  double frame_rate_{60.0};
  double exposure_time_us_{5000.0};
  double gain_db_{8.0};
  int grab_timeout_ms_{1000};
  bool publish_bayer_{false};
  bool publish_camera_info_{false};
  bool diagnostics_enabled_{false};

  void * handle_{nullptr};
  bool sdk_initialized_{false};
  bool device_open_{false};
  bool grabbing_{false};
  std::atomic_bool running_{false};
  std::thread grab_thread_;
  std::vector<unsigned char> bgr_buffer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
};
}  // namespace hik_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<hik_camera::HikCameraNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("hik_camera"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
