#include <MvCameraControl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
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
    publish_camera_info_ = declare_parameter("publish_camera_info", false);
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
    shutdown_camera();
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
    RCLCPP_INFO(
      get_logger(),
      "Camera opened: model=%s serial=%s frame_rate=%.1f exposure=%.1fus gain=%.1fdB",
      selected_model.c_str(), selected_serial.c_str(), frame_rate_, exposure_time_us_, gain_db_);
  }

  void grab_loop()
  {
    uint64_t frame_count = 0;
    auto report_started = std::chrono::steady_clock::now();

    while (running_.load() && rclcpp::ok()) {
      MV_FRAME_OUT frame{};
      const int result = MV_CC_GetImageBuffer(handle_, &frame, grab_timeout_ms_);
      if (result != MV_OK) {
        if (running_.load()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "GetImageBuffer failed: %s",
            error_hex(result).c_str());
        }
        continue;
      }

      const unsigned int width = frame.stFrameInfo.nExtendWidth;
      const unsigned int height = frame.stFrameInfo.nExtendHeight;
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
        const auto stamp = now();
        auto image = std::make_unique<sensor_msgs::msg::Image>();
        image->header.stamp = stamp;
        image->header.frame_id = frame_id_;
        image->height = height;
        image->width = width;
        image->encoding = "bgr8";
        image->is_bigendian = false;
        image->step = width * 3U;
        image->data.assign(bgr_buffer_.begin(), bgr_buffer_.begin() + convert.nDstLen);
        image_pub_->publish(std::move(image));

        if (publish_camera_info_) {
          auto info = camera_info_manager_->getCameraInfo();
          info.header.stamp = stamp;
          info.header.frame_id = frame_id_;
          info.width = width;
          info.height = height;
          camera_info_pub_->publish(info);
        }
        ++frame_count;
      } else {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "Bayer-to-BGR conversion failed: %s",
          error_hex(convert_result).c_str());
      }

      const int free_result = MV_CC_FreeImageBuffer(handle_, &frame);
      if (free_result != MV_OK) {
        RCLCPP_ERROR(
          get_logger(), "FreeImageBuffer failed: %s", error_hex(free_result).c_str());
      }

      const auto current = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(current - report_started).count();
      if (elapsed >= 5.0) {
        RCLCPP_INFO(
          get_logger(), "Publishing %.1f FPS at %ux%u bgr8",
          static_cast<double>(frame_count) / elapsed, width, height);
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
  bool publish_camera_info_{false};

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
