// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "armor_tracker/tracker_node.hpp"

// STD
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "armor_tracker/ekf_models.hpp"
#include "armor_tracker/time_utils.hpp"

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  diagnostic_boundaries_ = declare_parameter("diagnostic_boundaries", false);
  latency_trace_path_ = declare_parameter("latency_trace_path", "");
  if (!latency_trace_path_.empty()) {
    if (std::ifstream(latency_trace_path_).good())
      throw std::runtime_error("Use new tracker trace path");
    timing_rows_.reserve(20000);
    timing_running_.store(true);
    timing_writer_ = std::thread([this]() {
      std::ofstream out(latency_trace_path_);
      out << "source_stamp_ns,callback_begin_ns,transformed_ns,ready_ns,previous_state_stamp_ns,"
             "tracker_state,tracking,target_id,event,source_frame,target_frame,detail\n";
      do {
        std::vector<TimingRow> batch;
        batch.reserve(20000);
        {
          std::lock_guard<std::mutex> lock(timing_mutex_);
          batch.swap(timing_rows_);
        }
        for (const auto & r : batch)
          out << r.stamp << ',' << r.begin << ',' << r.transformed << ',' << r.ready << ','
              << r.previous << ',' << r.state << ',' << r.tracking << ',' << r.id << ',' << r.event
              << ',' << r.source_frame << ',' << r.target_frame << ',' << r.detail << '\n';
        out.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      } while (timing_running_.load());
      std::lock_guard<std::mutex> lock(timing_mutex_);
      for (const auto & r : timing_rows_)
        out << r.stamp << ',' << r.begin << ',' << r.transformed << ',' << r.ready << ','
            << r.previous << ',' << r.state << ',' << r.tracking << ',' << r.id << ',' << r.event
            << ',' << r.source_frame << ',' << r.target_frame << ',' << r.detail << '\n';
      std::ofstream meta(latency_trace_path_ + ".dropped.txt");
      meta << timing_dropped_ << '\n';
    });
  }
  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  double max_match_yaw_diff_ = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  const auto motion_model = declare_parameter<std::string>("tracker.motion_model", "rotating");
  if (motion_model != "rotating" && motion_model != "single_armor") {
    throw std::invalid_argument("tracker.motion_model must be rotating or single_armor");
  }
  const bool single_armor = motion_model == "single_armor";
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff_, single_armor);
  RCLCPP_INFO(get_logger(), "Tracker motion model: %s", motion_model.c_str());
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  tracker_->lost_time_thres = this->declare_parameter("tracker.lost_time_thres", 0.3);
  max_dt_ = this->declare_parameter("tracker.max_dt", 0.2);
  if (!isPositiveFiniteDt(tracker_->lost_time_thres)) {
    throw std::invalid_argument(
      "tracker.lost_time_thres must be finite and greater than zero seconds");
  }
  if (!isPositiveFiniteDt(max_dt_)) {
    throw std::invalid_argument("tracker.max_dt must be finite and greater than zero seconds");
  }

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
  // measurement: xa, ya, za, yaw
  // f - Process function
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_;
    x_new(2) += x(3) * dt_;
    x_new(4) += x(5) * dt_;
    x_new(6) += x(7) * dt_;
    return x_new;
  };
  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
          0,   1,   0,   0,   0,   0,   0,   0,   0,
          0,   0,   1,   dt_, 0,   0,   0,   0,   0, 
          0,   0,   0,   1,   0,   0,   0,   0,   0,
          0,   0,   0,   0,   1,   dt_, 0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   dt_, 0,
          0,   0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
    // clang-format on
    return f;
  };
  // h - Observation function
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    z(0) = xc - r * cos(yaw);  // xa
    z(1) = yc - r * sin(yaw);  // ya
    z(2) = x(4);               // za
    z(3) = x(6);               // yaw
    return z;
  };
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(4, 9);
    double yaw = x(6), r = x(8);
    // clang-format off
    //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
    h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
          0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
          0,   0,   0,   0,   1,   0,   0,          0,   0,
          0,   0,   0,   0,   0,   0,   1,          0,   0;
    // clang-format on
    return h;
  };
  // update_Q - process noise covariance matrix
  s2qxyz_max_ = declare_parameter("ekf.sigma2_q_xyz_max", 0.1);
  s2qxyz_min_ = declare_parameter("ekf.sigma2_q_xyz_min", 0.05);
  s2qyaw_max_ = declare_parameter("ekf.sigma2_q_yaw_max", 10.0);
  s2qyaw_min_ = declare_parameter("ekf.sigma2_q_yaw_min", 5.0);
  s2qr_ = declare_parameter("ekf.sigma2_q_r", 80.0);
  auto u_q = [this](const Eigen::VectorXd & x_p) {
    double vx = x_p(1), vy = x_p(3), v_yaw = x_p(7);
    double dx = pow(pow(vx, 2) + pow(vy, 2), 0.5);
    double dy = abs(v_yaw);
    const double position_variance = exp(-dy) * (s2qxyz_max_ - s2qxyz_min_) + s2qxyz_min_;
    const double yaw_variance = exp(-dx) * (s2qyaw_max_ - s2qyaw_min_) + s2qyaw_min_;
    return makeProcessNoiseMatrix(dt_, position_variance, yaw_variance, s2qr_);
  };
  // update_R - measurement noise covariance matrix
  r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05);
  r_xyz_min = declare_parameter("ekf.r_xyz_min", 1e-4);
  r_yaw = declare_parameter("ekf.r_yaw", 0.02);
  if (!std::isfinite(r_xyz_min) || r_xyz_min <= 0.0) {
    throw std::invalid_argument("ekf.r_xyz_min must be a finite, positive variance in m^2");
  }
  auto u_r = [this](const Eigen::VectorXd & z) {
    return makeMeasurementNoiseMatrix(z, r_xyz_factor, r_xyz_min, r_yaw);
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  if (single_armor) {
    // The rotating model's translation Q at zero angular velocity is sigma2_q_xyz_max.
    tracker_->ekf =
      makeSingleArmorFilter([this]() { return dt_; }, s2qxyz_max_, r_xyz_factor, r_xyz_min);
  } else {
    tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
  }

  // Reset tracker service
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/reset", [this](
                        const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
      boundary("reset_enter", std_msgs::msg::Header{});
      tracker_->reset();
      boundary("reset_exit", std_msgs::msg::Header{});
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Tracker reset!");
      return;
    });

  // Change target service
  change_target_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/change", [this](
                         const std_srvs::srv::Trigger::Request::SharedPtr,
                         std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_->requestTargetChange();
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Target change!");
      return;
    });

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  if (diagnostic_boundaries_ && !latency_trace_path_.empty()) {
    // Same MessageEvent, queue size, timeout and subscription; only bracket add().
    tf2_filter_ = std::make_shared<tf2_filter>(
      *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
      this->get_node_clock_interface(), std::chrono::duration<int>(1));
    using Event = message_filters::MessageEvent<auto_aim_interfaces::msg::Armors const>;
    armors_sub_.registerCallback(std::function<void(const Event &)>([this](const Event & event) {
      const auto & msg = event.getMessage();
      boundary("filter_input", msg->header, std::to_string(msg->armors.size()));
      std::string reason;
      // Exact message timestamp; zero timeout probe. Never substitutes latest TF.
      const bool available = tf2_buffer_->canTransform(
        target_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
        rclcpp::Duration(0, 0), &reason);
      boundary(
        "tf_probe", msg->header, std::string(available ? "available " : "unavailable ") + reason);
      boundary("filter_add_begin", msg->header);
      tf2_filter_->add(event);
      boundary("filter_add_return", msg->header);
    }));
  } else {
    tf2_filter_ = std::make_shared<tf2_filter>(
      armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
      this->get_node_clock_interface(), std::chrono::duration<int>(1));
  }
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);
}

void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  const auto & diagnostic_header = armors_msg->header;
  std::string exit_reason = "normal";
  const int exceptions_at_entry = std::uncaught_exceptions();
  boundary("callback_enter_filter_pass", diagnostic_header);
  struct DiagnosticExit
  {
    ArmorTrackerNode * node;
    const std_msgs::msg::Header & header;
    const std::string & reason;
    int exceptions;
    ~DiagnosticExit()
    {
      node->boundary(
        "callback_exit", header, std::uncaught_exceptions() > exceptions ? "exception" : reason);
    }
  } exit_guard{this, diagnostic_header, exit_reason, exceptions_at_entry};
  auto timing_now = []() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
  };
  const int64_t timing_begin = latency_trace_path_.empty() ? 0 : timing_now();
  const int64_t timing_previous = last_time_.nanoseconds();
  if (timing_begin)
    timing_record(
      {rclcpp::Time(armors_msg->header.stamp).nanoseconds(), timing_begin, 0, 0, timing_previous,
       -1, false, ""});
  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      exit_reason = "transform_extrapolation";
      boundary("transform_failure", diagnostic_header, ex.what());
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  const int64_t timing_transformed = latency_trace_path_.empty() ? 0 : timing_now();
  // Filter abnormal armors
  armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
               Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                 max_armor_distance_;
      }),
    armors_msg->armors.end());

  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    if (shouldResetTrackerForDt(dt_, max_dt_)) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid Armors dt=%.9f s (valid range: 0 < dt <= %.6f s); resetting from the current "
        "message",
        dt_, max_dt_);
      tracker_->reset();
      tracker_->init(armors_msg);
      target_msg.tracking = false;
    } else {
      tracker_->update(armors_msg, dt_);

      // Publish Info
      info_msg.position_diff = tracker_->info_position_diff;
      info_msg.yaw_diff = tracker_->info_yaw_diff;
      info_msg.position.x = tracker_->measurement(0);
      info_msg.position.y = tracker_->measurement(1);
      info_msg.position.z = tracker_->measurement(2);
      info_msg.yaw = tracker_->measurement(3);
      info_pub_->publish(info_msg);

      if (tracker_->tracker_state == Tracker::DETECTING) {
        target_msg.tracking = false;
      } else if (
        tracker_->tracker_state == Tracker::TRACKING ||
        tracker_->tracker_state == Tracker::TEMP_LOST) {
        target_msg.tracking = true;
        // Fill target message
        const auto & state = tracker_->target_state;
        target_msg.id = tracker_->tracked_id;
        target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
        target_msg.position.x = state(0);
        target_msg.velocity.x = state(1);
        target_msg.position.y = state(2);
        target_msg.velocity.y = state(3);
        target_msg.position.z = state(4);
        target_msg.velocity.z = state(5);
        target_msg.yaw = state(6);
        target_msg.v_yaw = state(7);
        target_msg.radius_1 = state(8);
        target_msg.radius_2 = tracker_->another_r;
        target_msg.dz = tracker_->dz;
      } else if (tracker_->tracker_state == Tracker::CHANGE_TARGET) {
        target_msg.tracking = false;
      }
    }
  }

  last_time_ = time;

  if (!latency_trace_path_.empty()) {
    timing_record(
      {time.nanoseconds(), timing_begin, timing_transformed, timing_now(), timing_previous,
       static_cast<int>(tracker_->tracker_state), target_msg.tracking, target_msg.id});
  }
  boundary(
    "target_publish_begin", diagnostic_header,
    target_msg.tracking ? "tracking_true" : "tracking_false");
  target_pub_->publish(target_msg);
  boundary(
    "target_publish_return", diagnostic_header,
    target_msg.tracking ? "tracking_true" : "tracking_false");

  publishMarkers(target_msg);
}

void ArmorTrackerNode::boundary(
  const std::string & event, const std_msgs::msg::Header & header, const std::string & detail)
{
  if (!diagnostic_boundaries_ || latency_trace_path_.empty()) return;
  auto clean = [](std::string text) {
    for (auto & c : text)
      if (c == ',' || c == '\n' || c == '\r') c = ';';
    return text;
  };
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::steady_clock::now().time_since_epoch())
                     .count();
  timing_record(
    {rclcpp::Time(header.stamp).nanoseconds(), now, 0, 0, 0, -2, false, "", event,
     clean(header.frame_id), clean(target_frame_), clean(detail)});
}

void ArmorTrackerNode::timing_record(TimingRow row)
{
  std::lock_guard<std::mutex> lock(timing_mutex_);
  if (timing_rows_.size() < 20000)
    timing_rows_.push_back(std::move(row));
  else
    ++timing_dropped_;
}

ArmorTrackerNode::~ArmorTrackerNode()
{
  timing_running_.store(false);
  if (timing_writer_.joinable()) timing_writer_.join();
}

void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double dz = target_msg.dz;

    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = za;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armor_marker_.id = i;
      armor_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armor_marker_);
    }
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;

    armor_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.emplace_back(armor_marker_);
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)
