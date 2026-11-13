#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "armor_tracker/tracker_node.hpp"

namespace rm_auto_aim
{
namespace
{

class MotionModelNodeTest : public testing::TestWithParam<std::string>
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_P(MotionModelNodeTest, PublishesSelectedModelGeometry)
{
  const bool single = GetParam() == "single_armor";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("tracker.motion_model", GetParam()),
     rclcpp::Parameter("tracker.tracking_thres", 2), rclcpp::Parameter("target_frame", "odom")});
  auto tracker = std::make_shared<ArmorTrackerNode>(options);
  auto probe = std::make_shared<rclcpp::Node>("motion_model_probe");
  std::vector<auto_aim_interfaces::msg::Target> received;
  auto subscription = probe->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    [&received](const auto_aim_interfaces::msg::Target & target) { received.push_back(target); });
  auto publisher = probe->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tracker);
  executor.add_node(probe);
  auto spin_for = [&executor](std::chrono::milliseconds duration) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  };
  spin_for(std::chrono::milliseconds(500));
  auto_aim_interfaces::msg::Armors message;
  message.header.frame_id = "odom";
  auto_aim_interfaces::msg::Armor armor;
  armor.number = "3";
  armor.type = "small";
  armor.pose.position.x = 2.0;
  armor.pose.position.y = 0.1;
  armor.pose.position.z = 0.2;
  for (int frame = 0; frame < 10; ++frame) {
    const double yaw = single && frame % 2 ? M_PI : 0.0;
    armor.pose.orientation.z = std::sin(yaw / 2.0);
    armor.pose.orientation.w = std::cos(yaw / 2.0);
    message.armors = {armor};
    message.header.stamp = probe->now();
    publisher->publish(message);
    spin_for(std::chrono::milliseconds(20));
  }
  ASSERT_FALSE(received.empty());
  const auto & target = received.back();
  ASSERT_TRUE(target.tracking);
  EXPECT_EQ(target.armors_num, single ? 1 : 4);
  EXPECT_NEAR(target.position.x, single ? 2.0 : 2.2, 1e-9);
  EXPECT_NEAR(target.position.y, 0.1, 1e-9);
  EXPECT_NEAR(target.position.z, 0.2, 1e-9);
  EXPECT_DOUBLE_EQ(target.radius_1, single ? 0.0 : 0.2);
  EXPECT_DOUBLE_EQ(target.radius_2, single ? 0.0 : 0.2);
  EXPECT_DOUBLE_EQ(target.dz, 0.0);
  EXPECT_NEAR(target.velocity.x, 0.0, 1e-9);
  EXPECT_NEAR(target.yaw, 0.0, 1e-9);
  EXPECT_NEAR(target.v_yaw, 0.0, 1e-9);
  EXPECT_EQ(target.header.frame_id, "odom");
  EXPECT_EQ(target.header.stamp, message.header.stamp);
}

INSTANTIATE_TEST_SUITE_P(Models, MotionModelNodeTest, testing::Values("single_armor", "rotating"));

}  // namespace
}  // namespace rm_auto_aim
