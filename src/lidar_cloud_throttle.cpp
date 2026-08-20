#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "x2_navigation/timing.hpp"

namespace x2_navigation
{

class LidarCloudThrottle : public rclcpp::Node
{
public:
  LidarCloudThrottle()
  : Node("lidar_cloud_throttle")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/scan_nav/cloud");
    const auto max_rate_hz = declare_parameter<double>("max_rate_hz", 5.0);
    timestamp_offset_sec_ = declare_parameter<double>("timestamp_offset_sec", 0.0);

    if (input_topic.empty() || output_topic.empty()) {
      throw std::invalid_argument("input_topic and output_topic must not be empty");
    }
    const auto minimum_period = periodFromRateHz(max_rate_hz);
    if (!minimum_period) {
      throw std::invalid_argument(
              "max_rate_hz must produce a positive, representable nanosecond period");
    }
    const auto timestamp_offset = nanosecondsFromSeconds(timestamp_offset_sec_);
    if (!timestamp_offset) {
      throw std::invalid_argument(
              "timestamp_offset_sec must be finite and representable in nanoseconds");
    }

    minimum_period_ = *minimum_period;
    timestamp_offset_ = rclcpp::Duration::from_nanoseconds(timestamp_offset->count());
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, qos,
      std::bind(&LidarCloudThrottle::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Forwarding %s to %s at up to %.1f Hz with a %.3f s timestamp offset",
      input_topic.c_str(), output_topic.c_str(), max_rate_hz, timestamp_offset_sec_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_publish_time_ != std::chrono::steady_clock::time_point{} &&
      now - last_publish_time_ < minimum_period_)
    {
      return;
    }

    auto output = *message;
    auto timestamp = rclcpp::Time(output.header.stamp);
    try {
      timestamp += timestamp_offset_;
    } catch (const std::overflow_error &) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Discarded point cloud because its timestamp and configured offset overflowed");
      return;
    }
    output.header.stamp = timestamp;
    publisher_->publish(output);
    last_publish_time_ = now;
  }

  std::chrono::nanoseconds minimum_period_{0};
  double timestamp_offset_sec_{0.0};
  rclcpp::Duration timestamp_offset_{0, 0};
  std::chrono::steady_clock::time_point last_publish_time_{};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace x2_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<x2_navigation::LidarCloudThrottle>());
  rclcpp::shutdown();
  return 0;
}
