#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "x2_navigation/point_cloud_voxel_filter.hpp"
#include "x2_navigation/timing.hpp"

namespace x2_navigation
{

class LidarCloudThrottle : public rclcpp::Node
{
public:
  LidarCloudThrottle()
  : Node("lidar_cloud_throttle"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/scan_nav/cloud");
    const auto max_rate_hz = declare_parameter<double>("max_rate_hz", 10.0);
    timestamp_offset_sec_ = declare_parameter<double>("timestamp_offset_sec", 0.0);
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    filter_config_.voxel_size = declare_parameter<double>("voxel_size", 0.05);
    filter_config_.min_height = declare_parameter<double>("min_height", -0.45);
    filter_config_.max_height = declare_parameter<double>("max_height", 0.30);
    const auto max_input_points = declare_parameter<std::int64_t>("max_input_points", 40000);

    if (input_topic.empty() || output_topic.empty() || target_frame_.empty()) {
      throw std::invalid_argument("input_topic, output_topic, and target_frame must not be empty");
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
    if (!std::isfinite(tf_timeout_sec_) || tf_timeout_sec_ < 0.0) {
      throw std::invalid_argument("tf_timeout_sec must be finite and non-negative");
    }
    if (!std::isfinite(filter_config_.voxel_size) || filter_config_.voxel_size <= 0.0 ||
      !std::isfinite(filter_config_.min_height) || !std::isfinite(filter_config_.max_height) ||
      filter_config_.min_height > filter_config_.max_height || max_input_points <= 0)
    {
      throw std::invalid_argument(
              "voxel_size and max_input_points must be positive and height bounds must be finite and ordered");
    }

    processing_period_ = *minimum_period;
    timestamp_offset_ = rclcpp::Duration::from_nanoseconds(timestamp_offset->count());
    filter_config_.max_input_points = static_cast<std::size_t>(max_input_points);
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    input_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    processing_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = input_callback_group_;
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, qos,
      std::bind(&LidarCloudThrottle::cloudCallback, this, std::placeholders::_1),
      subscription_options);
    processing_timer_ = create_wall_timer(
      processing_period_, std::bind(&LidarCloudThrottle::processLatestCloud, this),
      processing_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "Processing the newest cloud from %s to %s in %s at %.1f Hz with %.3f m voxels, "
      "at most %ld input points, height [%.2f, %.2f], and a %.3f s timestamp offset",
      input_topic.c_str(), output_topic.c_str(), target_frame_.c_str(), max_rate_hz,
      filter_config_.voxel_size, max_input_points, filter_config_.min_height, filter_config_.max_height,
      timestamp_offset_sec_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
    latest_cloud_ = message;
    ++latest_cloud_sequence_;
  }

  void processLatestCloud()
  {
    sensor_msgs::msg::PointCloud2::SharedPtr message;
    std::uint64_t sequence = 0U;
    {
      std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
      if (!latest_cloud_ || latest_cloud_sequence_ == last_published_sequence_) {
        return;
      }
      message = latest_cloud_;
      sequence = latest_cloud_sequence_;
    }

    const auto processing_start = std::chrono::steady_clock::now();

    auto timestamp = rclcpp::Time(message->header.stamp);
    try {
      timestamp += timestamp_offset_;
    } catch (const std::overflow_error &) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Discarded point cloud because its timestamp and configured offset overflowed");
      return;
    }
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Discarded point cloud without a frame ID");
      return;
    }

    geometry_msgs::msg::TransformStamped target_from_source;
    try {
      target_from_source = tf_buffer_.lookupTransform(
        target_frame_, message->header.frame_id, timestamp,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Discarded point cloud until %s <- %s is available: %s",
        target_frame_.c_str(), message->header.frame_id.c_str(), error.what());
      return;
    }

    auto output_header = message->header;
    output_header.frame_id = target_frame_;
    output_header.stamp = timestamp;
    const auto output = voxel_filter_.filter(
      *message, target_from_source, output_header, filter_config_);
    if (!output) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Discarded point cloud with an unsupported or malformed XYZ layout");
      last_published_sequence_ = sequence;
      return;
    }

    publisher_->publish(output->cloud);
    last_published_sequence_ = sequence;
    const auto processing_time = std::chrono::steady_clock::now() - processing_start;
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published %zu voxels from %zu/%zu input points in %.1f ms",
      output->retained_point_count, output->sampled_point_count, output->input_point_count,
      std::chrono::duration<double, std::milli>(processing_time).count());
  }

  std::chrono::nanoseconds processing_period_{0};
  double timestamp_offset_sec_{0.0};
  rclcpp::Duration timestamp_offset_{0, 0};
  std::string target_frame_;
  double tf_timeout_sec_{0.05};
  PointCloudVoxelFilterConfig filter_config_{0.05, -0.45, 0.30, 40000U};
  PointCloudVoxelFilterWorkspace voxel_filter_;
  std::mutex latest_cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::uint64_t latest_cloud_sequence_{0U};
  std::uint64_t last_published_sequence_{0U};
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr processing_callback_group_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace x2_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<x2_navigation::LidarCloudThrottle>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
