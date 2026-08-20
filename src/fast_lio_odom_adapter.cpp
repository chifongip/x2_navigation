#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "x2_navigation/odometry_transform.hpp"

namespace x2_navigation
{

class FastLioOdomAdapter : public rclcpp::Node
{
public:
  FastLioOdomAdapter()
  : Node("fast_lio_odom_adapter"), tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry_loc");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    tracking_frame_ = declare_parameter<std::string>(
      "tracking_frame", "lidar_imu_chest_front");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 50);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::QoS(50),
      std::bind(&FastLioOdomAdapter::odometryCallback, this, std::placeholders::_1));
  }

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr input)
  {
    if (input->header.frame_id != odom_frame_ ||
      input->child_frame_id != tracking_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring odometry with frames '%s' -> '%s'; expected '%s' -> '%s'",
        input->header.frame_id.c_str(), input->child_frame_id.c_str(),
        odom_frame_.c_str(), tracking_frame_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped base_from_tracking;
    try {
      base_from_tracking = tf_buffer_.lookupTransform(
        base_frame_, tracking_frame_, rclcpp::Time(input->header.stamp),
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Not publishing transformed odometry until %s <- %s is available: %s",
        base_frame_.c_str(), tracking_frame_.c_str(), error.what());
      return;
    }

    const auto output = transformFastLioOdometry(
      *input, base_from_tracking, base_frame_);
    odometry_publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string tracking_frame_;
  std::string base_frame_;
  double tf_timeout_sec_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace x2_navigation

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<x2_navigation::FastLioOdomAdapter>());
  rclcpp::shutdown();
  return 0;
}
