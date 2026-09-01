#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

#include <agibot_x2_manipulation_msgs/msg/manipulation_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace x2_navigation
{

class PayloadCloudFilter : public rclcpp::Node
{
public:
  PayloadCloudFilter()
  : Node("payload_cloud_filter")
  {
    min_x_ = declare_parameter("min_x", 0.20);
    max_x_ = declare_parameter("max_x", 0.50);
    min_y_ = declare_parameter("min_y", -0.22);
    max_y_ = declare_parameter("max_y", 0.22);
    min_z_ = declare_parameter("min_z", 0.03);
    max_z_ = declare_parameter("max_z", 0.56);
    const auto input = declare_parameter("input_topic", "/scan_nav/self_filtered_cloud");
    const auto output = declare_parameter("output_topic", "/scan_nav/payload_filtered_cloud");
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, rclcpp::SensorDataQoS());
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS(), std::bind(&PayloadCloudFilter::filter, this, std::placeholders::_1));
    state_sub_ = create_subscription<agibot_x2_manipulation_msgs::msg::ManipulationState>(
      "/manipulation_state", rclcpp::QoS(1).reliable().transient_local(),
      [this](agibot_x2_manipulation_msgs::msg::ManipulationState::SharedPtr message) {
        state_ = message->state;
      });
  }

private:
  static std::optional<std::size_t> fieldOffset(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
  {
    const auto found = std::find_if(cloud.fields.begin(), cloud.fields.end(), [&name](const auto & field) {
      return field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    });
    return found == cloud.fields.end() ? std::nullopt : std::optional<std::size_t>(found->offset);
  }

  void filter(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    using State = agibot_x2_manipulation_msgs::msg::ManipulationState;
    if (state_.load() != State::HOLDING) {
      publisher_->publish(*input);
      return;
    }
    const auto x_offset = fieldOffset(*input, "x");
    const auto y_offset = fieldOffset(*input, "y");
    const auto z_offset = fieldOffset(*input, "z");
    const auto field_fits = [input](const std::optional<std::size_t> & offset) {
        return offset && *offset <= input->point_step &&
               sizeof(float) <= input->point_step - *offset;
      };
    if (!field_fits(x_offset) || !field_fits(y_offset) || !field_fits(z_offset) ||
      input->point_step == 0)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Payload cloud lacks valid float32 x/y/z fields within point_step");
      return;
    }
    sensor_msgs::msg::PointCloud2 output = *input;
    output.height = 1;
    output.width = 0;
    output.row_step = 0;
    output.data.clear();
    output.data.reserve(input->data.size());
    for (std::size_t row = 0; row < input->height; ++row) {
      for (std::size_t column = 0; column < input->width; ++column) {
        const std::size_t offset = row * input->row_step + column * input->point_step;
        if (offset + input->point_step > input->data.size()) {
          continue;
        }
        float x, y, z;
        std::memcpy(&x, input->data.data() + offset + *x_offset, sizeof(float));
        std::memcpy(&y, input->data.data() + offset + *y_offset, sizeof(float));
        std::memcpy(&z, input->data.data() + offset + *z_offset, sizeof(float));
        const bool payload = std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
          x >= min_x_ && x <= max_x_ && y >= min_y_ && y <= max_y_ && z >= min_z_ && z <= max_z_;
        if (!payload) {
          output.data.insert(
            output.data.end(), input->data.begin() + offset,
            input->data.begin() + offset + input->point_step);
          ++output.width;
        }
      }
    }
    output.row_step = output.width * output.point_step;
    publisher_->publish(output);
  }

  std::atomic<uint8_t> state_{agibot_x2_manipulation_msgs::msg::ManipulationState::UNKNOWN};
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<agibot_x2_manipulation_msgs::msg::ManipulationState>::SharedPtr state_sub_;
};

}  // namespace x2_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<x2_navigation::PayloadCloudFilter>());
  rclcpp::shutdown();
  return 0;
}
