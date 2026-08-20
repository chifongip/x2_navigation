#include <cerrno>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <zmq.h>

#include "x2_navigation/timing.hpp"
#include "x2_navigation/velocity_command.hpp"

namespace x2_navigation
{

class ZmqVelocityPublisher
{
public:
  explicit ZmqVelocityPublisher(const std::string & endpoint)
  {
    context_ = zmq_ctx_new();
    if (!context_) {
      throw std::runtime_error("Could not create ZMQ context: " + zmqError());
    }

    socket_ = zmq_socket(context_, ZMQ_PUB);
    if (!socket_) {
      const auto error = zmqError();
      close();
      throw std::runtime_error("Could not create ZMQ PUB socket: " + error);
    }

    const int linger = 0;
    const int high_water_mark = 1;
    if (zmq_setsockopt(socket_, ZMQ_LINGER, &linger, sizeof(linger)) != 0 ||
      zmq_setsockopt(socket_, ZMQ_SNDHWM, &high_water_mark, sizeof(high_water_mark)) != 0)
    {
      const auto error = zmqError();
      close();
      throw std::runtime_error("Could not configure ZMQ PUB socket: " + error);
    }

    if (zmq_bind(socket_, endpoint.c_str()) != 0) {
      const auto error = zmqError();
      close();
      throw std::runtime_error(
              "Could not bind velocity ZMQ endpoint " + endpoint + ": " + error);
    }
  }

  ~ZmqVelocityPublisher()
  {
    close();
  }

  ZmqVelocityPublisher(const ZmqVelocityPublisher &) = delete;
  ZmqVelocityPublisher & operator=(const ZmqVelocityPublisher &) = delete;

  bool publish(const VelocityCommand & command)
  {
    const auto payload = velocityCommandJson(command);
    const auto result = zmq_send(socket_, payload.data(), payload.size(), ZMQ_DONTWAIT);
    return result >= 0 || errno == EAGAIN;
  }

private:
  static std::string zmqError()
  {
    return zmq_strerror(errno);
  }

  void close()
  {
    if (socket_) {
      zmq_close(socket_);
      socket_ = nullptr;
    }
    if (context_) {
      zmq_ctx_term(context_);
      context_ = nullptr;
    }
  }

  void * context_{nullptr};
  void * socket_{nullptr};
};

class Nav2ZmqVelocityBridge : public rclcpp::Node
{
public:
  Nav2ZmqVelocityBridge()
  : Node("nav2_zmq_velocity_bridge")
  {
    const auto command_topic = declare_parameter<std::string>("command_topic", "/cmd_vel");
    const auto endpoint = declare_parameter<std::string>("zmq_endpoint", "tcp://*:8558");
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);
    const auto command_timeout_sec = declare_parameter<double>("command_timeout_sec", 0.20);

    if (command_topic.empty()) {
      throw std::invalid_argument("command_topic must not be empty");
    }
    if (endpoint.empty()) {
      throw std::invalid_argument("zmq_endpoint must not be empty");
    }
    const auto publish_period = periodFromRateHz(publish_rate_hz);
    if (!publish_period) {
      throw std::invalid_argument(
              "publish_rate_hz must produce a positive, representable nanosecond period");
    }
    if (!std::isfinite(command_timeout_sec) || command_timeout_sec <= 0.0) {
      throw std::invalid_argument("command_timeout_sec must be finite and positive");
    }

    publisher_ = std::make_unique<ZmqVelocityPublisher>(endpoint);
    command_timeout_ = std::chrono::duration<double>(command_timeout_sec);
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic, rclcpp::QoS(10),
      std::bind(&Nav2ZmqVelocityBridge::commandCallback, this, std::placeholders::_1));

    publish_timer_ = create_wall_timer(
      *publish_period,
      std::bind(&Nav2ZmqVelocityBridge::publishCommand, this));

    RCLCPP_INFO(
      get_logger(), "Forwarding %s to %s at %.1f Hz with a %.3f s watchdog",
      command_topic.c_str(), endpoint.c_str(), publish_rate_hz, command_timeout_sec);
  }

  ~Nav2ZmqVelocityBridge() override
  {
    if (publisher_) {
      publisher_->publish(zeroVelocityCommand());
    }
  }

private:
  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!watchdog_.update(*message, VelocityCommandWatchdog::Clock::now())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Discarded a non-finite velocity command and cleared the velocity output");
    }
  }

  void publishCommand()
  {
    const auto command = watchdog_.commandAt(
      VelocityCommandWatchdog::Clock::now(), command_timeout_);
    if (!publisher_->publish(command)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "ZMQ velocity send failed: %s", zmq_strerror(errno));
    }
  }

  std::unique_ptr<ZmqVelocityPublisher> publisher_;
  VelocityCommandWatchdog watchdog_;
  std::chrono::duration<double> command_timeout_{0.20};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace x2_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<x2_navigation::Nav2ZmqVelocityBridge>());
  rclcpp::shutdown();
  return 0;
}
