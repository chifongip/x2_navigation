#ifndef X2_NAVIGATION__VELOCITY_COMMAND_HPP_
#define X2_NAVIGATION__VELOCITY_COMMAND_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

namespace x2_navigation
{

struct VelocityCommand
{
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
};

inline VelocityCommand zeroVelocityCommand()
{
  return VelocityCommand{};
}

inline bool hasFiniteComponents(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) &&
         std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) &&
         std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) &&
         std::isfinite(twist.angular.z);
}

inline std::optional<VelocityCommand> navigationVelocityCommand(
  const geometry_msgs::msg::Twist & twist)
{
  if (!hasFiniteComponents(twist)) {
    return std::nullopt;
  }

  VelocityCommand command;
  command.linear_x = std::clamp(twist.linear.x, -0.5, 1.0);
  command.linear_y = std::clamp(twist.linear.y, -1.0, 1.0);
  command.angular_z = std::clamp(twist.angular.z, -1.0, 1.0);
  return command;
}

inline std::string velocityCommandJson(const VelocityCommand & command)
{
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::max_digits10)
         << "{\"linear\":{\"x\":" << command.linear_x
         << ",\"y\":" << command.linear_y
         << ",\"z\":" << command.linear_z
         << "},\"angular\":{\"x\":" << command.angular_x
         << ",\"y\":" << command.angular_y
         << ",\"z\":" << command.angular_z << "}}";
  return stream.str();
}

class VelocityCommandWatchdog
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  bool update(const geometry_msgs::msg::Twist & twist, const TimePoint now)
  {
    const auto command = navigationVelocityCommand(twist);
    if (!command) {
      clear();
      return false;
    }

    command_ = *command;
    received_at_ = now;
    return true;
  }

  void clear()
  {
    command_.reset();
    received_at_.reset();
  }

  VelocityCommand commandAt(
    const TimePoint now, const std::chrono::duration<double> timeout) const
  {
    if (!std::isfinite(timeout.count()) || timeout <= timeout.zero() ||
      !command_ || !received_at_ || now - *received_at_ > timeout)
    {
      return zeroVelocityCommand();
    }
    return *command_;
  }

private:
  std::optional<VelocityCommand> command_;
  std::optional<TimePoint> received_at_;
};

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__VELOCITY_COMMAND_HPP_
