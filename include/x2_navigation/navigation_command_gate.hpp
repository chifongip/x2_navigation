#ifndef X2_NAVIGATION__NAVIGATION_COMMAND_GATE_HPP_
#define X2_NAVIGATION__NAVIGATION_COMMAND_GATE_HPP_

#include <chrono>
#include <optional>

#include <geometry_msgs/msg/twist.hpp>

#include "x2_navigation/velocity_command.hpp"

namespace x2_navigation
{

// Receipt times use a steady clock: smoother publications cannot renew the
// original controller stream's lifetime. Callers serialize access.
class NavigationCommandGate
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Timeout = std::chrono::duration<double>;

  explicit NavigationCommandGate(bool monitor_raw = false)
  : monitor_raw_(monitor_raw) {}

  void updateRaw(
    const geometry_msgs::msg::Twist & command, TimePoint now, Timeout timeout)
  {
    if (!hasFiniteComponents(command) || command == geometry_msgs::msg::Twist{}) {
      clear();
      return;
    }
    if (!fresh(raw_received_at_, now, timeout)) {
      clear();
    }
    raw_received_at_ = now;
  }

  void update(
    const geometry_msgs::msg::Twist & command, TimePoint now, Timeout timeout)
  {
    if (!hasFiniteComponents(command) ||
      (monitor_raw_ && !fresh(raw_received_at_, now, timeout)))
    {
      clear();
      return;
    }
    command_ = command;
    received_at_ = now;
  }

  geometry_msgs::msg::Twist commandAt(TimePoint now, Timeout timeout)
  {
    if (monitor_raw_ && !fresh(raw_received_at_, now, timeout)) {
      clear();
      return geometry_msgs::msg::Twist{};
    }
    if (!fresh(received_at_, now, timeout)) {
      command_ = geometry_msgs::msg::Twist{};
      received_at_.reset();
      return geometry_msgs::msg::Twist{};
    }
    return command_;
  }

private:
  static bool fresh(const std::optional<TimePoint> & received, TimePoint now, Timeout timeout)
  {
    return std::isfinite(timeout.count()) && timeout.count() > 0.0 &&
           received && now >= *received && now - *received <= timeout;
  }

  void clear()
  {
    command_ = geometry_msgs::msg::Twist{};
    received_at_.reset();
    raw_received_at_.reset();
  }

  bool monitor_raw_;
  geometry_msgs::msg::Twist command_;
  std::optional<TimePoint> received_at_, raw_received_at_;
};

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__NAVIGATION_COMMAND_GATE_HPP_
