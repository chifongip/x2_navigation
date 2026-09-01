#ifndef X2_NAVIGATION__HOLONOMIC_FINE_ALIGN_HPP_
#define X2_NAVIGATION__HOLONOMIC_FINE_ALIGN_HPP_

#include <algorithm>
#include <cmath>
#include <optional>

#include <geometry_msgs/msg/twist.hpp>

#include "x2_navigation/table_dock_geometry.hpp"

namespace x2_navigation
{

struct HolonomicFineAlignConfig
{
  double translation_gain{0.5};
  double yaw_gain{1.0};
  double translation_speed_min{0.11};
  double translation_speed_max{0.15};
  double angular_speed_min{0.11};
  double angular_speed_max{0.25};
  double translation_yaw_stop{0.3490658504};
  double x_position_tolerance{0.05};
  double y_position_tolerance{0.05};
  double yaw_tolerance{0.0872664626};
  bool allow_reverse_x{false};
};

inline bool validHolonomicFineAlignConfig(const HolonomicFineAlignConfig & config)
{
  return std::isfinite(config.translation_gain) && config.translation_gain > 0.0 &&
         std::isfinite(config.yaw_gain) && config.yaw_gain > 0.0 &&
         std::isfinite(config.translation_speed_min) &&
         std::isfinite(config.translation_speed_max) &&
         config.translation_speed_min > 0.0 &&
         config.translation_speed_max >= config.translation_speed_min &&
         std::isfinite(config.angular_speed_min) &&
         std::isfinite(config.angular_speed_max) && config.angular_speed_min > 0.0 &&
         config.angular_speed_max >= config.angular_speed_min &&
         std::isfinite(config.translation_yaw_stop) && config.translation_yaw_stop > 0.0 &&
         std::isfinite(config.x_position_tolerance) && config.x_position_tolerance > 0.0 &&
         std::isfinite(config.y_position_tolerance) && config.y_position_tolerance > 0.0 &&
         std::isfinite(config.yaw_tolerance) && config.yaw_tolerance > 0.0;
}

inline bool fineAlignAtGoal(
  const PlanarError & error, const HolonomicFineAlignConfig & config)
{
  return std::isfinite(error.x) && std::isfinite(error.y) && std::isfinite(error.yaw) &&
         std::abs(error.x) <= config.x_position_tolerance &&
         std::abs(error.y) <= config.y_position_tolerance &&
         std::abs(error.yaw) <= config.yaw_tolerance;
}

inline std::optional<geometry_msgs::msg::Twist> holonomicFineAlignCommand(
  const PlanarError & error, const HolonomicFineAlignConfig & config)
{
  if (!validHolonomicFineAlignConfig(config) || !std::isfinite(error.x) ||
    !std::isfinite(error.y) || !std::isfinite(error.yaw))
  {
    return std::nullopt;
  }

  geometry_msgs::msg::Twist command;
  const double x_error = std::abs(error.x) > config.x_position_tolerance ? error.x : 0.0;
  const double y_error = std::abs(error.y) > config.y_position_tolerance ? error.y : 0.0;
  const double command_x = config.allow_reverse_x ? x_error : std::max(0.0, x_error);
  const double command_y = y_error;
  const double command_distance = std::hypot(command_x, command_y);
  const double yaw_error = std::abs(error.yaw);

  if (command_distance > 0.0 && yaw_error < config.translation_yaw_stop)
  {
    const double yaw_fraction = yaw_error / config.translation_yaw_stop;
    const double yaw_limited_maximum = config.translation_speed_min +
      (config.translation_speed_max - config.translation_speed_min) * (1.0 - yaw_fraction);
    const double speed = std::clamp(
      config.translation_gain * command_distance,
      config.translation_speed_min, yaw_limited_maximum);
    command.linear.x = speed * command_x / command_distance;
    command.linear.y = speed * command_y / command_distance;
  }

  if (yaw_error > config.yaw_tolerance) {
    command.angular.z = std::copysign(
      std::clamp(
        config.yaw_gain * yaw_error, config.angular_speed_min, config.angular_speed_max),
      error.yaw);
  }
  return command;
}

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__HOLONOMIC_FINE_ALIGN_HPP_
