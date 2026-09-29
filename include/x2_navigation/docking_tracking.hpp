#ifndef X2_NAVIGATION__DOCKING_TRACKING_HPP_
#define X2_NAVIGATION__DOCKING_TRACKING_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "x2_navigation/holonomic_fine_align.hpp"

namespace x2_navigation
{

// Predict only to compare/filter a new observation. Never drive from prediction alone.
inline PlanarError predictRelativeError(
  const PlanarError & error, const geometry_msgs::msg::Twist & command, double dt)
{
  const double angle = command.angular.z * dt;
  const double x = error.x - command.linear.x * dt;
  const double y = error.y - command.linear.y * dt;
  return {
    std::cos(angle) * x + std::sin(angle) * y,
    -std::sin(angle) * x + std::cos(angle) * y,
    wrapAngle(error.yaw - angle)};
}

struct TagTrackingConfig
{
  std::size_t acquisition_samples{3};
  double timeout{1.2};
  double filter_time_constant{0.15};
  double position_slack{0.02};
  double angular_slack{0.0523598776};
  double translation_rate{0.15};
  double angular_rate{0.25};
};

class RelativeTagTracker
{
public:
  explicit RelativeTagTracker(TagTrackingConfig config) : config_(config) {}

  void reset()
  {
    filtered_.reset();
    previous_raw_.reset();
    count_ = 0;
  }

  bool observe(
    const PlanarError & raw, double stamp, const geometry_msgs::msg::Twist & command)
  {
    if (!std::isfinite(raw.x) || !std::isfinite(raw.y) || !std::isfinite(raw.yaw) ||
      !std::isfinite(stamp) || (previous_raw_ && stamp <= stamp_))
    {
      return false;
    }
    if (previous_raw_ && stamp - stamp_ > config_.timeout) {
      reset();
    }
    if (previous_raw_) {
      const double dt = stamp - stamp_;
      const double distance = std::hypot(previous_raw_->x, previous_raw_->y);
      // Allow real robot motion, including rotation of the relative translation.
      const double position_limit = config_.position_slack +
        (config_.translation_rate + config_.angular_rate * distance) * dt;
      const double angular_limit = config_.angular_slack + config_.angular_rate * dt;
      if (std::hypot(raw.x - previous_raw_->x, raw.y - previous_raw_->y) > position_limit ||
        std::abs(wrapAngle(raw.yaw - previous_raw_->yaw)) > angular_limit)
      {
        // Reject isolated jumps without moving the reference to the rejected pose.
        return false;
      }
      const auto predicted = predictRelativeError(*filtered_, command, dt);
      const double alpha = config_.filter_time_constant == 0.0 ? 1.0 :
        dt / (config_.filter_time_constant + dt);
      filtered_ = PlanarError{
        predicted.x + alpha * (raw.x - predicted.x),
        predicted.y + alpha * (raw.y - predicted.y),
        wrapAngle(predicted.yaw + alpha * wrapAngle(raw.yaw - predicted.yaw))};
    } else {
      filtered_ = raw;
    }
    previous_raw_ = raw;
    stamp_ = stamp;
    count_ = std::min(count_ + 1, config_.acquisition_samples);
    return true;
  }

  bool ready() const {return filtered_ && count_ >= config_.acquisition_samples;}
  const PlanarError & error() const {return *filtered_;}

private:
  TagTrackingConfig config_;
  std::optional<PlanarError> filtered_, previous_raw_;
  double stamp_{0.0};
  std::size_t count_{0};
};

struct DockingMotionConfig
{
  double position_hysteresis{0.02};
  double yaw_hysteresis{0.034906585};
  std::size_t confirmation_samples{2};
  double direction_rate{1.0};
};

inline double dockingStopTolerance(double tolerance, double hysteresis)
{
  // Keep the entire hysteresis band inside the completion tolerance.
  return std::max(tolerance - hysteresis, tolerance * 0.5);
}

class ConfirmedAxis
{
public:
  int update(double error, double tolerance, double hysteresis, std::size_t confirmations)
  {
    const double magnitude = std::abs(error);
    const int requested = magnitude <= dockingStopTolerance(tolerance, hysteresis) ?
      0 : (error > 0.0 ? 1 : -1);
    if (requested == 0) {
      active_ = pending_ = 0;
      count_ = 0;
    } else if (requested != active_) {
      // Stop before reversing. Every error outside completion can request motion.
      active_ = 0;
      if (magnitude <= tolerance) {
        pending_ = 0;
        count_ = 0;
      } else {
        count_ = requested == pending_ ? count_ + 1 : 1;
        pending_ = requested;
        if (count_ >= confirmations) {
          active_ = requested;
        }
      }
    }
    return active_;
  }

private:
  int active_{0}, pending_{0};
  std::size_t count_{0};
};

class DockingMotionController
{
public:
  DockingMotionController(HolonomicFineAlignConfig controller, DockingMotionConfig motion)
  : controller_(controller), motion_(motion) {}

  std::optional<geometry_msgs::msg::Twist> update(
    const PlanarError & error, double dt, bool new_observation)
  {
    if (!std::isfinite(error.x) || !std::isfinite(error.y) || !std::isfinite(error.yaw)) {
      return std::nullopt;
    }
    if (new_observation) {
      x_active_ = x_.update(
        controller_.allow_reverse_x ? error.x : std::max(0.0, error.x),
        controller_.x_position_tolerance, motion_.position_hysteresis,
        motion_.confirmation_samples);
      y_active_ = y_.update(
        error.y, controller_.y_position_tolerance, motion_.position_hysteresis,
        motion_.confirmation_samples);
      yaw_active_ = yaw_.update(
        error.yaw, controller_.yaw_tolerance, motion_.yaw_hysteresis,
        motion_.confirmation_samples);
      if (std::abs(error.yaw) >= controller_.translation_yaw_stop) {
        translation_enabled_ = false;
        translation_confirmations_ = 0;
      } else if (!translation_enabled_) {
        if (std::abs(error.yaw) < controller_.translation_yaw_stop - motion_.yaw_hysteresis) {
          translation_enabled_ = ++translation_confirmations_ >= motion_.confirmation_samples;
        } else {
          translation_confirmations_ = 0;
        }
      }
    }
    PlanarError selected{
      x_active_ ? error.x : 0.0, y_active_ ? error.y : 0.0, error.yaw};
    // Preserve the original yaw-based speed ceiling even when yaw is in its deadband.
    auto moving_config = controller_;
    moving_config.x_position_tolerance = dockingStopTolerance(
      controller_.x_position_tolerance, motion_.position_hysteresis);
    moving_config.y_position_tolerance = dockingStopTolerance(
      controller_.y_position_tolerance, motion_.position_hysteresis);
    moving_config.yaw_tolerance = dockingStopTolerance(
      controller_.yaw_tolerance, motion_.yaw_hysteresis);
    auto command = holonomicFineAlignCommand(selected, moving_config);
    if (!command) {
      return command;
    }
    if (!yaw_active_) {
      command->angular.z = 0.0;
    }
    if (!translation_enabled_) {
      command->linear.x = command->linear.y = 0.0;
    }
    const double speed = std::hypot(command->linear.x, command->linear.y);
    if (speed > 0.0) {
      if (direction_reconfirmations_ > 0) {
        if (!new_observation || ++direction_reconfirmations_ <= motion_.confirmation_samples) {
          command->linear.x = command->linear.y = 0.0;
          return command;
        }
        direction_reconfirmations_ = 0;
      }
      double direction = std::atan2(command->linear.y, command->linear.x);
      if (direction_) {
        // Stop before a large turn, rather than sweeping through an unsafe direction.
        const double delta = wrapAngle(direction - *direction_);
        if (std::abs(delta) > 1.5707963268) {
          direction_.reset();
          direction_reconfirmations_ = 1;
          command->linear.x = command->linear.y = 0.0;
          return command;
        }
        direction = *direction_ + std::clamp(
          delta, -motion_.direction_rate * dt, motion_.direction_rate * dt);
      }
      direction_ = direction;
      command->linear.x = speed * std::cos(direction);
      command->linear.y = speed * std::sin(direction);
    } else {
      direction_.reset();
      direction_reconfirmations_ = 0;
    }
    return command;
  }

private:
  HolonomicFineAlignConfig controller_;
  DockingMotionConfig motion_;
  ConfirmedAxis x_, y_, yaw_;
  int x_active_{0}, y_active_{0}, yaw_active_{0};
  bool translation_enabled_{false};
  std::size_t translation_confirmations_{0};
  std::size_t direction_reconfirmations_{0};
  std::optional<double> direction_;
};

inline Eigen::Isometry3d planarPose(const PlanarError & error)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = error.x;
  pose.translation().y() = error.y;
  pose.linear() = Eigen::AngleAxisd(error.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

inline Eigen::Isometry3d tagRelativeUndockTarget(
  const Eigen::Isometry3d & initial_to_dock, const Eigen::Isometry3d & current_to_dock,
  double distance)
{
  Eigen::Isometry3d retreat = Eigen::Isometry3d::Identity();
  retreat.translation().x() = -distance;
  return current_to_dock * initial_to_dock.inverse() * retreat;
}

// Visual settling requires a zero command and fresh, consistent relative poses.
// It is not an independent measurement of robot velocity.
class TagPoseSettling
{
public:
  bool update(
    bool valid, double now, std::uint64_t sequence, const PlanarError & observed,
    double duration, double position_spread, double angular_spread)
  {
    if (!valid) {
      started_.reset();
      return false;
    }
    if (!started_) {
      start(now, sequence, observed);
      return false;
    }
    if (sequence <= sequence_) {
      return false;
    }
    sequence_ = sequence;
    if (std::hypot(observed.x - anchor_.x, observed.y - anchor_.y) > position_spread ||
      std::abs(wrapAngle(observed.yaw - anchor_.yaw)) > angular_spread)
    {
      start(now, sequence, observed);
      return false;
    }
    return now - *started_ >= duration;
  }

private:
  void start(double now, std::uint64_t sequence, const PlanarError & observed)
  {
    started_ = now;
    sequence_ = sequence;
    anchor_ = observed;
  }

  std::optional<double> started_;
  std::uint64_t sequence_{0};
  PlanarError anchor_;
};

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__DOCKING_TRACKING_HPP_
