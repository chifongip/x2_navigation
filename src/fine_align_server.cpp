#include "x2_navigation/holonomic_fine_align.hpp"
#include "x2_navigation/table_dock_geometry.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <agibot_x2_manipulation_msgs/msg/manipulation_state.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <x2_navigation/action/fine_align.hpp>

namespace x2_navigation
{

using namespace std::chrono_literals;

class FineAlignServer : public rclcpp::Node
{
public:
  using FineAlign = x2_navigation::action::FineAlign;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FineAlign>;

  FineAlignServer()
  : Node("fine_align_server"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    fixed_frame_ = declare_parameter("fixed_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    tag_frame_ = declare_parameter("tag_frame", "tag9");
    tag_id_ = declare_parameter("tag_id", 9);
    minimum_decision_margin_ = declare_parameter("minimum_decision_margin", 20.0);
    standoff_ = declare_parameter("standoff", 0.70);
    lateral_offset_ = declare_parameter("lateral_offset", 0.0);
    yaw_offset_ = declare_parameter("yaw_offset", 0.0);
    maximum_pose_age_ = declare_parameter("maximum_pose_age", 2.5);
    maximum_sample_gap_ = declare_parameter("maximum_sample_gap", 2.5);
    const auto stable_sample_count = declare_parameter("stable_sample_count", 3);
    stable_sample_count_ = static_cast<std::size_t>(std::max(1L, stable_sample_count));
    maximum_position_spread_ = declare_parameter("maximum_position_spread", 0.02);
    maximum_angular_spread_ = declare_parameter("maximum_angular_spread", 0.0523598776);
    capture_distance_ = declare_parameter("capture_distance", 1.5);
    capture_lateral_ = declare_parameter("capture_lateral", 0.30);
    capture_yaw_ = declare_parameter("capture_yaw", 0.5235987756);
    reverse_capture_distance_ = declare_parameter("reverse_capture_distance", 0.15);
    acquisition_timeout_ = declare_parameter("acquisition_timeout", 6.0);
    approach_timeout_ = declare_parameter("approach_timeout", 45.0);
    command_timeout_ = declare_parameter("command_timeout", 0.20);
    collision_stop_timeout_ = declare_parameter("collision_stop_timeout", 1.0);
    odometry_timeout_ = declare_parameter("odometry_timeout", 0.50);
    settled_linear_velocity_ = declare_parameter("settled_linear_velocity", 0.03);
    settled_angular_velocity_ = declare_parameter("settled_angular_velocity", 0.05);
    const auto settled_sample_count = declare_parameter("settled_sample_count", 3);
    settled_sample_count_ = static_cast<std::size_t>(std::max(1L, settled_sample_count));
    const double controller_frequency = declare_parameter("controller_frequency", 20.0);
    progress_log_interval_ = declare_parameter("progress_log_interval", 1.0);

    controller_config_.translation_gain = declare_parameter("translation_gain", 0.5);
    controller_config_.yaw_gain = declare_parameter("yaw_gain", 1.0);
    controller_config_.translation_speed_min = declare_parameter("translation_speed_min", 0.11);
    controller_config_.translation_speed_max = declare_parameter("translation_speed_max", 0.15);
    controller_config_.angular_speed_min = declare_parameter("angular_speed_min", 0.11);
    controller_config_.angular_speed_max = declare_parameter("angular_speed_max", 0.25);
    controller_config_.translation_yaw_stop = declare_parameter(
      "translation_yaw_stop", 0.3490658504);
    controller_config_.x_position_tolerance = declare_parameter("x_position_tolerance", 0.05);
    controller_config_.y_position_tolerance = declare_parameter("y_position_tolerance", 0.05);
    controller_config_.yaw_tolerance = declare_parameter("yaw_tolerance", 0.0872664626);
    controller_config_.allow_reverse_x = declare_parameter("allow_reverse_x", false);

    if (!validHolonomicFineAlignConfig(controller_config_) ||
      !std::isfinite(controller_frequency) || controller_frequency <= 0.0 ||
      !std::isfinite(maximum_pose_age_) || maximum_pose_age_ <= 0.0 ||
      !std::isfinite(reverse_capture_distance_) || reverse_capture_distance_ <= 0.0 ||
      !std::isfinite(progress_log_interval_) || progress_log_interval_ <= 0.0 ||
      !std::isfinite(odometry_timeout_) || odometry_timeout_ <= 0.0 ||
      !std::isfinite(settled_linear_velocity_) || settled_linear_velocity_ < 0.0 ||
      !std::isfinite(settled_angular_velocity_) || settled_angular_velocity_ < 0.0)
    {
      throw std::invalid_argument("invalid fine-align controller configuration");
    }
    controller_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / controller_frequency));

    const auto nav_cmd_topic = declare_parameter("nav_cmd_topic", "/cmd_vel_nav");
    const auto raw_cmd_topic = declare_parameter("raw_cmd_topic", "/cmd_vel_raw");
    const auto odom_topic = declare_parameter("odom_topic", "/odom");
    const auto detections_topic = declare_parameter(
      "detections_topic", "/front_center_rectify/detections");

    detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      detections_topic, rclcpp::SensorDataQoS(),
      std::bind(&FineAlignServer::onDetections, this, std::placeholders::_1));
    state_sub_ = create_subscription<agibot_x2_manipulation_msgs::msg::ManipulationState>(
      "/manipulation_state", rclcpp::QoS(1).reliable().transient_local(),
      [this](agibot_x2_manipulation_msgs::msg::ManipulationState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        manipulation_state_ = message->state;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(odometry_mutex_);
        linear_velocity_ = std::hypot(message->twist.twist.linear.x, message->twist.twist.linear.y);
        angular_velocity_ = std::abs(message->twist.twist.angular.z);
        odometry_received_at_ = std::chrono::steady_clock::now();
      });
    nav_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", 10,
      [this](action_msgs::msg::GoalStatusArray::SharedPtr message) {
        bool active = false;
        for (const auto & status : message->status_list) {
          active = active || status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
            status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
            status.status == action_msgs::msg::GoalStatus::STATUS_CANCELING;
        }
        nav_active_.store(active);
      });
    nav_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_cmd_topic, 10, [this](geometry_msgs::msg::Twist::SharedPtr message) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        nav_command_ = *message;
        nav_command_time_ = std::chrono::steady_clock::now();
      });
    collision_sub_ = create_subscription<nav2_msgs::msg::CollisionMonitorState>(
      "/collision_monitor_state", 10,
      [this](nav2_msgs::msg::CollisionMonitorState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(collision_mutex_);
        if (message->action_type == nav2_msgs::msg::CollisionMonitorState::STOP) {
          if (!collision_stopped_) {
            collision_stop_since_ = std::chrono::steady_clock::now();
          }
          collision_stopped_ = true;
        } else {
          collision_stopped_ = false;
          collision_stop_since_.reset();
        }
      });

    raw_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(raw_cmd_topic, 10);
    mux_timer_ = create_wall_timer(
      controller_period_, std::bind(&FineAlignServer::publishSelectedCommand, this));
    server_ = rclcpp_action::create_server<FineAlign>(
      this, "/fine_align",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FineAlign::Goal>) {
        bool expected = false;
        return operation_active_.compare_exchange_strong(expected, true) ?
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
               rclcpp_action::GoalResponse::REJECT;
      },
      [](std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandle> handle) {
        std::thread(&FineAlignServer::execute, this, std::move(handle)).detach();
      });
  }

private:
  void onDetections(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message)
  {
    const auto found = std::find_if(
      message->detections.begin(), message->detections.end(), [this](const auto & item) {
        return item.id == tag_id_ && item.decision_margin >= minimum_decision_margin_;
      });
    const rclcpp::Time stamp(message->header.stamp);
    if (found == message->detections.end() || stamp.nanoseconds() == 0) {
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        fixed_frame_, tag_frame_, tf2::TimePointZero);
      const rclcpp::Time transform_stamp(transform.header.stamp);
      if (transform_stamp.nanoseconds() == 0 ||
        std::abs((now() - transform_stamp).seconds()) > maximum_pose_age_ ||
        std::abs((stamp - transform_stamp).seconds()) > maximum_pose_age_)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Fine-align rejected incoherent tag TF for detection at %.3f", stamp.seconds());
        return;
      }
      const auto target = tableDockPose(
        tf2::transformToEigen(transform), standoff_, lateral_offset_, yaw_offset_);
      std::lock_guard<std::mutex> lock(measurement_mutex_);
      if (last_sample_stamp_.nanoseconds() != 0 && stamp <= last_sample_stamp_) {
        return;
      }
      if (last_sample_stamp_.nanoseconds() != 0 &&
        (stamp - last_sample_stamp_).seconds() > maximum_sample_gap_)
      {
        samples_.clear();
      }
      last_sample_stamp_ = stamp;
      samples_.push_back(target);
      while (samples_.size() > stable_sample_count_) {
        samples_.pop_front();
      }
      if (samples_.size() < stable_sample_count_) {
        return;
      }
      const double yaw = std::atan2(target.linear()(1, 0), target.linear()(0, 0));
      for (const auto & sample : samples_) {
        const double sample_yaw = std::atan2(sample.linear()(1, 0), sample.linear()(0, 0));
        if ((sample.translation().head<2>() - target.translation().head<2>()).norm() >
          maximum_position_spread_ ||
          std::abs(wrapAngle(sample_yaw - yaw)) > maximum_angular_spread_)
        {
          return;
        }
      }
      stable_target_ = target;
      stable_target_stamp_ = stamp;
      ++stable_target_sequence_;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Fine-align tag transform unavailable: %s", error.what());
    }
  }

  bool validState(uint8_t state) const
  {
    using State = agibot_x2_manipulation_msgs::msg::ManipulationState;
    return state == State::EMPTY || state == State::HOLDING;
  }

  bool stableMeasurement(
    Eigen::Isometry3d & target, uint8_t & state, std::uint64_t & sequence)
  {
    std::lock_guard<std::mutex> lock(measurement_mutex_);
    state = manipulation_state_;
    if (!stable_target_ || stable_target_stamp_.nanoseconds() == 0 ||
      (now() - stable_target_stamp_).seconds() > maximum_pose_age_)
    {
      return false;
    }
    target = *stable_target_;
    sequence = stable_target_sequence_;
    return true;
  }

  bool waitForMeasurement(
    const std::shared_ptr<GoalHandle> & handle, Eigen::Isometry3d & target,
    uint8_t & state, std::uint64_t & sequence)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(acquisition_timeout_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (handle->is_canceling() || nav_active_.load()) {
        return false;
      }
      if (stableMeasurement(target, state, sequence)) {
        return true;
      }
      auto feedback = std::make_shared<FineAlign::Feedback>();
      feedback->stage = FineAlign::Feedback::ACQUIRING;
      feedback->tag_visible = false;
      handle->publish_feedback(feedback);
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  PlanarError currentError(const Eigen::Isometry3d & target)
  {
    const auto transform = tf_buffer_.lookupTransform(
      fixed_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
    return planarError(tf2::transformToEigen(transform), target);
  }

  static geometry_msgs::msg::Pose2D errorMessage(const PlanarError & error)
  {
    geometry_msgs::msg::Pose2D result;
    result.x = error.x;
    result.y = error.y;
    result.theta = error.yaw;
    return result;
  }

  bool insideCaptureEnvelope(const PlanarError & error) const
  {
    const double minimum_x = controller_config_.allow_reverse_x ?
      -reverse_capture_distance_ : -controller_config_.x_position_tolerance;
    return error.x >= minimum_x &&
           std::hypot(error.x, error.y) <= capture_distance_ &&
           std::abs(error.y) <= capture_lateral_ && std::abs(error.yaw) <= capture_yaw_;
  }

  bool odometrySettled() const
  {
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    return odometry_received_at_ &&
           std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *odometry_received_at_).count() <= odometry_timeout_ &&
           linear_velocity_ <= settled_linear_velocity_ &&
           angular_velocity_ <= settled_angular_velocity_;
  }

  void execute(std::shared_ptr<GoalHandle> handle)
  {
    auto result = std::make_shared<FineAlign::Result>();
    result->final_error.x = std::numeric_limits<double>::quiet_NaN();
    result->final_error.y = std::numeric_limits<double>::quiet_NaN();
    result->final_error.theta = std::numeric_limits<double>::quiet_NaN();
    Eigen::Isometry3d target;
    uint8_t state = agibot_x2_manipulation_msgs::msg::ManipulationState::UNKNOWN;
    std::uint64_t sequence = 0;
    result->manipulation_state = state;
    if (nav_active_.load()) {
      finish(handle, result, FineAlign::Result::NAVIGATION_ACTIVE, "Nav2 is active");
      return;
    }
    if (!waitForMeasurement(handle, target, state, sequence)) {
      if (nav_active_.load()) {
        finish(handle, result, FineAlign::Result::NAVIGATION_ACTIVE, "Nav2 became active");
      } else {
        finishCanceledOrFailed(
          handle, result, FineAlign::Result::NO_STABLE_TAG, "no stable 1 Hz tag pose");
      }
      return;
    }
    result->manipulation_state = state;
    if (!validState(state)) {
      finish(
        handle, result, FineAlign::Result::INVALID_STATE,
        "manipulation state is not EMPTY or HOLDING");
      return;
    }

    PlanarError error;
    try {
      error = currentError(target);
      result->final_error = errorMessage(error);
    } catch (const tf2::TransformException & exception) {
      finish(handle, result, FineAlign::Result::SAFETY_ABORT, exception.what());
      return;
    }
    if (!insideCaptureEnvelope(error)) {
      finish(
        handle, result, FineAlign::Result::OUTSIDE_CAPTURE_ENVELOPE,
        "robot is outside the configured fine-align capture envelope");
      return;
    }
    if (!handle->get_goal()->execute) {
      result->success = true;
      result->error_code = FineAlign::Result::SUCCESS;
      result->message = "fine-alignment inputs and capture pose are ready";
      handle->succeed(result);
      operation_active_.store(false);
      return;
    }

    alignment_active_.store(true);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(approach_timeout_);
    const auto progress_log_period = std::max(
      controller_period_, std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(progress_log_interval_)));
    auto next_progress_log = std::chrono::steady_clock::time_point::min();
    std::uint64_t checked_sequence = sequence;
    std::size_t settled_samples = 0;
    while (rclcpp::ok()) {
      if (handle->is_canceling()) {
        finishCanceledOrFailed(
          handle, result, FineAlign::Result::ALIGNMENT_TIMEOUT, "fine alignment canceled");
        return;
      }
      if (nav_active_.load()) {
        finish(handle, result, FineAlign::Result::NAVIGATION_ACTIVE, "Nav2 became active");
        return;
      }
      if (collisionStopTimedOut()) {
        finish(
          handle, result, FineAlign::Result::COLLISION_STOPPED,
          "collision monitor stop persisted");
        return;
      }
      if (std::chrono::steady_clock::now() > deadline) {
        finish(
          handle, result, FineAlign::Result::ALIGNMENT_TIMEOUT, "fine alignment timed out");
        return;
      }
      if (!stableMeasurement(target, state, sequence)) {
        finish(
          handle, result, FineAlign::Result::NO_STABLE_TAG,
          "last stable tag target exceeded the pose-age limit");
        return;
      }
      result->manipulation_state = state;
      if (!validState(state)) {
        finish(
          handle, result, FineAlign::Result::INVALID_STATE,
          "manipulation state became invalid during fine alignment");
        return;
      }
      try {
        error = currentError(target);
        result->final_error = errorMessage(error);
      } catch (const tf2::TransformException & exception) {
        finish(handle, result, FineAlign::Result::SAFETY_ABORT, exception.what());
        return;
      }
      if (!insideCaptureEnvelope(error)) {
        finish(
          handle, result, FineAlign::Result::OUTSIDE_CAPTURE_ENVELOPE,
          "refined target moved outside the configured capture envelope");
        return;
      }
      const auto command = holonomicFineAlignCommand(error, controller_config_);
      if (!command) {
        finish(
          handle, result, FineAlign::Result::SAFETY_ABORT,
          "holonomic controller rejected its input");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        alignment_command_ = *command;
        alignment_command_time_ = std::chrono::steady_clock::now();
      }

      const bool at_goal = fineAlignAtGoal(error, controller_config_);
      const bool odometry_settled = at_goal && odometrySettled();
      if (!at_goal) {
        settled_samples = 0;
      } else if (sequence != checked_sequence) {
        checked_sequence = sequence;
        settled_samples = odometry_settled ? settled_samples + 1 : 0;
      }

      const auto current_steady_time = std::chrono::steady_clock::now();
      if (current_steady_time >= next_progress_log) {
        RCLCPP_INFO(
          get_logger(),
          "Fine-align progress: sequence=%llu; error_base=(x=%.3f m, y=%.3f m, yaw=%.3f rad); "
          "command=(linear.x=%.3f m/s, linear.y=%.3f m/s, angular.z=%.3f rad/s); "
          "stage=%s; odometry_settled=%s; settled_samples=%zu/%zu",
          static_cast<unsigned long long>(sequence), error.x, error.y, error.yaw,
          command->linear.x, command->linear.y, command->angular.z,
          at_goal ? "settling" : "controlling", odometry_settled ? "true" : "false",
          settled_samples, settled_sample_count_);
        next_progress_log = current_steady_time + progress_log_period;
      }

      auto feedback = std::make_shared<FineAlign::Feedback>();
      feedback->stage = at_goal ? FineAlign::Feedback::SETTLING : FineAlign::Feedback::CONTROLLING;
      feedback->current_error = errorMessage(error);
      feedback->tag_visible = true;
      feedback->progress = static_cast<float>(
        1.0 - std::min(1.0, std::hypot(error.x, error.y) / capture_distance_));
      handle->publish_feedback(feedback);

      if (settled_samples >= settled_sample_count_) {
        stopAlignment();
        result->success = true;
        result->error_code = FineAlign::Result::SUCCESS;
        result->message = "table fine alignment succeeded";
        handle->succeed(result);
        operation_active_.store(false);
        return;
      }
      std::this_thread::sleep_for(controller_period_);
    }
    finish(handle, result, FineAlign::Result::SAFETY_ABORT, "ROS shutdown interrupted alignment");
  }

  void stopAlignment()
  {
    alignment_active_.store(false);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      alignment_command_ = geometry_msgs::msg::Twist{};
      alignment_command_time_.reset();
    }
    raw_cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void logAbortDiagnostic(
    const std::shared_ptr<GoalHandle> & handle, const FineAlign::Result & result,
    uint16_t code, const std::string & message)
  {
    constexpr double unavailable = std::numeric_limits<double>::quiet_NaN();
    const auto current_time = now();
    const auto current_steady_time = std::chrono::steady_clock::now();
    bool stable_target_available = false;
    std::uint64_t stable_target_sequence = 0;
    uint8_t current_manipulation_state =
      agibot_x2_manipulation_msgs::msg::ManipulationState::UNKNOWN;
    double stable_target_x = unavailable;
    double stable_target_y = unavailable;
    double stable_target_yaw = unavailable;
    double stable_target_age = unavailable;
    {
      std::lock_guard<std::mutex> lock(measurement_mutex_);
      stable_target_available = stable_target_.has_value();
      stable_target_sequence = stable_target_sequence_;
      current_manipulation_state = manipulation_state_;
      if (stable_target_available) {
        stable_target_x = stable_target_->translation().x();
        stable_target_y = stable_target_->translation().y();
        stable_target_yaw = std::atan2(
          stable_target_->linear()(1, 0), stable_target_->linear()(0, 0));
        stable_target_age = std::abs((current_time - stable_target_stamp_).seconds());
      }
    }

    bool collision_stopped = false;
    double collision_stop_age = unavailable;
    {
      std::lock_guard<std::mutex> lock(collision_mutex_);
      collision_stopped = collision_stopped_;
      if (collision_stop_since_) {
        collision_stop_age = std::chrono::duration<double>(
          current_steady_time - *collision_stop_since_).count();
      }
    }

    bool odometry_available = false;
    double odometry_age = unavailable;
    double linear_velocity = unavailable;
    double angular_velocity = unavailable;
    {
      std::lock_guard<std::mutex> lock(odometry_mutex_);
      odometry_available = odometry_received_at_.has_value();
      if (odometry_available) {
        odometry_age = std::chrono::duration<double>(
          current_steady_time - *odometry_received_at_).count();
        linear_velocity = linear_velocity_;
        angular_velocity = angular_velocity_;
      }
    }

    RCLCPP_ERROR(
      get_logger(),
      "Fine-align action abort: code=%u; reason='%s'; execute=%s; "
      "final_error_base=(x=%.3f m, y=%.3f m, yaw=%.3f rad); "
      "manipulation_state=(result=%u, current=%u); "
      "stable_target_%s=(sequence=%llu, x=%.3f m, y=%.3f m, yaw=%.3f rad, "
      "age=%.3f s, frame=%s); nav_active=%s; "
      "collision_stopped=%s (age=%.3f s); odometry_%s=(age=%.3f s, "
      "linear=%.3f m/s, angular=%.3f rad/s)",
      static_cast<unsigned int>(code), message.c_str(),
      handle->get_goal()->execute ? "true" : "false",
      result.final_error.x, result.final_error.y, result.final_error.theta,
      static_cast<unsigned int>(result.manipulation_state),
      static_cast<unsigned int>(current_manipulation_state),
      stable_target_available ? "available" : "unavailable",
      static_cast<unsigned long long>(stable_target_sequence), stable_target_x, stable_target_y,
      stable_target_yaw, stable_target_age, fixed_frame_.c_str(),
      nav_active_.load() ? "true" : "false", collision_stopped ? "true" : "false",
      collision_stop_age, odometry_available ? "available" : "unavailable", odometry_age,
      linear_velocity, angular_velocity);
  }

  void finish(
    const std::shared_ptr<GoalHandle> & handle, const std::shared_ptr<FineAlign::Result> & result,
    uint16_t code, const std::string & message)
  {
    result->success = false;
    result->error_code = code;
    result->message = message;
    logAbortDiagnostic(handle, *result, code, message);
    stopAlignment();
    handle->abort(result);
    operation_active_.store(false);
  }

  void finishCanceledOrFailed(
    const std::shared_ptr<GoalHandle> & handle, const std::shared_ptr<FineAlign::Result> & result,
    uint16_t code, const std::string & message)
  {
    if (handle->is_canceling()) {
      stopAlignment();
      result->success = false;
      result->error_code = code;
      result->message = "fine alignment canceled";
      handle->canceled(result);
      operation_active_.store(false);
      return;
    }
    finish(handle, result, code, message);
  }

  void publishSelectedCommand()
  {
    geometry_msgs::msg::Twist command;
    const auto now_steady = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (operation_active_.load()) {
      if (!nav_active_.load() && alignment_active_.load() && alignment_command_time_ &&
        std::chrono::duration<double>(now_steady - *alignment_command_time_).count() <=
        command_timeout_)
      {
        command = alignment_command_;
      }
    } else if (nav_command_time_ &&
      std::chrono::duration<double>(now_steady - *nav_command_time_).count() <= command_timeout_)
    {
      command = nav_command_;
    }
    raw_cmd_pub_->publish(command);
  }

  bool collisionStopTimedOut()
  {
    std::lock_guard<std::mutex> lock(collision_mutex_);
    return collision_stopped_ && collision_stop_since_ &&
           std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *collision_stop_since_).count() > collision_stop_timeout_;
  }

  rclcpp_action::Server<FineAlign>::SharedPtr server_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<agibot_x2_manipulation_msgs::msg::ManipulationState>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
  rclcpp::Subscription<nav2_msgs::msg::CollisionMonitorState>::SharedPtr collision_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_pub_;
  rclcpp::TimerBase::SharedPtr mux_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  mutable std::mutex measurement_mutex_, command_mutex_, collision_mutex_, odometry_mutex_;
  std::deque<Eigen::Isometry3d> samples_;
  std::optional<Eigen::Isometry3d> stable_target_;
  rclcpp::Time last_sample_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stable_target_stamp_{0, 0, RCL_ROS_TIME};
  std::uint64_t stable_target_sequence_{0};
  uint8_t manipulation_state_{agibot_x2_manipulation_msgs::msg::ManipulationState::UNKNOWN};
  geometry_msgs::msg::Twist nav_command_, alignment_command_;
  std::optional<std::chrono::steady_clock::time_point> nav_command_time_;
  std::optional<std::chrono::steady_clock::time_point> alignment_command_time_;
  std::optional<std::chrono::steady_clock::time_point> collision_stop_since_;
  std::optional<std::chrono::steady_clock::time_point> odometry_received_at_;
  std::atomic_bool operation_active_{false}, alignment_active_{false}, nav_active_{false};
  bool collision_stopped_{false};
  std::string fixed_frame_, base_frame_, tag_frame_;
  int tag_id_{9};
  std::size_t stable_sample_count_{3}, settled_sample_count_{3};
  HolonomicFineAlignConfig controller_config_;
  std::chrono::nanoseconds controller_period_{50ms};
  double minimum_decision_margin_, standoff_, lateral_offset_, yaw_offset_;
  double maximum_pose_age_, maximum_sample_gap_, maximum_position_spread_, maximum_angular_spread_;
  double capture_distance_, capture_lateral_, capture_yaw_, reverse_capture_distance_;
  double acquisition_timeout_, approach_timeout_, command_timeout_, collision_stop_timeout_;
  double odometry_timeout_, settled_linear_velocity_, settled_angular_velocity_, progress_log_interval_;
  double linear_velocity_{0.0}, angular_velocity_{0.0};
};

}  // namespace x2_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<x2_navigation::FineAlignServer>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
