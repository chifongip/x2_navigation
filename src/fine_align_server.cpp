#include "x2_navigation/docking_tracking.hpp"
#include "x2_navigation/docking_profiles.hpp"
#include "x2_navigation/navigation_command_gate.hpp"
#include "x2_navigation/table_dock_geometry.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <agibot_x2_manipulation_msgs/msg/manipulation_state.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <x2_navigation/action/fine_align.hpp>
#include <x2_navigation/action/undock.hpp>

namespace x2_navigation
{

using namespace std::chrono_literals;

class FineAlignServer : public rclcpp::Node
{
public:
  using FineAlign = x2_navigation::action::FineAlign;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FineAlign>;
  using Undock = x2_navigation::action::Undock;
  using UndockGoalHandle = rclcpp_action::ServerGoalHandle<Undock>;

  FineAlignServer()
  : Node("fine_align_server"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter("base_frame", "base_link");
    rcl_interfaces::msg::ParameterDescriptor profile_descriptor;
    profile_descriptor.read_only = true;
    const auto tag_frame = declare_parameter("tag_frame", "tag9", profile_descriptor);
    const auto tag_id = declare_parameter("tag_id", 9, profile_descriptor);
    minimum_decision_margin_ = declare_parameter("minimum_decision_margin", 20.0);
    const auto standoff = declare_parameter("standoff", 0.70, profile_descriptor);
    const auto lateral_offset = declare_parameter("lateral_offset", 0.0, profile_descriptor);
    const auto yaw_offset = declare_parameter("yaw_offset", 0.0, profile_descriptor);
    profiles_.add({"default", tag_id, tag_frame, standoff, lateral_offset, yaw_offset});
    const auto profile_names = declare_parameter<std::vector<std::string>>(
      "docking_profile_names", std::vector<std::string>{}, profile_descriptor);
    std::set<std::string> profile_names_seen{"default"};
    for (const auto & name : profile_names) {
      if (!profile_names_seen.insert(name).second) {
        throw std::invalid_argument("duplicate docking profile: " + name);
      }
      if (name.empty() || name.find_first_not_of(
          "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos)
      {
        throw std::invalid_argument("invalid docking profile name: " + name);
      }
      const auto prefix = "docking_profiles." + name + ".";
      const auto required = [this, &prefix, &profile_descriptor](
        const std::string & field, rclcpp::ParameterType type) {
          const auto value = declare_parameter(prefix + field, type, profile_descriptor);
          if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            throw std::invalid_argument("missing docking profile parameter: " + prefix + field);
          }
          return value;
        };
      profiles_.add({
        name, required("tag_id", rclcpp::ParameterType::PARAMETER_INTEGER).get<std::int64_t>(),
        required("tag_frame", rclcpp::ParameterType::PARAMETER_STRING).get<std::string>(),
        required("standoff", rclcpp::ParameterType::PARAMETER_DOUBLE).get<double>(),
        required("lateral_offset", rclcpp::ParameterType::PARAMETER_DOUBLE).get<double>(),
        required("yaw_offset", rclcpp::ParameterType::PARAMETER_DOUBLE).get<double>()});
    }
    default_profile_ = declare_parameter("default_docking_profile", "default", profile_descriptor);
    active_profile_ = profiles_.resolve("", default_profile_);
    maximum_pose_age_ = declare_parameter("maximum_pose_age", 2.5);
    declare_parameter("maximum_sample_gap", 2.5);  // Legacy parameter; tracking_timeout replaces it.
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
    const auto maximum_retries = declare_parameter("maximum_retries", 2);
    if (maximum_retries < 0 || maximum_retries > 10) {
      throw std::invalid_argument("maximum_retries must be between 0 and 10");
    }
    maximum_retries_ = static_cast<std::size_t>(maximum_retries);
    retry_delay_ = declare_parameter("retry_delay", 1.0);
    command_timeout_ = declare_parameter("command_timeout", 0.20);
    collision_stop_timeout_ = declare_parameter("collision_stop_timeout", 1.0);
    const auto settled_sample_count = declare_parameter("settled_sample_count", 3);
    if (settled_sample_count < 1) {
      throw std::invalid_argument("settled_sample_count must be positive");
    }
    settling_duration_ = declare_parameter("settling_duration", 0.5);
    settling_position_spread_ = declare_parameter("settling_position_spread", 0.02);
    settling_angular_spread_ = declare_parameter("settling_angular_spread", 0.034906585);
    tracking_timeout_ = declare_parameter("tracking_timeout", 1.2);
    filter_time_constant_ = declare_parameter("tag_filter_time_constant", 0.15);
    motion_config_.position_hysteresis = declare_parameter("position_hysteresis", 0.02);
    motion_config_.yaw_hysteresis = declare_parameter("yaw_hysteresis", 0.034906585);
    const auto confirmations = declare_parameter("motion_confirmation_samples", 2);
    if (confirmations < 1 || confirmations > 20) {
      throw std::invalid_argument("motion_confirmation_samples must be between 1 and 20");
    }
    motion_config_.confirmation_samples = static_cast<std::size_t>(confirmations);
    motion_config_.direction_rate = declare_parameter("direction_change_rate", 1.0);
    const double controller_frequency = declare_parameter("controller_frequency", 20.0);
    progress_log_interval_ = declare_parameter("progress_log_interval", 1.0);
    undock_distance_ = declare_parameter("undock_distance", 0.30);
    undock_timeout_ = declare_parameter("undock_timeout", 10.0);

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

    undock_controller_config_ = controller_config_;
    undock_controller_config_.translation_speed_min = declare_parameter(
      "undock_translation_speed_min", 0.10);
    undock_controller_config_.translation_speed_max = declare_parameter(
      "undock_translation_speed_max", 0.10);
    undock_controller_config_.angular_speed_min = declare_parameter(
      "undock_angular_speed_min", 0.10);
    undock_controller_config_.angular_speed_max = declare_parameter(
      "undock_angular_speed_max", 0.10);
    undock_controller_config_.allow_reverse_x = true;

    if (!validHolonomicFineAlignConfig(controller_config_) ||
      !validHolonomicFineAlignConfig(undock_controller_config_) ||
      !std::isfinite(controller_frequency) || controller_frequency <= 0.0 ||
      !std::isfinite(maximum_pose_age_) || maximum_pose_age_ <= 0.0 ||
      !std::isfinite(reverse_capture_distance_) || reverse_capture_distance_ <= 0.0 ||
      !std::isfinite(retry_delay_) || retry_delay_ < 0.0 ||
      !std::isfinite(progress_log_interval_) || progress_log_interval_ <= 0.0 ||
      !std::isfinite(undock_distance_) || undock_distance_ <= 0.0 ||
      undock_controller_config_.translation_speed_max > 0.5 ||
      undock_controller_config_.angular_speed_max > 1.0 ||
      !std::isfinite(undock_timeout_) || undock_timeout_ <= 0.0 ||
      !std::isfinite(settling_position_spread_) || settling_position_spread_ <= 0.0 ||
      !std::isfinite(settling_angular_spread_) || settling_angular_spread_ <= 0.0 ||
      !std::isfinite(settling_duration_) || settling_duration_ <= 0.0 ||
      !std::isfinite(tracking_timeout_) || tracking_timeout_ <= 0.0 ||
      !std::isfinite(filter_time_constant_) || filter_time_constant_ < 0.0 ||
      !std::isfinite(maximum_position_spread_) || maximum_position_spread_ <= 0.0 ||
      !std::isfinite(maximum_angular_spread_) || maximum_angular_spread_ <= 0.0 ||
      !std::isfinite(motion_config_.position_hysteresis) || motion_config_.position_hysteresis < 0.0 ||
      !std::isfinite(motion_config_.yaw_hysteresis) || motion_config_.yaw_hysteresis < 0.0 ||
      motion_config_.yaw_hysteresis >= controller_config_.translation_yaw_stop ||
      !std::isfinite(motion_config_.direction_rate) || motion_config_.direction_rate <= 0.0)
    {
      throw std::invalid_argument("invalid fine-align or undock controller configuration");
    }
    tracker_ = std::make_unique<RelativeTagTracker>(TagTrackingConfig{
      stable_sample_count_, std::min(tracking_timeout_, maximum_pose_age_), filter_time_constant_,
      maximum_position_spread_, maximum_angular_spread_,
      std::max(controller_config_.translation_speed_max, undock_controller_config_.translation_speed_max),
      std::max(controller_config_.angular_speed_max, undock_controller_config_.angular_speed_max)});
    RCLCPP_INFO(
      get_logger(),
      "Docking uses timestamped %s <- %s; tracking_timeout=%.3f s; settling_duration=%.3f s; "
      "translation=(min=%.3f, max=%.3f m/s); angular=(min=%.3f, max=%.3f rad/s). "
      "settled_sample_count is retained for compatibility; completion uses settling_duration.",
      base_frame_.c_str(), active_profile_.tag_frame.c_str(), tracking_timeout_, settling_duration_,
      controller_config_.translation_speed_min, controller_config_.translation_speed_max,
      controller_config_.angular_speed_min, controller_config_.angular_speed_max);
    controller_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / controller_frequency));

    const auto nav_cmd_topic = declare_parameter("nav_cmd_topic", "/cmd_vel_nav");
    const auto nav_raw_cmd_topic = declare_parameter("nav_raw_cmd_topic", "");
    nav_command_gate_ = NavigationCommandGate(!nav_raw_cmd_topic.empty());
    const auto raw_cmd_topic = declare_parameter("raw_cmd_topic", "/cmd_vel_raw");
    const auto detections_topic = declare_parameter(
      "detections_topic", "/front_center_rectify/detections");

    detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      detections_topic, rclcpp::SensorDataQoS(),
      [this](apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message) {
        std::lock_guard<std::mutex> lock(pending_detection_mutex_);
        pending_detection_ = message;
      });
    state_sub_ = create_subscription<agibot_x2_manipulation_msgs::msg::ManipulationState>(
      "/manipulation_state", rclcpp::QoS(1).reliable().transient_local(),
      [this](agibot_x2_manipulation_msgs::msg::ManipulationState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        manipulation_state_ = message->state;
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
        nav_command_gate_.update(
          *message, std::chrono::steady_clock::now(),
          std::chrono::duration<double>(command_timeout_));
      });
    if (!nav_raw_cmd_topic.empty()) {
      nav_raw_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        nav_raw_cmd_topic, rclcpp::QoS(1),
        [this](geometry_msgs::msg::Twist::SharedPtr message) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          nav_command_gate_.updateRaw(
            *message, std::chrono::steady_clock::now(),
            std::chrono::duration<double>(command_timeout_));
        });
    }
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
    undock_server_ = rclcpp_action::create_server<Undock>(
      this, "/undock",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Undock::Goal>) {
        bool expected = false;
        return operation_active_.compare_exchange_strong(expected, true) ?
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
               rclcpp_action::GoalResponse::REJECT;
      },
      [](std::shared_ptr<UndockGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<UndockGoalHandle> handle) {
        std::thread(&FineAlignServer::executeUndock, this, std::move(handle)).detach();
      });
  }

private:
  struct AttemptFailure
  {
    uint16_t code;
    std::string message;
  };

  bool activateProfile(const std::string & requested, bool undocking, std::string & resolved)
  {
    std::scoped_lock lock(measurement_mutex_, pending_detection_mutex_);
    try {
      active_profile_ = profiles_.resolve(
        requested, default_profile_, undocking ? last_docked_profile_ : "");
    } catch (const std::invalid_argument &) {
      return false;
    }
    resolved = active_profile_.name;
    ++profile_generation_;
    activation_stamp_ = now();
    pending_detection_.reset();
    tracker_->reset();
    stable_target_.reset();
    raw_error_ = PlanarError{};
    last_sample_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    stable_target_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    RCLCPP_INFO(
      get_logger(), "Docking acquisition: profile=%s; tag_id=%lld; tag_frame=%s; "
      "standoff=%.3f m; lateral_offset=%.3f m; yaw_offset=%.3f rad",
      resolved.c_str(), static_cast<long long>(active_profile_.tag_id),
      active_profile_.tag_frame.c_str(), active_profile_.standoff,
      active_profile_.lateral_offset, active_profile_.yaw_offset);
    return true;
  }

  void logProfileContext(const char * event)
  {
    std::lock_guard<std::mutex> lock(measurement_mutex_);
    RCLCPP_INFO(
      get_logger(), "%s profile=%s; tag_id=%lld; tag_frame=%s", event,
      active_profile_.name.c_str(), static_cast<long long>(active_profile_.tag_id),
      active_profile_.tag_frame.c_str());
  }

  void processPendingDetection()
  {
    DockingProfile profile;
    std::uint64_t generation;
    rclcpp::Time activation_stamp(0, 0, RCL_ROS_TIME);
    {
      std::lock_guard<std::mutex> lock(measurement_mutex_);
      profile = active_profile_;
      generation = profile_generation_;
      activation_stamp = activation_stamp_;
    }
    apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message;
    {
      std::lock_guard<std::mutex> lock(pending_detection_mutex_);
      message = pending_detection_;
    }
    if (!message) {
      return;
    }
    const auto found = std::find_if(
      message->detections.begin(), message->detections.end(), [this, &profile](const auto & item) {
        return item.id == profile.tag_id && std::isfinite(item.decision_margin) &&
               item.decision_margin >= minimum_decision_margin_;
      });
    const rclcpp::Time stamp(message->header.stamp);
    if (found == message->detections.end() || stamp.nanoseconds() == 0 ||
      stamp < activation_stamp)
    {
      return;
    }
    const double age = (now() - stamp).seconds();
    if (age < 0.0 || age > std::min(tracking_timeout_, maximum_pose_age_)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(measurement_mutex_);
      if (generation != profile_generation_ ||
        (last_sample_stamp_.nanoseconds() != 0 && stamp <= last_sample_stamp_))
      {
        return;
      }
    }
    try {
      // Retry queued detections on the timer if their matching TF has not arrived yet.
      const auto transform = tf_buffer_.lookupTransform(base_frame_, profile.tag_frame, stamp);
      const auto target = tableDockPose(
        tf2::transformToEigen(transform), profile.standoff, profile.lateral_offset, profile.yaw_offset);
      const auto raw = planarError(Eigen::Isometry3d::Identity(), target);
      geometry_msgs::msg::Twist command;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (alignment_active_.load()) {
          command = alignment_command_;
        }
      }
      {
        std::lock_guard<std::mutex> lock(collision_mutex_);
        if (collision_stopped_) {
          command = geometry_msgs::msg::Twist{};
        }
      }
      std::lock_guard<std::mutex> lock(measurement_mutex_);
      if (generation != profile_generation_ ||
        (last_sample_stamp_.nanoseconds() != 0 && stamp <= last_sample_stamp_))
      {
        return;
      }
      last_sample_stamp_ = stamp;
      if (!tracker_->observe(raw, stamp.seconds(), command)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Fine-align rejected tag jump: raw_error=(%.3f, %.3f, %.3f)", raw.x, raw.y, raw.yaw);
        return;
      }
      if (!tracker_->ready()) {
        stable_target_.reset();
        return;
      }
      const auto filtered = tracker_->error();
      const auto filtered_target = planarPose(filtered);
      stable_target_ = filtered_target;
      raw_error_ = raw;
      stable_target_stamp_ = stamp;
      ++stable_target_sequence_;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Fine-align tag transform unavailable: %s", error.what());
    } catch (const std::exception & error) {
      {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        if (generation != profile_generation_) {
          return;
        }
        tracker_->reset();
        stable_target_.reset();
        last_sample_stamp_ = stamp;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Fine-align rejected invalid tag measurement: %s", error.what());
    }
  }

  bool validState(uint8_t state) const
  {
    using State = agibot_x2_manipulation_msgs::msg::ManipulationState;
    return state == State::EMPTY || state == State::HOLDING;
  }

  bool stableMeasurement(
    Eigen::Isometry3d & target, uint8_t & state, std::uint64_t & sequence,
    PlanarError * raw_error = nullptr, double * observation_age = nullptr)
  {
    std::lock_guard<std::mutex> lock(measurement_mutex_);
    state = manipulation_state_;
    if (!stable_target_ || stable_target_stamp_.nanoseconds() == 0 ||
      (now() - stable_target_stamp_).seconds() < 0.0 ||
      (now() - stable_target_stamp_).seconds() > std::min(tracking_timeout_, maximum_pose_age_))
    {
      return false;
    }
    target = *stable_target_;
    sequence = stable_target_sequence_;
    if (raw_error) {
      *raw_error = raw_error_;
    }
    if (observation_age) {
      *observation_age = (now() - stable_target_stamp_).seconds();
    }
    return true;
  }

  bool waitForMeasurement(
    const std::shared_ptr<GoalHandle> & handle, Eigen::Isometry3d & target,
    uint8_t & state, std::uint64_t & sequence, std::uint64_t minimum_sequence,
    uint16_t feedback_stage)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(acquisition_timeout_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (handle->is_canceling() || nav_active_.load()) {
        return false;
      }
      const bool target_available = stableMeasurement(target, state, sequence);
      if (target_available && sequence > minimum_sequence) {
        return true;
      }
      auto feedback = std::make_shared<FineAlign::Feedback>();
      feedback->profile_id = handle->get_goal()->profile_id.empty() ?
        default_profile_ : handle->get_goal()->profile_id;
      feedback->stage = feedback_stage;
      feedback->tag_visible = target_available;
      handle->publish_feedback(feedback);
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  std::uint64_t latestStableTargetSequence() const
  {
    std::lock_guard<std::mutex> lock(measurement_mutex_);
    return stable_target_sequence_;
  }

  PlanarError currentError(const Eigen::Isometry3d & target)
  {
    return planarError(Eigen::Isometry3d::Identity(), target);
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

  static bool retryableFailure(uint16_t code)
  {
    return code == FineAlign::Result::NO_STABLE_TAG ||
           code == FineAlign::Result::OUTSIDE_CAPTURE_ENVELOPE ||
           code == FineAlign::Result::DOCKING_FAILED ||
           code == FineAlign::Result::ALIGNMENT_TIMEOUT;
  }

  bool waitForRetryDelay(
    const std::shared_ptr<GoalHandle> & handle, const FineAlign::Result & result)
  {
    auto feedback = std::make_shared<FineAlign::Feedback>();
    feedback->profile_id = result.profile_id;
    feedback->stage = FineAlign::Feedback::REACQUIRING;
    feedback->current_error = result.final_error;
    feedback->tag_visible = false;
    feedback->progress = 0.0F;
    handle->publish_feedback(feedback);

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(retry_delay_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (handle->is_canceling() || nav_active_.load()) {
        return false;
      }
      std::this_thread::sleep_for(100ms);
    }
    return rclcpp::ok();
  }

  std::optional<AttemptFailure> runAttempt(
    const std::shared_ptr<GoalHandle> & handle, const std::shared_ptr<FineAlign::Result> & result,
    std::uint64_t minimum_sequence, uint16_t acquisition_stage, std::size_t attempt,
    std::size_t maximum_attempts)
  {
    Eigen::Isometry3d target;
    uint8_t state = agibot_x2_manipulation_msgs::msg::ManipulationState::UNKNOWN;
    std::uint64_t sequence = 0;
    if (!waitForMeasurement(
        handle, target, state, sequence, minimum_sequence, acquisition_stage))
    {
      if (handle->is_canceling()) {
        return AttemptFailure{FineAlign::Result::ALIGNMENT_TIMEOUT, "fine alignment canceled"};
      }
      if (nav_active_.load()) {
        return AttemptFailure{FineAlign::Result::NAVIGATION_ACTIVE, "Nav2 became active"};
      }
      if (!rclcpp::ok()) {
        return AttemptFailure{
          FineAlign::Result::SAFETY_ABORT, "ROS shutdown interrupted alignment"};
      }
      return AttemptFailure{
        FineAlign::Result::NO_STABLE_TAG,
        minimum_sequence == 0 ? "no stable 1 Hz tag pose" : "no newer stable 1 Hz tag pose"};
    }
    result->manipulation_state = state;
    if (!validState(state)) {
      return AttemptFailure{
        FineAlign::Result::INVALID_STATE, "manipulation state is not EMPTY or HOLDING"};
    }

    PlanarError error;
    try {
      error = currentError(target);
      result->final_error = errorMessage(error);
    } catch (const tf2::TransformException & exception) {
      return AttemptFailure{FineAlign::Result::SAFETY_ABORT, exception.what()};
    }
    if (!insideCaptureEnvelope(error)) {
      return AttemptFailure{
        FineAlign::Result::OUTSIDE_CAPTURE_ENVELOPE,
        "robot is outside the configured fine-align capture envelope"};
    }
    if (!handle->get_goal()->execute) {
      return std::nullopt;
    }

    alignment_active_.store(true);
    DockingMotionController motion_controller(controller_config_, motion_config_);
    TagPoseSettling settling;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(approach_timeout_);
    const auto progress_log_period = std::max(
      controller_period_, std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(progress_log_interval_)));
    auto next_progress_log = std::chrono::steady_clock::time_point::min();
    std::uint64_t checked_sequence = 0;
    auto previous_control_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (handle->is_canceling()) {
        return AttemptFailure{FineAlign::Result::ALIGNMENT_TIMEOUT, "fine alignment canceled"};
      }
      if (nav_active_.load()) {
        return AttemptFailure{FineAlign::Result::NAVIGATION_ACTIVE, "Nav2 became active"};
      }
      if (collisionStopTimedOut()) {
        return AttemptFailure{
          FineAlign::Result::COLLISION_STOPPED, "collision monitor stop persisted"};
      }
      if (std::chrono::steady_clock::now() > deadline) {
        return AttemptFailure{FineAlign::Result::ALIGNMENT_TIMEOUT, "fine alignment timed out"};
      }
      PlanarError raw_error;
      double observation_age = 0.0;
      if (!stableMeasurement(target, state, sequence, &raw_error, &observation_age)) {
        return AttemptFailure{
          FineAlign::Result::NO_STABLE_TAG,
          "robot-relative tag target is unavailable or stale"};
      }
      result->manipulation_state = state;
      if (!validState(state)) {
        return AttemptFailure{
          FineAlign::Result::INVALID_STATE,
          "manipulation state became invalid during fine alignment"};
      }
      try {
        error = currentError(target);
        result->final_error = errorMessage(error);
      } catch (const tf2::TransformException & exception) {
        return AttemptFailure{FineAlign::Result::SAFETY_ABORT, exception.what()};
      }
      if (!insideCaptureEnvelope(error) || !insideCaptureEnvelope(raw_error)) {
        return AttemptFailure{
          FineAlign::Result::OUTSIDE_CAPTURE_ENVELOPE,
          "refined target moved outside the configured capture envelope"};
      }
      const auto control_time = std::chrono::steady_clock::now();
      const bool new_observation = sequence != checked_sequence;
      const double dt = std::chrono::duration<double>(control_time - previous_control_time).count();
      previous_control_time = control_time;
      const auto command = motion_controller.update(error, dt, new_observation);
      checked_sequence = sequence;
      if (!command) {
        return AttemptFailure{
          FineAlign::Result::SAFETY_ABORT, "holonomic controller rejected its input"};
      }
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        alignment_command_ = *command;
        alignment_command_time_ = std::chrono::steady_clock::now();
      }

      const bool at_goal = fineAlignAtGoal(error, controller_config_) &&
        fineAlignAtGoal(raw_error, controller_config_);
      const bool command_stopped = command->linear.x == 0.0 && command->linear.y == 0.0 &&
        command->angular.z == 0.0;
      const bool settling_pose = at_goal && command_stopped;
      const bool completed = settling.update(
        settling_pose, std::chrono::duration<double>(control_time.time_since_epoch()).count(),
        sequence, raw_error, settling_duration_, settling_position_spread_, settling_angular_spread_);

      const auto current_steady_time = std::chrono::steady_clock::now();
      if (current_steady_time >= next_progress_log) {
        logProfileContext("Fine-align progress:");
        RCLCPP_INFO(
          get_logger(),
          "Fine-align progress: attempt=%zu/%zu; sequence=%llu; "
          "error_base=(x=%.3f m, y=%.3f m, yaw=%.3f rad); "
          "command=(linear.x=%.3f m/s, linear.y=%.3f m/s, angular.z=%.3f rad/s); "
          "stage=%s; settling_duration=%.3f s; "
          "tag_age=%.3f s; raw_error=(%.3f, %.3f, %.3f); new_observation=%s; "
          "pose_within_tolerance=%s; command_stopped=%s; tag_settled=%s",
          attempt, maximum_attempts, static_cast<unsigned long long>(sequence),
          error.x, error.y, error.yaw,
          command->linear.x, command->linear.y, command->angular.z,
          settling_pose ? "settling" : "controlling",
          settling_duration_, observation_age, raw_error.x, raw_error.y, raw_error.yaw,
          new_observation ? "true" : "false", at_goal ? "true" : "false",
          command_stopped ? "true" : "false", completed ? "true" : "false");
        next_progress_log = current_steady_time + progress_log_period;
      }

      auto feedback = std::make_shared<FineAlign::Feedback>();
      feedback->profile_id = result->profile_id;
      feedback->stage = settling_pose ?
        FineAlign::Feedback::SETTLING : FineAlign::Feedback::CONTROLLING;
      feedback->current_error = errorMessage(error);
      feedback->tag_visible = true;
      feedback->progress = static_cast<float>(
        1.0 - std::min(1.0, std::hypot(error.x, error.y) / capture_distance_));
      handle->publish_feedback(feedback);

      if (completed) {
        return std::nullopt;
      }
      std::this_thread::sleep_for(controller_period_);
    }
    return AttemptFailure{FineAlign::Result::SAFETY_ABORT, "ROS shutdown interrupted alignment"};
  }

  void execute(std::shared_ptr<GoalHandle> handle)
  {
    auto result = std::make_shared<FineAlign::Result>();
    result->final_error.x = std::numeric_limits<double>::quiet_NaN();
    result->final_error.y = std::numeric_limits<double>::quiet_NaN();
    result->final_error.theta = std::numeric_limits<double>::quiet_NaN();
    result->manipulation_state =
      agibot_x2_manipulation_msgs::msg::ManipulationState::UNKNOWN;
    if (!activateProfile(handle->get_goal()->profile_id, false, result->profile_id)) {
      finish(handle, result, FineAlign::Result::INVALID_PROFILE,
        "unknown docking profile: " + handle->get_goal()->profile_id);
      return;
    }
    if (nav_active_.load()) {
      finish(handle, result, FineAlign::Result::NAVIGATION_ACTIVE, "Nav2 is active");
      return;
    }
    const std::size_t maximum_attempts = handle->get_goal()->execute ? maximum_retries_ + 1U : 1U;
    std::uint64_t minimum_sequence = 0;
    for (std::size_t attempt = 1; attempt <= maximum_attempts; ++attempt) {
      if (attempt > 1U && !waitForRetryDelay(handle, *result)) {
        if (nav_active_.load()) {
          finish(handle, result, FineAlign::Result::NAVIGATION_ACTIVE, "Nav2 became active");
        } else {
          finishCanceledOrFailed(
            handle, result, FineAlign::Result::ALIGNMENT_TIMEOUT,
            rclcpp::ok() ? "fine alignment canceled" : "ROS shutdown interrupted alignment");
        }
        return;
      }

      const auto failure = runAttempt(
        handle, result, minimum_sequence,
        attempt == 1U ? FineAlign::Feedback::ACQUIRING : FineAlign::Feedback::REACQUIRING,
        attempt, maximum_attempts);
      if (!failure) {
        stopAlignment();
        result->success = true;
        result->error_code = FineAlign::Result::SUCCESS;
        result->message = handle->get_goal()->execute ?
          "tag fine alignment succeeded on attempt " + std::to_string(attempt) + " of " +
          std::to_string(maximum_attempts) :
          "fine-alignment inputs and capture pose are ready";
        if (handle->get_goal()->execute) {
          std::lock_guard<std::mutex> lock(measurement_mutex_);
          last_docked_profile_ = result->profile_id;
        }
        handle->succeed(result);
        operation_active_.store(false);
        return;
      }

      stopAlignment();
      if (handle->is_canceling()) {
        finishCanceledOrFailed(handle, result, failure->code, failure->message);
        return;
      }
      const bool will_retry = retryableFailure(failure->code) && attempt < maximum_attempts;
      if (!will_retry) {
        std::string message = failure->message;
        if (retryableFailure(failure->code) && maximum_attempts > 1U) {
          message += "; retries exhausted after " + std::to_string(attempt) + " attempts";
        }
        finish(handle, result, failure->code, message);
        return;
      }

      minimum_sequence = latestStableTargetSequence();
      logProfileContext("Fine-align retry:");
      RCLCPP_WARN(
        get_logger(),
        "Fine-align retry: attempt=%zu/%zu failed; code=%u; reason='%s'; "
        "next_attempt=%zu/%zu; retry_delay=%.3f s; require_target_sequence>%llu",
        attempt, maximum_attempts, static_cast<unsigned int>(failure->code),
        failure->message.c_str(), attempt + 1U, maximum_attempts, retry_delay_,
        static_cast<unsigned long long>(minimum_sequence));
    }
  }

  uint8_t currentManipulationState() const
  {
    std::lock_guard<std::mutex> lock(measurement_mutex_);
    return manipulation_state_;
  }

  void logUndockAbort(
    const Undock::Result & result, uint16_t code, const std::string & message) const
  {
    constexpr double unavailable = std::numeric_limits<double>::quiet_NaN();
    bool collision_stopped = false;
    double collision_stop_age = unavailable;
    {
      std::lock_guard<std::mutex> lock(collision_mutex_);
      collision_stopped = collision_stopped_;
      if (collision_stop_since_) {
        collision_stop_age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - *collision_stop_since_).count();
      }
    }
    RCLCPP_ERROR(
      get_logger(),
      "Undock action abort: code=%u; reason='%s'; distance=(traveled=%.3f m, target=%.3f m); "
      "manipulation_state=(result=%u, current=%u); nav_active=%s; "
      "collision_stopped=%s (age=%.3f s)",
      static_cast<unsigned int>(code), message.c_str(), result.distance_traveled,
      undock_distance_, static_cast<unsigned int>(result.manipulation_state),
      static_cast<unsigned int>(currentManipulationState()), nav_active_.load() ? "true" : "false",
      collision_stopped ? "true" : "false", collision_stop_age);
  }

  void finishUndock(
    const std::shared_ptr<UndockGoalHandle> & handle,
    const std::shared_ptr<Undock::Result> & result, uint16_t code,
    const std::string & message)
  {
    stopAlignment();
    result->success = false;
    result->error_code = code;
    result->message = message;
    if (handle->is_canceling()) {
      result->error_code = Undock::Result::CANCELED;
      result->message = "undocking canceled";
      handle->canceled(result);
    } else {
      logProfileContext("Undock action abort:");
      logUndockAbort(*result, code, message);
      handle->abort(result);
    }
    operation_active_.store(false);
  }

  void executeUndock(std::shared_ptr<UndockGoalHandle> handle)
  {
    auto result = std::make_shared<Undock::Result>();
    result->manipulation_state = currentManipulationState();

    if (!activateProfile(handle->get_goal()->profile_id, true, result->profile_id)) {
      finishUndock(handle, result, Undock::Result::INVALID_PROFILE,
        "unknown docking profile: " + handle->get_goal()->profile_id);
      return;
    }
    auto validating = std::make_shared<Undock::Feedback>();
    validating->profile_id = result->profile_id;
    validating->stage = Undock::Feedback::VALIDATING;
    validating->distance_remaining = undock_distance_;
    handle->publish_feedback(validating);

    if (nav_active_.load()) {
      finishUndock(handle, result, Undock::Result::NAVIGATION_ACTIVE, "Nav2 is active");
      return;
    }
    if (!validState(result->manipulation_state)) {
      finishUndock(
        handle, result, Undock::Result::INVALID_STATE,
        "manipulation state is not EMPTY or HOLDING");
      return;
    }
    Eigen::Isometry3d initial;
    uint8_t state = result->manipulation_state;
    std::uint64_t sequence = 0;
    const auto acquisition_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(acquisition_timeout_);
    while (rclcpp::ok() && !stableMeasurement(initial, state, sequence)) {
      if (handle->is_canceling() || nav_active_.load() || collisionStopTimedOut() ||
        !validState(currentManipulationState()))
      {
        finishUndock(handle, result,
          handle->is_canceling() ? Undock::Result::CANCELED :
          nav_active_.load() ? Undock::Result::NAVIGATION_ACTIVE :
          !validState(currentManipulationState()) ? Undock::Result::INVALID_STATE :
          Undock::Result::COLLISION_STOPPED, "undock acquisition interrupted");
        return;
      }
      if (std::chrono::steady_clock::now() >= acquisition_deadline) {
        finishUndock(handle, result, Undock::Result::NO_STABLE_TAG,
          "no fresh stable tag for undocking");
        return;
      }
      handle->publish_feedback(validating);
      std::this_thread::sleep_for(controller_period_);
    }
    if (!rclcpp::ok()) {
      finishUndock(handle, result, Undock::Result::SAFETY_ABORT,
        "ROS shutdown interrupted undock acquisition");
      return;
    }
    alignment_active_.store(true);
    DockingMotionController motion_controller(undock_controller_config_, motion_config_);
    TagPoseSettling settling;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(undock_timeout_);
    auto previous_control_time = std::chrono::steady_clock::now();
    auto next_progress_log = std::chrono::steady_clock::time_point::min();
    std::uint64_t checked_sequence = 0;
    while (rclcpp::ok()) {
      if (handle->is_canceling() || nav_active_.load() || collisionStopTimedOut()) {
        finishUndock(handle, result,
          handle->is_canceling() ? Undock::Result::CANCELED :
          nav_active_.load() ? Undock::Result::NAVIGATION_ACTIVE : Undock::Result::COLLISION_STOPPED,
          "undocking interrupted by cancellation, navigation, or collision stop");
        return;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        finishUndock(handle, result, Undock::Result::UNDOCK_TIMEOUT, "undocking timed out");
        return;
      }
      Eigen::Isometry3d current;
      PlanarError raw;
      double age = 0.0;
      if (!stableMeasurement(current, state, sequence, &raw, &age)) {
        finishUndock(handle, result, Undock::Result::NO_STABLE_TAG,
          "tag became unavailable or stale during undocking");
        return;
      }
      result->manipulation_state = state;
      if (!validState(state)) {
        finishUndock(handle, result, Undock::Result::INVALID_STATE,
          "manipulation state became invalid during undocking");
        return;
      }
      const auto target = tagRelativeUndockTarget(initial, current, undock_distance_);
      const auto error = planarError(Eigen::Isometry3d::Identity(), target);
      const auto raw_target = tagRelativeUndockTarget(initial, planarPose(raw), undock_distance_);
      const auto raw_error = planarError(Eigen::Isometry3d::Identity(), raw_target);
      const auto robot_from_initial = initial * planarPose(raw).inverse();
      result->distance_traveled = std::max(0.0, -robot_from_initial.translation().x());
      const auto control_time = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(control_time - previous_control_time).count();
      previous_control_time = control_time;
      const auto command = motion_controller.update(error, dt, sequence != checked_sequence);
      checked_sequence = sequence;
      if (!command) {
        finishUndock(handle, result, Undock::Result::SAFETY_ABORT,
          "controller rejected tag-relative undocking error");
        return;
      }
      const bool at_goal = fineAlignAtGoal(error, undock_controller_config_) &&
        fineAlignAtGoal(raw_error, undock_controller_config_);
      const bool command_stopped = command->linear.x == 0.0 && command->linear.y == 0.0 &&
        command->angular.z == 0.0;
      const bool settling_pose = at_goal && command_stopped;
      const bool completed = settling.update(
        settling_pose, std::chrono::duration<double>(control_time.time_since_epoch()).count(),
        sequence, raw, settling_duration_, settling_position_spread_, settling_angular_spread_);
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        alignment_command_ = *command;
        alignment_command_time_ = control_time;
      }
      const double remaining = std::max(0.0, undock_distance_ - result->distance_traveled);
      if (control_time >= next_progress_log) {
        logProfileContext("Undock progress:");
        RCLCPP_INFO(get_logger(),
          "Undock progress: distance=(traveled=%.3f m, remaining=%.3f m, target=%.3f m); "
          "error_base=(x=%.3f m, y=%.3f m, yaw=%.3f rad); "
          "command=(linear.x=%.3f m/s, linear.y=%.3f m/s, angular.z=%.3f rad/s); "
          "tag_age=%.3f s; sequence=%llu; pose_within_tolerance=%s; command_stopped=%s; "
          "stage=%s; tag_settled=%s",
          result->distance_traveled, remaining, undock_distance_, error.x, error.y, error.yaw,
          command->linear.x, command->linear.y, command->angular.z, age,
          static_cast<unsigned long long>(sequence), at_goal ? "true" : "false",
          command_stopped ? "true" : "false", settling_pose ? "settling" : "moving",
          completed ? "true" : "false");
        next_progress_log = control_time + std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(progress_log_interval_));
      }
      auto feedback = std::make_shared<Undock::Feedback>();
      feedback->profile_id = result->profile_id;
      feedback->stage = settling_pose ? Undock::Feedback::SETTLING : Undock::Feedback::MOVING;
      feedback->distance_traveled = result->distance_traveled;
      feedback->distance_remaining = remaining;
      feedback->commanded_speed = command->linear.x;
      feedback->commanded_lateral_speed = command->linear.y;
      feedback->commanded_yaw_speed = command->angular.z;
      feedback->progress = static_cast<float>(
        std::min(1.0, result->distance_traveled / undock_distance_));
      handle->publish_feedback(feedback);
      if (completed) {
        stopAlignment();
        result->success = true;
        result->error_code = Undock::Result::SUCCESS;
        result->message = "tag-relative undocking succeeded";
        handle->succeed(result);
        operation_active_.store(false);
        return;
      }
      std::this_thread::sleep_for(controller_period_);
    }
    finishUndock(handle, result, Undock::Result::SAFETY_ABORT,
      "ROS shutdown interrupted undocking");
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

    RCLCPP_ERROR(
      get_logger(),
      "Fine-align action abort: code=%u; reason='%s'; execute=%s; "
      "final_error_base=(x=%.3f m, y=%.3f m, yaw=%.3f rad); "
      "manipulation_state=(result=%u, current=%u); "
      "stable_target_%s=(sequence=%llu, x=%.3f m, y=%.3f m, yaw=%.3f rad, "
      "age=%.3f s, frame=%s); nav_active=%s; "
      "collision_stopped=%s (age=%.3f s)",
      static_cast<unsigned int>(code), message.c_str(),
      handle->get_goal()->execute ? "true" : "false",
      result.final_error.x, result.final_error.y, result.final_error.theta,
      static_cast<unsigned int>(result.manipulation_state),
      static_cast<unsigned int>(current_manipulation_state),
      stable_target_available ? "available" : "unavailable",
      static_cast<unsigned long long>(stable_target_sequence), stable_target_x, stable_target_y,
      stable_target_yaw, stable_target_age, base_frame_.c_str(),
      nav_active_.load() ? "true" : "false", collision_stopped ? "true" : "false",
      collision_stop_age);
  }

  void finish(
    const std::shared_ptr<GoalHandle> & handle, const std::shared_ptr<FineAlign::Result> & result,
    uint16_t code, const std::string & message)
  {
    result->success = false;
    result->error_code = code;
    result->message = message;
    logProfileContext("Fine-align action abort:");
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
    processPendingDetection();
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
    } else {
      command = nav_command_gate_.commandAt(
        now_steady, std::chrono::duration<double>(command_timeout_));
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
  rclcpp_action::Server<Undock>::SharedPtr undock_server_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<agibot_x2_manipulation_msgs::msg::ManipulationState>::SharedPtr state_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_raw_cmd_sub_;
  NavigationCommandGate nav_command_gate_;
  rclcpp::Subscription<nav2_msgs::msg::CollisionMonitorState>::SharedPtr collision_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_pub_;
  rclcpp::TimerBase::SharedPtr mux_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex pending_detection_mutex_;
  apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr pending_detection_;
  std::unique_ptr<RelativeTagTracker> tracker_;
  PlanarError raw_error_;
  DockingMotionConfig motion_config_;
  double settling_duration_, tracking_timeout_, filter_time_constant_;
  double settling_position_spread_, settling_angular_spread_;
  mutable std::mutex measurement_mutex_, command_mutex_, collision_mutex_;
  std::optional<Eigen::Isometry3d> stable_target_;
  rclcpp::Time last_sample_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stable_target_stamp_{0, 0, RCL_ROS_TIME};
  std::uint64_t stable_target_sequence_{0};
  uint8_t manipulation_state_{agibot_x2_manipulation_msgs::msg::ManipulationState::UNKNOWN};
  geometry_msgs::msg::Twist alignment_command_;
  std::optional<std::chrono::steady_clock::time_point> alignment_command_time_;
  std::optional<std::chrono::steady_clock::time_point> collision_stop_since_;
  std::atomic_bool operation_active_{false}, alignment_active_{false}, nav_active_{false};
  bool collision_stopped_{false};
  std::string base_frame_, default_profile_, last_docked_profile_;
  DockingProfiles profiles_;
  DockingProfile active_profile_;
  std::uint64_t profile_generation_{0};
  rclcpp::Time activation_stamp_{0, 0, RCL_ROS_TIME};
  std::size_t stable_sample_count_{3}, maximum_retries_{2};
  HolonomicFineAlignConfig controller_config_, undock_controller_config_;
  std::chrono::nanoseconds controller_period_{50ms};
  double minimum_decision_margin_;
  double maximum_pose_age_, maximum_position_spread_, maximum_angular_spread_;
  double capture_distance_, capture_lateral_, capture_yaw_, reverse_capture_distance_;
  double acquisition_timeout_, approach_timeout_, retry_delay_, command_timeout_;
  double collision_stop_timeout_;
  double progress_log_interval_;
  double undock_distance_, undock_timeout_;
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
