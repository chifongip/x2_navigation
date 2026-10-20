#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "x2_navigation/odin_transform.hpp"

namespace x2_navigation
{

class OdinLocalizationAdapter : public rclcpp::Node
{
public:
  OdinLocalizationAdapter()
  : Node("odin_localization_adapter"), robot_tf_(get_clock()), robot_listener_(robot_tf_),
    broadcaster_(*this)
  {
    const auto input = declare_parameter<std::string>("input_topic", "/odin1/odometry_highfreq");
    const auto native_tf = declare_parameter<std::string>("native_tf_topic", "/odin1/tf");
    if (native_tf == "/tf" || native_tf == "tf" || native_tf.empty()) {
      throw std::invalid_argument("native_tf_topic must be isolated from robot /tf");
    }
    native_odom_ = declare_parameter<std::string>("native_odom_frame", "odom");
    native_map_ = declare_parameter<std::string>("native_map_frame", "map");
    native_imu_ = declare_parameter<std::string>("native_imu_frame", "imu");
    tracking_ = declare_parameter<std::string>("tracking_frame", "odin_imu");
    base_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_ = declare_parameter<std::string>("odom_frame", "odom");
    map_ = declare_parameter<std::string>("map_frame", "map");
    offset_ = declare_parameter<double>("sensor_time_offset_sec", 0.0);
    local_timeout_ = positive("local_timeout_sec", 0.25);
    correction_timeout_ = positive("correction_timeout_sec", 1.0);
    future_tolerance_ = positive("future_tolerance_sec", 0.05);
    max_step_ = positive("max_translation_step_m", 0.5);
    max_angle_ = positive("max_rotation_step_rad", 0.5);
    const auto capacity = declare_parameter<int64_t>("max_pending_samples", 256);
    if (capacity < 1 || capacity > 4096) {
      throw std::invalid_argument("max_pending_samples must be between 1 and 4096");
    }
    max_pending_samples_ = static_cast<std::size_t>(capacity);
    pose_variance_ = positive("pose_variance", 0.01);
    twist_variance_ = positive("twist_variance", 0.1);
    alignment_confirmed_ = declare_parameter<bool>("map_alignment_confirmed", false);
    const auto xyz = declare_parameter<std::vector<double>>("map_alignment_xyz", {0.0, 0.0, 0.0});
    const auto rpy = declare_parameter<std::vector<double>>("map_alignment_rpy", {0.0, 0.0, 0.0});
    if (xyz.size() != 3 || rpy.size() != 3 || !std::isfinite(offset_) ||
      std::abs(offset_) > 3600.0 || !Eigen::Map<const Eigen::Vector3d>(xyz.data()).allFinite() ||
      !Eigen::Map<const Eigen::Vector3d>(rpy.data()).allFinite())
    {
      throw std::invalid_argument("Invalid timestamp offset or map alignment");
    }
    alignment_.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    alignment_.linear() = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    for (const auto & frame : {native_odom_, native_map_, native_imu_, tracking_, base_, odom_, map_}) {
      if (frame.empty() || frame.front() == '/') {
        throw std::invalid_argument("Frame IDs must be nonempty without a leading slash");
      }
    }
    if (odom_ == map_ || odom_ == base_ || map_ == base_) {
      throw std::invalid_argument("Robot root frames must be distinct");
    }
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/odin1/localization_ready", 1);
    status_pub_ = create_publisher<std_msgs::msg::String>("/odin1/localization_status", 1);
    // All callbacks share the default mutually exclusive callback group.
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input, rclcpp::SensorDataQoS().keep_last(max_pending_samples_),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {receiveOdometry(msg);});
    native_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      native_tf, rclcpp::SensorDataQoS().keep_last(10),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
        for (const auto & tf : msg->transforms) {
          if (tf.header.frame_id != native_odom_ || tf.child_frame_id != native_map_) {
            continue;
          }
          try {
            const auto stamp = shifted(tf.header.stamp);
            if (!fresh(stamp, correction_timeout_) ||
              (last_correction_stamp_ && stamp <= *last_correction_stamp_))
            {
              continue;
            }
            const auto correction = alignment_ * odinTransform(tf.transform).inverse();
            last_correction_stamp_ = stamp;
            // A replayed or delayed packet from before a local reset cannot
            // establish the relationship with the new odometry epoch.
            if (recovery_after_ && stamp <= *recovery_after_) {
              continue;
            }
            correction_ = correction;
            correction_stamp_ = stamp;
            correction_received_at_ = std::chrono::steady_clock::now();
          } catch (const std::exception & error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
            correction_stamp_.reset();
          }
        }
      });
    // Publish at most one local sample per tick, retaining newer pending samples
    // until timestamped robot TF covers them. Never forge future TF.
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {process();});
  }

private:
  double positive(const std::string & name, double value)
  {
    value = declare_parameter<double>(name, value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be positive and finite");
    }
    return value;
  }

  rclcpp::Time shifted(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
      (stamp.sec == 0 && stamp.nanosec == 0))
    {
      throw std::invalid_argument("Invalid Odin timestamp");
    }
    return rclcpp::Time(stamp, get_clock()->get_clock_type()) +
           rclcpp::Duration::from_seconds(offset_);
  }

  bool fresh(const rclcpp::Time & stamp, double timeout) const
  {
    const auto age = (now() - stamp).seconds();
    return age >= -future_tolerance_ && age <= timeout;
  }

  void status(const std::string & reason)
  {
    std_msgs::msg::Bool ready;
    ready.data = reason == "ready";
    ready_pub_->publish(ready);
    std_msgs::msg::String message;
    message.data = reason;
    status_pub_->publish(message);
  }

  void sendTf(
    const Eigen::Isometry3d & pose, const std::string & parent,
    const std::string & child, const rclcpp::Time & stamp)
  {
    auto tf = tf2::eigenToTransform(pose);
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.header.stamp = stamp;
    broadcaster_.sendTransform(tf);
  }

  struct PendingOdometry
  {
    nav_msgs::msg::Odometry::ConstSharedPtr message;
    rclcpp::Time stamp;
    std::chrono::steady_clock::time_point received;
  };

  void invalidateLocal(const std::string & reason)
  {
    // Keep timestamp watermarks across reset: discarding the differentiation
    // baseline must not allow an old correction packet to be accepted again.
    const auto boundary = now();
    if (!recovery_after_ || boundary > *recovery_after_) {
      recovery_after_ = boundary;
    }
    pending_.clear();
    previous_stamp_.reset();
    local_accepted_at_.reset();
    correction_stamp_.reset();
    correction_received_at_.reset();
    differentiated_ = false;
    local_reason_ = reason;
  }

  void receiveOdometry(nav_msgs::msg::Odometry::ConstSharedPtr input)
  {
    try {
      const auto stamp = shifted(input->header.stamp);
      if (!fresh(stamp, local_timeout_)) {
        throw std::invalid_argument("Invalid local measurement time");
      }
      if (input->header.frame_id != native_odom_ || input->child_frame_id != native_imu_) {
        throw std::invalid_argument("Unexpected native odometry frames");
      }
      odinTransform(input->pose.pose);  // Validate before queuing or publishing.
      if (last_local_stamp_ && stamp == *last_local_stamp_) {
        return;  // Duplicate packets do not refresh liveness or the baseline.
      }
      if (last_local_stamp_ && stamp < *last_local_stamp_) {
        last_local_stamp_ = stamp;
        invalidateLocal("local_time_regression");
        return;
      }
      last_local_stamp_ = stamp;
      pending_.push_back({input, stamp, std::chrono::steady_clock::now()});
      while (pending_.size() > max_pending_samples_) {
        pending_.pop_front();
      }
    } catch (const std::exception & error) {
      invalidateLocal("invalid_local_input");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
    }
  }

  void process()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (previous_stamp_ && (!fresh(*previous_stamp_, local_timeout_) ||
      !local_accepted_at_ ||
      std::chrono::duration<double>(steady_now - *local_accepted_at_).count() > local_timeout_))
    {
      invalidateLocal("local_odometry_not_advancing");
    }
    while (!pending_.empty() && (!fresh(pending_.front().stamp, local_timeout_) ||
      std::chrono::duration<double>(steady_now - pending_.front().received).count() > local_timeout_))
    {
      pending_.pop_front();
    }

    std::optional<rclcpp::Time> latest_robot_stamp;
    bool robot_connected = false;
    if (!pending_.empty()) {
      try {
        const auto latest = robot_tf_.lookupTransform(base_, tracking_, tf2::TimePointZero);
        robot_connected = true;
        // A zero stamp denotes an entirely static chain, valid at all times.
        if (latest.header.stamp.sec != 0 || latest.header.stamp.nanosec != 0) {
          latest_robot_stamp = rclcpp::Time(latest.header.stamp, get_clock()->get_clock_type());
        }
      } catch (const tf2::TransformException &) {
        // Query once per tick when the chain is absent; avoid one warning or
        // lookup for every pending sample while shared bringup is unavailable.
      }
    }
    // Select the newest transformable sample. A newer untransformable sample
    // never replaces a usable older one; both queue length and age are bounded.
    for (std::size_t index = pending_.size(); robot_connected && index > 0; --index) {
      const auto candidate = pending_[index - 1];
      if (latest_robot_stamp && candidate.stamp > *latest_robot_stamp) {
        continue;
      }
      if (!robot_tf_.canTransform(base_, tracking_, candidate.stamp)) {
        continue;
      }
      try {
        const auto mount = robot_tf_.lookupTransform(base_, tracking_, candidate.stamp);
        const auto pose = odinBasePose(
          odinTransform(candidate.message->pose.pose), odinTransform(mount.transform));
        nav_msgs::msg::Odometry output;
        output.header.stamp = candidate.stamp;
        output.header.frame_id = odom_;
        output.child_frame_id = base_;
        output.pose.pose = tf2::toMsg(pose);
        if (previous_stamp_) {
          const auto dt = (candidate.stamp - *previous_stamp_).seconds();
          const auto distance = (pose.translation() - previous_pose_.translation()).norm();
          const auto angle = Eigen::AngleAxisd(
            previous_pose_.rotation().transpose() * pose.rotation()).angle();
          if (dt <= 0.0 || dt > local_timeout_ || distance > max_step_ || angle > max_angle_) {
            invalidateLocal("local_odometry_discontinuity");
            status(local_reason_);
            return;
          }
          output.twist.twist = odinLocalTwist(previous_pose_, pose, dt);
        }
        // SDK covariance axes are undocumented; use explicit diagonal estimates.
        for (int i = 0; i < 6; ++i) {
          output.pose.covariance[i * 7] = pose_variance_;
          output.twist.covariance[i * 7] = twist_variance_;
        }
        differentiated_ = previous_stamp_.has_value();
        previous_pose_ = pose;
        previous_stamp_ = candidate.stamp;
        local_accepted_at_ = steady_now;
        pending_.erase(pending_.begin(), pending_.begin() + index);
        odometry_pub_->publish(output);
        sendTf(pose, odom_, base_, candidate.stamp);
        break;
      } catch (const tf2::TransformException &) {
        // The buffer may change after canTransform. Retain samples and history
        // for a retry; a transient lookup error is not a new odometry baseline.
        continue;
      } catch (const std::exception & error) {
        invalidateLocal("invalid_transformed_local_pose");
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
        break;
      }
    }
    if (!previous_stamp_) {
      status(pending_.empty() ? local_reason_ : "waiting_for_timestamped_robot_tf");
    } else if (!alignment_confirmed_) {
      status("map_alignment_unconfirmed");
    } else if (!correction_stamp_ || !fresh(*correction_stamp_, correction_timeout_) ||
      !correction_received_at_ ||
      std::chrono::duration<double>(steady_now - *correction_received_at_).count() > correction_timeout_)
    {
      status("waiting_for_fresh_relocalization");
    } else if (!differentiated_) {
      status("waiting_for_velocity_baseline");
    } else {
      if (!published_correction_stamp_ || *published_correction_stamp_ != *correction_stamp_) {
        sendTf(correction_, map_, odom_, *correction_stamp_);
        published_correction_stamp_ = correction_stamp_;
      }
      status("ready");
    }
  }

  tf2_ros::Buffer robot_tf_;
  tf2_ros::TransformListener robot_listener_;
  tf2_ros::TransformBroadcaster broadcaster_;
  std::deque<PendingOdometry> pending_;
  std::size_t max_pending_samples_;
  std::string local_reason_{"waiting_for_local_odometry"};
  std::string native_odom_, native_map_, native_imu_, tracking_, base_, odom_, map_;
  double offset_, local_timeout_, correction_timeout_, future_tolerance_;
  double max_step_, max_angle_, pose_variance_, twist_variance_;
  bool alignment_confirmed_, differentiated_{false};
  Eigen::Isometry3d alignment_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d correction_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d previous_pose_{Eigen::Isometry3d::Identity()};
  std::optional<rclcpp::Time> correction_stamp_, previous_stamp_, published_correction_stamp_;
  std::optional<rclcpp::Time> last_local_stamp_, last_correction_stamp_, recovery_after_;
  std::optional<std::chrono::steady_clock::time_point> local_accepted_at_, correction_received_at_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr native_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace x2_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<x2_navigation::OdinLocalizationAdapter>());
  rclcpp::shutdown();
  return 0;
}
