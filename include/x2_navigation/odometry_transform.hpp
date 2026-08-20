#ifndef X2_NAVIGATION__ODOMETRY_TRANSFORM_HPP_
#define X2_NAVIGATION__ODOMETRY_TRANSFORM_HPP_

#include <array>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace x2_navigation
{

inline Eigen::Matrix3d skew(const Eigen::Vector3d &vector)
{
  Eigen::Matrix3d result;
  result << 0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return result;
}

inline nav_msgs::msg::Odometry transformFastLioOdometry(
  const nav_msgs::msg::Odometry &input,
  const geometry_msgs::msg::TransformStamped &base_from_tracking,
  const std::string &base_frame)
{
  Eigen::Isometry3d odom_to_tracking = Eigen::Isometry3d::Identity();
  tf2::fromMsg(input.pose.pose, odom_to_tracking);

  const auto &translation = base_from_tracking.transform.translation;
  const auto &orientation = base_from_tracking.transform.rotation;
  const Eigen::Quaterniond rotation_quaternion(
    orientation.w, orientation.x, orientation.y, orientation.z);
  const Eigen::Isometry3d base_from_tracking_isometry =
    Eigen::Translation3d(translation.x, translation.y, translation.z) *
    rotation_quaternion.normalized();
  const Eigen::Isometry3d odom_to_base =
    odom_to_tracking * base_from_tracking_isometry.inverse();

  nav_msgs::msg::Odometry output = input;
  output.child_frame_id = base_frame;
  output.pose.pose = tf2::toMsg(odom_to_base);

  Eigen::Matrix<double, 6, 6> input_pose_covariance;
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      input_pose_covariance(row, column) = input.pose.covariance[row * 6 + column];
    }
  }
  Eigen::Matrix<double, 6, 6> pose_jacobian =
    Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Vector3d tracking_offset_in_odom =
    odom_to_base.rotation() * base_from_tracking_isometry.translation();
  // FAST-LIO's EKF stores attitude errors as right perturbations in the
  // tracking frame. PoseWithCovariance uses fixed odom axes.
  const Eigen::Matrix3d tracking_to_odom = odom_to_tracking.rotation();
  pose_jacobian.block<3, 3>(0, 3) =
    skew(tracking_offset_in_odom) * tracking_to_odom;
  pose_jacobian.block<3, 3>(3, 3) = tracking_to_odom;
  const Eigen::Matrix<double, 6, 6> output_pose_covariance =
    pose_jacobian * input_pose_covariance * pose_jacobian.transpose();
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      output.pose.covariance[row * 6 + column] = output_pose_covariance(row, column);
    }
  }

  const Eigen::Matrix3d rotation = base_from_tracking_isometry.rotation();
  const Eigen::Vector3d tracking_linear(
    input.twist.twist.linear.x,
    input.twist.twist.linear.y,
    input.twist.twist.linear.z);
  const Eigen::Vector3d tracking_angular(
    input.twist.twist.angular.x,
    input.twist.twist.angular.y,
    input.twist.twist.angular.z);
  const Eigen::Vector3d base_angular = rotation * tracking_angular;
  const Eigen::Vector3d base_linear = rotation * tracking_linear -
    base_angular.cross(base_from_tracking_isometry.translation());

  output.twist.twist.linear.x = base_linear.x();
  output.twist.twist.linear.y = base_linear.y();
  output.twist.twist.linear.z = base_linear.z();
  output.twist.twist.angular.x = base_angular.x();
  output.twist.twist.angular.y = base_angular.y();
  output.twist.twist.angular.z = base_angular.z();

  Eigen::Matrix<double, 6, 6> input_twist_covariance;
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      input_twist_covariance(row, column) = input.twist.covariance[row * 6 + column];
    }
  }

  Eigen::Matrix<double, 6, 6> twist_jacobian =
    Eigen::Matrix<double, 6, 6>::Zero();
  twist_jacobian.block<3, 3>(0, 0) = rotation;
  twist_jacobian.block<3, 3>(0, 3) =
    -skew(base_from_tracking_isometry.translation()) * rotation;
  twist_jacobian.block<3, 3>(3, 3) = rotation;
  const Eigen::Matrix<double, 6, 6> output_twist_covariance =
    twist_jacobian * input_twist_covariance * twist_jacobian.transpose();
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      output.twist.covariance[row * 6 + column] = output_twist_covariance(row, column);
    }
  }

  return output;
}

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__ODOMETRY_TRANSFORM_HPP_
