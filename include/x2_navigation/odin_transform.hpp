#ifndef X2_NAVIGATION__ODIN_TRANSFORM_HPP_
#define X2_NAVIGATION__ODIN_TRANSFORM_HPP_

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace x2_navigation
{

inline Eigen::Isometry3d odinTransform(
  double x, double y, double z, double qx, double qy, double qz, double qw)
{
  const Eigen::Vector3d translation(x, y, z);
  Eigen::Quaterniond rotation(qw, qx, qy, qz);
  if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
    std::abs(rotation.norm() - 1.0) > 0.01)
  {
    throw std::invalid_argument("Non-finite translation or invalid unit quaternion");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = translation;
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}

inline Eigen::Isometry3d odinTransform(const geometry_msgs::msg::Pose & pose)
{
  return odinTransform(
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

inline Eigen::Isometry3d odinTransform(const geometry_msgs::msg::Transform & transform)
{
  return odinTransform(
    transform.translation.x, transform.translation.y, transform.translation.z,
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
}

inline Eigen::Isometry3d odinBasePose(
  const Eigen::Isometry3d & odom_from_imu, const Eigen::Isometry3d & base_from_imu)
{
  return odom_from_imu * base_from_imu.inverse();
}

inline geometry_msgs::msg::Twist odinLocalTwist(
  const Eigen::Isometry3d & previous, const Eigen::Isometry3d & current, double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("Differentiation requires positive finite dt");
  }
  // Differentiate base poses in the continuous local frame, then express in
  // current base axes. This also removes relative torso/joint motion.
  const Eigen::Vector3d linear =
    current.rotation().transpose() * (current.translation() - previous.translation()) / dt;
  const Eigen::AngleAxisd delta(current.rotation().transpose() * previous.rotation());
  const Eigen::Vector3d angular = -delta.axis() * delta.angle() / dt;
  geometry_msgs::msg::Twist result;
  result.linear.x = linear.x();
  result.linear.y = linear.y();
  result.linear.z = linear.z();
  result.angular.x = angular.x();
  result.angular.y = angular.y();
  result.angular.z = angular.z();
  return result;
}

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__ODIN_TRANSFORM_HPP_
