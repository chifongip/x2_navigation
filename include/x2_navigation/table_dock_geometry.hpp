#ifndef X2_NAVIGATION__TABLE_DOCK_GEOMETRY_HPP_
#define X2_NAVIGATION__TABLE_DOCK_GEOMETRY_HPP_

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace x2_navigation
{

inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline Eigen::Isometry3d tableDockPose(
  const Eigen::Isometry3d & fixed_to_tag, double standoff,
  double lateral_offset, double yaw_offset)
{
  Eigen::Vector2d outward(
    fixed_to_tag.linear()(0, 2), fixed_to_tag.linear()(1, 2));
  Eigen::Vector2d right(
    fixed_to_tag.linear()(0, 0), fixed_to_tag.linear()(1, 0));
  if (outward.norm() < 0.5 || right.norm() < 0.5) {
    throw std::runtime_error("table tag normal or horizontal axis is not horizontal");
  }
  outward.normalize();
  right.normalize();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = fixed_to_tag.translation().x() + standoff * outward.x() +
    lateral_offset * right.x();
  pose.translation().y() = fixed_to_tag.translation().y() + standoff * outward.y() +
    lateral_offset * right.y();
  const double yaw = std::atan2(-outward.y(), -outward.x()) + yaw_offset;
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

struct PlanarError
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline PlanarError planarError(
  const Eigen::Isometry3d & fixed_to_base, const Eigen::Isometry3d & fixed_to_target)
{
  const Eigen::Isometry3d base_to_target = fixed_to_base.inverse() * fixed_to_target;
  return {
    base_to_target.translation().x(),
    base_to_target.translation().y(),
    wrapAngle(std::atan2(base_to_target.linear()(1, 0), base_to_target.linear()(0, 0)))};
}

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__TABLE_DOCK_GEOMETRY_HPP_
