#include "x2_navigation/table_dock_geometry.hpp"

#include <gtest/gtest.h>

TEST(TableDockGeometry, BuildsPoseOutwardAndFacingTable)
{
  Eigen::Isometry3d tag = Eigen::Isometry3d::Identity();
  // Tag +Z points out of the table along fixed +X; tag +X points fixed -Y.
  tag.linear().col(2) = Eigen::Vector3d::UnitX();
  tag.linear().col(0) = -Eigen::Vector3d::UnitY();
  tag.linear().col(1) = Eigen::Vector3d::UnitZ();
  const auto target = x2_navigation::tableDockPose(tag, 0.7, 0.0, 0.0);
  EXPECT_NEAR(target.translation().x(), 0.7, 1e-9);
  EXPECT_NEAR(target.translation().y(), 0.0, 1e-9);
  const double yaw = std::atan2(target.linear()(1, 0), target.linear()(0, 0));
  EXPECT_NEAR(x2_navigation::wrapAngle(yaw - M_PI), 0.0, 1e-9);
}

TEST(TableDockGeometry, ReportsTargetInBaseFrame)
{
  Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(0.4, -0.1, 0.0);
  const auto error = x2_navigation::planarError(base, target);
  EXPECT_NEAR(error.x, 0.4, 1e-9);
  EXPECT_NEAR(error.y, -0.1, 1e-9);
  EXPECT_NEAR(error.yaw, 0.0, 1e-9);
}
