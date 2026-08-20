#include <cmath>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "x2_navigation/odometry_transform.hpp"

namespace
{

TEST(OdometryTransform, ConvertsPoseTwistAndTwistCovarianceToBaseFrame)
{
  nav_msgs::msg::Odometry input;
  input.header.frame_id = "odom";
  input.child_frame_id = "lidar_imu_chest_front";
  input.pose.pose.position.x = 5.0;
  input.pose.pose.orientation.w = 1.0;
  input.pose.covariance[0] = 1.0;
  input.pose.covariance[35] = 4.0;
  input.twist.twist.linear.x = 1.0;
  input.twist.twist.angular.z = 2.0;
  input.twist.covariance[0] = 1.0;
  input.twist.covariance[35] = 4.0;

  geometry_msgs::msg::TransformStamped base_from_tracking;
  base_from_tracking.header.frame_id = "base_link";
  base_from_tracking.child_frame_id = "lidar_imu_chest_front";
  base_from_tracking.transform.translation.x = 1.0;
  base_from_tracking.transform.rotation.w = 1.0;

  const auto output = x2_navigation::transformFastLioOdometry(
    input, base_from_tracking, "base_link");

  EXPECT_EQ(output.header.frame_id, "odom");
  EXPECT_EQ(output.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(output.pose.covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(output.pose.covariance[7], 4.0);
  EXPECT_DOUBLE_EQ(output.twist.twist.angular.z, 2.0);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.y, -2.0);
  EXPECT_DOUBLE_EQ(output.twist.covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(output.twist.covariance[7], 4.0);
  EXPECT_DOUBLE_EQ(output.twist.covariance[35], 4.0);
}

TEST(OdometryTransform, RotatesTrackingFrameTwistIntoBaseFrame)
{
  nav_msgs::msg::Odometry input;
  input.header.frame_id = "odom";
  input.child_frame_id = "lidar_imu_chest_front";
  input.pose.pose.orientation.w = 1.0;
  input.twist.twist.linear.x = 1.0;
  input.twist.covariance[0] = 1.0;

  geometry_msgs::msg::TransformStamped base_from_tracking;
  base_from_tracking.header.frame_id = "base_link";
  base_from_tracking.child_frame_id = "lidar_imu_chest_front";
  base_from_tracking.transform.rotation.z = std::sqrt(0.5);
  base_from_tracking.transform.rotation.w = std::sqrt(0.5);

  const auto output = x2_navigation::transformFastLioOdometry(
    input, base_from_tracking, "base_link");

  EXPECT_NEAR(output.pose.pose.orientation.z, -std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(output.pose.pose.orientation.w, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(output.twist.twist.linear.x, 0.0, 1e-12);
  EXPECT_NEAR(output.twist.twist.linear.y, 1.0, 1e-12);
  EXPECT_NEAR(output.twist.covariance[0], 0.0, 1e-12);
  EXPECT_NEAR(output.twist.covariance[7], 1.0, 1e-12);
}

TEST(OdometryTransform, ConvertsFastLioPoseCovarianceToFixedOdomAxes)
{
  nav_msgs::msg::Odometry input;
  input.pose.pose.orientation.z = std::sqrt(0.5);
  input.pose.pose.orientation.w = std::sqrt(0.5);
  // FAST-LIO reports attitude covariance as a right perturbation in the
  // tracking frame. A tracking-frame roll is an odom-frame pitch here.
  input.pose.covariance[21] = 4.0;

  geometry_msgs::msg::TransformStamped base_from_tracking;
  base_from_tracking.transform.translation.x = 1.0;
  base_from_tracking.transform.rotation.w = 1.0;

  const auto output = x2_navigation::transformFastLioOdometry(
    input, base_from_tracking, "base_link");

  EXPECT_NEAR(output.pose.covariance[21], 0.0, 1e-12);
  EXPECT_NEAR(output.pose.covariance[28], 4.0, 1e-12);
  // The transformed roll is parallel to the lever arm and must not add
  // position uncertainty.
  EXPECT_NEAR(output.pose.covariance[0], 0.0, 1e-12);
  EXPECT_NEAR(output.pose.covariance[7], 0.0, 1e-12);
  EXPECT_NEAR(output.pose.covariance[14], 0.0, 1e-12);
}

}  // namespace
