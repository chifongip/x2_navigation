#include <limits>

#include <gtest/gtest.h>

#include "x2_navigation/localization_gate.hpp"
#include "x2_navigation/odin_transform.hpp"

using x2_navigation::odinTransform;
using x2_navigation::odinBasePose;
using x2_navigation::odinLocalTwist;

TEST(OdinTransform, RotatedMountAndArticulation)
{
  const auto mount = odinTransform(0.2, -0.1, 0.5, 0, 0, std::sin(0.4), std::cos(0.4));
  const auto base = odinTransform(3, 4, 0.7, 0, 0, std::sin(0.7), std::cos(0.7));
  EXPECT_TRUE(odinBasePose(base * mount, mount).matrix().isApprox(base.matrix()));
  const auto moved_mount = odinTransform(0.3, -0.2, 0.6, std::sin(0.1), 0, 0, std::cos(0.1));
  const auto converted = odinBasePose(base * moved_mount, moved_mount);
  const auto twist = odinLocalTwist(base, converted, 0.1);
  EXPECT_NEAR(twist.linear.x, 0.0, 1e-12);
  EXPECT_NEAR(twist.angular.z, 0.0, 1e-12);
}

TEST(OdinTransform, LocalVelocityAndCorrectionDirection)
{
  const auto previous = odinTransform(0, 0, 0, 0, 0, 0, 1);
  const auto current = odinTransform(0.1, 0, 0, 0, 0, std::sin(0.05), std::cos(0.05));
  const auto twist = odinLocalTwist(previous, current, 0.1);
  EXPECT_NEAR(twist.linear.x, std::cos(0.1), 1e-10);
  EXPECT_NEAR(twist.linear.y, -std::sin(0.1), 1e-10);
  EXPECT_NEAR(twist.angular.z, 1.0, 1e-10);
  const auto native = odinTransform(2, 3, 0, 0, 0, std::sin(0.3), std::cos(0.3));
  const auto alignment = odinTransform(-4, 0, 0, 0, 0, 0, 1);
  EXPECT_TRUE(((alignment * native.inverse()) * native).matrix().isApprox(alignment.matrix()));
  // Corrections have no input to local differentiation.
  EXPECT_DOUBLE_EQ(odinLocalTwist(previous, current, 0.1).linear.x, twist.linear.x);
}

TEST(OdinTransform, RejectMalformedGeometry)
{
  EXPECT_THROW(odinTransform(0, 0, 0, 0, 0, 0, 0), std::invalid_argument);
  EXPECT_THROW(odinTransform(std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 0, 0, 1),
    std::invalid_argument);
  EXPECT_THROW(odinLocalTwist(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 0),
    std::invalid_argument);
}

TEST(LocalizationGate, MissingFalseAndStaleReadiness)
{
  x2_navigation::LocalizationGate gate;
  const auto now = x2_navigation::LocalizationGate::Clock::now();
  const auto timeout = std::chrono::duration<double>(0.25);
  EXPECT_FALSE(gate.ready(now, timeout));
  gate.update(true, now);
  EXPECT_TRUE(gate.ready(now + std::chrono::milliseconds(200), timeout));
  EXPECT_FALSE(gate.ready(now + std::chrono::milliseconds(300), timeout));
  gate.update(false, now);
  EXPECT_FALSE(gate.ready(now, timeout));
}
