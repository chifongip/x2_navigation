#include "x2_navigation/docking_tracking.hpp"

#include <gtest/gtest.h>

namespace
{
using namespace x2_navigation;

TEST(RelativeTagTracker, AcquiresThenUpdatesWithoutAnotherBatch)
{
  RelativeTagTracker tracker({3, 0.5, 0.15, 0.02, 0.05, 0.1, 0.1});
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.1;
  ASSERT_TRUE(tracker.observe({0.6, 0.0, 0.0}, 1.0, command));
  EXPECT_FALSE(tracker.ready());
  ASSERT_TRUE(tracker.observe({0.59, 0.0, 0.0}, 1.1, command));
  ASSERT_TRUE(tracker.observe({0.58, 0.0, 0.0}, 1.2, command));
  EXPECT_TRUE(tracker.ready());
  ASSERT_TRUE(tracker.observe({0.57, 0.0, 0.0}, 1.3, command));
  EXPECT_NEAR(tracker.error().x, 0.57, 1e-9);
}

TEST(RelativeTagTracker, RejectsOutliersAndDuplicateEvidence)
{
  RelativeTagTracker tracker({1, 0.5, 0.15, 0.02, 0.05, 0.1, 0.1});
  geometry_msgs::msg::Twist command;
  ASSERT_TRUE(tracker.observe({0.6, 0.0, 0.0}, 1.0, command));
  EXPECT_FALSE(tracker.observe({3.0, 0.0, 0.0}, 1.1, command));
  EXPECT_FALSE(tracker.observe({0.6, 0.0, 0.0}, 1.0, command));
  ASSERT_TRUE(tracker.observe({0.61, 0.0, 0.0}, 1.2, command));
  EXPECT_LT(tracker.error().x, 0.61);
}

TEST(RelativeTagTracker, DropoutRequiresNewAcquisition)
{
  RelativeTagTracker tracker({2, 0.5, 0.0, 0.02, 0.05, 0.1, 0.1});
  geometry_msgs::msg::Twist command;
  tracker.observe({0.6, 0.0, 0.0}, 1.0, command);
  tracker.observe({0.6, 0.0, 0.0}, 1.1, command);
  ASSERT_TRUE(tracker.ready());
  tracker.observe({0.6, 0.0, 0.0}, 2.0, command);
  EXPECT_FALSE(tracker.ready());
  tracker.observe({0.6, 0.0, 0.0}, 2.1, command);
  EXPECT_TRUE(tracker.ready());
}

TEST(RelativeTagTracker, DefaultTimeoutSupportsOneHertzAcquisition)
{
  RelativeTagTracker tracker(TagTrackingConfig{});
  geometry_msgs::msg::Twist command;
  tracker.observe({0.6, 0.0, 0.0}, 1.0, command);
  tracker.observe({0.6, 0.0, 0.0}, 2.0, command);
  tracker.observe({0.6, 0.0, 0.0}, 3.0, command);
  EXPECT_TRUE(tracker.ready());
  ASSERT_TRUE(tracker.observe({0.59, 0.0, 0.0}, 4.0, command));
  EXPECT_TRUE(tracker.ready());
}

TEST(RelativeTagTracker, FiltersYawAcrossPiWithoutJump)
{
  RelativeTagTracker tracker({1, 0.5, 0.15, 0.02, 0.05, 0.1, 0.1});
  geometry_msgs::msg::Twist command;
  tracker.observe({0.6, 0.0, 3.13}, 1.0, command);
  ASSERT_TRUE(tracker.observe({0.6, 0.0, -3.13}, 1.1, command));
  EXPECT_LT(std::abs(wrapAngle(tracker.error().yaw - 3.13)), 0.03);
}

TEST(RelativeTagTracker, AccountsForRotationDuringMotion)
{
  RelativeTagTracker tracker({1, 0.5, 0.15, 0.02, 0.05, 0.1, 0.1});
  geometry_msgs::msg::Twist command;
  command.angular.z = 0.1;
  const PlanarError initial{1.5, 0.2, 0.3};
  tracker.observe(initial, 1.0, command);
  const auto expected = predictRelativeError(initial, command, 0.1);
  ASSERT_TRUE(tracker.observe(expected, 1.1, command));
  EXPECT_NEAR(tracker.error().x, expected.x, 1e-9);
  EXPECT_NEAR(tracker.error().y, expected.y, 1e-9);
}

HolonomicFineAlignConfig minimumSpeedConfig()
{
  HolonomicFineAlignConfig config;
  config.translation_gain = 1.0;
  config.translation_speed_min = config.translation_speed_max = 0.1;
  config.angular_speed_min = config.angular_speed_max = 0.1;
  config.allow_reverse_x = true;
  return config;
}

TEST(DockingMotion, ConfirmsStartsAndPreservesMinimumMagnitude)
{
  DockingMotionController controller(minimumSpeedConfig(), {});
  auto command = controller.update({0.6, 0.2, 0.1}, 0.05, true);
  ASSERT_TRUE(command);
  EXPECT_DOUBLE_EQ(command->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command->angular.z, 0.0);
  command = controller.update({0.6, 0.2, 0.1}, 0.05, false);
  EXPECT_DOUBLE_EQ(command->linear.x, 0.0);
  command = controller.update({0.6, 0.2, 0.1}, 0.05, true);
  EXPECT_NEAR(std::hypot(command->linear.x, command->linear.y), 0.1, 1e-9);
}

TEST(DockingMotion, DoesNotChatterAtPositionTolerance)
{
  DockingMotionController controller(minimumSpeedConfig(), {});
  controller.update({0.2, 0.0, 0.0}, 0.05, true);
  ASSERT_GT(controller.update({0.2, 0.0, 0.0}, 0.05, true)->linear.x, 0.0);
  EXPECT_GT(controller.update({0.049, 0.0, 0.0}, 0.05, true)->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(controller.update({0.029, 0.0, 0.0}, 0.05, true)->linear.x, 0.0);
  for (int i = 0; i < 10; ++i) {
    EXPECT_DOUBLE_EQ(controller.update({0.049, 0.0, 0.0}, 0.05, true)->linear.x, 0.0);
  }
  EXPECT_DOUBLE_EQ(controller.update({0.051, 0.0, 0.0}, 0.05, true)->linear.x, 0.0);
  EXPECT_GT(controller.update({0.051, 0.0, 0.0}, 0.05, true)->linear.x, 0.0);
}

TEST(DockingMotion, CorrectsReportedErrorInsteadOfWaitingOutsideCompletion)
{
  auto config = minimumSpeedConfig();
  config.x_position_tolerance = config.y_position_tolerance = 0.1;
  config.yaw_tolerance = 0.174532925;
  DockingMotionController controller(config, {});
  const PlanarError reported{0.112, -0.085, 0.007};
  EXPECT_DOUBLE_EQ(controller.update(reported, 0.05, true)->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(controller.update(reported, 0.05, false)->linear.x, 0.0);
  const auto moving = controller.update(reported, 0.05, true);
  EXPECT_NEAR(moving->linear.x, 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(moving->linear.y, 0.0);
  EXPECT_GT(controller.update({0.09, -0.085, 0.007}, 0.05, true)->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(controller.update({0.079, -0.085, 0.007}, 0.05, true)->linear.x, 0.0);
}

TEST(DockingMotion, CorrectsYawJustOutsideCompletionTolerance)
{
  auto config = minimumSpeedConfig();
  config.yaw_tolerance = 0.174532925;
  DockingMotionController controller(config, {});
  controller.update({0.0, 0.0, -0.18}, 0.05, true);
  EXPECT_NEAR(controller.update({0.0, 0.0, -0.18}, 0.05, true)->angular.z, -0.1, 1e-9);
  EXPECT_NEAR(controller.update({0.0, 0.0, -0.15}, 0.05, true)->angular.z, -0.1, 1e-9);
  EXPECT_DOUBLE_EQ(controller.update({0.0, 0.0, -0.13}, 0.05, true)->angular.z, 0.0);
  EXPECT_GT(dockingStopTolerance(0.01, 0.02), 0.0);
}

TEST(DockingMotion, StopsBeforeConfirmedReversal)
{
  DockingMotionController controller(minimumSpeedConfig(), {});
  controller.update({0.2, 0.0, 0.0}, 0.05, true);
  controller.update({0.2, 0.0, 0.0}, 0.05, true);
  EXPECT_DOUBLE_EQ(controller.update({-0.2, 0.0, 0.0}, 0.05, true)->linear.x, 0.0);
  EXPECT_LT(controller.update({-0.2, 0.0, 0.0}, 0.05, true)->linear.x, 0.0);
}

TEST(DockingMotion, LimitsDirectionChangeWithoutReducingSpeed)
{
  DockingMotionController controller(minimumSpeedConfig(), {});
  controller.update({0.6, 0.2, 0.0}, 0.05, true);
  const auto before = controller.update({0.6, 0.2, 0.0}, 0.05, true);
  const auto after = controller.update({0.2, 0.6, 0.0}, 0.05, true);
  const double delta = wrapAngle(std::atan2(after->linear.y, after->linear.x) -
    std::atan2(before->linear.y, before->linear.x));
  EXPECT_LE(std::abs(delta), 0.05 + 1e-9);
  EXPECT_NEAR(std::hypot(after->linear.x, after->linear.y), 0.1, 1e-9);
}

TEST(DockingMotion, YawGateRequiresLowerThresholdToResume)
{
  DockingMotionController controller(minimumSpeedConfig(), {});
  controller.update({0.6, 0.0, 0.0}, 0.05, true);
  controller.update({0.6, 0.0, 0.0}, 0.05, true);
  EXPECT_DOUBLE_EQ(controller.update({0.6, 0.0, 0.36}, 0.05, true)->linear.x, 0.0);
  for (int i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(controller.update({0.6, 0.0, 0.34}, 0.05, true)->linear.x, 0.0);
  }
  controller.update({0.6, 0.0, 0.3}, 0.05, true);
  EXPECT_GT(controller.update({0.6, 0.0, 0.3}, 0.05, true)->linear.x, 0.0);
}

TEST(TagPoseSettling, RequiresNewObservationsAndResetsOnMotion)
{
  TagPoseSettling settling;
  const PlanarError initial{0.05, 0.01, 3.13};
  EXPECT_FALSE(settling.update(true, 1.0, 1, initial, 0.5, 0.02, 0.04));
  EXPECT_FALSE(settling.update(true, 2.0, 1, initial, 0.5, 0.02, 0.04));
  EXPECT_TRUE(settling.update(true, 2.1, 2, {0.051, 0.01, -3.13}, 0.5, 0.02, 0.04));
  EXPECT_FALSE(settling.update(true, 2.2, 3, {0.08, 0.01, -3.13}, 0.5, 0.02, 0.04));
  EXPECT_FALSE(settling.update(true, 2.3, 4, {0.08, 0.01, -3.13}, 0.5, 0.02, 0.04));
  EXPECT_TRUE(settling.update(true, 2.8, 5, {0.08, 0.01, -3.13}, 0.5, 0.02, 0.04));
  EXPECT_FALSE(settling.update(false, 2.9, 6, initial, 0.5, 0.02, 0.04));
  EXPECT_FALSE(settling.update(true, 3.0, 7, initial, 0.5, 0.02, 0.04));
}

TEST(TagPoseSettling, CanStartAtZeroAndNeedsANewObservation)
{
  TagPoseSettling settling;
  const PlanarError pose{};
  EXPECT_FALSE(settling.update(true, 0.0, 0, pose, 0.5, 0.02, 0.04));
  EXPECT_FALSE(settling.update(true, 1.0, 0, pose, 0.5, 0.02, 0.04));
  EXPECT_TRUE(settling.update(true, 1.1, 1, pose, 0.5, 0.02, 0.04));
}

TEST(TagPoseSettling, InvalidObservationsRestartTheFullSettlingWindow)
{
  TagPoseSettling settling;
  const PlanarError pose{};
  EXPECT_FALSE(settling.update(false, 0.0, 0, pose, 0.5, 0.02, 0.04));
  EXPECT_FALSE(settling.update(true, 1.0, 1, pose, 0.5, 0.02, 0.04));
  EXPECT_TRUE(settling.update(true, 2.0, 2, pose, 0.5, 0.02, 0.04));
  EXPECT_FALSE(settling.update(false, 3.0, 3, pose, 0.5, 0.02, 0.04));
  EXPECT_FALSE(settling.update(true, 4.0, 4, pose, 0.5, 0.02, 0.04));
  EXPECT_FALSE(settling.update(true, 4.25, 5, pose, 0.5, 0.02, 0.04));
  EXPECT_TRUE(settling.update(true, 4.5, 6, pose, 0.5, 0.02, 0.04));
}

TEST(TagRelativeUndocking, MeasuresRetreatInInitialRobotFrame)
{
  const auto initial_to_dock = planarPose({0.6, 0.2, 0.3});
  const auto robot_pose = planarPose({-0.3, 0.0, 0.0});
  const auto current_to_dock = robot_pose.inverse() * initial_to_dock;
  const auto target = tagRelativeUndockTarget(initial_to_dock, current_to_dock, 0.3);
  const auto error = planarError(Eigen::Isometry3d::Identity(), target);
  EXPECT_NEAR(error.x, 0.0, 1e-9);
  EXPECT_NEAR(error.y, 0.0, 1e-9);
  EXPECT_NEAR(error.yaw, 0.0, 1e-9);
  const auto measured = initial_to_dock * current_to_dock.inverse();
  EXPECT_NEAR(-measured.translation().x(), 0.3, 1e-9);
}

TEST(TagRelativeUndocking, CorrectsLateralAndYawDrift)
{
  const auto initial = planarPose({0.6, -0.2, -0.3});
  const auto robot = planarPose({-0.05, 0.1, 0.2});
  const auto current = robot.inverse() * initial;
  const auto target = tagRelativeUndockTarget(initial, current, 0.3);
  const auto expected = robot.inverse() * planarPose({-0.3, 0.0, 0.0});
  EXPECT_TRUE(target.matrix().isApprox(expected.matrix(), 1e-9));
  const auto error = planarError(Eigen::Isometry3d::Identity(), target);
  EXPECT_LT(error.x, 0.0);
  EXPECT_LT(error.y, 0.0);
  EXPECT_NEAR(error.yaw, -0.2, 1e-9);
}
}  // namespace
