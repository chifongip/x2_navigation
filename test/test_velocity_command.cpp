#include <chrono>
#include <limits>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include "x2_navigation/timing.hpp"
#include "x2_navigation/velocity_command.hpp"

namespace
{

TEST(VelocityCommand, ClampsNavigationVelocityAndForcesUnusedAxesToZero)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  twist.linear.y = 0.25;
  twist.linear.z = -0.25;
  twist.angular.x = 0.25;
  twist.angular.y = -0.25;
  twist.angular.z = -1.0;

  const auto command = x2_navigation::navigationVelocityCommand(twist);

  ASSERT_TRUE(command.has_value());
  EXPECT_DOUBLE_EQ(command->linear_x, 0.5);
  EXPECT_DOUBLE_EQ(command->linear_y, 0.0);
  EXPECT_DOUBLE_EQ(command->linear_z, 0.0);
  EXPECT_DOUBLE_EQ(command->angular_x, 0.0);
  EXPECT_DOUBLE_EQ(command->angular_y, 0.0);
  EXPECT_DOUBLE_EQ(command->angular_z, -0.5);
}

TEST(VelocityCommand, RejectsNonFiniteTwistComponents)
{
  geometry_msgs::msg::Twist twist;
  twist.angular.y = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(x2_navigation::navigationVelocityCommand(twist).has_value());
}

TEST(VelocityCommand, SerializesCompleteTwistShape)
{
  x2_navigation::VelocityCommand command;
  command.linear_x = 0.25;
  command.angular_z = -0.5;

  EXPECT_EQ(
    x2_navigation::velocityCommandJson(command),
    "{\"linear\":{\"x\":0.25,\"y\":0,\"z\":0},\"angular\":{\"x\":0,\"y\":0,\"z\":-0.5}}");
}

TEST(VelocityCommand, PublishesZeroWhenCommandIsMissingOrStale)
{
  x2_navigation::VelocityCommandWatchdog watchdog;
  const auto start = x2_navigation::VelocityCommandWatchdog::Clock::now();
  const auto timeout = std::chrono::duration<double>(0.20);

  EXPECT_DOUBLE_EQ(watchdog.commandAt(start, timeout).linear_x, 0.0);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.3;
  ASSERT_TRUE(watchdog.update(twist, start));
  EXPECT_DOUBLE_EQ(
    watchdog.commandAt(start + std::chrono::milliseconds(199), timeout).linear_x, 0.3);
  EXPECT_DOUBLE_EQ(
    watchdog.commandAt(start + std::chrono::milliseconds(201), timeout).linear_x, 0.0);
}

TEST(VelocityCommand, ClearsOutputForInvalidCommand)
{
  x2_navigation::VelocityCommandWatchdog watchdog;
  const auto start = x2_navigation::VelocityCommandWatchdog::Clock::now();
  const auto timeout = std::chrono::duration<double>(0.20);

  geometry_msgs::msg::Twist valid;
  valid.linear.x = 0.3;
  ASSERT_TRUE(watchdog.update(valid, start));

  geometry_msgs::msg::Twist invalid;
  invalid.linear.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(watchdog.update(invalid, start + std::chrono::milliseconds(10)));
  EXPECT_DOUBLE_EQ(
    watchdog.commandAt(start + std::chrono::milliseconds(10), timeout).linear_x, 0.0);
}

TEST(VelocityCommand, RejectsInvalidWatchdogTimeouts)
{
  x2_navigation::VelocityCommandWatchdog watchdog;
  const auto start = x2_navigation::VelocityCommandWatchdog::Clock::now();
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.3;
  ASSERT_TRUE(watchdog.update(twist, start));

  EXPECT_DOUBLE_EQ(
    watchdog.commandAt(start, std::chrono::duration<double>::zero()).linear_x, 0.0);
  EXPECT_DOUBLE_EQ(
    watchdog.commandAt(
      start, std::chrono::duration<double>(std::numeric_limits<double>::quiet_NaN())).linear_x,
    0.0);
}

TEST(VelocityCommand, ConvertsOnlyRepresentablePositiveRatesToPeriods)
{
  const auto period = x2_navigation::periodFromRateHz(20.0);
  ASSERT_TRUE(period.has_value());
  EXPECT_EQ(*period, std::chrono::milliseconds(50));

  EXPECT_FALSE(x2_navigation::periodFromRateHz(0.0).has_value());
  EXPECT_FALSE(x2_navigation::periodFromRateHz(-1.0).has_value());
  EXPECT_FALSE(
    x2_navigation::periodFromRateHz(std::numeric_limits<double>::infinity()).has_value());
  EXPECT_FALSE(
    x2_navigation::periodFromRateHz(std::numeric_limits<double>::max()).has_value());
  EXPECT_FALSE(x2_navigation::periodFromRateHz(std::numeric_limits<double>::min()).has_value());
}

TEST(VelocityCommand, ConvertsOnlyRepresentableTimestampOffsets)
{
  const auto offset = x2_navigation::nanosecondsFromSeconds(-0.125);
  ASSERT_TRUE(offset.has_value());
  EXPECT_EQ(*offset, std::chrono::milliseconds(-125));

  EXPECT_FALSE(
    x2_navigation::nanosecondsFromSeconds(std::numeric_limits<double>::quiet_NaN()).has_value());
  EXPECT_FALSE(
    x2_navigation::nanosecondsFromSeconds(std::numeric_limits<double>::max()).has_value());
}

}  // namespace
