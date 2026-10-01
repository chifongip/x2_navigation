#include <chrono>
#include <limits>

#include <gtest/gtest.h>

#include "x2_navigation/navigation_command_gate.hpp"

namespace
{
using namespace std::chrono_literals;
using x2_navigation::NavigationCommandGate;
const NavigationCommandGate::TimePoint start{};
const NavigationCommandGate::Timeout timeout{0.2};

geometry_msgs::msg::Twist movingCommand()
{
  geometry_msgs::msg::Twist command;
  command.angular.z = 0.7;
  return command;
}

TEST(NavigationCommandGate, LegacyInputNeedsNoRawStream)
{
  NavigationCommandGate gate;
  gate.update(movingCommand(), start, timeout);
  EXPECT_DOUBLE_EQ(gate.commandAt(start + 200ms, timeout).angular.z, 0.7);
  EXPECT_EQ(gate.commandAt(start + 201ms, timeout), geometry_msgs::msg::Twist{});
}

TEST(NavigationCommandGate, SmootherCannotExtendOriginalCommandLifetime)
{
  NavigationCommandGate gate(true);
  gate.updateRaw(movingCommand(), start, timeout);
  gate.update(movingCommand(), start + 190ms, timeout);
  EXPECT_DOUBLE_EQ(gate.commandAt(start + 200ms, timeout).angular.z, 0.7);
  EXPECT_EQ(gate.commandAt(start + 201ms, timeout), geometry_msgs::msg::Twist{});
  gate.update(movingCommand(), start + 210ms, timeout);
  EXPECT_EQ(gate.commandAt(start + 210ms, timeout), geometry_msgs::msg::Twist{});
}

TEST(NavigationCommandGate, ExplicitStopClearsCacheAndRequiresBothStreamsToResume)
{
  NavigationCommandGate gate(true);
  gate.updateRaw(movingCommand(), start, timeout);
  gate.update(movingCommand(), start + 1ms, timeout);
  gate.updateRaw(geometry_msgs::msg::Twist{}, start + 2ms, timeout);
  EXPECT_EQ(gate.commandAt(start + 2ms, timeout), geometry_msgs::msg::Twist{});
  gate.update(movingCommand(), start + 3ms, timeout);
  gate.updateRaw(movingCommand(), start + 4ms, timeout);
  EXPECT_EQ(gate.commandAt(start + 4ms, timeout), geometry_msgs::msg::Twist{});
  gate.update(movingCommand(), start + 6ms, timeout);
  EXPECT_DOUBLE_EQ(gate.commandAt(start + 6ms, timeout).angular.z, 0.7);
}

TEST(NavigationCommandGate, RawResumptionDoesNotReuseExpiredSmoothedCommand)
{
  NavigationCommandGate gate(true);
  gate.updateRaw(movingCommand(), start, timeout);
  gate.update(movingCommand(), start + 1ms, timeout);
  gate.updateRaw(movingCommand(), start + 300ms, timeout);
  EXPECT_EQ(gate.commandAt(start + 300ms, timeout), geometry_msgs::msg::Twist{});
}

TEST(NavigationCommandGate, FreshRawStreamCannotHideMissingSmoother)
{
  NavigationCommandGate gate(true);
  gate.updateRaw(movingCommand(), start, timeout);
  gate.update(movingCommand(), start + 1ms, timeout);
  gate.updateRaw(movingCommand(), start + 150ms, timeout);
  gate.updateRaw(movingCommand(), start + 250ms, timeout);
  EXPECT_EQ(gate.commandAt(start + 250ms, timeout), geometry_msgs::msg::Twist{});
}

TEST(NavigationCommandGate, NonFiniteInputInvalidatesEitherStream)
{
  for (const bool invalid_raw : {false, true}) {
    NavigationCommandGate gate(true);
    gate.updateRaw(movingCommand(), start, timeout);
    gate.update(movingCommand(), start + 1ms, timeout);
    auto invalid = movingCommand();
    invalid.linear.y = std::numeric_limits<double>::quiet_NaN();
    if (invalid_raw) {
      gate.updateRaw(invalid, start + 2ms, timeout);
    } else {
      gate.update(invalid, start + 2ms, timeout);
    }
    EXPECT_EQ(gate.commandAt(start + 2ms, timeout), geometry_msgs::msg::Twist{});
  }
}
}  // namespace
