#include "x2_navigation/holonomic_fine_align.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace
{

using x2_navigation::HolonomicFineAlignConfig;
using x2_navigation::PlanarError;
using x2_navigation::fineAlignAtGoal;
using x2_navigation::holonomicFineAlignCommand;

TEST(HolonomicFineAlign, CommandsForwardLateralAndYawTogether)
{
  const auto command = holonomicFineAlignCommand({0.4, -0.3, 0.1}, {});
  ASSERT_TRUE(command);
  EXPECT_GT(command->linear.x, 0.0);
  EXPECT_LT(command->linear.y, 0.0);
  EXPECT_GT(command->angular.z, 0.0);
  EXPECT_NEAR(std::hypot(command->linear.x, command->linear.y), 0.1385408, 1e-6);
}

TEST(HolonomicFineAlign, PreservesDirectionAndBoundsPlanarMagnitude)
{
  const auto maximum = holonomicFineAlignCommand({2.0, 2.0, 0.0}, {});
  ASSERT_TRUE(maximum);
  EXPECT_NEAR(maximum->linear.x, maximum->linear.y, 1e-9);
  EXPECT_NEAR(std::hypot(maximum->linear.x, maximum->linear.y), 0.15, 1e-9);

  const auto minimum = holonomicFineAlignCommand({0.051, 0.0, 0.0}, {});
  ASSERT_TRUE(minimum);
  EXPECT_NEAR(minimum->linear.x, 0.11, 1e-9);
}

TEST(HolonomicFineAlign, StopsTranslationAtLargeYawAndBoundsRotation)
{
  const auto command = holonomicFineAlignCommand({0.4, 0.2, 0.4}, {});
  ASSERT_TRUE(command);
  EXPECT_DOUBLE_EQ(command->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command->linear.y, 0.0);
  EXPECT_DOUBLE_EQ(command->angular.z, 0.25);
}

TEST(HolonomicFineAlign, ReverseXIsOptInAndUsesPlanarSpeedRange)
{
  const auto forward_only = holonomicFineAlignCommand({-0.2, 0.0, 0.0}, {});
  ASSERT_TRUE(forward_only);
  EXPECT_DOUBLE_EQ(forward_only->linear.x, 0.0);

  HolonomicFineAlignConfig reverse;
  reverse.allow_reverse_x = true;
  const auto reverse_command = holonomicFineAlignCommand({-0.2, 0.0, 0.0}, reverse);
  ASSERT_TRUE(reverse_command);
  EXPECT_DOUBLE_EQ(reverse_command->linear.x, -0.11);
  EXPECT_NEAR(
    std::hypot(reverse_command->linear.x, reverse_command->linear.y),
    reverse.translation_speed_min, 1e-9);
}

TEST(HolonomicFineAlign, ReportsGoalOnlyInsideAxisAndYawTolerances)
{
  const HolonomicFineAlignConfig config;
  EXPECT_TRUE(fineAlignAtGoal({0.03, 0.04, 0.08}, config));
  EXPECT_FALSE(fineAlignAtGoal({0.051, 0.0, 0.0}, config));
  EXPECT_FALSE(fineAlignAtGoal({0.0, 0.051, 0.0}, config));
  EXPECT_FALSE(fineAlignAtGoal({0.0, 0.0, 0.09}, config));
  const auto command = holonomicFineAlignCommand({0.03, 0.04, 0.08}, config);
  ASSERT_TRUE(command);
  EXPECT_DOUBLE_EQ(command->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command->linear.y, 0.0);
  EXPECT_DOUBLE_EQ(command->angular.z, 0.0);
}

TEST(HolonomicFineAlign, AppliesIndependentPositionTolerancesToEachAxis)
{
  HolonomicFineAlignConfig config;
  config.x_position_tolerance = 0.04;
  config.y_position_tolerance = 0.02;

  EXPECT_TRUE(fineAlignAtGoal({0.04, 0.02, 0.0}, config));
  EXPECT_FALSE(fineAlignAtGoal({0.041, 0.02, 0.0}, config));
  EXPECT_FALSE(fineAlignAtGoal({0.04, 0.021, 0.0}, config));

  const auto lateral_only = holonomicFineAlignCommand({0.03, 0.04, 0.0}, config);
  ASSERT_TRUE(lateral_only);
  EXPECT_DOUBLE_EQ(lateral_only->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(lateral_only->linear.y, config.translation_speed_min);

  const auto forward_only = holonomicFineAlignCommand({0.05, 0.01, 0.0}, config);
  ASSERT_TRUE(forward_only);
  EXPECT_DOUBLE_EQ(forward_only->linear.x, config.translation_speed_min);
  EXPECT_DOUBLE_EQ(forward_only->linear.y, 0.0);
}

TEST(HolonomicFineAlign, RejectsInvalidInputsAndConfiguration)
{
  HolonomicFineAlignConfig invalid;
  invalid.translation_speed_max = invalid.translation_speed_min - 0.01;
  EXPECT_FALSE(holonomicFineAlignCommand({0.1, 0.0, 0.0}, invalid));
  invalid = {};
  invalid.y_position_tolerance = 0.0;
  EXPECT_FALSE(holonomicFineAlignCommand({0.1, 0.0, 0.0}, invalid));
  EXPECT_FALSE(holonomicFineAlignCommand(
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, {}));
}

}  // namespace
