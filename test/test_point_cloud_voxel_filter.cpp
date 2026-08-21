#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "x2_navigation/point_cloud_voxel_filter.hpp"

namespace
{

sensor_msgs::msg::PointCloud2 makeCloud(const std::vector<float> & points)
{
  const auto make_field = [](const std::string & name, std::uint32_t offset) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1U;
      return field;
    };

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar_chest_front";
  cloud.height = 1U;
  cloud.width = static_cast<std::uint32_t>(points.size() / 3U);
  cloud.fields = {make_field("x", 0U), make_field("y", 4U), make_field("z", 8U)};
  cloud.is_bigendian = false;
  cloud.point_step = 3U * sizeof(float);
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
  cloud.is_dense = true;
  return cloud;
}

geometry_msgs::msg::TransformStamped makeTransform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.translation.x = 1.0;
  transform.transform.translation.z = -0.1;
  transform.transform.rotation.w = 1.0;
  return transform;
}

float readOutputValue(const sensor_msgs::msg::PointCloud2 & cloud, std::size_t point, std::size_t offset)
{
  float value;
  std::memcpy(&value, cloud.data.data() + point * cloud.point_step + offset, sizeof(value));
  return value;
}

}  // namespace

TEST(PointCloudVoxelFilter, TransformsCropsAndDownsamplesToCompactXyz)
{
  const auto input = makeCloud({
      0.01F, 0.00F, 0.10F,
      0.04F, 0.02F, 0.12F,
      0.06F, 0.00F, 0.10F,
      0.10F, 0.00F, 0.50F,
      std::numeric_limits<float>::quiet_NaN(), 0.00F, 0.10F,
    });
  std_msgs::msg::Header output_header;
  output_header.frame_id = "base_link";
  output_header.stamp.sec = 42;

  const auto result = x2_navigation::filterPointCloudToVoxels(
    input, makeTransform(), output_header,
    x2_navigation::PointCloudVoxelFilterConfig{0.05, -0.45, 0.30, 40000U});

  ASSERT_TRUE(result);
  EXPECT_EQ(result->input_point_count, 5U);
  EXPECT_EQ(result->sampled_point_count, 5U);
  EXPECT_EQ(result->retained_point_count, 2U);
  EXPECT_EQ(result->cloud.header.frame_id, "base_link");
  EXPECT_EQ(result->cloud.header.stamp.sec, 42);
  EXPECT_EQ(result->cloud.height, 1U);
  EXPECT_EQ(result->cloud.width, 2U);
  EXPECT_EQ(result->cloud.point_step, 12U);
  ASSERT_EQ(result->cloud.fields.size(), 3U);
  EXPECT_EQ(result->cloud.fields[0].name, "x");
  EXPECT_EQ(result->cloud.fields[1].name, "y");
  EXPECT_EQ(result->cloud.fields[2].name, "z");

  EXPECT_NEAR(readOutputValue(result->cloud, 0U, 0U), 1.025F, 1e-6F);
  EXPECT_NEAR(readOutputValue(result->cloud, 0U, 4U), 0.01F, 1e-6F);
  EXPECT_NEAR(readOutputValue(result->cloud, 0U, 8U), 0.01F, 1e-6F);
  EXPECT_NEAR(readOutputValue(result->cloud, 1U, 0U), 1.06F, 1e-6F);
  EXPECT_NEAR(readOutputValue(result->cloud, 1U, 4U), 0.0F, 1e-6F);
  EXPECT_NEAR(readOutputValue(result->cloud, 1U, 8U), 0.0F, 1e-6F);
}

TEST(PointCloudVoxelFilter, RejectsCloudsWithoutFloat32XyzFields)
{
  auto input = makeCloud({0.0F, 0.0F, 0.0F});
  input.fields[2].datatype = sensor_msgs::msg::PointField::UINT32;

  const auto result = x2_navigation::filterPointCloudToVoxels(
    input, makeTransform(), std_msgs::msg::Header{},
    x2_navigation::PointCloudVoxelFilterConfig{0.05, -0.45, 0.30, 40000U});

  EXPECT_FALSE(result);
}

TEST(PointCloudVoxelFilter, RejectsMalformedCloudWithZeroPointStride)
{
  auto input = makeCloud({0.0F, 0.0F, 0.0F});
  input.point_step = 0U;

  const auto result = x2_navigation::filterPointCloudToVoxels(
    input, makeTransform(), std_msgs::msg::Header{},
    x2_navigation::PointCloudVoxelFilterConfig{0.05, -0.45, 0.30, 40000U});

  EXPECT_FALSE(result);
}

TEST(PointCloudVoxelFilter, BoundsInputWorkWithUniformSampling)
{
  const auto input = makeCloud({
      0.00F, 0.00F, 0.00F,
      0.10F, 0.00F, 0.00F,
      0.20F, 0.00F, 0.00F,
      0.30F, 0.00F, 0.00F,
      0.40F, 0.00F, 0.00F,
    });

  const auto result = x2_navigation::filterPointCloudToVoxels(
    input, makeTransform(), std_msgs::msg::Header{},
    x2_navigation::PointCloudVoxelFilterConfig{0.05, -0.45, 0.30, 2U});

  ASSERT_TRUE(result);
  EXPECT_EQ(result->input_point_count, 5U);
  EXPECT_EQ(result->sampled_point_count, 2U);
  EXPECT_EQ(result->retained_point_count, 2U);
  EXPECT_NEAR(readOutputValue(result->cloud, 0U, 0U), 1.0F, 1e-6F);
  EXPECT_NEAR(readOutputValue(result->cloud, 1U, 0U), 1.4F, 1e-6F);
}

TEST(PointCloudVoxelFilter, SamplesExactlyTheLimitJustAboveTheBoundary)
{
  const auto input = makeCloud({
      0.00F, 0.00F, 0.00F,
      0.10F, 0.00F, 0.00F,
      0.20F, 0.00F, 0.00F,
      0.30F, 0.00F, 0.00F,
      0.40F, 0.00F, 0.00F,
      0.50F, 0.00F, 0.00F,
    });

  const auto result = x2_navigation::filterPointCloudToVoxels(
    input, makeTransform(), std_msgs::msg::Header{},
    x2_navigation::PointCloudVoxelFilterConfig{0.05, -0.45, 0.30, 5U});

  ASSERT_TRUE(result);
  EXPECT_EQ(result->input_point_count, 6U);
  EXPECT_EQ(result->sampled_point_count, 5U);
  EXPECT_EQ(result->retained_point_count, 5U);
  EXPECT_NEAR(readOutputValue(result->cloud, 0U, 0U), 1.0F, 1e-6F);
  EXPECT_NEAR(readOutputValue(result->cloud, 4U, 0U), 1.5F, 1e-6F);
}

TEST(PointCloudVoxelFilter, SamplesOrganizedCloudsWithPaddedRows)
{
  const auto source = makeCloud({
      0.00F, 0.00F, 0.00F,
      0.10F, 0.00F, 0.00F,
      0.20F, 0.00F, 0.00F,
      0.30F, 0.00F, 0.00F,
      0.40F, 0.00F, 0.00F,
      0.50F, 0.00F, 0.00F,
    });
  auto input = source;
  input.width = 3U;
  input.height = 2U;
  input.row_step = input.width * input.point_step + 8U;
  input.data.assign(static_cast<std::size_t>(input.height) * input.row_step, 0U);
  for (std::size_t index = 0U; index < 6U; ++index) {
    const auto row = index / input.width;
    const auto column = index % input.width;
    std::memcpy(
      input.data.data() + row * input.row_step + column * input.point_step,
      source.data.data() + index * source.point_step, source.point_step);
  }

  const auto result = x2_navigation::filterPointCloudToVoxels(
    input, makeTransform(), std_msgs::msg::Header{},
    x2_navigation::PointCloudVoxelFilterConfig{0.05, -0.45, 0.30, 2U});

  ASSERT_TRUE(result);
  EXPECT_EQ(result->sampled_point_count, 2U);
  ASSERT_EQ(result->retained_point_count, 2U);
  EXPECT_NEAR(readOutputValue(result->cloud, 0U, 0U), 1.0F, 1e-6F);
  EXPECT_NEAR(readOutputValue(result->cloud, 1U, 0U), 1.5F, 1e-6F);
}
