#ifndef X2_NAVIGATION__POINT_CLOUD_VOXEL_FILTER_HPP_
#define X2_NAVIGATION__POINT_CLOUD_VOXEL_FILTER_HPP_

#include <cstddef>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace x2_navigation
{

struct PointCloudVoxelFilterConfig
{
  double voxel_size;
  double min_height;
  double max_height;
};

struct PointCloudVoxelFilterResult
{
  sensor_msgs::msg::PointCloud2 cloud;
  std::size_t input_point_count;
  std::size_t retained_point_count;
};

std::optional<PointCloudVoxelFilterResult> filterPointCloudToVoxels(
  const sensor_msgs::msg::PointCloud2 & input,
  const geometry_msgs::msg::TransformStamped & target_from_source,
  const std_msgs::msg::Header & output_header,
  const PointCloudVoxelFilterConfig & config);

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__POINT_CLOUD_VOXEL_FILTER_HPP_
