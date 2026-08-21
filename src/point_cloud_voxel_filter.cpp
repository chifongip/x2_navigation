#include "x2_navigation/point_cloud_voxel_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory_resource>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace x2_navigation
{
namespace
{

struct PointFieldOffsets
{
  std::uint32_t x;
  std::uint32_t y;
  std::uint32_t z;
};

struct VoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto hash_x = std::hash<std::int64_t>{}(key.x);
    const auto hash_y = std::hash<std::int64_t>{}(key.y);
    const auto hash_z = std::hash<std::int64_t>{}(key.z);
    return hash_x ^ (hash_y << 1U) ^ (hash_z << 2U);
  }
};

struct VoxelAccumulator
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  std::size_t count{0};
};

bool findFloat32Field(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name,
  std::uint32_t * offset)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1U ||
        field.offset > cloud.point_step - sizeof(float))
      {
        return false;
      }
      *offset = field.offset;
      return true;
    }
  }
  return false;
}

std::optional<PointFieldOffsets> getPointFieldOffsets(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (cloud.point_step < 3U * sizeof(float)) {
    return std::nullopt;
  }

  PointFieldOffsets offsets{};
  if (!findFloat32Field(cloud, "x", &offsets.x) ||
    !findFloat32Field(cloud, "y", &offsets.y) ||
    !findFloat32Field(cloud, "z", &offsets.z))
  {
    return std::nullopt;
  }
  return offsets;
}

bool hasValidLayout(const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (cloud.is_bigendian) {
    return false;
  }
  if (cloud.height == 0U || cloud.width == 0U) {
    return true;
  }
  if (cloud.point_step == 0U) {
    return false;
  }
  if (cloud.width > std::numeric_limits<std::uint32_t>::max() / cloud.point_step ||
    cloud.row_step < cloud.width * cloud.point_step ||
    cloud.height > std::numeric_limits<std::size_t>::max() / cloud.row_step)
  {
    return false;
  }
  const auto required_size = static_cast<std::size_t>(cloud.height) * cloud.row_step;
  return required_size <= cloud.data.size();
}

float readFloat(const std::uint8_t * point_data, std::uint32_t offset)
{
  float value;
  std::memcpy(&value, point_data + offset, sizeof(value));
  return value;
}

bool makeVoxelKey(const tf2::Vector3 & point, double voxel_size, VoxelKey * key)
{
  const auto scaled_x = std::floor(point.x() / voxel_size);
  const auto scaled_y = std::floor(point.y() / voxel_size);
  const auto scaled_z = std::floor(point.z() / voxel_size);
  const auto minimum = static_cast<double>(std::numeric_limits<std::int64_t>::min());
  const auto maximum = static_cast<double>(std::numeric_limits<std::int64_t>::max());
  if (scaled_x < minimum || scaled_x > maximum ||
    scaled_y < minimum || scaled_y > maximum ||
    scaled_z < minimum || scaled_z > maximum)
  {
    return false;
  }
  *key = VoxelKey{
    static_cast<std::int64_t>(scaled_x),
    static_cast<std::int64_t>(scaled_y),
    static_cast<std::int64_t>(scaled_z)};
  return true;
}

sensor_msgs::msg::PointCloud2 makeEmptyCloud(const std_msgs::msg::Header & header)
{
  const auto make_field = [](const std::string & name, std::uint32_t offset) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1U;
      return field;
    };

  sensor_msgs::msg::PointCloud2 output;
  output.header = header;
  output.height = 1U;
  output.fields.resize(3U);
  output.fields[0] = make_field("x", 0U);
  output.fields[1] = make_field("y", 4U);
  output.fields[2] = make_field("z", 8U);
  output.is_bigendian = false;
  output.point_step = 3U * sizeof(float);
  output.is_dense = true;
  return output;
}

}  // namespace

struct PointCloudVoxelFilterWorkspace::Impl
{
  std::pmr::unsynchronized_pool_resource memory_resource;
  std::pmr::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> voxel_indices{&memory_resource};
  std::pmr::vector<VoxelAccumulator> voxels{&memory_resource};
  bool initial_capacity_reserved{false};
};

PointCloudVoxelFilterWorkspace::PointCloudVoxelFilterWorkspace()
: impl_(std::make_unique<Impl>())
{
}

PointCloudVoxelFilterWorkspace::~PointCloudVoxelFilterWorkspace() = default;

std::optional<PointCloudVoxelFilterResult> PointCloudVoxelFilterWorkspace::filter(
  const sensor_msgs::msg::PointCloud2 & input,
  const geometry_msgs::msg::TransformStamped & target_from_source,
  const std_msgs::msg::Header & output_header,
  const PointCloudVoxelFilterConfig & config)
{
  if (!std::isfinite(config.voxel_size) || config.voxel_size <= 0.0 ||
    !std::isfinite(config.min_height) || !std::isfinite(config.max_height) ||
    config.min_height > config.max_height || config.max_input_points == 0U ||
    !hasValidLayout(input))
  {
    return std::nullopt;
  }

  const auto field_offsets = getPointFieldOffsets(input);
  if (!field_offsets) {
    return std::nullopt;
  }

  const auto input_point_count = static_cast<std::size_t>(input.width) * input.height;
  if (input_point_count == 0U) {
    PointCloudVoxelFilterResult result;
    result.cloud = makeEmptyCloud(output_header);
    result.input_point_count = 0U;
    result.sampled_point_count = 0U;
    result.retained_point_count = 0U;
    return result;
  }

  tf2::Transform transform;
  tf2::fromMsg(target_from_source.transform, transform);

  const auto sampled_point_count = std::min(input_point_count, config.max_input_points);
  const auto initial_voxel_capacity = std::min<std::size_t>(sampled_point_count, 16384U);
  auto & voxel_indices = impl_->voxel_indices;
  auto & voxels = impl_->voxels;
  voxel_indices.clear();
  voxels.clear();
  if (!impl_->initial_capacity_reserved) {
    voxel_indices.reserve(initial_voxel_capacity);
    voxels.reserve(initial_voxel_capacity);
    impl_->initial_capacity_reserved = true;
  }

  std::size_t next_sample_index = sampled_point_count == 1U ?
    (input_point_count - 1U) / 2U : 0U;
  const auto sample_interval_count = sampled_point_count > 1U ? sampled_point_count - 1U : 1U;
  const auto sample_span = input_point_count - 1U;
  const auto base_sample_interval = sample_span / sample_interval_count;
  const auto sample_interval_remainder = sample_span % sample_interval_count;
  std::size_t sample_interval_error = 0U;
  for (std::size_t samples_visited = 0U; samples_visited < sampled_point_count;
    ++samples_visited)
  {
    const auto input_index = next_sample_index;
    if (samples_visited + 1U < sampled_point_count) {
      next_sample_index += base_sample_interval;
      sample_interval_error += sample_interval_remainder;
      if (sample_interval_error >= sample_interval_count) {
        ++next_sample_index;
        sample_interval_error -= sample_interval_count;
      }
    }

    const auto row = input_index / input.width;
    const auto column = input_index % input.width;
    const auto * point_data = input.data.data() + row * input.row_step +
      column * input.point_step;
    const auto source_x = readFloat(point_data, field_offsets->x);
    const auto source_y = readFloat(point_data, field_offsets->y);
    const auto source_z = readFloat(point_data, field_offsets->z);
    if (!std::isfinite(source_x) || !std::isfinite(source_y) || !std::isfinite(source_z)) {
      continue;
    }

    const auto point = transform * tf2::Vector3(source_x, source_y, source_z);
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z()) ||
      point.z() < config.min_height || point.z() > config.max_height)
    {
      continue;
    }

    VoxelKey key{};
    if (!makeVoxelKey(point, config.voxel_size, &key)) {
      continue;
    }

    const auto [iterator, inserted] = voxel_indices.emplace(key, voxels.size());
    if (inserted) {
      voxels.push_back(VoxelAccumulator{});
    }
    auto & accumulator = voxels[iterator->second];
    accumulator.x += point.x();
    accumulator.y += point.y();
    accumulator.z += point.z();
    ++accumulator.count;
  }

  if (voxels.size() > std::numeric_limits<std::uint32_t>::max() / (3U * sizeof(float))) {
    return std::nullopt;
  }

  PointCloudVoxelFilterResult result;
  result.cloud = makeEmptyCloud(output_header);
  result.input_point_count = input_point_count;
  result.sampled_point_count = sampled_point_count;
  result.retained_point_count = voxels.size();
  result.cloud.width = static_cast<std::uint32_t>(voxels.size());
  result.cloud.row_step = result.cloud.width * result.cloud.point_step;
  result.cloud.data.resize(result.cloud.row_step);

  for (std::size_t index = 0U; index < voxels.size(); ++index) {
    const auto & voxel = voxels[index];
    const auto inverse_count = 1.0 / static_cast<double>(voxel.count);
    const float x = static_cast<float>(voxel.x * inverse_count);
    const float y = static_cast<float>(voxel.y * inverse_count);
    const float z = static_cast<float>(voxel.z * inverse_count);
    auto * output_data = result.cloud.data.data() + index * result.cloud.point_step;
    std::memcpy(output_data, &x, sizeof(x));
    std::memcpy(output_data + sizeof(x), &y, sizeof(y));
    std::memcpy(output_data + 2U * sizeof(x), &z, sizeof(z));
  }

  return result;
}

std::optional<PointCloudVoxelFilterResult> filterPointCloudToVoxels(
  const sensor_msgs::msg::PointCloud2 & input,
  const geometry_msgs::msg::TransformStamped & target_from_source,
  const std_msgs::msg::Header & output_header,
  const PointCloudVoxelFilterConfig & config)
{
  PointCloudVoxelFilterWorkspace workspace;
  return workspace.filter(input, target_from_source, output_header, config);
}

}  // namespace x2_navigation
