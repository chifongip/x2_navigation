#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "x2_navigation/point_cloud_obstacle_layer.hpp"

using namespace std::chrono_literals;

namespace
{

class RaceBuffer : public tf2_ros::Buffer
{
public:
  using tf2_ros::Buffer::Buffer;
  mutable bool clear_on_lookup{false};
  mutable std::vector<tf2::Duration> lookup_timeouts;

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target, const std::string & source,
    const tf2::TimePoint & time, const tf2::Duration timeout) const override
  {
    lookup_timeouts.push_back(timeout);
    if (clear_on_lookup) {
      clear_on_lookup = false;
      const_cast<RaceBuffer *>(this)->clear();
    }
    return tf2_ros::Buffer::lookupTransform(target, source, time, timeout);
  }
};

class InspectableLayer : public x2_navigation::PointCloudObstacleLayer
{
public:
  std::vector<nav2_costmap_2d::Observation> observations()
  {
    std::vector<nav2_costmap_2d::Observation> result;
    getMarkingObservations(result);
    return result;
  }

  std::size_t asyncFilterCount() const {return observation_notifiers_.size();}
};

class PointCloudObstacleLayerTest : public testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("obstacles.observation_sources", "cloud"),
      rclcpp::Parameter("obstacles.cloud.topic", "/test/obstacle_cloud"),
      rclcpp::Parameter("obstacles.cloud.data_type", "PointCloud2"),
      rclcpp::Parameter("obstacles.cloud.sensor_frame", "sensor"),
      rclcpp::Parameter("obstacles.cloud.clearing", true),
      rclcpp::Parameter("obstacles.cloud.min_obstacle_height", 0.1),
      rclcpp::Parameter("obstacles.cloud.max_obstacle_height", 0.4),
      rclcpp::Parameter("obstacles.cloud.obstacle_max_range", 5.0),
      rclcpp::Parameter("obstacles.cloud.obstacle_min_range", 0.2),
      rclcpp::Parameter("obstacles.cloud.raytrace_max_range", 5.5),
      rclcpp::Parameter("obstacles.cloud.raytrace_min_range", 0.3),
    });
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_obstacle_layer", "", options);
    node_->declare_parameter("track_unknown_space", false);
    node_->declare_parameter("transform_tolerance", 0.15);
    tf_ = std::make_unique<RaceBuffer>(node_->get_clock());
    tf_->setUsingDedicatedThread(true);
    map_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", false, false);
    map_->resizeMap(120, 120, 0.05, -3.0, -3.0);
    layer_ = std::make_shared<InspectableLayer>();
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    layer_->initialize(map_.get(), "obstacles", tf_.get(), node_, group_);
    map_->addPlugin(layer_);
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/test/obstacle_cloud", rclcpp::SensorDataQoS());
    executor_->add_node(node_->get_node_base_interface());
    layer_->activate();
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (publisher_->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor_->spin_some();
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_GT(publisher_->get_subscription_count(), 0U);
    cloud_.header.frame_id = "cloud_frame";
    cloud_.header.stamp = node_->now();
    sensor_msgs::PointCloud2Modifier modifier(cloud_);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float>(cloud_, "x")[0] = 1.0F;
    sensor_msgs::PointCloud2Iterator<float>(cloud_, "y")[0] = 0.0F;
    sensor_msgs::PointCloud2Iterator<float>(cloud_, "z")[0] = 0.2F;
  }

  void TearDown() override
  {
    layer_->deactivate();
    executor_->remove_node(node_->get_node_base_interface());
    map_.reset();
    layer_.reset();
    publisher_.reset();
    tf_.reset();
    node_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  void publish()
  {
    publisher_->publish(cloud_);
    const auto deadline = std::chrono::steady_clock::now() + 40ms;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  void transform(const std::string & child, double x, bool is_static = true)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "odom";
    t.child_frame_id = child;
    t.header.stamp = cloud_.header.stamp;
    t.transform.translation.x = x;
    t.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_->setTransform(t, "test", is_static));
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::unique_ptr<RaceBuffer> tf_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map_;
  std::shared_ptr<InspectableLayer> layer_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  sensor_msgs::msg::PointCloud2 cloud_;
};

TEST_F(PointCloudObstacleLayerTest, WaitsForBothCloudAndSensorTfWithoutBlocking)
{
  EXPECT_EQ(layer_->asyncFilterCount(), 0U);
  publish();
  auto start = std::chrono::steady_clock::now();
  map_->updateMap(0.0, 0.0, 0.0);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
  EXPECT_TRUE(layer_->observations().empty());
  transform("cloud_frame", 0.5);
  map_->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(layer_->observations().empty());
  transform("sensor", -0.25);
  map_->updateMap(0.0, 0.0, 0.0);
  const auto observations = layer_->observations();
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_DOUBLE_EQ(observations[0].origin_.x, -0.25);
  EXPECT_FLOAT_EQ(sensor_msgs::PointCloud2ConstIterator<float>(
    *observations[0].cloud_, "x")[0], 1.5F);
  EXPECT_EQ(observations[0].cloud_->header.stamp, cloud_.header.stamp);
  unsigned int x, y;
  ASSERT_TRUE(map_->getCostmap()->worldToMap(1.5, 0.0, x, y));
  EXPECT_EQ(map_->getCostmap()->getCost(x, y), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(PointCloudObstacleLayerTest, ExpiresCloudsWithoutTransformingAtLatestTime)
{
  // A dynamic TF at the wrong time must not admit the cloud.
  auto original_stamp = cloud_.header.stamp;
  cloud_.header.stamp.sec -= 1;
  transform("cloud_frame", 0.0, false);
  transform("sensor", 0.0, false);
  cloud_.header.stamp = original_stamp;
  publish();
  map_->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(layer_->observations().empty());
  std::this_thread::sleep_for(170ms);
  // TF arriving after the deadline must not resurrect the queued cloud.
  transform("cloud_frame", 0.5, false);
  transform("sensor", 0.0, false);
  map_->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(layer_->observations().empty());
}

TEST_F(PointCloudObstacleLayerTest, DropsUnstampedClouds)
{
  transform("cloud_frame", 0.0);
  transform("sensor", 0.0);
  cloud_.header.stamp = builtin_interfaces::msg::Time();
  publish();
  map_->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(layer_->observations().empty());
}

TEST_F(PointCloudObstacleLayerTest, MissingTfDoesNotBlockOrLaterReplaceNewerObservation)
{
  const auto missing = cloud_.header.frame_id;
  cloud_.header.frame_id = "missing_frame";
  publish();
  cloud_.header.frame_id = missing;
  cloud_.header.stamp = node_->now();
  transform("cloud_frame", 0.5);
  transform("sensor", 0.0);
  publish();
  map_->updateMap(0.0, 0.0, 0.0);
  auto observations = layer_->observations();
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_FLOAT_EQ(sensor_msgs::PointCloud2ConstIterator<float>(
    *observations[0].cloud_, "x")[0], 1.5F);
  transform("missing_frame", 1.0);
  map_->updateMap(0.0, 0.0, 0.0);
  observations = layer_->observations();
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_FLOAT_EQ(sensor_msgs::PointCloud2ConstIterator<float>(
    *observations[0].cloud_, "x")[0], 1.5F);
}

TEST_F(PointCloudObstacleLayerTest, TfDisappearingAfterReadinessDoesNotWaitOrStrandBuffer)
{
  // BufferCore::clear retains static transforms; use dynamic TF to model a
  // clock jump actually invalidating the just-checked sensor transforms.
  transform("cloud_frame", 0.5, false);
  transform("sensor", 0.0, false);
  publish();
  tf_->clear_on_lookup = true;
  const auto start = std::chrono::steady_clock::now();
  map_->updateMap(0.0, 0.0, 0.0);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
  EXPECT_TRUE(layer_->observations().empty());
  ASSERT_FALSE(tf_->lookup_timeouts.empty());
  for (const auto timeout : tf_->lookup_timeouts) {
    EXPECT_EQ(timeout, tf2::Duration::zero());
  }
  transform("cloud_frame", 0.5);
  transform("sensor", 0.0);
  publish();
  map_->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(layer_->observations().size(), 1U);
}

TEST_F(PointCloudObstacleLayerTest, ResetAndDeactivateDiscardQueuedClouds)
{
  publish();
  layer_->reset();
  transform("cloud_frame", 0.0);
  transform("sensor", 0.0);
  map_->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(layer_->observations().empty());
  tf_->clear();
  publish();
  layer_->deactivate();
  layer_->activate();
  transform("cloud_frame", 0.0);
  transform("sensor", 0.0);
  map_->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(layer_->observations().empty());
  publish();
  map_->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(layer_->observations().size(), 1U);
}

TEST_F(PointCloudObstacleLayerTest, PreservesRaytracingFromPhysicalSensorOrigin)
{
  transform("cloud_frame", 0.5);
  transform("sensor", -0.25);
  publish();
  map_->updateMap(0.0, 0.0, 0.0);
  unsigned int x, y;
  ASSERT_TRUE(map_->getCostmap()->worldToMap(1.5, 0.0, x, y));
  ASSERT_EQ(map_->getCostmap()->getCost(x, y), nav2_costmap_2d::LETHAL_OBSTACLE);
  sensor_msgs::PointCloud2Iterator<float>(cloud_, "x")[0] = 2.0F;
  publish();
  map_->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(map_->getCostmap()->getCost(x, y), nav2_costmap_2d::FREE_SPACE);
  ASSERT_TRUE(map_->getCostmap()->worldToMap(2.5, 0.0, x, y));
  EXPECT_EQ(map_->getCostmap()->getCost(x, y), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(PointCloudObstacleLayerTest, PreservesSourceHeightAndRangeParameters)
{
  transform("cloud_frame", 0.0);
  transform("sensor", 0.0);
  sensor_msgs::PointCloud2Modifier(cloud_).resize(3);
  sensor_msgs::PointCloud2Iterator<float> x(cloud_, "x"), y(cloud_, "y"), z(cloud_, "z");
  for (const auto height : {0.05F, 0.2F, 0.5F}) {
    *x = 1.0F;
    *y = 0.0F;
    *z = height;
    ++x;
    ++y;
    ++z;
  }
  publish();
  map_->updateMap(0.0, 0.0, 0.0);
  const auto observations = layer_->observations();
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations[0].cloud_->width, 1U);
  EXPECT_FLOAT_EQ(sensor_msgs::PointCloud2ConstIterator<float>(
    *observations[0].cloud_, "z")[0], 0.2F);
  EXPECT_DOUBLE_EQ(observations[0].obstacle_max_range_, 5.0);
  EXPECT_DOUBLE_EQ(observations[0].obstacle_min_range_, 0.2);
  EXPECT_DOUBLE_EQ(observations[0].raytrace_max_range_, 5.5);
  EXPECT_DOUBLE_EQ(observations[0].raytrace_min_range_, 0.3);
}

}  // namespace
