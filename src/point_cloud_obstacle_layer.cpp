#include "x2_navigation/point_cloud_obstacle_layer.hpp"

#include <cmath>
#include <iterator>
#include <sstream>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace x2_navigation
{

PointCloudObstacleLayer::~PointCloudObstacleLayer()
{
  // Connection is not RAII: disconnect explicitly while derived state exists.
  // Signal1 serializes disconnect with an in-flight enqueue callback. Never
  // hold pending_mutex_ here, since that callback also needs it.
  for (auto & connection : connections_) {
    connection.disconnect();
  }
}

void PointCloudObstacleLayer::onInitialize()
{
  ObstacleLayer::onInitialize();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Point-cloud obstacle layer lost its lifecycle node");
  }
  double timeout;
  node->get_parameter("transform_tolerance", timeout);
  if (!std::isfinite(timeout) || timeout < 0.0) {
    throw std::runtime_error("transform_tolerance must be finite and nonnegative");
  }
  wait_timeout_ = std::chrono::duration<double>(timeout);

  // Base initialization leaves subscribers unsubscribed. Disconnect its TF
  // filters before activation, so no waitForTransform request can be created.
  observation_notifiers_.clear();
  std::string names;
  node->get_parameter(name_ + ".observation_sources", names);
  std::istringstream stream(names);
  std::string source;
  while (stream >> source) {
    const auto index = sources_.size();
    auto sub = std::dynamic_pointer_cast<
      message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
      rclcpp_lifecycle::LifecycleNode>>(observation_subscribers_.at(index));
    if (!sub) {
      throw std::runtime_error("PointCloudObstacleLayer supports only PointCloud2 sources");
    }
    Source state;
    node->get_parameter(name_ + "." + source + ".sensor_frame", state.sensor_frame);
    sources_.push_back(std::move(state));
    configureBuffer(source, index);
    connections_.push_back(sub->registerCallback(
      [this, index](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        enqueue(index, std::move(cloud));
      }));
  }
}

void PointCloudObstacleLayer::configureBuffer(const std::string & source, std::size_t index)
{
  auto node = node_.lock();
  const auto prefix = name_ + "." + source + ".";
  const auto number = [&node, &prefix](const std::string & key) {
      return node->get_parameter(prefix + key).as_double();
    };
  // Reuse the official buffer and every source parameter, but make its final
  // synchronous TF lookups nonblocking too. TF can disappear between readiness
  // checks and bufferCloud (for example, on a clock jump).
  auto buffer = std::make_shared<nav2_costmap_2d::ObservationBuffer>(
    node, node->get_parameter(prefix + "topic").as_string(),
    number("observation_persistence"), number("expected_update_rate"),
    number("min_obstacle_height"), number("max_obstacle_height"),
    number("obstacle_max_range"), number("obstacle_min_range"),
    number("raytrace_max_range"), number("raytrace_min_range"), *tf_,
    global_frame_, sources_.at(index).sensor_frame, tf2::Duration::zero());
  const auto original = observation_buffers_.at(index);
  for (auto & marking : marking_buffers_) {
    if (marking == original) {
      marking = buffer;
    }
  }
  for (auto & clearing : clearing_buffers_) {
    if (clearing == original) {
      clearing = buffer;
    }
  }
  observation_buffers_[index] = std::move(buffer);
}

void PointCloudObstacleLayer::activate()
{
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    active_ = true;
  }
  ObstacleLayer::activate();
}

void PointCloudObstacleLayer::deactivate()
{
  ObstacleLayer::deactivate();
  std::lock_guard<std::mutex> lock(pending_mutex_);
  active_ = false;
  for (auto & source : sources_) {
    source.pending.clear();
  }
}

void PointCloudObstacleLayer::reset()
{
  std::lock_guard<Costmap2D::mutex_t> layer_lock(*getMutex());
  std::lock_guard<std::mutex> lock(pending_mutex_);
  for (auto & source : sources_) {
    source.pending.clear();
  }
  ObstacleLayer::reset();
}

void PointCloudObstacleLayer::enqueue(
  std::size_t source, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  // A zero stamp asks TF for the latest pose, which is unsuitable for a sensor
  // observation. Never mark or clear using an unstamped cloud.
  if (cloud->header.frame_id.empty() ||
    (cloud->header.stamp.sec == 0 && cloud->header.stamp.nanosec == 0))
  {
    return;
  }
  std::lock_guard<std::mutex> lock(pending_mutex_);
  if (!active_) {
    return;
  }
  auto & queue = sources_.at(source).pending;
  constexpr std::size_t max_pending_clouds = 50;
  if (queue.size() >= max_pending_clouds) {
    queue.pop_front();
  }
  queue.push_back({std::move(cloud), std::chrono::steady_clock::now()});
}

void PointCloudObstacleLayer::processPending()
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  if (!active_) {
    return;
  }
  for (std::size_t i = 0; i < sources_.size(); ++i) {
    auto & source = sources_[i];
    auto it = source.pending.begin();
    while (it != source.pending.end()) {
      const auto & pending = *it;
      const auto & cloud = pending.cloud;
      if (wait_timeout_.count() > 0.0 &&
        std::chrono::steady_clock::now() - pending.received >= wait_timeout_)
      {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "Dropping expired obstacle cloud from %s", cloud->header.frame_id.c_str());
        it = source.pending.erase(it);
        continue;
      }
      const rclcpp::Time stamp(cloud->header.stamp);
      const auto & origin = source.sensor_frame.empty() ?
        cloud->header.frame_id : source.sensor_frame;
      // Zero-duration queries only inspect the buffer; they never register
      // transformable requests or invoke callbacks from the TF listener.
      const bool ready =
        tf_->canTransform(global_frame_, cloud->header.frame_id, stamp) &&
        tf_->canTransform(global_frame_, origin, stamp);
      if (ready) {
        // Same operation as the upstream PointCloud2 callback, with an RAII
        // lock so exceptions cannot strand the observation-buffer mutex.
        auto & buffer = *observation_buffers_.at(i);
        std::lock_guard<nav2_costmap_2d::ObservationBuffer> buffer_lock(buffer);
        buffer.bufferCloud(*cloud);
        // A newer usable observation supersedes older pending ones. They must
        // not replace it later when their delayed TF finally arrives.
        it = source.pending.erase(source.pending.begin(), std::next(it));
      } else if (wait_timeout_.count() <= 0.0) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "Dropping obstacle cloud: timestamp-valid TF is unavailable for %s",
          cloud->header.frame_id.c_str());
        it = source.pending.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void PointCloudObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  // Keep the same layer -> queue -> observation lock order as reset().
  std::lock_guard<Costmap2D::mutex_t> layer_lock(*getMutex());
  processPending();
  ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

}  // namespace x2_navigation

PLUGINLIB_EXPORT_CLASS(x2_navigation::PointCloudObstacleLayer, nav2_costmap_2d::Layer)
