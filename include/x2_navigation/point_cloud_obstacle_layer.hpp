#ifndef X2_NAVIGATION__POINT_CLOUD_OBSTACLE_LAYER_HPP_
#define X2_NAVIGATION__POINT_CLOUD_OBSTACLE_LAYER_HPP_

#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/obstacle_layer.hpp"

namespace x2_navigation
{

// Keep Nav2 obstacle geometry and observation buffers, but do not register
// asynchronous TF requests (geometry2 #992 can deadlock the shared TF buffer).
class PointCloudObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  ~PointCloudObstacleLayer() override;
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

private:
  struct PendingCloud
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    std::chrono::steady_clock::time_point received;
  };

  struct Source
  {
    std::string sensor_frame;
    std::deque<PendingCloud> pending;
  };

  void enqueue(
    std::size_t source, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  void processPending();
  void configureBuffer(const std::string & source, std::size_t index);

  std::mutex pending_mutex_;
  std::vector<Source> sources_;
  std::vector<message_filters::Connection> connections_;
  std::chrono::duration<double> wait_timeout_{0.0};
  bool active_{false};
};

}  // namespace x2_navigation

#endif  // X2_NAVIGATION__POINT_CLOUD_OBSTACLE_LAYER_HPP_
