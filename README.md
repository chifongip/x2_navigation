# X2 Navigation2

`x2_navigation` is the X2's minimal Navigation2 stack. It consumes existing
robot state and localization; it never starts a controller manager, a robot
state publisher, FAST-LIO, or a global-localization node.

The included map is `2026-08-18-Lab_voxel_0_05m.yaml` at 0.05 m resolution.

## Prerequisites

Before launching navigation, start the existing X2 state publisher and
FAST-LIO. The runtime TF graph must provide all of the following:

```bash
ros2 launch x2_bringup state_publisher.launch.py command_transport:=zmq
```

Start this once. Navigation consumes the resulting `/joint_states`, `/tf`, and
`/tf_static`; it does not start another controller manager or robot-state
publisher.

```
map -> odom                         E1R global-localization owner
odom -> base_link                   E1R localization from /Odometry_loc
base_link -> lidar_imu_chest_front  existing sensor or robot TF publisher
base_link -> lidar_chest_front      existing sensor or robot TF publisher
```

The adapter uses the last transform to publish `/odom` with a base-frame pose
and twist. It publishes nothing if that transform is unavailable at the
odometry stamp.

E1R localization owns both `map -> odom` and `odom -> base_link`; keep
`publish_robot_root_tf` enabled. The adapter intentionally never broadcasts
TF.

## Launch

```bash
ros2 launch x2_navigation navigation.launch.py
```

The stack forwards normal, unstamped `geometry_msgs/msg/Twist` commands from
`/cmd_vel` to the RoboJuDo velocity interface. `nav2_zmq_velocity_bridge`
binds `tcp://*:8558` and publishes full Twist-shaped JSON at 20 Hz. It clamps
`linear.x` to 0.0-0.5 m/s, forces all unsupported axes to zero, clamps
`angular.z` to -0.5-0.5 rad/s, and sends zero velocity if a command is absent,
invalid, or older than 0.20 s.

RoboJuDo must be started separately with `VelocityZmqCtrl` enabled and
connected to `tcp://127.0.0.1:8558`; put it before any joystick controller if
it should take priority. This package does not modify RoboJuDo configuration.
Only one process may bind port 8558. Do not use the X2 upper-body command port
8559 for navigation.

Override the bridge transport or watchdog only when the matching RoboJuDo
receiver configuration is changed:

```bash
ros2 launch x2_navigation navigation.launch.py \
  velocity_zmq_endpoint:=tcp://*:8558 velocity_command_timeout_sec:=0.20
```

Both costmaps consume dynamic obstacles through this bounded pipeline:

```
/aima/hal/sensor/lidar_chest_front/lidar_pointcloud
  -> lidar_cloud_throttle (newest cloud every 100 ms)
  -> base_link transform, height crop, 0.05 m voxel downsampling
  -> /scan_nav/cloud (compact XYZ PointCloud2)
  -> ground_segmentation_ros2 (two-phase ground segmentation)
  -> /scan_nav/ground_filtered_cloud (non-ground points)
  -> robot_self_filter (live X2 link TF plus collision-box proxy)
  -> /scan_nav/self_filtered_cloud
     +-> pointcloud_to_laserscan -> /scan_nav/laser (panel visualization only)
     +-> local and global obstacle layers (marking and clearing)
```

The throttle uses the live `base_link <- lidar_chest_front` transform because
the chest LiDAR axes are rotated. It retains points from -0.45 m to 0.30 m in
the `base_link` frame and publishes one XYZ centroid per 0.05 m voxel. The
GSeg3D wrapper then receives `/scan_nav/cloud` with sensor-data QoS and sends
only its non-ground `obstacle_points` result to `/scan_nav/ground_filtered_cloud`.
Its X2 configuration uses the lower edge of the retained height band as the
ground seed because the cloud has already been transformed to `base_link`.
Retune `lidar_to_ground` together with the throttle height bounds after a
standing-posture or sensor-mount change; inspect `/scan_nav/ground_points` and
`/scan_nav/ground_filter_input` before deploying a new value. It processes the
newest received cloud from a 10 Hz timer, so callback arrival
phase does not cause avoidable rate-gate misses. Each output still requires a
fresh input cloud and a timestamp-valid transform. A reusable allocator avoids
per-frame voxel storage churn, and `max_input_points` bounds work to 40,000
uniformly sampled raw points per output. The costmap applies the 0.20-5.0 m
obstacle ranges. `x2_self_filter.urdf` preserves the X2 kinematic tree and
visual meshes, but replaces every production collision mesh with a local
bounding box. The boxes follow the same live link frames as the shared state
publisher and avoid the expensive convex-hull construction that the filter
performs for STL geometry at startup. The proxy dimensions in
`tools/generate_self_filter_urdf.py` are explicit, so review and retune
`PROXY_BOXES` after a collision-mesh or geometry change before regenerating
`x2_self_filter.urdf`. The deployed
`self_filter.yaml` deliberately selects the arms and hand pads only: the chest
LiDAR lies within the conservative torso proxy, so filtering that link would
classify every outgoing ray as self-shadow and erase the environment. The
boxes are expanded by 25 percent plus one centimetre to absorb voxel-centroid
error. The filter and both Nav2 obstacle layers use `lidar_chest_front` as the
sensor origin even though the cloud frame is `base_link`, so points shadowed by
the visible upper body are removed and raytracing starts at the physical LiDAR.
The filter drops a cloud until timestamp-valid transforms for all
configured links are available; it never reuses an old robot pose. Add another
link only after checking that its live filter output does not mask the scene.
Tune the height limits, voxel size, proxy expansion, and input bound for the
robot posture and environment; the navigation stack adds only one raw-cloud
subscriber and keeps it at a bounded rate. The global costmap combines the
static map with these live obstacles, and Nav2 does not consume FAST-LIO's
registered clouds. Both costmaps use a 1.0 m inflation radius to preserve the
required clearance around obstacles.

Point-cloud clearing raytraces only to retained returns. Unlike a LaserScan,
it has no infinity returns to clear empty sectors out to maximum range.

`pointcloud_to_laserscan` is a standard ROS 2 package used only to make a
lightweight map-alignment view for `x2_operator_panel`; Nav2 continues to use
`/scan_nav/self_filtered_cloud`. The converter consumes the filtered
`/scan_nav/self_filtered_cloud` input, outputs `/scan_nav/laser` in `base_link`,
uses one-degree rays over a full turn, and sends infinity for empty sectors.
Its display range defaults to 0.20-12.0 m and can be changed without altering
costmap ranges:

```bash
ros2 launch x2_navigation navigation.launch.py \
  laser_scan_range_min:=0.20 laser_scan_range_max:=12.0
```

Verify the package before deployment. On a ROS 2 Humble image without it,
install the matching system package and rebuild this workspace:

```bash
ros2 pkg prefix pointcloud_to_laserscan
sudo apt install ros-humble-pointcloud-to-laserscan
```

The default timestamp offset is `0.0`, so the navigation pipeline preserves the
raw LiDAR stamp. Override `lidar_timestamp_offset_sec` only after measuring the
relative LiDAR-to-TF timing. This does not modify the raw driver topic or
FAST-LIO. A wrong fixed offset creates a pose error whenever the robot moves.
