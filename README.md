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

The local costmap consumes dynamic obstacles through this bounded pipeline:

```
/aima/hal/sensor/lidar_chest_front/lidar_pointcloud
  -> lidar_cloud_throttle (newest cloud every 100 ms)
  -> base_link transform, height crop, 0.05 m voxel downsampling
  -> /scan_nav/cloud (compact XYZ PointCloud2)
  -> local obstacle layer (marking and clearing)
```

The throttle uses the live `base_link <- lidar_chest_front` transform because
the chest LiDAR axes are rotated. It retains points from -0.45 m to 0.30 m in
the `base_link` frame and publishes one XYZ centroid per 0.05 m voxel. It
processes the newest received cloud from a 10 Hz timer, so callback arrival
phase does not cause avoidable rate-gate misses. Each output still requires a
fresh input cloud and a timestamp-valid transform. A reusable allocator avoids
per-frame voxel storage churn, and `max_input_points` bounds work to 40,000
uniformly sampled raw points per output. The costmap applies the 0.20-5.0 m
obstacle ranges. Tune the height limits, voxel size, and input bound for the
robot posture and environment; the navigation stack adds only one raw-cloud
subscriber and keeps it at a bounded rate. The global costmap remains
static-map only, and Nav2 does not consume FAST-LIO's registered clouds.

Point-cloud clearing raytraces only to retained returns. Unlike a LaserScan,
it has no infinity returns to clear empty sectors out to maximum range.

The default timestamp offset is `0.0`, so the navigation pipeline preserves the
raw LiDAR stamp. Override `lidar_timestamp_offset_sec` only after measuring the
relative LiDAR-to-TF timing. This does not modify the raw driver topic or
FAST-LIO. A wrong fixed offset creates a pose error whenever the robot moves.
