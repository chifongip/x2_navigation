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

DWB's `PreferForward` critic favors forward travel while retaining reverse motion
when needed. The trial settings use penalty 10.0, scale 1.0, angular weight 1.0,
and slow-motion thresholds of 0.05 m/s and 0.2 rad/s. Tune these under `FollowPath`
in `nav2_params.yaml` and validate goals behind the robot, tight turns, and
obstacle avoidance in simulation before hardware use.

The stack forwards normal, unstamped `geometry_msgs/msg/Twist` commands from
`/cmd_vel` to the RoboJuDo velocity interface. `nav2_zmq_velocity_bridge`
binds `tcp://*:8558` and publishes full Twist-shaped JSON at 20 Hz. It clamps
`linear.x` to -0.5-1.0 m/s, `linear.y` to -1.0-1.0 m/s, forces the remaining
unsupported axes to zero, clamps `angular.z` to -1.0-1.0 rad/s, and sends zero
velocity if a command is absent, invalid, or older than 0.20 s.

RoboJuDo must be started separately with `VelocityZmqCtrl` enabled and
connected to `tcp://127.0.0.1:8558`; put it before any joystick controller if
it should take priority. This package does not modify RoboJuDo configuration.
Only one process may bind port 8558. Do not use the X2 upper-body command port
8559 for navigation.

## AprilTag table fine alignment

Fine alignment is a separate operation after coarse `NavigateToPose` completes.
The public `/fine_align` (`x2_navigation/action/FineAlign`) action defaults to
measurement only (`execute: false`). It requires Nav2 to be idle, a stable pose for the selected tag, and `/manipulation_state` to report `EMPTY` or `HOLDING`. Execution uses a
local holonomic controller that independently corrects forward, lateral, and yaw
error while all output continues through Collision Monitor and the ZMQ deadman.

The table detector defaults to 1 Hz. The tracking timeout defaults to 1.2 seconds
to allow that cadence with modest scheduling margin. Acquisition requires three
accepted observations; completion uses a time-based dwell with new evidence.
Faster detection is preferable for motion, but the existing detector rate is
preserved. The target is centered and square to the table at a configured 0.50 m
standoff; commission this parameter on hardware before manipulation.

```bash
# No motion: validate state, tag/TF stability, and capture geometry.
ros2 action send_goal /fine_align x2_navigation/action/FineAlign \
  "{execute: false}" --feedback

# Physical fine alignment.
ros2 action send_goal /fine_align x2_navigation/action/FineAlign \
  "{execute: true}" --feedback
```

### Named docking configurations

Both action goals accept an optional `profile_id`; feedback and results report
its resolved name. Empty `/fine_align` selection uses `default_docking_profile`
(default: `default`). The reserved `default` profile uses the existing top-level
`tag_id`, `tag_frame`, `standoff`, `lateral_offset`, and `yaw_offset` parameters,
including the currently configured tag9 and 0.50 m stand-off.

Additional profiles are startup ROS parameters. For example, merge the following
into `fine_align_server.ros__parameters` in your navigation parameter file:

```yaml
# Illustrative values only; commission offsets before physical execution.
docking_profile_names: [table_side, other_station]
default_docking_profile: default
docking_profiles:
  table_side:
    tag_id: 9
    tag_frame: tag9
    standoff: 0.70
    lateral_offset: 0.10
    yaw_offset: 0.10
  other_station:
    tag_id: 10
    tag_frame: tag10
    standoff: 0.60
    lateral_offset: 0.0
    yaw_offset: 0.0
```

Each additional profile requires all five fields. Names contain only letters,
numbers, or underscores, must be unique, and cannot reuse `default`. Tag IDs
must be nonnegative, frames nonempty, stand-off positive and finite, and offsets
finite. The configured default must exist. Profile parameters are read-only;
restart the server after editing them. Omit `docking_profile_names` when no
additional profiles are needed.

Stand-off is measured along the tag's projected outward `+Z` normal; lateral
offset follows its projected `+X` axis, **not robot left**. Yaw offset is added
to the heading facing the tag. Distances use meters and angles use radians.
Profiles share capture limits, speeds, tolerances, tracking settings, retries,
and retreat distance. They select final poses, not staged approach maneuvers.
Each operation discards cached tracking and requires fresh observations;
retries keep the selected profile. The detector must publish that tag ID and
its matching timestamped TF frame; profiles do not configure the detector.

```bash
# Validate a named profile without motion.
ros2 action send_goal /fine_align x2_navigation/action/FineAlign \
  "{profile_id: table_side, execute: false}" --feedback

# Explicitly select the tag profile for retreat (physical motion).
ros2 action send_goal /undock x2_navigation/action/Undock \
  "{profile_id: table_side}" --feedback
```

An empty `/undock` request uses the last successfully executed docking profile,
falling back to the configured default after restart or before the first
successful dock. Measurement-only docking, failed/canceled docking, and
undocking do not change this history. Explicit undocking selection overrides
history for that operation only. Unknown profiles abort without motion using
`INVALID_PROFILE` (FineAlign code 10; Undock code 9), without silent fallback.

These action interface additions require rebuilding and restarting action
clients together with the server. The operator panel continues sending empty
profile selections and needs no UI changes.

Undocking is a separate operation with shared retreat settings exposed as `/undock`
(`x2_navigation/action/Undock`). Both docking and undocking now require the same
visible selected tag and have no odometry subscription or measured-velocity gate.
Undocking first acquires a fresh stable relative target, then defines a robot pose
`undock_distance` behind the initial robot pose, preserving the initial heading.
It computes the current error through the full planar relative transform:
`current_base_to_dock * inverse(initial_base_to_dock) * initial_base_to_retreat`.
The dock frame is derived from the tag; standoff and lateral offsets cancel in
this relative calculation. The controller corrects backward, lateral, and yaw
error at the configured undock speeds. Distance traveled is the backward
projection of the tag-derived robot displacement in the initial robot frame.

The provided profile retreats 0.30 m with a 10.0 s movement timeout. Acquisition
has a separate `acquisition_timeout`. Independent undocking translation and
angular minimum/maximum speeds remain unchanged. Undocking shares the existing
position/yaw tolerances, motion hysteresis, tag tracking, command mux, Collision
Monitor, watchdog, manipulation-state gate, and Nav2-idle gate.

```bash
ros2 action send_goal /undock x2_navigation/action/Undock "{}" --feedback
```

Undocking has no automatic retry. Missing/stale/invalid tag tracking commands zero
and aborts with `NO_STABLE_TAG` (code 4), replacing the old
`ODOMETRY_UNAVAILABLE` name. Clients should rebuild the regenerated action
interface. Cancellation, persistent collision stop, Nav2 activation, invalid
manipulation state, timeout, and ROS shutdown also command zero. The tag must
remain visible throughout retreat; there is no blind odometry or timed fallback.

Nav2 publishes `/cmd_vel_nav`. The lifecycle-managed `velocity_smoother` consumes
that stream and publishes `/cmd_vel_nav_smoothed`; the fine-align server selects
either this smoothed navigation command or its internal alignment command. Its
output `/cmd_vel_raw` passes through Collision Monitor to `/cmd_vel`, which the
ZMQ bridge forwards to RoboJuDo. Collision Monitor is downstream of smoothing,
so collision stops are immediate. Fine-align and undock commands bypass smoothing.

The smoother uses `OPEN_LOOP` feedback: it limits changes relative to its previous
command without requiring reliable measured velocity from LiDAR odometry. The
initial trial limits are 0.3 m/s² linear and 0.3 rad/s² angular acceleration,
with deceleration magnitudes of 0.5 m/s² and 0.5 rad/s². At 20 Hz, angular changes
are limited to 0.015 rad/s per accelerating update and 0.025 rad/s per decelerating
update. Velocity bounds match DWB: x is -0.2 to 0.5 m/s, y is zero, and yaw is
-1.0 to 1.0 rad/s. Tune the `velocity_smoother` block through `params_file`; DWB,
odometry, and docking parameters are unchanged. This limits acceleration, not jerk.

Arbitration independently monitors the original and smoothed streams with its
existing 0.20 s steady-clock command timeout. Continued smoother output cannot
extend a lost Nav2 command's lifetime. Explicit all-zero original commands and
non-finite input invalidate the navigation cache; arbitration publishes zero at
its next 20 Hz update. These stop overrides are exempt from acceleration limits.
Resumption requires a new valid nonzero original command followed by a fresh
smoothed command. Collision Monitor and the ZMQ watchdog retain their stop rules.
For standalone `fine_align_server` usage, `nav_raw_cmd_topic` defaults to empty,
preserving the previous single-input behavior. The navigation launch sets this
parameter to `nav_cmd_topic` and sets the arbitration input to
`smoothed_nav_cmd_topic`. Both launch topic arguments may be overridden.

Validate the full command chain in simulation before hardware motion. Compare
`/cmd_vel_nav`, `/cmd_vel_nav_smoothed`, `/cmd_vel_raw`, and `/cmd_vel` for alternating
0.1/0.7 rad/s yaw requests, reversals, goal stops, controller input loss, and
collision stops. Check tracking and stopping distance when choosing gentler limits;
command smoothing does not correct noisy or delayed odometry. The low-level gait
controller must be able to follow the configured limits.

Fine-align planar speed is bounded by the configured vector
magnitude, rather than independently on x and y. Setting translation minimum and
maximum to 0.1 preserves a 0.1 m/s moving command; setting angular minimum and
maximum to 0.1 preserves a 0.1 rad/s rotating command. Gains and speed limits are
not changed by the tracking workflow. Inspect the running node's parameters when
using launch overrides; the startup log reports effective limits.

Docking looks up `base_link <- tag9` at each detection timestamp, through the
camera/base TF branch. It no longer calculates docking error through `odom`.
Camera calibration, dynamic camera/base transforms, and detection timestamps must
be correct. Detections are queued until their matching TF arrives, without blocking
the command timer. Future, stale, duplicate, and out-of-order detections are ignored.
`maximum_sample_gap` remains declared for launch compatibility. Legacy
`fixed_frame` and odometry-related YAML settings are retained but ignored.

`stable_sample_count` gates initial acquisition and reacquisition. Once acquired,
each accepted observation updates the target without waiting for another batch.
The tracker rejects jumps exceeding `maximum_position_spread` and
`maximum_angular_spread` plus allowed robot motion over the observation interval.
It filters translation and wrapped yaw using `tag_filter_time_constant` (0.15 s).
Previous commanded motion predicts the relative error only for filtering a new
observation; it is an approximation, not measured motion, and never extends tag
freshness. Prediction disagreement therefore needs validation with the real gait.

`tracking_timeout` (1.2 s), capped by `maximum_pose_age`, limits how long a cached
relative observation may command motion. Invalid geometry clears tracking
immediately; isolated pose outliers leave only the last accepted observation valid
until its original timeout. On timeout the server stops, reports `REACQUIRING`,
and applies the existing bounded retry policy. A gap longer than the timeout
requires a new acquisition batch. The tag frequency must support this interval. For example, reducing the timeout
to 0.5 s requires a faster detector than the existing 1 Hz default. Validate
detection cadence, latency, and stopping distance before changing this limit.

`motion_confirmation_samples` (2) counts distinct accepted observations before
starting an axis or reversing it. Reversals first stop the affected axis.
`position_hysteresis` (0.02 m) and `yaw_hysteresis` (0.0349 rad) define tighter
stop thresholds inside the configured completion tolerances. A stopped axis can
restart after confirmed observations outside its completion tolerance. An active
axis continues until its error reaches `max(tolerance - hysteresis, tolerance / 2)`.
For a 0.10 m tolerance and 0.02 m hysteresis, restart is above 0.10 m and stop is
at or below 0.08 m. There is no stopped waiting band outside completion.
Translation also requires confirmation below its yaw resume threshold.
`direction_change_rate` (1.0 rad/s) bounds changes in translation heading while
preserving the configured speed magnitude. Large heading changes stop translation
before selecting the new direction. There is no ramp below the configured minimum
speed; gait-level acceleration/jerk handling remains the receiver's responsibility.
Collision Monitor and watchdog zero commands are not smoothed.

Completion of both operations requires raw and filtered poses within their
existing target tolerances, a zero movement command, and stable fresh relative tag
poses over `settling_duration` (0.5 s). The raw relative pose must stay within
`settling_position_spread` (0.02 m) and `settling_angular_spread` (0.0349 rad) of
the first qualifying observation. Motion, pose disagreement, a nonzero command,
or loss of eligibility resets the dwell. Repeated ticks using the same observation
cannot complete it. At 1 Hz, completion requires a subsequent fresh observation
and therefore takes at least one observation interval. `settled_sample_count`
remains accepted for compatibility; it does not determine completion.

Visual settling observes motion relative to the table; it cannot independently
prove that the robot body has stopped. The table/tag must be stationary and camera
calibration and timestamps must be accurate. Future reliable robot odometry can
be added as an optional motion-feedback gate alongside this visual check. There
is currently no odometry feedback, pose, velocity, freshness, or sample-sequence
dependency in either action. Navigation's FAST_LIO adapter, `/odom`, and TF pipeline
are unchanged. Legacy YAML odometry topics/timeouts, measured-speed settling
limits, and undock jump limits are ignored; existing tuned values are preserved.

During physical alignment, the server writes an INFO-level progress log every
`progress_log_interval` seconds (default: 1.0). It includes the current base-frame
x/y/yaw error, commanded `linear.x`, `linear.y`, and `angular.z`, settling state,
accepted-tag sequence number, observation age, raw error, settling duration, and
current attempt. It reports pose eligibility, zero-command status, and visual
settling completion separately. Undocking logs the same tag diagnostics and its
relative retreat progress.

Physical alignment retries recoverable failures up to `maximum_retries` times
(default: 2, for three total attempts). Tag loss, capture-envelope drift, and
approach timeout are retryable. Before another attempt, the server publishes the
`REACQUIRING` action-feedback stage, commands zero velocity, waits `retry_delay`
(default: 1.0 s), and requires a newer stable tag target. Measurement-only goals
do not retry. Cancellation, a persistent Collision Monitor stop, Nav2 activation,
invalid manipulation state, TF/controller safety failures, and ROS shutdown abort
immediately. Every scheduled retry is logged with its failure code, reason,
attempt count, delay, and required target sequence; the final abort log reports
when all retries have been exhausted.

Collision Monitor and the local/global costmaps use the tuned X2 robot
footprint: -0.15 m to 0.15 m along x and +/-0.30 m along y in `base_link`.
This is the robot-body collision envelope; it is independent of the carrying
box geometry.

While `HOLDING`, `payload_cloud_filter` removes only self-returns from the
carrying-box envelope: 0.20 m to 0.50 m along x and +/-0.22 m along y in
`base_link` (with the configured vertical bounds). It prevents the held box
from appearing as a LiDAR obstacle; it does **not** expand the robot footprint
or reserve the carrying-box volume for collision checking. Recalibrate the
filter whenever the box dimensions or carry pose changes, and configure a
separate payload collision envelope if the workflow needs collision protection
for the held box.

`fine_align_server.ros__parameters` configures the tag admission gate, target,
capture envelope, controller, settling criteria, and timeouts. A stable target
is held in the robot frame only until the configured tracking timeout.
Invalid tag geometry is rejected with a throttled warning and clears the cached
target and stability samples. The server keeps running and accepts subsequent
valid measurements. If acquisition times out, the action reports `NO_STABLE_TAG`
using the configured retry policy rather than terminating the process. During
motion, an invalidated target follows the existing tag-loss stop behavior.
The previous OpenNav-based prototype is preserved on the local
`archive/opennav-table-docking` branch.

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
