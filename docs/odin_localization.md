# Odin localization on X2

Select Odin for localization while retaining the chest LiDAR obstacle pipeline.
FAST-LIO remains the default. Navigation never starts an Odin device driver,
robot_state_publisher, or controller manager. Do not run both localization
backends or both standalone and navigation-owned Odin adapters simultaneously.

## Interfaces and TF ownership

| Data | FAST-LIO backend | Odin backend |
|---|---|---|
| Local input | `/Odometry_loc` | `/odin1/odometry_highfreq`, configurable `input_topic` |
| Robot odometry | adapter publishes `/odom` | adapter publishes `/odom` |
| Root TF owner | E1R/Open3D localization | `odin_localization_adapter` |
| Global correction | E1R/Open3D | native `odom -> map` on isolated `/odin1/tf` |
| Robot state | shared `x2_bringup` | same shared `x2_bringup` |
| Obstacles | chest LiDAR | same chest LiDAR |

In Odin mode, robot TF is `map -> odom -> base_link -> ... -> torso_link ->
odin_imu`. Only the adapter publishes the first two edges. The driver continues
publishing its native `odom -> imu`, `imu -> lidar`, `odom -> camera_0`, and
`odom -> map` on **a separate TF topic**, `/odin1/tf`. These names deliberately
remain separate from URDF frames. Do not bridge native TF wholesale into `/tf`,
attach native `imu` to the URDF, or publish a second static root transform.
Native camera visualization must listen to the isolated native graph.

The local driver source and the [manufacturer data-output documentation](https://manifoldtechltd.github.io/wiki/en/odin_series/odin1/5.%20Data%20output_.html)
agree that the correction is the pose of map in odom, `T_odin_odom_odin_map`.
The adapter publishes
`T_nav_map_odom = T_nav_map_odin_map * inverse(T_odin_odom_odin_map)`.
Its local base pose is
`T_odom_base = T_odom_imu * inverse(T_base_odin_imu)`.
Robot TF is queried at the corrected measurement timestamp, including waist
and other articulated joints. There is no latest-TF fallback.

Local poses are sampled at up to 100 Hz from the approximately 400 Hz input.
A queue holds up to `max_pending_samples` (default 256), bounded additionally
by `local_timeout_sec`. Each tick selects the newest sample covered by exact
timestamped robot TF, retaining newer samples for delayed TF. This prevents
new arrivals from starving older usable poses. `/odin1/odometry` (approximately 10 Hz) is an explicit alternative input;
there is no automatic stream switching. Velocity is differentiated from local
base poses and expressed in current base axes, accounting for relative torso
motion. Global corrections never enter differentiation. Native twist and
covariance are not assumed to have ROS-compatible axes: output covariances use
configurable diagonal estimates, `pose_variance` and `twist_variance`.

## Calibration and passive findings

The robot anchor is the existing `torso_to_odin_imu` from session_005. That
calibration is **provisional**: RMS 4.82 px, p95 10.26 px, one head posture.
Neither its values nor factory sensor geometry were changed.

During the 2026-10-20 passive inspection, native correction packets arrived at
approximately 10 Hz. The driver emits these only in mode 2 after relocalization
succeeds; this supports successful relocalization, rather than merely assuming
the checked-in YAML reflects the running device. The running node did not
expose its effective device map mode/path as ROS parameters. The local default
YAML was still mapping mode, so it must not be treated as the live configuration.

High-frequency local odometry arrived near 390 Hz. Its stamps led this host by
about 0.28 seconds; standard odometry and corrections led it by about 0.15 seconds.
These are receive-age observations, **not a measured clock offset**. Measure
device/host clock alignment independently before choosing
`sensor_time_offset_sec`. That same offset applies to local poses and native
corrections. Do not increase future tolerance to conceal clock misalignment or
enable the driver's future-stamped TF bursts for navigation.

Runtime WIWC IMU-LiDAR geometry differed from the factory geometry by roughly
2.8 cm and 0.8 degrees; runtime camera-LiDAR geometry differed too. Native camera
TF is computed from those same WIWC matrices and does not independently resolve
the discrepancy. This integration does not use those transforms or Odin clouds.
It assumes the documented native IMU pose refers to the physical calibrated IMU;
verify base pose against the robot posture before accepting runtime use.
No shared robot-state publisher or `base_link` chain was visible during the
initial passive sample, so timestamped robot TF coverage remains a runtime check.
The final passive sample again saw mode-2 correction packets at about 10 Hz and
local odometry at about 396 Hz, with no `/odom` publisher or shared joint-state
stream. High-frequency stamps then led the host by 0.15–0.18 seconds, demonstrating
that the initial receive-age observation must not be hard-coded as a clock offset.
The live driver remained on its original `/tf`; no private native TF publisher
or new adapter was started on the robot domain.

## Occupancy map

The generated local artifacts are in `/home/ubuntu/x2_ws/odin_maps/`:

- `map_20261015_153035.yaml` and `.pgm`: 748 by 1017 cells at 0.05 m.
- `map_20261015_153035.conversion.json`: source hashes, conversion parameters,
  exact origin, floor-fit evidence, and cell-round-trip verification.
- `odin_driver_localization.yaml`: a new startup copy pointing to the supplied
  `.bin`, with mode 2. Review against the effective live config before use.

The source PCD has 7,820,940 points. Its sampled floor fit is near Z = -0.70 m,
tilt 0.017 degrees, RMS 0.012 m; no leveling or coordinate transform was applied.
Odin-map-to-occupancy-map alignment is therefore identity, provided the PCD export
corresponds to the selected `.bin`. An occupancy YAML origin is the grid's lower
corner in map coordinates; it is **not** an extra localization transform.
The saved origin was corrected to the converter's full-precision value:
`[-3.567807197570801, -36.13247299194336, 0]`.

Reproduce conversion on an isolated domain, preserving existing converter tuning:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash
export ROS_DOMAIN_ID=96
ros2 run pcd2pgm pcd2pgm_node --ros-args \
  -p pcd_file:=/home/ubuntu/x2_ws/odin_maps/map_20261015_153035.pcd \
  -p odom_to_lidar_odom:='[0.0,0.0,0.0,0.0,0.0,0.0]' \
  -p flag_pass_through:=false -p map_resolution:=0.05 \
  -p thre_z_min:=-0.60 -p thre_z_max:=1.30 \
  -p thre_radius:=0.1 -p thres_point_count:=10 \
  -p map_topic_name:=/odin_map_conversion/map
```

In another terminal with the same isolated environment, save under a **new** name:

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/odin_map_review \
  --ros-args -r map:=/odin_map_conversion/map -p save_map_timeout:=10.0
```

Preserve the exact `/odin_map_conversion/map.info.origin` when saving: Humble's
saver rounds large coordinates in YAML. Conversion retained 2,347,063 points and
produced 122,274 occupied cells; all 760,716 saved cells matched the source grid.
This converter marks the remaining cells free; it does not reconstruct surveyed
free-space bounds. Inspect map boundaries and obstacle coverage before use.
Keep map artifacts outside source repositories; no maps are added to package Git.

## Readiness and failure behavior

`/odin1/localization_ready` (`std_msgs/Bool`) and
`/odin1/localization_status` (`std_msgs/String`) publish at 100 Hz. Readiness
requires advancing valid local input, timestamp-valid robot TF, a fresh valid
native correction, a velocity baseline, and `map_alignment_confirmed: true`.
That flag defaults false; identity alignment is never silently accepted.

With `localization_backend:=odin autostart:=true`, navigation starts its processes
but leaves the managed Nav2 nodes unconfigured until the startup gate observes
sustained readiness. The lifecycle manager itself has immediate autostart disabled.
The gate requires true readiness for 0.5 seconds, heartbeats no older than 0.25
seconds, and a complete `map -> odom -> base_link` chain in its own TF buffer
with transform ages between -0.05 and 1.0 seconds. It also waits for
`/lifecycle_manager_navigation/manage_nodes`, then sends exactly one `STARTUP`
request. Waiting is indefinite, with informational messages at most every five
seconds. Successful lifecycle startup ends the gate process normally. A failed
request exits with an error and is not retried automatically because some nodes
may already have transitioned; inspect their states before a manual recovery.

The gate's private parameters are `ready_hold_sec`, `readiness_timeout_sec`,
`tf_max_age_sec`, `future_tolerance_sec`, `readiness_topic`, `manager_service`,
`map_frame`, `odom_frame`, and `base_frame`. Standard frame/topic defaults match
the Odin adapter defaults. No extra launch flags are required. `autostart:=false`
retains manual lifecycle control without a gate; FAST-LIO startup is unchanged.
This gate controls initial Nav2 startup only. A loss after activation leaves
Nav2 running, with motion blocked by the velocity bridge as described below.
Localization can change after a startup request is submitted; the velocity
bridge remains the authority for suppressing transport commands during that loss.
Operator-panel and cloud-preprocessor startup TF diagnostics are independent
and are not suppressed by this gate.

Defaults: local timeout 0.25 s, correction timeout 1.0 s, future tolerance 0.05 s.
Both measurement time and steady-clock progress are checked. Repeated packets
do not keep old corrections alive. Map TF retains the correction's measurement
stamp and stops updating when unready; TF buffers can retain historical samples.
The readiness gate handles loss even before TF caches expire.

Large local pose steps (0.5 m or 0.5 rad), backward stamps, invalid local input,
or local input/TF timeouts reset the baseline and invalidate the cached
correction. Recovery requires a correction measured strictly after the reset
boundary as well as a new local velocity baseline. Separate correction timestamp
watermarks reject both exact replays and previously unseen pre-reset packets.
Transient TF lookup misses retain history while bounded pending samples wait. Local odometry remains available during correction loss
when its own input and robot TF are valid.

The velocity bridge requires readiness only in Odin mode. Missing/false readiness
or a heartbeat older than 0.25 s forces zero transport output and clears commands.
Recovery requires a newly received command. No localization-loss fallback to
FAST-LIO is automatic. The driver exposes no independent tracking-confidence or
loss flag; fresh correction packets are evidence of correction availability,
not proof of localization accuracy. Inactive Nav2 is preferred for initial checks.

## Deploying the readiness-gated startup

Deploy the changed `x2_navigation` launch file, gate executable, CMake install/test
registration, tests, and documentation to the container source checkout. Preserve
its existing `config/` files and map artifacts. Build inside `robojudo-plus`:

```bash
cd /home/agi/workspace/x2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
(unset FASTRTPS_DEFAULT_PROFILES_FILE; colcon build --symlink-install --packages-select x2_navigation)
source install/setup.bash
```

At a coordinated restart, use the existing startup script and Odin profile with
`NAVIGATION_AUTOSTART=true`. No script changes are required. Preview first:

```bash
NAVIGATION_AUTOSTART=true ENABLE_VELOCITY_BRIDGE=false \
  bash /home/agi/workspace/run_ros_stack.sh --localization true --navigation true --dry-run
```

If the isolated driver already runs, use `--localization false`; if shared state
already runs, also set `START_STATE_BRINGUP=false`. Remove `--dry-run` only after
coordinating process ownership. Keep command forwarding disabled for this passive
validation. Before relocalization, check that `/planner_server` is unconfigured;
after the gate reports startup success, check that it is active:

```bash
ros2 lifecycle get /planner_server
ros2 topic echo /odin1/localization_ready --once
```

Confirm the global costmap no longer prints the missing-map timeout during this
initial wait. Later localization-loss diagnostics remain enabled. Roll back this
feature by restoring the previous package revision and rebuilding at another
coordinated restart, preserving all runtime configuration files.

## Controlled startup and validation — not executed on the live robot

Coordinate all live stack changes first. Use the live robot's existing ROS domain
and DDS profile; do not carry the isolated test environment into runtime commands.
On each host, source ROS Humble and this workspace's `install/setup.bash`.

1. Review the new driver config against the actual driver's settings. Confirm
   calibration serial, firmware/driver compatibility, mode 2, map path, and clock
   alignment. Do not reset the algorithm or change tuned files as part of checking.
2. Stop FAST-LIO/Open3D localization and the old navigation adapter if running.
   Keep shared state alive. If absent, start shared bringup **once**:

   ```bash
   ros2 launch x2_bringup state_publisher.launch.py command_transport:=zmq
   ```

3. Coordinate replacement of the existing Odin driver with native TF isolation.
   Do not start a second driver against the same device:

   ```bash
   ros2 launch x2_navigation odin_driver.launch.py \
     config_file:=/home/ubuntu/x2_ws/odin_maps/odin_driver_localization.yaml
   ```

   This X2 wrapper starts only the official `host_sdk_sample` executable and
   remaps its native TF to `/odin1/tf`. It rejects `senddepth`, `sendreprojection`,
   or `sendoverlay` when enabled, rather than starting auxiliary executables that
   internally ignore custom configuration. Use a separate startup copy with
   those flags disabled; no source tuning is overwritten. The official Odin
   launch file and auxiliary C++ implementations are unchanged. Only an eight-line
   SDK-node patch remains: honor `config_file` and clean up missing-file errors.
   Confirm `/tf` no longer contains native `odom -> map` or `odom -> imu`.

4. Copy `config/odin_localization.yaml` to a new runtime file. Set the independently
   measured timing offset. Set `map_alignment_confirmed: true` only after matching
   the active `.bin` to the supplied PCD/occupancy map and inspecting alignment.
   Do not overwrite source calibration or tuned navigation parameters.

   ```bash
   cp /home/ubuntu/x2_ws/src/x2_navigation/config/odin_localization.yaml \
     /tmp/x2_odin_runtime.yaml
   # Review/edit the new runtime copy as described above.
   ros2 launch x2_navigation odin_localization.launch.py \
     params_file:=/tmp/x2_odin_runtime.yaml
   ```

5. Passively inspect status, `/odom`, timestamped robot TF, and base pose in map.
   Check exactly one publisher for each root edge and shared state component:

   ```bash
   ros2 topic echo /odin1/localization_status
   ros2 topic info /odom --verbose
   ros2 run tf2_ros tf2_echo map base_link
   ros2 run tf2_ros tf2_echo base_link odin_imu
   ```

   Compare physical torso posture/base heading with the converted pose. Require
   advancing stamps and bounded timestamped TF coverage; do not substitute latest
   TF or claim the provisional calibration is accepted.

6. Stop only the standalone adapter before starting navigation's adapter. Initially
   disable the velocity bridge and lifecycle autostart:

   ```bash
   ros2 launch x2_navigation navigation.launch.py \
     localization_backend:=odin odin_params_file:=/tmp/x2_odin_runtime.yaml \
     map:=/home/ubuntu/x2_ws/odin_maps/map_20261015_153035.yaml \
     enable_velocity_bridge:=false autostart:=false rviz:=false
   ```

   Inspect chest LiDAR -> bounded preprocessing -> ground segmentation -> self
   filter -> payload filter -> costmap input. Preserve existing height bounds,
   sensor origin, confidence settings, and timestamps. Configure/activate Nav2
   only after coordinated approval for that runtime validation step. Validate
   map overlay, floor removal, robot rejection, and near-obstacle coverage with
   no goals or motion commands. The hardware path has not been accepted by the
   offline tests. Later operational startup can enable the bridge explicitly
   after those checks and coordination; its existing endpoint is unchanged.

Rollback: stop Odin navigation/adapter, keep native Odin TF isolated, restart the
existing E1R localization workflow with `publish_robot_root_tf` enabled, and
launch navigation with `localization_backend:=fast_lio` and its original map.
Keep shared bringup running once. No tuned parameter restoration is needed.

## Offline checks

Changed files in `x2_navigation`:

- Runtime: `src/odin_localization_adapter.cpp`, `src/nav2_zmq_velocity_bridge.cpp`,
  `include/x2_navigation/odin_transform.hpp`, `include/x2_navigation/localization_gate.hpp`.
- Launch/configuration: `launch/navigation.launch.py`, `launch/odin_localization.launch.py`,
  `launch/odin_driver.launch.py`, `config/odin_localization.yaml`, `config/x2_self_filter.urdf`.
- Packaging/documentation: `CMakeLists.txt`, `package.xml`, `README.md`, this document.
- Tests: `test/test_odin_transform.cpp`, `test/test_localization_backend.py`,
  `test/test_odin_localization.launch.py`, `test/test_odin_recovery.launch.py`,
  `test/test_odin_driver_launch.py`, `test/test_odin_navigation.launch.py`,
  `test/odin_navigation_inputs.py`, and the shared fixture in `test/test_navigation.launch.py`.
  The Odin/Nav2 fixture uses actual adapter root TF and synthetic joint/IMU poses;
  it does not supply static map/odom/base transforms.

Changed files in `odin_ros_driver`: `src/host_sdk_sample.cpp` (eight added lines)
and `test/test_launch_configuration.py`. The earlier official-launch changes
were removed in favor of the X2 wrapper.
Generated map/startup artifacts are listed above. The pre-existing changes in
`x2_navigation/config/nav2_params.yaml`, `test/test_navigation_config.py`, and
`pcd2pgm` were preserved; they are not integration edits.

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash
colcon build --symlink-install --packages-select x2_navigation odin_ros_driver \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
ROS_DOMAIN_ID=96 ROS_LOG_DIR=/tmp/x2-odin-tests PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  colcon test --packages-select x2_navigation --event-handlers console_direct+ \
  --ctest-args -R 'odin|localization_backend|navigation_config|command_topic_routing|odometry_transform|velocity_command|navigation_command_gate|fast_lio_odom_adapter|test_test_navigation.launch|nav2_zmq_velocity_bridge|velocity_send_failure' \
  --output-on-failure
colcon test-result --test-result-base build/x2_navigation/test_results --verbose
ROS_DOMAIN_ID=96 ROS_LOG_DIR=/tmp/x2-odin-driver-tests PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -q src/odin_ros_driver/test/test_launch_configuration.py
```

Tests cover transform direction, rotated mounting, articulation compensation,
local velocity independent of correction jumps, malformed/future/stale inputs,
readiness heartbeat loss and recovery, delayed dynamic TF, outage/reset and old
correction replay rejection, auxiliary-request validation, backend selection,
TF isolation, and the retained synthetic Nav2 obstacle pipeline. The missing-config driver test exits
before SDK/device initialization. No hardware motion or live stack restart is
part of these checks.

Validation on 2026-10-20 after the robustness fixes: both packages built and all
15 focused CTest suites passed on localhost-only domain 97. The package result
inventory reports 205 tests with zero errors/failures/skips; that inventory also
includes earlier results outside the focused selection. The missing-config
driver regression passed separately. Re-running the original review reproductions
on domain 98 produced about 95 local outputs per second with delayed TF, and
kept readiness false after the outage/reset until a new correction arrived.
No hardware motion or live driver restart was performed. User-specified
`nav2_params.yaml` still has Git blob prefix `e13cdc7`.
