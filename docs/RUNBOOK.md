# Runbook — step by step, and what every file does

Two parts: [**how to run it**](#part-1--running-it-step-by-step) and
[**what each file is for**](#part-2--file-map).

---

# Part 1 — Running it, step by step

Each step is independently verifiable. If a step fails, stop there — the later
ones assume it worked.

## Step 0 · Install dependencies (once)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-smac-planner \
  ros-humble-slam-toolbox \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  python3-colcon-common-extensions python3-rosdep
```

**Check:** `ros2 --version` prints something.

---

## Step 1 · Build

```bash
cd rse_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

**Check:** `colcon build` ends with `9 packages finished`. No warnings expected.

> First build takes ~3–5 min. If a package fails, build it alone to see the
> error clearly: `colcon build --packages-select amr_core`.

---

## Step 2 · Run the tests (no simulator needed)

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

**Check:** 250 tests, 0 failures. This alone validates the motion smoother, the
safety envelope, conflict detection, the yielding protocol, the sensor
validators, selective mapping, map fusion, the slope cost model, the world
geometry and every config loader.

`colcon test` also runs the style linters (`uncrustify`, `cpplint`, `flake8`,
`pep257`) alongside the functional tests, so a *style* failure and a *logic*
failure look alike in the summary. To tell them apart:

```bash
colcon test-result --verbose | grep -E 'gtest|pytest'   # the ones that matter
```

If uncrustify ever does complain, it can rewrite the files itself — it is
idempotent and authoritative:

```bash
for pkg in amr_core amr_msgs amr_sensor_bsp amr_fleet_control \
           amr_mapping amr_navigation; do
  (cd src/$pkg && ament_uncrustify --reformat)
done
```

---

## Step 3 · Sanity-check the configuration

```bash
ros2 run amr_bringup check_model_consistency.py
```

**Check:** `All checks passed.` Confirms every roster resolves, priorities are
unique, each robot's safety envelope is within its own LiDAR range, and every
model can climb the steepest ramp in the world.

---

## Step 4 · (Optional) Regenerate the world

Only needed if you edit the layout. The generated files are committed.

```bash
ros2 run amr_gazebo generate_world.py --package-dir src/amr_gazebo
colcon build --packages-select amr_gazebo && source install/setup.bash
```

**Check:** prints 4 ramps with their gradients and the paths it wrote.

---

## Step 5 · World only (no robots)

Start here rather than with the full stack — it isolates Gazebo problems from
navigation problems.

```bash
ros2 launch amr_bringup simulation.launch.py
```

**Check:** Gazebo opens showing racks, the central firewall with its doorway,
the Hump Bridge, the mezzanine, and moving pedestrians/robots.

To run without the GUI or without moving obstacles:

```bash
ros2 launch amr_bringup simulation.launch.py gui:=false
ros2 launch amr_bringup simulation.launch.py dynamic_obstacles:=false
```

---

## Step 5b · Check the robot description expands

Two seconds, and it isolates any xacro problem from the launch machinery:

```bash
xacro $(ros2 pkg prefix amr_description)/share/amr_description/urdf/amr.urdf.xacro \
    robot_name:=amr1 robot_model:=heavy_mapper | head -30
```

**Check:** XML starting `<?xml ...` with `<robot name="amr">` and links called
`amr1/base_link`, `amr1/lidar_link`. An error here is a description problem;
anything else is a launch problem.

---

## Step 6 · One robot

`robot.launch.py` does **not** start Gazebo, so you need two terminals:

```bash
# Terminal A - the world (leave step 5 running, or start it again)
ros2 launch amr_bringup simulation.launch.py

# Terminal B - one robot
source install/setup.bash
ros2 launch amr_bringup robot.launch.py robot_name:=amr1
```

**Check:**

```bash
ros2 topic hz /amr1/scan          # validated LiDAR flowing
ros2 topic echo /amr1/sensor_health --once
ros2 topic echo /amr1/safety_status --once
```

**Check the TF tree is whole** -- this is the single most useful check at this
step, because a split tree makes everything downstream fail confusingly:

```bash
ros2 run tf2_ros tf2_echo map amr1/base_footprint   # must resolve, not time out
ros2 node list | grep amr1                          # every node must be /amr1/...
```

If `ros2 node list` shows a bare `/slam_toolbox` or `/controller_server` rather
than `/amr1/...`, the node is outside its namespace, its parameters have not
applied, and it is running on defaults. That is what
`test_launch_namespacing.py` exists to prevent.

`safety_status.halt_active` should be `false` once scans arrive. Before the
first scan it is `true` with `reason: 3` (SENSOR_TIMEOUT) — that is the
fail-closed posture working, not a bug.

Variants for narrowing a problem down:

```bash
ros2 launch amr_bringup robot.launch.py robot_name:=amr1 navigation:=false
ros2 launch amr_bringup robot.launch.py robot_name:=amr1 slam:=false
```

---

## Step 7 · The full fleet

```bash
ros2 launch amr_bringup fleet.launch.py
```

This runs everything: Gazebo, both robots, SLAM, nav2, traffic control, map
fusion, RViz.

**Check:** RViz shows both robot models, both laser scans, and the merged map
filling in. `ros2 node list | wc -l` should be around 25.

If nav2 lifecycle bonds time out on a slower machine:

```bash
ros2 launch amr_bringup fleet.launch.py spawn_delay:=10.0
```

---

## Step 8 · Concurrent goals — the brief's primary scenario

Keep step 7 running. In a new terminal:

```bash
source install/setup.bash
ros2 run amr_bringup send_goals.py --list          # see the named goals
ros2 run amr_bringup send_goals.py \
    --goal amr1=heavy_storage \
    --goal amr2=packing_bay_4
```

**Check:** both plans appear in RViz; the script prints `all goals reached` and
exits 0.

---

## Step 9 · Selective mapping (System Optimization)

```bash
ros2 topic echo /amr1/map_update_stats
```

**Check:** `suppression_ratio` climbs above 0.9 as the robot re-covers known
ground, `frontier_cells` stays non-zero, `policy_state` moves between
`frontier_burst`, `exploring` and `throttled`.

---

## Step 10 · Slope-aware planning (A/B/C)

```bash
ros2 run amr_bringup demo_slope_planning.py
```

**Check:** the printed table shows

| | Configuration | Route |
|---|---|---|
| A | slope ON, doorway open | `doorway` |
| B | slope ON, doorway blocked | `bridge` |
| C | slope OFF, doorway open | control |

then `PASS`. Also try the ramp-only goal:

```bash
ros2 run amr_bringup send_goals.py --goal amr1=mezzanine_storage
```

---

## Step 11 · Conflict and yielding

```bash
ros2 run amr_bringup demo_conflict.py
```

**Check:** a timeline showing `amr2: YIELD` with a reason naming `amr1`, the
hold duration, the minimum separation, then `PASS`. Conflict spheres appear in
RViz while it runs.

---

## Step 12 · Safety override

```bash
ros2 run amr_bringup demo_safety_override.py --robot amr1 --speed 0.5
```

**Check:** `OVERRIDE CONFIRMED`, with the obstacle distance, the envelope
`d_safe` at that speed, and the measured reaction latency.

Manual e-stop, for the video:

```bash
ros2 service call /amr1/set_safety_override amr_msgs/srv/SetSafetyOverride \
  "{engage: true, reason: 'demo'}"
ros2 service call /amr1/set_safety_override amr_msgs/srv/SetSafetyOverride \
  "{engage: false, reason: 'demo'}"
```

---

## Step 13 · Payload-dependent motion smoothing

```bash
ros2 topic echo /amr1/cmd_vel --field linear.x &
ros2 service call /amr1/set_payload amr_msgs/srv/SetPayload "{payload_kg: 0.0}"
ros2 run amr_bringup send_goals.py --goal amr1=east_staging

ros2 service call /amr1/set_payload amr_msgs/srv/SetPayload "{payload_kg: 110.0}"
ros2 run amr_bringup send_goals.py --goal amr1=west_staging
```

**Check:** the service reply reports `load_ratio`, and the loaded run takes
visibly longer to reach cruise speed.

---

## Step 14 · Sensor validation (BSP)

```bash
ros2 topic echo /amr1/sensor_health
```

**Check:** three streams (`lidar`, `imu`, `camera`), each with
received/rejected/degraded counts and a measured rate.

To make the IMU plausibility warning fire, drop the limit and rebuild:

```bash
# in src/amr_description/config/robot_models.yaml, heavy_mapper.imu:
#   max_angular_velocity: 0.3
colcon build --packages-select amr_description && source install/setup.bash
# relaunch, then drive the robot; the warning names the model and the limit.
```

---

## Step 15 · Ten robots

```bash
ros2 launch amr_bringup fleet.launch.py \
    fleet_config:=$(ros2 pkg prefix amr_bringup)/share/amr_bringup/config/fleet_ten_robots.yaml \
    spawn_delay:=8.0 rviz:=false
```

**Check:** ten robots spawn. Heavy on one machine — the cheap proof is the unit
tests (`FleetConfigTest.ScalesToTenRobotsWithNoCodeChange`), which run in
milliseconds.

---

## Quick diagnosis

| Symptom | First command | Likely cause |
|---|---|---|
| Robot won't move | `ros2 topic echo /amr1/safety_status --once` | `halt_active: true`; read `reason` |
| `reason: 3` | `ros2 topic hz /amr1/scan` | No validated scan — check `sensor_health` |
| No merged map | `ros2 topic hz /amr1/map_contribution` | SLAM not publishing, or fusion not up |
| No plans | `ros2 lifecycle get /amr1/planner_server` | nav2 not activated; raise `spawn_delay` |
| Phantom wall on a ramp | `ros2 topic echo /amr1/sensor_health` | IMU rejected, so ground-rejection is off |
| `Test::Run() is private` when building tests | — | A test helper collides with a `testing::Test` member (`Run`, `SetUp`, `HasFailure`…). Rename the helper. |
| `Tf has two or more unconnected trees` | `ros2 node list \| grep amr1` | A node launched outside its namespace, so `root_key` params did not apply and SLAM published `map -> odom` instead of `amr1/map -> amr1/odom`. |
| `No critics defined for FollowPath` | `ros2 node list \| grep controller_server` | Same cause: `/controller_server` instead of `/amr1/controller_server` means nav2 ran on defaults. |
| `Unable to parse the value of parameter robot_description as yaml` | — | A `Command()` substitution needs `ParameterValue(..., value_type=str)`. Fixed; rebuild `amr_bringup`. |
| Local costmap jumps / drifts as the map updates | `ros2 param get /amr1/local_costmap/local_costmap global_frame` | Must be `amr1/odom`, not `map`. |
| `model library not found: .../install/amr_bringup/share/amr_description/...` | — | Stale `fleet.yaml` using a relative path. Fixed: it is now a `package://` URI. Rebuild `amr_bringup`. |
| `Robot is out of bounds of the costmap!` | `ros2 topic info /map -v \| grep -c PUBLISHER` | Must be **1** (`map_fusion`). If `slam_toolbox` also publishes `/map`, the static layer resized the global costmap to SLAM's private grid. |
| ...and if `/map` has one publisher | `ros2 run tf2_ros tf2_echo map amr1/base_footprint` | If it reads roughly **twice** the roster's `x:`/`y:`, the spawn pose is in the TF chain twice. `<odometry_source>` must be `0` (encoder) in `amr.gazebo.xacro`. See DESIGN_NOTES 8b. |
| `merged map: 0.0% explored` | (read map_fusion's ERROR line) | It now prints the evidence bounding box against the grid extent, which names the offset directly. |
| `AttributeError: can't set attribute` from a script | - | A `Node` subclass assigned to one of rclpy's read-only properties (`clients`, `publishers`, `timers`...). Rename it; `test_node_attribute_shadowing.py` rejects all of them. |
| Goals accepted but the robot doesn't move | `ros2 topic echo /amr1/safety_status --once` | `halt_active: true` means the override is holding it. That is the override working, not nav2 failing. |
| `Starting point in lethal space!` | `ros2 topic echo /amr1/scan --once \| head -20` | If the ranges are all ~0.3 m the scanner is inside its own chassis and SLAM is mapping the robot. `lidar.height` must clear `base_z_offset + chassis_height + 0.024`. See DESIGN_NOTES 8d. |
| `map geometry changed to 19x13` (a tiny map) | `ros2 run amr_bringup check_model_consistency.py` | Same cause. Compare the map's metres against the chassis dimensions -- if they match, the LiDAR is seeing the robot. |
| `minimum laser range setting (0.0 m) exceeds...` | - | The key is `min_laser_range`, not `minimum_laser_range`. Fixed, and now derived per robot from `robot_models.yaml`. |
| `colcon test` fails but gtest/pytest all pass | `colcon test-result --verbose \| grep -E 'gtest\|pytest'` | Only the style linters failed. Run `ament_uncrustify --reformat` in the package. |

---

# Part 2 — File map

Only the files that do something. Config, tests and generated artefacts are
grouped rather than listed individually.

## `amr_core` — shared foundation (no ROS dependency)

| File | Task |
|---|---|
| `include/amr_core/robot_model.hpp` | `RobotProfile`, `DynamicLimits`, `SafetySpec`, `RobotRole` enum. **Computes payload/speed-derated accel and jerk limits, and `d_safe = k·v² + d_min`.** The typed refactoring deliverable. |
| `include/amr_core/fleet_config.hpp` | Fleet roster + `FleetPolicy`. Rejects duplicate names/priorities. |
| `include/amr_core/geometry.hpp` | Angle wrapping, quaternion→yaw/pitch, exact unicycle integration. |
| `src/model_library.cpp` | Parses `robot_models.yaml` into `RobotProfile`, with cross-field validation (e.g. rejects a safety envelope the LiDAR can't see). |
| `src/fleet_config.cpp` | Parses `fleet.yaml`, resolves each robot's model. |
| `src/robot_model.cpp` | Role enum ↔ string. |

## `amr_msgs` — interfaces

| File | Task |
|---|---|
| `msg/PredictedTrajectory.msg` | A robot's forward projection, shared fleet-wide. |
| `msg/TrafficDirective.msg` | PROCEED / SLOW / YIELD + speed scale + reason. |
| `msg/SafetyStatus.msg` | Halt state, obstacle distance, `d_safe`, measured latency. |
| `msg/SensorHealth.msg` | Per-stream BSP validation counters. |
| `msg/MapUpdateStats.msg` | Selective-mapping suppression telemetry. |
| `msg/PayloadState.msg`, `srv/SetPayload.srv` | Runtime payload injection. |
| `srv/SetSafetyOverride.srv` | Manual e-stop. |

## `amr_description` — the robots

| File | Task |
|---|---|
| `config/robot_models.yaml` | **Single source of truth** for both robot models: geometry, mass, dynamic limits, safety constants, sensor specs. Read by the xacro *and* the C++ *and* the launch files. |
| `urdf/amr.urdf.xacro` | Top-level; renders either model from the YAML. |
| `urdf/robot_models.xacro` | Loads the YAML into xacro, sets the frame prefix. |
| `urdf/amr_base.xacro` | Chassis, drive wheels, casters. |
| `urdf/sensors.xacro` | LiDAR / IMU / camera. **Publishes only to `*_raw`** — this is what makes the BSP gate structural. |
| `urdf/amr.gazebo.xacro` | Diff-drive and joint-state plugins. |
| `urdf/inertials.xacro` | Inertia tensor macros. |

## `amr_gazebo` — the world

| File | Task |
|---|---|
| `amr_gazebo/world_builder.py` | **Generates the world.** One geometric description emits the SDF, the elevation map and the waypoints, so they cannot disagree. |
| `scripts/generate_world.py` | CLI wrapper for the above. |
| `scripts/dynamic_obstacle_driver.py` | Drives 6 collision-bearing obstacles on seeded patrol loops. |
| `worlds/warehouse_multilevel.world` | Generated. The Pinch, Hump Bridge, mezzanine, racks. |
| `maps/warehouse_elevation.{pgm,yaml}` | Generated. What the slope planner reads. |
| `config/waypoints.yaml` | Generated. Named goals. |

## `amr_sensor_bsp` — sensor validation gate

| File | Task |
|---|---|
| `include/amr_sensor_bsp/validator.hpp` | Base class; hard (drop) vs soft (forward + flag) fault grading; statistics. |
| `include/.../sensor_validators.hpp` + `src/sensor_validators.cpp` | LiDAR, IMU and camera validators. **IMU angular-velocity plausibility check** (the brief's explicit requirement), camera blacked-out/saturated detection, and **IMU-informed ground-return rejection** that stops a 2D LiDAR painting a phantom wall across every ramp. |
| `src/bsp_validation_node.cpp` | Subscribes `*_raw`, republishes validated names, publishes `SensorHealth`. |

## `amr_fleet_control` — motion, safety, traffic

| File | Task |
|---|---|
| `src/motion_smoother.cpp` | **Acceleration + jerk limiting** with a discrete-corrected S-curve approach law. Applies the traffic speed scale, so a yield is a *controlled* stop. |
| `src/safety_monitor.cpp` | `d_safe = k·v² + d_min` envelope, latching with hysteresis, fail-closed on sensor loss. |
| `src/trajectory.cpp` | Forward projection along the global plan (or the current twist). |
| `src/traffic_manager.cpp` | **Space-time conflict detection** + priority yielding protocol + anti-starvation boost. |
| `src/velocity_smoother_node.cpp` | Node: `cmd_vel_nav` → `cmd_vel_smoothed`. Owns the payload service. |
| `src/safety_override_node.cpp` | Node: `cmd_vel_smoothed` → `cmd_vel`. **Sole `cmd_vel` publisher** — this is what makes the override authoritative. |
| `src/trajectory_broadcaster_node.cpp` | Node: publishes this robot's projection to `/fleet/trajectories`. |
| `src/traffic_control_node.cpp` | Node: one per fleet. Issues directives, draws conflict markers. |

## `amr_mapping` — cooperative SLAM

| File | Task |
|---|---|
| `src/selective_mapping.cpp` | **Selective iteration:** frontier cells always publish, significant changes always publish, repeatedly-traversed regions are throttled. |
| `src/map_fusion.cpp` | Log-odds fusion of both robots' maps (not max-occupancy, so a transient obstacle can be overturned). |
| `src/selective_mapping_node.cpp` | Node: filters this robot's SLAM output, publishes `MapUpdateStats`. |
| `src/map_fusion_node.cpp` | Node: merges into `/map`, anchors each robot's private SLAM frame into the shared one. |

## `amr_navigation` — nav2 costmap plugins

| File | Task |
|---|---|
| `src/slope_cost_model.cpp` | Elevation loading + the slope→cost curve. Ramps priced high but **strictly below LETHAL** so they stay usable. |
| `src/slope_layer.cpp` | nav2 plugin wrapping the above; per-robot climbing limit. |
| `src/fleet_trajectory_layer.cpp` | nav2 plugin: writes peers' **predicted** positions into the local costmap with time decay. This is how the local planner consumes peer trajectories. |
| `costmap_plugins.xml` | pluginlib registration. |

## `amr_bringup` — launch, config, demos

| File | Task |
|---|---|
| `config/fleet.yaml` | **The roster.** Which robots exist, their models, start poses, priorities, and the fleet traffic policy. |
| `config/fleet_ten_robots.yaml` | The same file with eight more entries — the scalability proof. |
| `config/nav2_params.yaml` | One nav2 template for every robot; per-robot values rewritten at launch. |
| `config/slam_toolbox.yaml` | Per-robot SLAM tuning. |
| `amr_bringup/fleet_loader.py` | Reads the roster at launch time and **derives nav2's parameters from the model library** — no duplicated constants. |
| `launch/fleet.launch.py` | Top level. Iterates the roster; contains no robot names. |
| `launch/simulation.launch.py` | Gazebo, world, dynamic obstacles. |
| `launch/robot.launch.py` | One robot's whole stack, namespaced. |
| `launch/slam.launch.py` | slam_toolbox + selective mapping. |
| `launch/navigation.launch.py` | nav2. **Remaps the controller to `cmd_vel_nav`** — the one line that inserts the smoother and safety override into the command path. |
| `launch/fleet_control.launch.py` | Traffic controller + map fusion. |
| `scripts/send_goals.py` | Concurrent goals by waypoint name. |
| `scripts/demo_conflict.py` | Forces the narrow-intersection conflict; asserts who yielded. |
| `scripts/demo_safety_override.py` | Drives a hostile command stream; proves and measures the override. |
| `scripts/demo_slope_planning.py` | The A/B/C slope experiment. |
| `scripts/check_model_consistency.py` | Verifies the single-source-of-truth claim. |

## Docs

| File | Task |
|---|---|
| `README.md` | Overview, command chain, demos, troubleshooting. |
| `docs/REQUIREMENTS.md` | Every requirement → implementation → test. **Read this first.** |
| `docs/REFACTORING.md` | The refactoring deliverable. |
| `docs/DESIGN_NOTES.md` | Decisions worth defending, and the three bugs the tests caught. |
| `docs/ARCHITECTURE.md` | Node graph, topics, frames. |
| `docs/VIDEO_WALKTHROUGH.md` | Screenshare script with timings. |
| `docs/RUNBOOK.md` | This file. |
