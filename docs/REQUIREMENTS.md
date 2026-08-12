# Requirements traceability

Every requirement in the brief, mapped to the code that implements it and the
test or demo that proves it. Section numbers follow the assignment.

Legend: **Test** means an automated assertion that fails the build if the
behaviour regresses. **Demo** means a script that exits non-zero on failure.

---

## 1. Navigation setup and environment

| # | Requirement | Implementation | Evidence |
|---|---|---|---|
| 1.1 | **AMR-1 (Mapper/Lead)** — accurate global localisation and path planning, higher payload | `robot_models.yaml:heavy_mapper` (120 kg, 1080-beam 25 m LiDAR), `fleet.yaml` priority 100 | Test: `ModelLibraryTest.TheHeavyModelIsTheLessAgileOne`, `test_the_shipped_roster_matches_the_brief` |
| 1.2 | **AMR-2 (Scout/Follower)** — speed and dynamic obstacle avoidance, lower payload, **higher acceleration limits** | `robot_models.yaml:light_scout` (30 kg, 30 Hz LiDAR, accel 1.10 vs 0.35 m/s²) | Test: `DynamicLimits.HeavyUnitIsLessAgileThanScoutInEveryState` sweeps all load/speed states |
| 1.3 | **Large multi-level structure with ramps** | 44×30 m world; Hump Bridge 0.55 m with 8.7° ramps, mezzanine deck 0.45 m with 7.8° and 14.8° ramps | Test: `test_ramp_gradients_match_documentation`, `test_ramp_slab_pose_matches_elevation_model` |
| 1.4 | **Complex, initially unknown aisle layouts** | Three rack blocks, 2.4 m aisles, cross-aisles; no prior map — SLAM starts blank | Test: `test_primary_goals_are_reachable_from_the_docks` |
| 1.5 | **Static storage racks** | `_storage_block()` generates segmented rows | Test: `test_no_waypoint_sits_inside_an_obstacle` |
| 1.6 | **Frequent randomly moving dynamic obstacles** | 6 collision-bearing models (4 pedestrians, 2 third-party robots) on seeded patrol loops with dwell and corner-cutting | Test: `test_no_dynamic_obstacle_spawns_inside_geometry`, `test_dynamic_obstacles_expose_a_cmd_vel_interface` |

> Gazebo Classic `<actor>` elements have no collision geometry and are invisible
> to a LiDAR. Real models are used so obstacle avoidance is genuine.

---

## 2. Core mapping and global path planning

| # | Requirement | Implementation | Evidence |
|---|---|---|---|
| 2.1 | **Both robots contribute to a single unified global occupancy grid** | `amr_mapping::MapFusion` — log-odds accumulator; `map_fusion_node` anchors each robot's private SLAM frame into the shared `map` frame and publishes `/map` | Test: `MapFusionTest.BothRobotsContributeToOneMap`, `ContributionsAreTransformedIntoTheGlobalFrame` |
| 2.2 | **Selective mapping: prioritise unexplored boundaries** | `SelectiveMappingPolicy::IsFrontier` — frontier cells bypass all throttling | Test: `SelectiveMappingTest.FrontierCellsAreNeverThrottled`, `TheEdgeOfTheKnownGridCountsAsFrontier` |
| 2.3 | **Selective mapping: reduce update frequency for repeatedly traversed areas** | Per-cell visit counters from actual traversal; `saturated_period` (5 s) vs `explored_period` (1 s) | Test: `RepeatedlyTraversedRegionsAreThrottled` (>90% suppression), `SuppressionRisesWithFamiliarity` |
| 2.4 | **Concurrent goals for both robots** | `send_goals.py` dispatches and awaits together | Demo: `send_goals.py --goal amr1=heavy_storage --goal amr2=packing_bay_4` |
| 2.5 | **Ramp/slope traversability cost function** | `amr_navigation::SlopeCostModel` — `cost = base + (253−base)·f(θ)^γ`, lethal beyond the per-model climbing limit; `SlopeLayer` nav2 plugin | Test: 19 tests incl. `TheGeneratedRampsMeasureTheirDocumentedGradient` against the real elevation map |
| 2.6 | **Minimise ramp use unless the only viable path** | Ramps priced high but strictly below `LETHAL`; the world provides a flat alternative (The Pinch) and a ramp-only goal (mezzanine) | Test: `EveryShippedRampIsAvoidableButUsable`, `TheFlatDoorwayIsCheaperThanTheSlopedBridge`, `test_mezzanine_is_reachable_only_by_ramp`. Demo: `demo_slope_planning.py` (A/B/C) |

> 2.6 is the requirement most easily faked. `demo_slope_planning.py` includes a
> control trial with the slope layer disabled, because "the planner avoided the
> ramp" is only evidence if the ramp would otherwise have been chosen.

---

## 3. Multi-robot local path planning and safety control

| # | Requirement | Implementation | Evidence |
|---|---|---|---|
| 3.1 | **Motion smoothing: acceleration and jerk limited** | `amr_fleet_control::MotionSmoother` — discrete-time S-curve terminal-approach law | Test: `NeverExceedsTheAccelerationLimit`, `NeverExceedsTheJerkLimit`, `JerkIsBoundedThroughAnAbruptReversal` |
| 3.2 | **Limits depend on dynamic state (speed, payload)** | `DynamicLimits::EffectiveAccelX(load_ratio, speed_ratio)`; payload injected at runtime via `SetPayload` | Test: `PayloadSlowsTheAccelerationRamp`, `AccelerationTapersAsSpeedRises` |
| 3.3 | **Physically appropriate per type — AMR-1 lower accel than AMR-2** | Model library; enforced by the consistency checker | Test: `HeavyUnitRampsSlowerThanTheScout` (measured end-to-end, not read from config) |
| 3.4 | **Local planner consumes peers' projected trajectories** | `TrajectoryPredictor` → `/fleet/trajectories` → `amr_navigation::FleetTrajectoryLayer` writes time-decayed cost into each robot's local costmap | Test: `TrajectoryPredictorTest.*` (6 tests); layer registered in `nav2_params.yaml:local_costmap.fleet_layer` |
| 3.5 | **Traffic Control Node enforces a yielding protocol — lighter AMR-2 yields to AMR-1 — via a temporary controlled stop** | `ConflictDetector` (space-time) + `YieldPolicy` (priority); the directive scales the *target* velocity so the smoother shapes the stop | Test: `TheScoutYieldsToTheHeavyMapper`, `YieldScaleProducesAJerkLimitedStopNotACut`. Demo: `demo_conflict.py` |
| 3.6 | **Safety node monitors local obstacle detection** | `safety_override_node` subscribes to the BSP-validated `scan`; decision runs in the scan callback, not on a timer | Test: 19 `SafetyMonitorTest` cases |
| 3.7 | **Speed-dependent threshold `d_safe = k·v² + d_min`** | `SafetySpec::SafeDistance` | Test: `SafeDistanceFollowsTheQuadraticLaw`, `TheEnvelopeGrowsWithSpeed` |
| 3.8 | **Immediate halt that bypasses/overrides the nav stack's velocity command** | `safety_override_node` is the *sole* publisher of `cmd_vel`; on violation it discards the upstream command outright | Test: `PedestrianStepsOutInFrontOfAMovingRobot`. Demo: `demo_safety_override.py` measures the override against a hostile command stream |

> 3.5 and 3.8 are deliberately different mechanisms. A yield is a *controlled*
> stop and passes through the jerk limiter; a safety halt bypasses it, because a
> smooth ramp-down is the wrong answer to "stop now".

---

## 4. Sensor / data flow integration and standards

| # | Requirement | Implementation | Evidence |
|---|---|---|---|
| 4.1 | **BSP-style validation routine (HAL alternative)** | `amr_sensor_bsp::SensorValidator` base + LiDAR/IMU/camera validators, with hard (drop) vs soft (forward and flag) grading | Test: 31 tests in `test_sensor_validators.cpp` |
| 4.2 | **Nav stack consumes sensor data only after validation** | Sensors publish `*_raw`; validators republish the plain names. The gate is structural — validated topics have no other publisher | `sensors.xacro` (raw only), `nav2_params.yaml` and `slam_toolbox.yaml` subscribe to `scan` only |
| 4.3 | **Log a warning when IMU angular velocity exceeds a physically plausible limit for the robot model** | Per-model `imu.max_angular_velocity` (2.50 heavy / 4.50 scout); dedicated `RCLCPP_WARN_THROTTLE` naming the model | Test: `ImplausibleAngularVelocityIsFlagged`, `TheLimitIsPerModel`, `TheLimitUsesTheVectorMagnitudeNotJustYaw` |
| 4.4 | **Validation output for IMU *and* camera** (rubric) | `CameraValidator` — structural checks plus mean-intensity content check catching blacked-out and saturated imagers; `SensorHealth` published per stream | Test: `ABlackedOutImagerIsCaught`, `ASaturatedImagerIsCaught` |
| 4.5 | **Cleanly namespaced multi-robot code** | Every robot's stack under `PushRosNamespace(robot_name)`; all frames prefixed in the URDF; fleet-wide topics under `/fleet/*` | `robot.launch.py`, `robot_models.xacro` |
| 4.6 | **Reusable class structure; fleet expandable to 10+ by minimal config change** | Roster-driven throughout; `fleet_ten_robots.yaml` is the two-robot file plus eight lines | Test: `FleetConfigTest.ScalesToTenRobotsWithNoCodeChange`, `YieldPolicyTest.WorksUnchangedForATenRobotFleet`, `test_an_arbitrary_fleet_size_loads` (25 robots) |

---

## 5. Deployment and code quality

| # | Requirement | Implementation | Evidence |
|---|---|---|---|
| 5.1 | **Complete, well-documented README covering launch of simulation, navigation and custom control** | [`README.md`](../README.md) — quick start, command chain, demos, troubleshooting | — |
| 5.2 | **Proper ROS 2 workspace, CMake/Colcon, dependencies for a clean build** | 9 packages, `ament_cmake` / `ament_cmake_python`, complete `package.xml` dependency sets, `rosdep`-installable | `colcon build` with `-Wall -Wextra -Wpedantic -Wshadow` |
| 5.3 | **Screenshare video explaining the problem statement** | Script and shot list in [`VIDEO_WALKTHROUGH.md`](VIDEO_WALKTHROUGH.md) | *(recording is the candidate's to produce)* |
| 5.4 | **Strict style guide (Google C++ / PEP 8)** | Google C++ throughout; PEP 8 for Python | `ament_uncrustify`, `ament_cpplint`, `ament_flake8`, `ament_pep257` in `colcon test` |
| 5.5 | **Identify a refactoring area, propose a plan, implement part of it** | [`REFACTORING.md`](REFACTORING.md) — two implemented (parameter files → typed C++; monolithic launch → composable tree), one proposed with a scoped first increment | — |

---

## Evaluation criteria

| Skill area | Success metric | Where it is met |
|---|---|---|
| **SLAM & Global Planning** | Complete high-quality map; navigates base → goal | Per-robot `slam_toolbox` fused into one `/map`; `send_goals.py` drives dock → Heavy Storage / Packing Bay 4 |
| **Safety & Override** | Controlled stop when an obstacle violates the safe distance, overriding user input; dynamic obstacles | `safety_override_node` as sole `cmd_vel` writer; `demo_safety_override.py` proves it against a hostile command stream and reports measured latency |
| **Motion Control** | Observably smooth; velocity profile modulates with environment | Jerk-limited S-curve; `SmootherDiagnostics` reports which constraint bound each tick, so smoothness is attributable rather than assumed |
| **Custom Integration** | Functional HAL class used by the nav node; validation runs and logs for IMU and camera | `amr_sensor_bsp` gates every sensor stream structurally; `SensorHealth` telemetry per stream |
| **System Optimization** | Programmatically restrict the map update area on a defined condition | `SelectiveMappingPolicy` with a four-way cell classification; `MapUpdateStats` publishes the achieved suppression ratio |
| **Code Quality & Build** | Clean, commented, modular; builds cleanly; compelling refactoring demo | 9 packages, ROS-free algorithm cores, 250 tests, `REFACTORING.md` |

---

## Deliberate deviations

Three places where the letter of the brief was not followed, and why.

**1. "HAL" is a BSP, and it is a gate rather than a wrapper.**
The brief asks for a BSP-style validation routine *instead of* a classic HAL,
and the distinction was taken seriously. A HAL abstracts *how* you talk to a
device; this layer asserts what a device on this chassis can physically do and
treats anything outside that envelope as a fault. It is implemented as a topic
gate rather than a library the nodes call, so a node cannot accidentally consume
unvalidated data — the raw topics simply are not what anything subscribes to.

**2. The IMU plausibility violation is a *soft* fault.**
The brief asks for a warning, and a warning is what it gets: the message is
forwarded, counted, and logged. Dropping it would be the more aggressive
reading, but a yaw rate of 40 rad/s is far more likely to be a units or
timestamp bug than a robot genuinely spinning, and starving the localisation of
IMU data mid-manoeuvre would destabilise the very estimate that has to recover
from the fault. The reasoning is recorded at
`sensor_validators.hpp:ImuValidator`.

**3. Multi-level navigation is 2.5D, not full 3D.**
Ramps and raised decks are handled as a traversability-cost problem over an
elevation field, not as separate map layers with a level-transition graph. Full
multi-floor navigation needs per-level costmaps and an explicit transition
planner, which is a substantially larger system. The 2.5D treatment is what
makes the slope-cost requirement meaningful and honest, and it is why the ramps
are gentle enough for a 2D LiDAR to cope with — helped by the IMU-informed
ground-return rejection described in [`DESIGN_NOTES.md`](DESIGN_NOTES.md), which
exists precisely because a level-mounted 2D LiDAR on a ramp otherwise paints a
phantom wall across it.
