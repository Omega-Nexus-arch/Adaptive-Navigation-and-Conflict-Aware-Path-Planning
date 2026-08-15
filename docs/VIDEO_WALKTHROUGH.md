# Screenshare walkthrough — script and shot list

> **Brief:** *"To submit a screenshare video explaining the various integrities
> of the problem statement."*

Target: **12–15 minutes**. The recording itself is yours to make; this is the
running order, the exact commands, and the sentence to say over each shot.

A note on framing that is worth more than any single demo: reviewers have seen
plenty of videos of a robot driving somewhere. What distinguishes this
submission is that most claims are *measured* rather than shown. Lean on that —
where a script prints a number, read the number out.

---

## Before recording

```bash
cd rse_ws
colcon build --symlink-install && source install/setup.bash
colcon test --event-handlers console_direct+ && colcon test-result --verbose
ros2 run amr_bringup check_model_consistency.py
```

Have four terminals ready: **fleet**, **demos**, **topic echo**, **scratch**.
Have `docs/REQUIREMENTS.md` and `docs/REFACTORING.md` open in a browser tab.

---

## 0 · Framing (0:00 – 1:00)

> "Two heterogeneous AMRs in a multi-level warehouse. I'll cover cooperative
> mapping with selective iteration, slope-aware global planning, conflict-aware
> local planning with a yielding protocol, and a low-latency safety override.
>
> The thing I'd most like you to take away is that these are tested rather than
> demonstrated — 274 automated tests, and the demo scripts exit non-zero if the
> behaviour they check isn't there."

**Shot:** the repository tree, then `docs/REQUIREMENTS.md` scrolled once.

---

## 1 · Architecture (1:00 – 3:00)

**Shot:** the command chain diagram in the README.

> "This is the most important diagram. nav2 publishes to `cmd_vel_nav`, not
> `cmd_vel`. The command passes through the velocity smoother, which applies
> the traffic directive and the acceleration and jerk limits, and then through
> the safety override, which is the *sole* publisher of `cmd_vel`. There is no
> path from a planner to the wheels that skips it — that's what makes the
> override an override rather than a suggestion.
>
> Two arbitration layers, failing in opposite directions on purpose. Traffic
> control fails open: if it dies you lose throughput, not collision avoidance.
> The safety override fails closed: no sensor data means halt."

**Shot:** `src/amr_core/include/amr_core/robot_model.hpp`.

> "Every physical constant lives in one YAML file and is parsed into these
> structs. The xacro reads it, the C++ reads it, and the launch files derive
> nav2's parameters from it. That's refactoring deliverable number one."

---

## 2 · Bring the fleet up (3:00 – 4:30)

```bash
ros2 launch amr_bringup fleet.launch.py
```

> "The warehouse is generated, not hand-authored. One geometric description
> emits the SDF, the elevation map the slope planner reads, and the named
> waypoints — so they can't disagree, and there's a test asserting the ramp
> geometry matches the elevation model to within a millimetre."

**Shot:** Gazebo — pan across the racks, The Pinch, the Hump Bridge, the
mezzanine. Then RViz as the merged map fills in.

> "Each robot runs its own SLAM in a private map frame. The fusion node anchors
> those into one shared frame and merges them — that's the single unified
> occupancy grid, and it's what every planner in the fleet uses."

---

## 3 · Concurrent goals (4:30 – 5:30)

```bash
ros2 run amr_bringup send_goals.py --goal amr1=heavy_storage --goal amr2=packing_bay_4
```

> "Both goals dispatched together. AMR-1 to Heavy Storage, AMR-2 to Packing Bay
> 4 — the brief's scenario."

**Shot:** RViz with both global plans visible.

---

## 4 · Selective mapping (5:30 – 7:00)

```bash
ros2 topic echo /amr1/map_update_stats
```

> "This is the System Optimization criterion — programmatically restricting the
> map update area on a defined condition.
>
> Four classes of cell. Frontier cells, next to unknown space, always publish.
> Cells that changed significantly always publish. Everything else is
> throttled, and how hard depends on how many times the robot has actually
> *driven* there — not how often it's been seen."

**Shot:** the `suppression_ratio` field climbing as the robot re-covers ground.

> "Read that number: it's above ninety per cent on a well-travelled aisle, and
> it drops back the moment the robot reaches somewhere new. `policy_state` says
> which regime it's in."

---

## 5 · Slope-aware planning (7:00 – 9:00)

The centrepiece. Do not rush it.

```bash
ros2 run amr_bringup demo_slope_planning.py
```

> "The requirement has two halves that pull in opposite directions: minimise
> ramp use, *unless* the ramp is the only viable path. So the ramp is priced
> high but strictly below lethal. Lethal would satisfy the first half and break
> the second.
>
> Three trials. A: slope cost on, flat doorway open — expect the doorway. B:
> doorway blocked — expect the bridge, because it's now the only crossing. C is
> the control: slope cost *off*, doorway open."

**Shot:** the printed table.

> "C is what makes A mean anything. Without it, 'the planner avoided the ramp'
> could just mean the flat route happened to be shorter."

Then:

```bash
ros2 run amr_bringup send_goals.py --goal amr1=mezzanine_storage
```

> "The mezzanine is ramp-only. There's a test that seals both ramp mouths and
> requires the deck to become unreachable — so this scenario provably tests
> something."

---

## 6 · Conflict and yielding (9:00 – 10:30)

```bash
ros2 run amr_bringup demo_conflict.py
```

> "The Pinch is two metres wide. One robot fits; two don't. Both are sent
> through from opposite sides at once.
>
> Conflicts are detected in space *and* time. Two paths that cross are only a
> problem if the robots are at the crossing together — a purely geometric test
> would stop the fleet at every junction in a warehouse where every aisle
> crosses every other one."

**Shot:** RViz conflict markers appearing, then the printed timeline.

> "AMR-2 yields, because AMR-1 outranks it. Note *how* it yields: the directive
> scales the target velocity upstream of the smoother, so the jerk limiter
> shapes the deceleration. That's the 'temporary, controlled stop' the brief
> asks for — not a zero written to the wheels.
>
> Strict priority starves the bottom of the roster, so there's a time-boxed
> priority inversion after twelve seconds of continuous yielding. Three tests
> hold it to that, including one that checks the boost *expires*."

---

## 7 · Safety override (10:30 – 12:00)

The claim reviewers are most likely to be sceptical of. Make it measurable.

```bash
ros2 run amr_bringup demo_safety_override.py --robot amr1 --speed 0.5
```

> "Watching a robot stop in Gazebo doesn't distinguish an override from nav2
> simply planning around something. So this publishes a constant forward
> velocity straight into `cmd_vel_nav`, ignoring every obstacle, and records
> what actually reaches `cmd_vel`."

**Shot:** the report.

> "`cmd_vel` was zeroed while `cmd_vel_nav` still demanded full speed. The
> navigation command was discarded, not obeyed.
>
> The threshold is speed-dependent — `d_safe = k·v² + d_min`. Read the numbers:
> the obstacle distance, the envelope at that speed, and the measured reaction
> latency from the sensor's own timestamp. And the halt latches with hysteresis,
> because releasing at the same distance it triggers gives you a limit cycle —
> there's a test that drives it at exactly the boundary for two hundred cycles
> and allows at most one transition."

---

## 8 · Motion smoothing and BSP (12:00 – 13:30)

```bash
ros2 service call /amr1/set_payload amr_msgs/srv/SetPayload "{payload_kg: 110.0}"
```

> "Acceleration and jerk limits depend on payload and current speed. At full
> load AMR-1's effective acceleration drops from 0.35 to 0.16."

**Shot:** the ramp-to-speed visibly lengthening.

> "Worth mentioning: the first version of this limiter was wrong, and the tests
> caught it. Clamping the *velocity* at the ceiling truncates the acceleration
> in one tick, which is an unbounded jerk — measured at eight times the limit.
> The fix is a discrete-corrected S-curve approach law. It's in
> `docs/DESIGN_NOTES.md` with the parameter sweep."

```bash
ros2 topic echo /amr1/sensor_health
```

> "The BSP layer. Sensors publish to `*_raw`; validators republish the plain
> names; nothing in the navigation stack subscribes to a raw topic, so the gate
> is structural.
>
> Hard faults are dropped, soft faults are forwarded and flagged. The IMU
> plausibility check the brief asks for is soft, deliberately: forty radians per
> second is far more likely to be a units bug than a spinning robot, and
> starving localisation of IMU data mid-manoeuvre would destabilise the estimate
> that has to recover from it.
>
> And the BSP layer earns its keep somewhere unexpected — it's the only place
> holding both the raw scan and a validated attitude, so it's where the
> ground-return rejection lives that stops a 2D LiDAR painting a phantom wall
> across every ramp."

---

## 9 · Scaling and refactoring (13:30 – 15:00)

**Shot:** `config/fleet.yaml` beside `config/fleet_ten_robots.yaml`.

> "Ten robots is this file plus eight lines. No new message, no new node, no
> launch edit — and the same two robot models. Nothing in the code names a
> robot; everything iterates the roster."

```bash
ros2 run amr_bringup check_model_consistency.py
```

> "This checks the single-source-of-truth claim rather than asserting it: every
> roster resolves, priorities are unique, each model's safety envelope is within
> its own LiDAR range, and every model can climb the steepest ramp in the
> world."

**Shot:** `docs/REFACTORING.md`.

> "Two refactorings implemented. The parameter files became typed C++ structs —
> AMR-1's acceleration went from five places to one, and a mistyped key is now a
> startup error instead of a silent default. And the monolithic launch file
> became five composable ones, which is what makes the roster-driven scaling
> possible.
>
> Section three proposes the next one — the costmap layer stack, where ordering
> still carries semantics that only exist as a YAML comment — with a scoped
> first increment.
>
> `docs/DESIGN_NOTES.md` has the decisions I'd expect you to push back on, and
> the three real bugs the tests and a layout render caught. Happy to go deeper
> on any of it."

---

## If something breaks on camera

Say what you expected, check `safety_status` first — it fails closed, so a
stopped robot is usually a sensor problem — and move on. A candidate who
debugs calmly reads better than one whose demo was suspiciously perfect.

```bash
ros2 topic echo /amr1/safety_status --once
ros2 topic echo /amr1/sensor_health
ros2 topic hz /amr1/scan
```
