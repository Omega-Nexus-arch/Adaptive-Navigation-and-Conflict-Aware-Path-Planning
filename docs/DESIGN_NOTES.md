# Design notes

The decisions worth arguing about, and the bugs found on the way. Written for
a reviewer who wants to know whether the thinking was any good, not just
whether the boxes are ticked.

---

## 1. The jerk limiter was wrong, and the tests caught it

The obvious velocity smoother is three clamps in sequence:

```cpp
a_desired = (v_target - v_current) / dt;
a_bounded = clamp(a_desired, -decel_limit, +accel_limit);
a_final   = clamp(a_bounded, a_prev - jerk*dt, a_prev + jerk*dt);
v_command = clamp(v_current + a_final*dt, min_vel, max_vel);   // <-- the bug
```

It looks correct and it is not. `NeverExceedsTheJerkLimit` failed on the first
run with a measured jerk of **4.64 m/s³ against a 0.60 limit** — a factor of
almost eight.

The final velocity clamp is the culprit. As the robot approaches top speed the
acceleration is still large; the clamp truncates the velocity step; the
*achieved* acceleration collapses to near zero in a single tick; and that step
in acceleration is, by definition, an unbounded jerk. The limiter enforces the
jerk bound on the acceleration it *requests* and then silently violates it on
the acceleration it *delivers*.

Two further failures came from the same family:

- **Envelope switching.** Re-clamping to the acceleration envelope *after* the
  jerk clamp caused a 10.03 m/s³ spike during a full-speed reversal, because
  the braking bound (1.6) and the accelerating bound (1.10) swap as the robot
  passes through zero velocity, and forcing the acceleration inside a
  newly-shrunk envelope in one tick is itself a step.
- **Discrete lag.** Replacing the clamp with the textbook continuous-time
  S-curve cap `sqrt(2·j·|e|)` reduced the overshoot but did not remove it: the
  acceleration is chosen from the error measured at the *start* of the tick, so
  the ramp-down runs one full tick behind the continuous curve.

The fix is a discrete-corrected terminal-approach law. Unwinding an
acceleration `a` to zero at jerk limit `j` consumes `a²/(2j) + a·dt` of
velocity, not `a²/(2j)`. Solving for `a`:

```
a_cap = sqrt((j·dt)² + 2·j·|e|) − j·dt
```

A parameter sweep over both robot models, jerk limits from 0.6 to 4.0 and tick
rates from 10 to 50 Hz:

| Form | Peak jerk vs limit | Overshoot past target |
|---|---|---|
| Continuous `sqrt(2j·e)` | at limit | 28 mm/s |
| Half-step correction | at limit | 0.6 mm/s |
| **Full-step (shipped)** | **at limit** | **0** |

With zero overshoot the velocity clamp never fires, so it could be removed
entirely — which is what closes the loop, because that clamp was the original
bug. All 18 smoother tests now pass.

**The transferable lesson:** a limiter that enforces its bound on the value it
requests, rather than the value it achieves, is not a limiter. The test that
found this measured the *achieved* jerk from the emitted command stream. A test
that had checked the internal `a_final` variable would have passed happily.

---

## 2. Why the safety override and the traffic controller fail in opposite directions

Both can stop a robot. They are deliberately built with opposite failure
postures, and getting this backwards is a classic way to ship something that
looks safe.

| | Traffic controller | Safety override |
|---|---|---|
| Purpose | Throughput | Collision avoidance |
| Scope | Fleet-wide, centralised | Per robot, local |
| Authority | Can only *slow* | Can *halt*, and is the sole `cmd_vel` writer |
| Input | Predicted trajectories | Directly measured range |
| On failure | **Fails open** — robots stop being told to yield | **Fails closed** — no data means halt |

If the traffic controller dies, the fleet keeps working with collision
avoidance intact and throughput that degrades under congestion. If it failed
closed instead, a single crashed node would immobilise the warehouse — and
somebody would eventually disable it.

The safety override is the opposite. No scan, a stale scan, or a scan the BSP
layer rejected all halt the robot. A monitor that assumes the world is clear
when it cannot see is worse than no monitor at all, because it still carries
the authority to override the planner.

This is also why a yield and a halt travel different paths. A yield scales the
*target* velocity upstream of the smoother, so the jerk limiter shapes it into
the "temporary, controlled stop" the brief asks for. A safety halt bypasses the
smoother, because a smooth ramp-down is the wrong answer to "stop now".

---

## 3. The 2D LiDAR would have made the ramps impassable

The requirement wants ramps that the planner reasons about. A fixed, level 2D
LiDAR on a robot climbing a ramp pitches with the chassis, so its beam tilts
down and strikes the floor ahead: at AMR-1's 0.42 m mounting height on an 8.7°
ramp, that is 2.78 m in front. The costmap fills with a phantom wall across the
ramp, the planner declares it impassable, and the entire slope-cost experiment
quietly evaluates nothing.

Three options were considered:

1. **Tilt the LiDAR up.** Trades the phantom wall for a blind spot at the
   robot's feet — unacceptable when the same sensor feeds the safety envelope.
2. **Filter by costmap height.** A 2D scan has no height information to filter
   on.
3. **Reject returns consistent with the ground plane, using measured pitch.**
   The predicted ground range is `h / sin(|pitch|)`; anything at or beyond it,
   less a tolerance, is floor.

Option 3 needs the chassis attitude, and the only component holding both the
raw scan and a validated attitude estimate is the BSP layer. So the IMU
validator feeds pitch into the LiDAR validator, and the LiDAR validator
suppresses floor returns while the robot is on a slope:

```cpp
// LidarValidator
double GroundReturnRange() const {
  const double pitch = std::abs(pitch_);
  if (!options_.ground_rejection_enabled || pitch < options_.ground_rejection_pitch) {
    return std::numeric_limits<double>::infinity();
  }
  return spec_.height / std::sin(pitch);
}
```

Two properties make it safe, both tested:

- Attitude is only ever taken from an IMU sample the validator **accepted**
  (`PitchIsNotUpdatedFromARejectedSample`), so one sensor's failure cannot
  corrupt the other's output.
- A genuine obstacle on the ramp, closer than the predicted floor range,
  survives (`OnARampFloorReturnsAreSuppressedButObstaclesSurvive`).

This is the strongest argument for building the BSP layer as a real component
rather than a pass-through that ticks a box: it is the only place in the system
where this fix could live.

---

## 4. Space-time conflicts, not path intersections

In a warehouse where every aisle crosses every other aisle, a geometric
path-intersection test flags a conflict almost continuously and the fleet stops
moving. Two robots whose paths cross are only in conflict if they are at the
crossing *together*.

So `ConflictDetector` compares trajectories on a shared absolute time grid and
reports the earliest instant at which centre distance falls below
`r_a + r_b + margin`. The distinction is pinned by two tests that differ only
in where the second robot starts:

- `CrossingPathsAtDifferentTimesAreNotAConflict` — paths intersect, no conflict.
- `CrossingPathsAtTheSameTimeAreAConflict` — same geometry, conflict at 1.2 s.

The second test also documents something easy to get wrong: the conflict is
flagged at **1.2 s**, not at 2.0 s when the two centres coincide. With a
required separation of 1.27 m the robots are already too close at (−0.8, 0) and
(0, −0.8), which are 1.13 m apart. Warning only at the moment of collision
would leave nothing to react with.

Complexity is O(N²·S) — about 950 distance evaluations per cycle at ten robots
and 21 samples, negligible at 10 Hz. Because `Detect` is a pure function of the
trajectory set, spatial hashing could be dropped in later without touching a
caller.

---

## 5. Strict priority starves somebody

"The lighter AMR-2 always yields to the heavier AMR-1" is the brief's rule and
it is implemented exactly. It also has a failure mode: in a busy aisle there is
always someone more important, and the lowest-ranked robot never moves. At ten
robots this stops being hypothetical.

The mitigation is a time-boxed priority inversion. After
`max_yield_seconds` (12 s) of continuous yielding, the starved robot's
effective priority is lifted above the entire roster for
`yield_cooldown_seconds` (4 s), forcing the conflict to resolve the other way.

```cpp
int YieldPolicy::EffectivePriority(const std::string & robot_id, double now) const {
  const auto it = state_.find(robot_id);
  if (it == state_.end()) return 0;
  if (now < it->second.boost_until) {
    return max_base_priority_ + 1;   // above everyone, briefly
  }
  return it->second.base_priority;
}
```

Being aggressive here is affordable precisely because the per-robot safety
override runs underneath and is the actual collision guarantee. The traffic
controller is optimising throughput; it is allowed to be wrong.

Three tests hold the mechanism to its contract: the boost fires
(`AStarvedRobotIsEventuallyLetThrough`), it actually inverts the outcome rather
than merely releasing the loser, and it **expires** (`TheStarvationBoostExpires`)
— a temporary inversion that became permanent would be its own bug.

---

## 6. Generating the world instead of writing it

The slope planner needs an elevation field. Authoring that separately from the
`.world` file guarantees they diverge after the first edit, and every "the
planner avoided the ramp" claim becomes unverifiable.

So one geometric description in `world_builder.py` emits three artefacts: the
SDF, the elevation map, and the named waypoints. They cannot disagree, and the
test suite checks the seam directly — `test_ramp_slab_pose_matches_elevation_model`
samples five points along every ramp and requires the SDF slab's top face to
match the elevation model to within a millimetre.

Generation also made properties testable that would otherwise be assertions in
a README:

- `test_mezzanine_is_reachable_only_by_ramp` seals both ramp mouths and requires
  the deck to become unreachable under a step-height traversability model.
- `test_pinch_admits_one_robot_but_not_two` measures the doorway and requires it
  to be between one and two AMR-1 widths.
- `test_blocking_the_pinch_leaves_the_bridge_as_the_only_crossing` verifies the
  world's topology is what the slope demo assumes.
- `test_floor_surfaces_do_not_overlap` — added after the render revealed the
  gentle mezzanine ramp overlapping the Hump Bridge's east ramp. Two surfaces at
  one point means `sample_elevation` returns whichever was declared first, and
  the elevation map silently stops describing what Gazebo simulates.

---

## 7. Two bugs found by rendering and by arithmetic

**The overlapping ramps.** Caught by plotting the layout rather than by reading
the code. The elevation map and the physics disagreed over a 2.7 m² patch on a
route the planner would use. Now guarded by a test.

**Cell-boundary aliasing in the gradient.** `test_slope_cost_model` reported the
Hump Bridge ramps at **4.48° against a true 8.7°** — almost exactly half. The
cause: sampling `x ± resolution` from an arbitrary query point. When `x` lands
near a cell boundary, floating-point rounding puts the `+step` probe back in the
*same* cell, the two-cell stencil collapses to one, and the measured gradient
halves.

It is a nasty failure because it is silent and biased in the dangerous
direction: it *under*-reports slope, exactly the error that would let a robot
onto a ramp too steep for it. The fix is to snap to the containing cell's centre
before differencing, so the stencil is exact by construction:

```cpp
double sample_x = x, sample_y = y;
if (!map.CellCentre(x, y, &sample_x, &sample_y)) {
  return sample;   // outside the map
}
```

Fixing it exposed a second, smaller issue. The steep ramp still read 13.24°
against 14.8°, within quantisation: the elevation encoder was padding its range
to a round 1.0 m for a world whose tallest deck is 0.55 m, throwing away 45% of
its 8-bit dynamic range. Tightening the headroom to 25% brought every ramp
within 1.5° of its true gradient. Worth doing next to a 16° lethal threshold.

Both are now regression-tested:
`TheGradientDoesNotDependOnWhereInACellYouAsk` sweeps 49 query offsets within a
single cell and requires an identical answer.

---

## 7b. A test helper named `Run`, and a hole in the offline harness

Worth recording because the *harness* was the real defect, not the typo.

`test_motion_smoother.cpp` had a namespace-scope helper called `Run`. That
fails to compile under gtest, with an error that points nowhere useful:

```
error: 'void testing::Test::Run()' is private within this context
```

A `TEST()` body is a member function of a class derived from `testing::Test`,
and that class declares a private `Run()`. Unqualified lookup finds the member
first and never reaches namespace scope. The helper is now `Drive`.

The interesting part is why it survived local verification. The offline harness
used while writing these tests expanded `TEST()` to a *free function*, so there
was no enclosing class and no member to shadow. It accepted code that real
gtest rejects — a harness that is more permissive than the tool it stands in
for, which is the one property such a harness must not have.

Fixed by making the shim generate a class derived from a stub `testing::Test`
carrying the same private members (`Run`, `SetUp`, `TearDown`, `HasFailure`,
`RecordProperty`, `IsSkipped`). It now reproduces the error, and a scan across
every test file confirms no other helper collides.

**The transferable lesson:** a stand-in for a tool should err towards being
*stricter* than the real thing. A looser one converts compile errors into
someone else's problem, which is exactly what happened here.

---

## 7c. A relative path that only worked before installing

`fleet.yaml` referenced the model library as
`../../amr_description/config/robot_models.yaml`. From the source tree that is
correct. From the install space it is not:

```
src/amr_bringup/config/../../amr_description/config/
  -> src/amr_description/config/                                  correct

install/amr_bringup/share/amr_bringup/config/../../amr_description/config/
  -> install/amr_bringup/share/amr_description/config/            does not exist
```

Every package installs under *its own* prefix, so a relative hop between
packages is meaningless once installed. Unit tests passed, `colcon build`
passed, and the first `ros2 launch` failed.

The fix is a `package://amr_description/config/robot_models.yaml` URI resolved
in two places, in order: `AMENT_PREFIX_PATH` for the install space, then a walk
up the source tree for an unbuilt checkout. Both matter — the first is what
production uses, the second is what keeps the tests runnable without a sourced
workspace, and losing either would have hidden this bug in the other direction.

The resolver is reimplemented in ten lines rather than pulling in
`ament_index_cpp`, because `amr_core`'s one genuinely useful property is that
its 27 tests need no ROS installation at all.

**The transferable lesson:** relative paths *between packages* are wrong even
when they work. The thing that made this bug expensive was that the two spaces
diverge only after installation, so every cheap check — build, unit test, source
inspection — agreed the path was fine.

---

## 7d. Two launch-layer bugs with the same shape

Both came from a value being correct *in general* and wrong *in one place*.

**A URDF is not YAML.** `robot_description` was built with `Command(['xacro', ...])`
and handed straight to `robot_state_publisher`. Launch tries to YAML-parse a
parameter's value before passing it on, and a URDF is XML, so the whole launch
aborted with *"Unable to parse the value of parameter robot_description as
yaml"* before a single node started. The fix is one wrapper:

```python
robot_description = ParameterValue(Command([...]), value_type=str)
```

**A key-based rewrite cannot express a per-section value.** nav2's parameters
were generated from one template using `nav2_common.RewrittenYaml`, whose
`param_rewrites` match on *key name* and replace every occurrence. That is right
for `robot_radius` and wrong for a frame:

| key | global costmap | local costmap | blanket rewrite gives |
|---|---|---|---|
| `global_frame` | `map` | `<robot>/odom` | `map` everywhere |
| `local_frame` | — | (behavior_server) `<robot>/odom` | never rewritten, stays `odom` |

The first anchors a *rolling* local costmap to the map frame, so it lurches
every time SLAM corrects `map -> odom`. The second leaves an unprefixed frame
that does not resolve in a namespaced TF tree. Neither throws; both just
degrade.

Frames now use `<ns>` placeholders substituted textually before the file is
parsed, so each occurrence keeps its own value, and `nav2_substitutions()`
carries no frame keys at all. A test asserts that absence directly, because the
tempting fix -- adding `global_frame` back to the rewrites -- is exactly the bug.

**The transferable lesson:** a substitution mechanism that matches on key name
assumes the key means the same thing everywhere in the file. For nav2
parameters that assumption is false, and it fails quietly.

---

## 7e. One scoping mistake, three unrelated-looking failures

The most instructive bug in the project, because the symptoms were nowhere near
the cause.

SLAM and nav2 are started a few seconds after the rest of the robot, so the
Gazebo diff-drive plugin has time to begin publishing `odom -> base_footprint`.
That delay was implemented by putting a `TimerAction` inside the robot's
namespaced group:

```python
namespaced_nodes.append(TimerAction(period=delay, actions=[slam, navigation]))
...
GroupAction([PushRosNamespace(robot_name)] + namespaced_nodes)   # the bug
```

`PushRosNamespace` is **scoped**. `GroupAction` applies it while visiting its
children and pops it as soon as that visit finishes. `TimerAction` does not run
its children during the visit -- it defers them -- so by the time they start,
the namespace is gone. Everything in the timer launched into the **root**
namespace, silently.

Three failures followed, none of which points at namespacing:

| Symptom | Actual cause |
|---|---|
| `Could not find a connection between 'map' and 'amr1/base_footprint'. Tf has two or more unconnected trees.` | `slam_toolbox` started as `/slam_toolbox`, so its `root_key: amr1` parameters never matched. On defaults it published `map -> odom` while the robot's frames were `amr1/odom` and `amr1/base_footprint`. |
| `No critics defined for FollowPath`, lifecycle bringup aborts | nav2's servers started unnamespaced, so their parameters did not match either. `controller_server` came up with an empty critics list. |
| Local costmap silently loads `static/obstacle/inflation` | Same cause. The configured `obstacle/fleet/inflation` never applied, so the conflict-aware fleet layer was simply absent -- and nothing said so. |

The fix is that every deferred block carries its own push, with the timer on the
outside:

```python
TimerAction(period=delay, actions=[namespaced(robot_name, [slam, navigation])])
```

Wrapping now happens in exactly one helper, `namespaced()`, and
`test_launch_namespacing.py` walks the launch description that
`robot.launch.py` actually produces -- modelling the timer's cleared scope --
asserting no per-robot node is reachable without a push, for one robot and for
all ten. Two of its checks are pure source-shape assertions that run without ROS
installed, so the guarantee holds even in a bare checkout.

**The transferable lesson:** when a parameter file is keyed by node name, a node
in the wrong namespace does not fail -- it quietly runs on defaults. That makes
namespacing a *correctness* property worth asserting structurally, not a
cosmetic one. The generic version: any mechanism that silently substitutes a
default when a lookup misses will convert a wiring error into a behavioural one,
and behavioural errors are debugged far from where they were introduced.

---

## 7f. A costmap the robot was never inside

**Symptom.** With one robot spawned, `planner_server` logs, on repeat:

```
[planner_server-13] [WARN] [nav2_costmap_2d]: Robot is out of bounds of the costmap!
```

**Cause.** nav2's global costmap defaults to **5 x 5 m anchored at the
origin**, and a non-rolling costmap only resizes when its *static layer*
receives a map. Every robot in this fleet spawns near `x = -18.5`. So from
`planner_server` activating until the first fused map lands on `/map`, the
robot is outside its own global costmap and the planner refuses to plan.

The window is short when map fusion is running. It is unbounded when it is
not -- and it is not whenever someone launches a single robot with
`fleet_services:=false`, which is exactly what the runbook tells them to do to
isolate a problem. The debugging advice produced a failure of its own.

**Why it was invisible.** Same reason as 7e: nothing errors. A costmap of the
wrong size is a *valid* costmap. The only evidence is a warning that names the
symptom (`out of bounds`) rather than the cause (`5 x 5 at the origin`), and no
log line anywhere states the costmap's actual geometry.

**Fix.** The bounds of the shared grid became a first-class part of the roster
(`map_extents:` in `fleet.yaml`). Two consumers read that one block -- map
fusion allocates the merged grid over it, and the global costmap is sized and
placed from it -- so the fused map and the costmap it fills are the same
rectangle *by construction*.

**Why the geometry is a textual placeholder and not a `RewrittenYaml` key.**
The same trap as the frame names in 7d. `RewrittenYaml` matches on key name and
replaces every occurrence, and `width`, `height` and `resolution` all appear
again in the rolling *local* costmap. One `width` rewrite would have given
every robot a 48-metre local costmap updated at 10 Hz -- a worse bug than the
one being fixed, and one that would have looked like a performance problem
rather than a configuration one. `test_the_local_costmap_keeps_its_own_geometry`
holds that line.

**What stops it recurring.** Thirteen tests, of which two carry the weight:
`test_the_global_costmap_contains_every_spawn_pose` expands the real
`nav2_params.yaml` the way `navigation.launch.py` does and asserts every
robot's spawn pose lies inside the resulting grid with clearance for its own
footprint -- for both rosters, ten robots included. And `load_map_extents`
rejects a roster whose spawn point does not fit, so moving a robot outside the
map is a startup error rather than a warning nobody reads.

**The transferable lesson**, and it is 7e's wearing different clothes: a
library default that is *structurally valid but situationally wrong* is more
dangerous than a missing value. A missing value raises. A 5 x 5 costmap at the
origin runs perfectly and simply never contains the robot.

---

## 7g. A node that ignores its own namespace

7f sized the global costmap correctly and the warning kept coming. Three
commands explained why:

```
$ ros2 topic info /map -v
Publisher count: 2
  Node name: map_fusion     Node namespace: /
  Node name: slam_toolbox   Node namespace: /amr1     <-- publishing on /map

$ ros2 topic info /amr1/map -v
Publisher count: 0
Subscription count: 1        (selective_mapping, waiting for a map nobody sends)

[map_fusion]: merged map: 0.0% explored (0 cells observed by 2 robots)
```

**Cause.** `PushRosNamespace` only moves topics a node created with a
*relative* name. slam_toolbox constructs its map publishers with literal
absolute names -- `/map` and `/map_metadata` -- so no amount of correct
namespacing moves them. The launch file did carry a remap rule, but it was
written `('map', 'map')`: a relative source, matching a topic the node never
created. **An unmatched remap rule is not an error.** It silently does nothing.

Two consequences, and they were one bug:

1. **The pipeline was severed.** `selective_mapping` waits on `<robot>/map`,
   which now had no publisher, so it never emitted a contribution and
   `map_fusion` merged nothing -- printed once every ten seconds by a node that
   was working perfectly.
2. **The costmap was resized to the wrong map.** Both slam_toolbox's private
   grid and the fused grid arrived on `/map`. nav2's static layer takes the
   last one it receives, and slam_toolbox's is a small grid in the `amr1/map`
   frame near the origin. So the global costmap, correctly sized by 7f, was
   immediately resized to a few metres around the origin.

**Fix.** Name the topic in the remap exactly as the node created it:
`('/map', 'map')` -- absolute source, relative target.

**What stops it recurring.** Three tests, all running without ROS:
`test_absolutely_named_topics_are_remapped_with_a_leading_slash` keeps a table
of packages known to hardcode absolute names; `test_no_remap_rule_is_a_relative_no_op`
rejects any `('x', 'x')` rule outright, because that pair is always either dead
code or this bug and it *reads* as deliberate; and
`test_only_map_fusion_may_publish_the_shared_map` holds the fusion invariant.

**The transferable lesson.** ROS 2's remapping and namespacing are *matching*
mechanisms, and a match that fails is indistinguishable from one that was never
needed. 7e was a namespace push that did not reach its nodes, 7f a default that
was valid but wrong, this a rule that matched nothing -- three mechanisms, one
shared property: **the failure mode is silence, so the only durable defence is
a test that asserts the wiring rather than a reviewer who reads it.**

---

## 8. Log-odds map fusion, not maximum occupancy

The simple fusion is to take the maximum occupancy across robots. In this
warehouse it has a specific pathology: a pedestrian seen once by one robot
becomes permanently occupied in the merged map, and the aisle closes for good.

Accumulating evidence in log-odds makes a single observation a weak claim that
later contrary observations can overturn, while a rack seen repeatedly from both
robots converges hard. Two details matter:

- **The accumulator is clamped** (±3.5). Without it, a long-held belief becomes
  unfalsifiable and the map stops tracking reality.
  `ConfidenceIsClampedSoTheMapStaysResponsive` fires 500 stale "occupied"
  observations and then 12 current "free" ones, and requires the cell to end up
  free.
- **Suppressed cells are skipped, not treated as free.** This is the contract
  that lets selective mapping and fusion compose: a cell the policy withheld
  arrives as unknown, meaning "no new evidence", not "empty".

Per-observation weights are deliberately modest because the independence
assumption between two robots sharing a world and sometimes a viewpoint is not
strictly true. Overconfidence shows up as exactly the stale-obstacle problem the
accumulator exists to avoid.

---

## 8b. The start pose, applied twice

The real cause behind 7f and 7g's shared symptom. Both of those were genuine
defects; neither was *this*, which is why the warning survived them.

**Two symptoms that look unrelated:**

```
[planner_server]: Robot is out of bounds of the costmap!        (every cycle)
[map_fusion]: merged map: 0.0% explored (0 cells observed by 2 robots)
```

Meanwhile every component was healthy: `/amr1/map` publishing at 1 Hz,
`selective_mapping` writing 258 frontier cells, `/map` with exactly one
publisher, the costmap correctly sized to the warehouse.

**Cause.** Four components each contribute one link to
`map -> <robot>/base_footprint`:

| Link | Published by |
|---|---|
| `map -> <robot>/map` | `map_fusion`, from the roster's initial pose |
| `<robot>/map -> <robot>/odom` | `slam_toolbox`, the SLAM correction |
| `<robot>/odom -> <robot>/base_footprint` | the Gazebo diff-drive plugin |

The chain is correct only if the robot's start pose appears in **exactly one**
of them. It belongs in the first -- anchoring a private SLAM frame into the
shared one is what lets ten robots share a map.

The URDF had `<odometry_source>1</odometry_source>`, which is `WORLD`:
`gazebo_ros_diff_drive` publishes Gazebo's ground-truth pose as odometry. So
`<robot>/odom -> <robot>/base_footprint` *already* contained (-18.5, 2.2), and
the anchor added it again. The robot reported **(-37.0, 4.4)** in `map`.

That one number explains both symptoms:

* -37.0 is outside the global costmap's `x[-24, 24]`, so
  `LayeredCostmap::updateMap` calls `isOutofBounds` and warns every cycle;
* every map cell carries the same offset, so all of them fall outside the
  merged grid and `Integrate` discards the lot -- `0 cells observed`, while the
  robot's own SLAM map looked perfect.

**Fix.** `<odometry_source>0</odometry_source>` -- encoder odometry, starting
at zero.

There is a second reason to prefer it that matters more than the bug.
Ground-truth odometry means SLAM never has to correct a drift or close a loop,
so a cooperative-SLAM exercise built on it measures nothing. Encoder odometry
drifts. That is the entire reason SLAM exists.

**Why it took three passes.** The warning names a symptom -- "out of bounds" --
and never the quantity that is wrong. Three causes produced the same sentence.
The lesson is not "look harder"; it is that a symptom shared by several causes
should be the last thing consulted, not the first. The decisive evidence, once
frames were suspected, was arithmetic: the anchor is (-18.5, 2.2), the costmap
starts at -24, and -18.5 - 18.5 = -37.

**What stops it recurring.**

* `test_frame_chain.py` asserts the invariant directly: odometry must be
  ENCODER, and `map_fusion` must be the only component that turns the roster's
  initial pose into a transform. Both checks run without ROS.
* `MapFusion::IntegrationReport` ends the ambiguity that hid this. `Integrate`
  returned 0 for three unrelated reasons and the caller could not tell which,
  so the most specific thing the system could say was "0.0% explored". It now
  reports how many cells were unknown, how many fell outside the grid, and the
  **bounding box of the evidence in the shared frame** -- which prints the
  doubled offset directly:

```
amr1: all 258 observed cells fell OUTSIDE the merged grid. Their extent in
'map' is x[-37.2, -36.8] y[4.2, 4.6]; the grid covers x[-24.0, 24.0]
y[-17.0, 17.0]. This is a frame error, not a small map -- check that the
robot's odometry starts at zero, because if it already carries the spawn pose
then the roster offset (-18.5, 2.2) is being applied twice.
```

**The transferable lesson.** A count is a measurement, not a diagnosis. Any
function that can return zero for several unrelated reasons will eventually be
asked which one happened, and the cheapest moment to answer is before it is
asked. This three-field struct would have turned a three-round debugging
session into one glance -- and unlike a guard test, it also helps with the
failures nobody thought to predict.

---

## 8c. `self.clients = {}`

The demo dispatcher crashed before sending a single goal:

```
File "send_goals.py", line 75, in __init__
    self.clients = {}
AttributeError: can't set attribute 'clients'
```

`rclpy.node.Node` exposes `clients` as a **read-only property**, along with
`publishers`, `subscriptions`, `services`, `timers`, `guards`, `waitables`,
`context` and `handle`. Every one is the natural name for a dictionary of
things a node holds, which is what makes the collision easy: the line reads
correctly, and Python only resolves the property when the constructor runs.

This is 7b in a different language. There, a test helper named `Run()` collided
with a private member of `testing::Test` and broke the build. The C++ one at
least failed at compile time. This failed at *construction* time, in a script
whose unit tests never instantiate the node, so nothing caught it until a
scripted acceptance run reported "the robots don't move".

**Fix.** `self.goal_clients`.

**What stops it recurring.** `test_node_attribute_shadowing.py` walks the AST
of every Python file in the workspace, finds every class deriving from `Node`,
and rejects any `self.<name> = ...` where `<name>` is one of the base class's
read-only properties. It is source-level on purpose -- an import-level check
would not have caught this, because the collision needs a constructor to run.
A second test re-derives the reserved list from the real `rclpy.node.Node`
wherever ROS is installed, so the hardcoded copy cannot drift.

**The transferable lesson**, now the third instance in this project: inheriting
from a framework class means inheriting its whole namespace, and frameworks
reserve exactly the names that are most natural to reuse. The general defence
is not vigilance; it is a check that enumerates the base class and compares.

---

## 8d. A scanner that could only see itself

**Symptom.** Both robots accept their goals and neither moves. The planner
says:

```
[amr1.planner_server]: GridBased: failed to create plan, invalid use:
                       Starting point in lethal space! Cannot create feasible plan..
[send_goals]: amr1: failed (status 6)
```

Everything upstream looked healthy: TF connected, the global costmap resized to
960 x 680, `map_fusion` anchoring both robots, nav2 fully activated.

**The tell was two lines that read like startup noise:**

```
[amr1.selective_mapping]: map geometry changed to 19x13 at 0.050 m/cell
[amr2.selective_mapping]: map geometry changed to 13x10 at 0.050 m/cell
```

19 x 13 cells at 0.05 m is **0.95 x 0.65 m**. The heavy mapper's chassis is
0.90 x 0.62 m. 13 x 10 is 0.65 x 0.50 m; the scout's chassis is 0.58 x 0.44 m.
Both maps are the robot's own body, to within a cell.

**Cause.** A Gazebo ray sensor collides with its own model's links. The LiDAR
was mounted at 0.42 m; the chassis collision box runs from `base_z_offset`
(0.13) to `base_z_offset + chassis_height` (0.47), with the cargo deck on top
of that at 0.494. **The scanner was inside the robot.** Every beam terminated
on a chassis wall at 0.31-0.45 m -- far beyond `range_min` (0.12 m), so nothing
filtered them and they arrived as ordinary obstacles.

From there the failure propagates through four components, none of which
mentions the LiDAR:

1. slam_toolbox maps a chassis-sized box and nothing else;
2. the merged map stalls at 0.1% explored;
3. the inflation layer turns that ring of self-hits into
   `INSCRIBED_INFLATED_OBSTACLE` (253) at the robot's own cell;
4. Smac's `Node2D::isNodeValid` rejects any cost >= 253, so the **start pose**
   is invalid -- "Starting point in lethal space". nav2 runs its recoveries,
   they hit the same phantom obstacles ("Collision Ahead - Exiting Spin"), and
   the goal aborts with status 6.

**Fix.** Put the scanner on a mast above the cargo deck: 0.60 m for the heavy
mapper, 0.48 m for the scout, both with ~0.10 m of clearance. The mast and the
scanner housing are **visual-only** -- collision geometry at the beam origin
would be a self-hit waiting for a model whose `range_min` is smaller.

Two things fixed in the same pass, both found by reading that log properly:

* `slam_toolbox.yaml` said `minimum_laser_range`. slam_toolbox's parameter is
  `min_laser_range`. The misspelling was accepted silently and the default 0.0
  applied -- the exact silent-defaulting failure REFACTORING.md section 1 is
  about, sitting in this project's own config. The only evidence was a warning
  that read like advice: *"minimum laser range setting (0.0 m) exceeds the
  capabilities of the used Lidar (0.1 m)"*. Both bounds now come from
  `robot_models.yaml` per robot, so the heavy mapper stops having its 25 m
  scanner truncated to slam_toolbox's 20 m default.
* The camera was flush with the chassis front face, so its near plane began
  inside solid geometry. It now sits 2 cm proud.

**What stops it recurring.** `test_sensor_placement.py` computes the top of
each model's collision geometry from `robot_models.yaml` and requires the scan
plane to clear it, so a new robot model is covered the moment it is added. It
also asserts the reason the clearance is needed -- the nearest chassis wall is
always further out than `range_min`, so a self-hit can never be filtered -- and
that neither the mast nor the scanner housing has collision geometry.
`check_model_consistency.py` performs the same check before Gazebo starts.

**The transferable lesson.** The four components between the cause and the
symptom were each behaving correctly. SLAM mapped what it was told; inflation
inflated what it was given; the planner refused an invalid start. A message
that accurately describes a component's own state can still be useless, because
its *cause* is three components upstream. What actually located this was a
dimension check: 19 x 13 cells is 0.95 x 0.65 m, and something on the robot is
0.90 x 0.62 m. **When a symptom is a number, compare it against the physical
quantities in the system before reading any more logs.**

---

## 9. What I would do next

Honest gaps, roughly in the order I would close them.

1. **Integration tests under `launch_testing`.** The 250 automated tests cover the
   algorithms thoroughly and the node wiring not at all. A `launch_testing`
   suite that brings up two robots headless and asserts on `/cmd_vel` and
   `/fleet/traffic_directives` would cover the seam between them.
2. **The costmap-stack refactoring** in `REFACTORING.md` §3 — layer ordering
   currently carries semantics that exist only as a YAML comment.
3. **Localisation-uncertainty-aware conflict margins.** `conflict_margin` is a
   constant. It should grow with each robot's pose covariance, so a poorly
   localised robot is given more room.
4. **Replace the fixed yield priority with a cost-based auction.** Strict
   priority plus anti-starvation works, but a robot two minutes from a deadline
   should outrank one that is idle. The priority is already an integer in one
   place, so the change is contained.
5. **Sensor fusion for the safety envelope.** It currently trusts one LiDAR. A
   second sensor with independent failure modes is the standard answer, and the
   BSP layer is the natural place to arbitrate between them.
