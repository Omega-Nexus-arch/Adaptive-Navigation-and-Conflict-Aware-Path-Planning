# Refactoring deliverable

> **Brief:** *"identify one area of the standard ROS Navigation or Gazebo launch
> files they integrated and propose a refactoring plan (e.g. splitting a
> monolithic launch file into modular components or converting a complex
> parameter file into a clean C++ struct/enum representation) and implement a
> small part of that refactoring."*

Two areas were identified. Both are implemented, not merely proposed, because
neither could be left half-done without the rest of the system inheriting the
problem. Section 3 is what a fuller version of this work would tackle next.

---

## 1. The parameter files → a typed C++ representation

### The problem

Standard nav2 integration spreads a robot's physical constants across
independently-maintained files. Starting from the stock `nav2_bringup` layout
plus a Gazebo description, AMR-1's maximum acceleration would have appeared in
**five** places:

| Where | As what |
|---|---|
| `nav2_amr1.yaml` → `FollowPath.acc_lim_x` | Controller limit |
| `nav2_amr1.yaml` → `velocity_smoother.max_accel` | Smoother limit |
| `amr1.urdf.xacro` → `max_wheel_acceleration` | Gazebo plugin limit |
| `velocity_smoother_node.cpp` | Hard-coded default |
| `safety_override_node.cpp` | Hard-coded default |

Nothing enforced agreement. Two specific failure modes follow, and both are
quiet:

- **Silent divergence.** Retuning `acc_lim_x` in the nav2 file leaves the
  smoother enforcing the old value. The robot still moves, and the difference
  only shows up as a subtly wrong acceleration profile that nobody attributes
  to the edit that caused it.
- **Silent defaulting.** A mistyped key in a nav2 YAML is not an error. The
  parameter simply keeps its declared default, and a mistyped
  `max_traversible_angle_degrees` leaves the slope layer running on 16° while
  the operator believes they set 10°.

Duplication across a second robot doubles it, and the ten-robot roster would
have made it ten nav2 files.

### Before

```yaml
# nav2_amr1.yaml  (and a near-identical nav2_amr2.yaml)
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.75          # ...also in amr1.urdf.xacro
      acc_lim_x: 0.35          # ...also in the smoother's C++ defaults
      decel_lim_x: -0.7        # ...sign convention differs from the xacro
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.55       # ...also in the xacro's collision geometry
```

```cpp
// velocity_smoother_node.cpp — the numbers, again
declare_parameter("max_accel_x", 0.35);
declare_parameter("max_jerk_x", 0.60);
declare_parameter("payload_capacity", 120.0);
// ...twelve more, each a fresh opportunity to disagree with the YAML
double accel = get_parameter("max_accel_x").as_double();
```

### After

One YAML block per *model*, and one typed C++ representation of it.

```yaml
# amr_description/config/robot_models.yaml — the single source of truth
heavy_mapper:
  role: "mapper"
  payload_capacity_kg: 120.0
  max_vel_x: 0.75
  max_accel_x: 0.35
  max_decel_x: 0.70
  max_jerk_x: 0.60
  payload_derating: 0.55
  safety_k: 0.85
  safety_d_min: 0.45
  imu: {max_angular_velocity: 2.50, ...}
```

```cpp
// amr_core/include/amr_core/robot_model.hpp
enum class RobotRole : std::uint8_t { kMapper, kScout, kUnknown };

struct DynamicLimits {
  double max_accel_x = 0.8;
  double max_jerk_x = 1.5;
  double payload_derating = 0.3;
  double speed_derating = 0.2;

  double EffectiveAccelX(double load_ratio, double speed_ratio) const;
  double EffectiveDecelX(double load_ratio, double speed_ratio) const;
  double EffectiveJerkX(double load_ratio, double speed_ratio) const;
};

struct SafetySpec {
  double k = 0.5;
  double d_min = 0.35;
  double SafeDistance(double speed) const { return k * speed * speed + d_min; }
  double ReleaseDistance(double speed) const;
};

struct RobotProfile {
  std::string model_name;
  RobotRole role = RobotRole::kUnknown;
  DynamicLimits limits;
  SafetySpec safety;
  LidarSpec lidar;
  ImuSpec imu;
  CameraSpec camera;
  int yield_priority = 0;
};
```

Consumption at every site becomes one line:

```cpp
const amr_core::FleetConfig fleet = amr_core::FleetConfig::FromFile(fleet_config);
robot_ = fleet.Robot(robot_name);
smoother_ = std::make_unique<MotionSmoother>(robot_.profile);
```

And the xacro reads the same file:

```xml
<xacro:property name="model_library" value="${xacro.load_yaml(model_config_path)}"/>
<xacro:property name="M" value="${model_library[requested_model]}"/>
```

while the launch files derive nav2's parameters from it rather than restating
them (`amr_bringup/fleet_loader.py::nav2_substitutions`).

### What it bought

| | Before | After |
|---|---|---|
| Places AMR-1's acceleration is written | 5 | 1 |
| nav2 parameter files for a 10-robot fleet | 10 | 1 template |
| A mistyped key | Silently defaults | `ConfigError` at startup, naming key and model |
| A structurally impossible profile | Runs, misbehaves later | Rejected at startup |
| Testable without ROS | No | Yes — 27 tests in `amr_core` |

The validation is the part that matters most in practice. `ModelLibrary` does
not just parse; it rejects configurations that cannot work:

```cpp
// A safety envelope the robot's own LiDAR cannot see is not a safety envelope.
const double worst_case_trigger = safety.SafeDistance(limits.max_vel_x);
if (worst_case_trigger > profile.lidar.range_max) {
  throw ConfigError(
    where + ": safety envelope needs " + std::to_string(worst_case_trigger) +
    " m at top speed but the LiDAR only reaches " +
    std::to_string(profile.lidar.range_max) + " m");
}
```

Equally, `FleetConfig` refuses two robots with the same yield priority, because
that would make the yielding protocol depend on message arrival order — an
intermittent deadlock waiting for a busy day. Both rejections are tested
(`ModelLibraryTest.RejectsASafetyEnvelopeTheLidarCannotSee`,
`FleetConfigTest.RejectsDuplicatePriorities`).

The enum deserves a note. `RobotRole` degrades a typo to `kUnknown` rather than
guessing:

```cpp
RobotRole RoleFromString(const std::string & text) {
  if (text == "mapper") return RobotRole::kMapper;
  if (text == "scout") return RobotRole::kScout;
  // "Mapper" is a typo, not a mapper. Silently promoting it is how a scout
  // ends up with a mapper's priority.
  return RobotRole::kUnknown;
}
```

---

## 2. The monolithic launch file → composable units

### The problem

The idiomatic starting point — one `bringup.launch.py` that starts Gazebo,
spawns both robots, and includes `nav2_bringup` twice — has three concrete
costs:

1. **Nothing can be run alone.** Tuning the world means starting two nav2
   stacks. Debugging a costmap layer means starting Gazebo.
2. **Robot count is structural.** Adding a robot means editing launch *code*,
   which is where the "expand to ten robots" requirement dies.
3. **Hardware bringup is a fork.** The simulation is inseparable from the rest,
   so a real-robot launch file becomes a divergent copy.

### Before

```python
def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(gzserver...),
        IncludeLaunchDescription(gzclient...),
        # AMR-1
        Node(package='robot_state_publisher', namespace='amr1', ...),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'amr1', '-x', '-18.5', ...]),
        Node(package='slam_toolbox', namespace='amr1', parameters=[amr1_slam]),
        IncludeLaunchDescription(nav2_bringup, launch_arguments={
            'namespace': 'amr1', 'params_file': nav2_amr1_yaml}.items()),
        # AMR-2 — the same fifteen lines again, with '1' replaced by '2'
        Node(package='robot_state_publisher', namespace='amr2', ...),
        ...
    ])
```

### After

Five files, each owning one concern:

```
fleet.launch.py                 top level; iterates the roster
  ├── simulation.launch.py      Gazebo, world, dynamic obstacles
  ├── robot.launch.py    × N    one robot's entire stack
  │     ├── slam.launch.py      slam_toolbox + selective mapping
  │     └── navigation.launch.py nav2, parameterised from the model library
  └── fleet_control.launch.py   traffic controller, map fusion
```

The robot loop contains no robot names:

```python
for index, robot in enumerate(fleet['robots']):
    actions.append(TimerAction(
        period=spawn_delay * index,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                'robot_name': robot['name'],
                'fleet_config': fleet_config,
            }.items())]))
```

Composability that this buys immediately:

```bash
ros2 launch amr_bringup simulation.launch.py                  # world only
ros2 launch amr_bringup robot.launch.py robot_name:=amr1      # one robot
ros2 launch amr_bringup robot.launch.py robot_name:=amr1 navigation:=false
ros2 launch amr_bringup fleet.launch.py simulation:=false     # real hardware
ros2 launch amr_bringup fleet.launch.py fleet_config:=.../fleet_ten_robots.yaml
```

Two details in the "after" that came from the problems the "before" produces.

**The stagger.** `TimerAction(period=spawn_delay * index, ...)` exists because
spawning several models into Gazebo simultaneously reliably wedges the spawn
service, and bringing up N nav2 stacks at once trips the lifecycle bond
timeouts. It costs a few seconds and removes a class of "works on my machine"
startup failures. `spawn_delay` is exposed so a slow machine can raise it.

**The `cmd_vel` remap.** `navigation.launch.py` remaps the controller's output
to `cmd_vel_nav`:

```python
Node(package='nav2_controller', executable='controller_server',
     remappings=[('cmd_vel', 'cmd_vel_nav')])
```

This is what inserts the velocity smoother and the safety override into the
command path. Without it, nav2 writes the wheels directly and the override
becomes decorative. It is one line, and it is the single most safety-relevant
line in the launch tree — which is why it carries a comment saying so.

---

## 3. Proposed next: the costmap layer stack

Not implemented; this is the plan the brief asks for.

**The problem.** `nav2_params.yaml` still repeats the layer stack four times —
local and global costmaps, each duplicated in intent across robot models — and
the layer *ordering* carries semantics that only exist as a YAML comment:

```yaml
plugins: ["static_layer", "slope_layer", "obstacle_layer", "inflation_layer"]
# `slope` sits between the map and the inflation, so terrain cost is inflated
# like any other cost and the planner keeps clear of ramp edges.
```

Reorder those strings and the system still starts, still runs, and quietly
plans worse paths. That is exactly the property the section-1 refactoring
removed from the physical parameters, and it remains here.

**The proposal.** Give the layer stack the same treatment: a typed
representation with the ordering constraints expressed as code.

```cpp
enum class CostmapLayerKind : std::uint8_t {
  kStatic, kSlope, kObstacle, kFleetTrajectory, kInflation
};

struct LayerSpec {
  CostmapLayerKind kind;
  bool enabled;
  std::map<std::string, rclcpp::ParameterValue> parameters;
};

class CostmapStack {
 public:
  // Rejects a stack where inflation is not last, or where slope precedes
  // static, and explains which invariant was violated.
  static CostmapStack Validate(std::vector<LayerSpec> layers);
  std::vector<std::string> ToPluginNames() const;
};
```

**Scope of a first increment.** Add `CostmapStack::Validate` and call it from a
launch-time check in `check_model_consistency.py`, so a mis-ordered stack fails
before Gazebo starts. That is roughly a day, needs no change to any layer, and
converts a silent misconfiguration into a startup error — the same trade as
section 1, applied to the one place it has not been made yet.

**Why it was not done now.** It touches nav2's plugin loading rather than this
project's own code, and the payoff is smaller than the two refactorings above:
the layer stack is edited far less often than a robot's physical limits. Given
finite time, the duplication that actually bites went first.
