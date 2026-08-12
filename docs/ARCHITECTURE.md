# Architecture

Node graph, topics and frames. For *why* things are the way they are, see
[`DESIGN_NOTES.md`](DESIGN_NOTES.md).

---

## Node graph

### Per robot (namespace `/<robot>`)

| Node | Package | Role |
|---|---|---|
| `robot_state_publisher` | `robot_state_publisher` | URDF → TF |
| `bsp_validation` | `amr_sensor_bsp` | Gates every sensor stream |
| `slam_toolbox` | `slam_toolbox` | Per-robot SLAM in `<robot>/map` |
| `selective_mapping` | `amr_mapping` | Filters this robot's map contribution |
| `velocity_smoother` | `amr_fleet_control` | Traffic scaling + accel/jerk limits |
| `safety_override` | `amr_fleet_control` | Sole `cmd_vel` writer |
| `trajectory_broadcaster` | `amr_fleet_control` | Publishes this robot's projection |
| nav2 servers ×6 + lifecycle manager | `nav2_*` | Planning and control |

### Fleet-wide (global namespace)

| Node | Package | Role |
|---|---|---|
| `traffic_control` | `amr_fleet_control` | Conflict detection and yielding |
| `map_fusion` | `amr_mapping` | Merged `/map` + static frame anchors |
| `dynamic_obstacle_driver` | `amr_gazebo` | Pedestrians and third-party robots |

---

## Topics

### Sensor path — the BSP gate

```
Gazebo                          amr_sensor_bsp                nav2 / SLAM / safety
──────                          ──────────────                ────────────────────
<ns>/scan_raw                ──► LidarValidator            ──► <ns>/scan
<ns>/imu_raw                 ──► ImuValidator              ──► <ns>/imu
<ns>/camera/image_unvalidated──► CameraValidator           ──► <ns>/camera/image_raw
                                        │
                                        └────────────────────► <ns>/sensor_health
```

Nothing downstream subscribes to a `*_raw` topic. The validated topics have no
other publisher, so the gate is structural rather than a convention.

### Command path

```
nav2 controller ──► <ns>/cmd_vel_nav
                         │
     <ns>/payload_state  │  /fleet/traffic_directives
                    ╲    │    ╱
                  velocity_smoother
                         │
                    <ns>/cmd_vel_smoothed
                         │
      <ns>/scan ──► safety_override ──► <ns>/safety_status
                         │
                    <ns>/cmd_vel ──► Gazebo diff-drive
```

### Fleet coordination

| Topic | Type | Publisher → Subscriber |
|---|---|---|
| `/fleet/trajectories` | `amr_msgs/PredictedTrajectory` | every `trajectory_broadcaster` → `traffic_control`, every `FleetTrajectoryLayer` |
| `/fleet/traffic_directives` | `amr_msgs/TrafficDirective` | `traffic_control` → every `velocity_smoother` |
| `/fleet/conflict_markers` | `visualization_msgs/MarkerArray` | `traffic_control` → RViz |

One topic each, not one per robot: the graph stays flat as the fleet grows, and
a new robot needs no rewiring.

### Mapping

| Topic | Type | Publisher → Subscriber |
|---|---|---|
| `<ns>/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` → `selective_mapping` |
| `<ns>/map_contribution` | `nav_msgs/OccupancyGrid` | `selective_mapping` → `map_fusion` |
| `<ns>/map_update_stats` | `amr_msgs/MapUpdateStats` | `selective_mapping` → telemetry |
| `/map` | `nav_msgs/OccupancyGrid` | `map_fusion` → every global costmap |

### Services

| Service | Type | Purpose |
|---|---|---|
| `<ns>/set_payload` | `amr_msgs/SetPayload` | Inject a payload change; retunes the smoother live |
| `<ns>/set_safety_override` | `amr_msgs/SetSafetyOverride` | Manual e-stop engage/release |

---

## Frames

```
map                                    ← shared global frame, root of everything
├── amr1/map                           ← static, from amr1's roster start pose
│   └── amr1/odom                      ← slam_toolbox
│       └── amr1/base_footprint        ← Gazebo diff-drive
│           └── amr1/base_link
│               ├── amr1/lidar_link
│               ├── amr1/imu_link
│               ├── amr1/camera_link → amr1/camera_optical_link
│               ├── amr1/{left,right}_wheel_link
│               └── amr1/{front,rear}_caster_link
└── amr2/map  → ... (identical structure)
```

Two decisions worth noting:

**Every link is explicitly prefixed inside the URDF**, rather than relying on
`robot_state_publisher`'s `frame_prefix`. Gazebo plugins write `frame_name`
directly into their messages and do not know about `frame_prefix`, so mixing the
two produces frames that are prefixed in TF and unprefixed in sensor headers.
Explicit prefixing is unambiguous. `frame_prefix` is consequently set to `""`.

**Each robot owns a private `<robot>/map`**, anchored into the shared `map` by a
static transform derived from its roster start pose. This is why two independent
SLAM instances can coexist without either owning the global frame, and why
`map_fusion` is the only node that needs to know about both.

---

## Data flow, end to end

```
    LiDAR ─► BSP ─► scan ─┬─► slam_toolbox ─► selective_mapping ─┐
                          │                                      │
                          ├─► costmaps                           ▼
                          │                                  map_fusion
                          └─► safety_override                     │
                                                                  ▼
    IMU ─► BSP ─► pitch ─► LidarValidator                       /map
                           (ground rejection)                     │
                                                                  ▼
    odom ─┬─► trajectory_broadcaster ─► /fleet/trajectories ─► global costmap
          │                                    │                  │
          │                                    ▼                  ▼
          │                            traffic_control       nav2 planner
          │                                    │                  │
          │                                    ▼                  ▼
          └───────────────► velocity_smoother ◄─── cmd_vel_nav ◄──┘
                                     │
                                     ▼
                            safety_override ─► cmd_vel
```

---

## Regenerating the RViz config for a larger fleet

`rviz/fleet.rviz` is generated by iterating the roster, so it scales with it.
From `src/amr_bringup`:

```python
import sys, yaml, pathlib
sys.path.insert(0, '.')
from amr_bringup.fleet_loader import load_fleet

fleet = load_fleet('config/fleet_ten_robots.yaml')
# ...build one RobotModel / LaserScan / Path display per robot, as in the
# snippet that produced the shipped file.
```

The displays per robot are: `RobotModel`, `LaserScan` on the *validated* `scan`
topic, `Path` on the global plan, and a (disabled by default) local costmap.
Fleet-level displays are the fused `/map`, the slope-cost debug grid, and the
predicted-conflict markers.
