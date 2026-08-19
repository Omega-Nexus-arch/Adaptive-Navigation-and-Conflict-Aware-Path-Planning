# Reading the rqt_graph — every node and every topic chain

A complete walkthrough of the running system as `rqt_graph` shows it.

---

## 0. How to read the picture

| Shape | Meaning |
|---|---|
| **Ellipse / rounded** | a **node** (a process, or a plugin hosted inside `gzserver`) |
| **Rectangle** | a **topic** |
| **Large labelled box** | a **namespace** grouping (`/amr1`, `/amr2`, `/obstacles`, `Root`) |
| **Arrow into a rectangle** | that node **publishes** the topic |
| **Arrow out of a rectangle** | that node **subscribes** to the topic |

The graph has **five regions**:

1. **`/gazebo`** (top left) — the simulator itself
2. **`/obstacles`** (top left) — the moving third-party robot
3. **`Root`** (centre) — the two fleet-wide singletons
4. **`/amr1`** (large upper box) — everything for robot 1
5. **`/amr2`** (large lower box) — **byte-for-byte the same graph** as `/amr1`

The `/amr1` and `/amr2` boxes are structurally identical. That symmetry is the
visual proof of the scalability requirement: adding a robot adds another
identical box and changes nothing else.

---

## 1. Region: `/gazebo`

| Node | What it is |
|---|---|
| `/gazebo` | `gzserver`. Runs physics at 500 Hz, hosts every Gazebo plugin, publishes `/clock`. |
| `/gazebo/gazebo_ros_state` | The `gazebo_ros_state` plugin. Exposes model poses and accepts `SetEntityState` calls — how the obstacle driver teleports/commands the moving obstacles. |

**Important:** `gzserver` is a single process, but it *hosts* many ROS nodes —
every `<plugin>` in the URDF becomes a node inside it. That is why
`/amr1/amr1_diff_drive`, `/amr1/amr1_lidar_plugin`, etc. appear inside the
`/amr1` namespace box even though no separate process exists for them.

---

## 2. Region: `/obstacles`

| Node / topic | Role |
|---|---|
| `dynamic_obstacle_driver` (node) | Python node from `amr_gazebo`. Drives the **rigid** third-party obstacle around its seeded patrol loop. |
| `/obstacles/thirdparty_1/cmd_vel` (topic) | Velocity command the driver publishes for that obstacle. |
| `/obstacles/thirdparty_1/thirdparty_1_move` (node) | The obstacle's own Gazebo diff-drive plugin, hosted in `gzserver`. Consumes `cmd_vel`. |
| `/obstacles/thirdparty_1/odom` (topic) | Odometry the obstacle publishes back. The driver reads it as closed-loop feedback for its waypoint following. |

**Chain:**
```
dynamic_obstacle_driver ──► /obstacles/thirdparty_1/cmd_vel ──► thirdparty_1_move (Gazebo)
                        ◄── /obstacles/thirdparty_1/odom    ◄──
```

**Why only ONE obstacle appears.** The four pedestrians are Gazebo `<actor>`
elements with scripted trajectories baked into the world SDF. Actors are
kinematic and are animated by Gazebo itself — they have no ROS interface at all,
so they cannot appear in an rqt_graph. Only the rigid box obstacle is
ROS-driven. This is worth saying out loud: *the absence of the pedestrians from
this graph is expected, not a bug.*

---

## 3. Region: `Root` — the fleet singletons

These deliberately live in the **root namespace**, not inside a robot, because
they are fleet-wide services. There is exactly one of each no matter how many
robots run.

### 3.1 `/map_fusion`

| | |
|---|---|
| Package | `amr_mapping` |
| Subscribes | `/amr1/map_contribution`, `/amr2/map_contribution` |
| Publishes | **`/map`**, and static transforms on `/tf_static` |

Fuses both robots' partial occupancy grids into one warehouse-wide map using a
**log-odds accumulator** (occupied +0.65, free −0.45, clamped ±3.5). It also
publishes the static transforms `map → amr1/map` and `map → amr2/map` derived
from each robot's known dock pose — which is what gives the whole fleet a single
TF root.

In the graph you can see `/map_fusion` sitting on the far left with edges going
to `/map` and a thick bundle into `/tf`.

### 3.2 `/traffic_control`

| | |
|---|---|
| Package | `amr_fleet_control` |
| Subscribes | `/fleet/trajectories` |
| Publishes | `/fleet/traffic_directives`, `/fleet/conflict_markers` |

Runs `ConflictDetector` (space-time, all pairs) at 10 Hz and applies
`YieldPolicy`. Publishes one `TrafficDirective` per robot per cycle — including
an explicit `PROCEED` for robots with no conflict, so silence can be treated as
a fault.

### 3.3 The two `/fleet/*` topics

| Topic | Type | Direction |
|---|---|---|
| `/fleet/trajectories` | `amr_msgs/PredictedTrajectory` | **many → one**: both `trajectory_broadcaster`s publish, `traffic_control` subscribes |
| `/fleet/traffic_directives` | `amr_msgs/TrafficDirective` | **one → many**: `traffic_control` publishes, both `velocity_smoother`s subscribe |

**These are the only two topics that cross the robot namespace boundary**, and
that is the entire cooperative-navigation interface. In the graph they are the
edges that leave one big box and enter the other.

> **Design note worth volunteering:** `/fleet/traffic_directives` is a *single
> shared* topic, not one per robot. Every smoother receives every directive and
> filters by `robot_id`. That means a new robot needs no new topic and no
> remapping — it just starts listening. The cost is a small amount of wasted
> bandwidth, which is the right trade at fleet scale.

Also visible: `/amr1/map_contribution` and `/amr2/map_contribution` crossing
from each robot box to `/map_fusion`, and `/map` fanning back into both boxes.

---

## 4. Region: `/amr1` (identical for `/amr2`)

Walking the box left to right, following the data.

### 4.1 Sensor plugins — where data enters ROS

| Node | Hosted in | Publishes |
|---|---|---|
| `/amr1/amr1_lidar_plugin` | gzserver | `/amr1/scan_raw` |
| `/amr1/amr1_imu_plugin` | gzserver | `/amr1/imu_raw` |
| `/amr1/amr1_camera_plugin` | gzserver | `/amr1/camera_raw/image_raw` (+ `camera_info`) |
| `/amr1/amr1_joint_states` | gzserver | `/amr1/joint_states` |
| `/amr1/amr1_diff_drive` | gzserver | `/amr1/odom`, TF `odom → base_footprint` |

Note the `_raw` suffix on all three sensor topics. **That naming is load-bearing**
— it is what makes it visually obvious in this graph that nothing except the BSP
node consumes them.

### 4.2 The BSP validation gate

```
/amr1/scan_raw            ┐
/amr1/imu_raw             ├─► /amr1/bsp_validation ─┬─► /amr1/scan
/amr1/camera_raw/image_raw┘                          ├─► /amr1/imu
                                                     ├─► /amr1/camera/image_raw
                                                     └─► /amr1/sensor_health
```

`/amr1/bsp_validation` (package `amr_sensor_bsp`) is the **only** subscriber of
the three `_raw` topics. It validates each message — geometry, beam count,
finiteness, staleness, IMU plausibility, camera intensity — and republishes the
validated version on the unsuffixed name.

**This is the single most important structural fact in the graph.** Trace any
arrow out of `scan_raw`: it goes to exactly one place. Every downstream
consumer — SLAM, both costmaps, the safety monitor — reads `/amr1/scan`, the
validated topic. Requirement §4.1 is satisfied *by topology*, not by convention.

### 4.3 SLAM and the mapping chain

```
/amr1/scan ─► /amr1/slam_toolbox ─┬─► /amr1/map ─► /amr1/selective_mapping
                                  └─► TF  amr1/map → amr1/odom
                                                          │
                                        /amr1/map_contribution
                                                          │
                                                          ▼
                                                    /map_fusion ─► /map
```

| Node | Role |
|---|---|
| `/amr1/slam_toolbox` | Stock `async_slam_toolbox_node` in mapping mode. Builds this robot's private grid on `/amr1/map` and publishes the `amr1/map → amr1/odom` correction transform. |
| `/amr1/selective_mapping` | `amr_mapping`. Subscribes to the robot's own SLAM grid and decides **which cells to forward**. Frontier cells always pass; explored cells throttle to 1 Hz; saturated cells to 0.2 Hz. Publishes the filtered grid as `/amr1/map_contribution` and statistics on `/amr1/map_update_stats`. |

**The chain answers requirement §2.2/§2.3 directly:** `slam_toolbox` produces
everything, `selective_mapping` is the throttle, `map_fusion` is the merge.

### 4.4 The costmaps

Two sub-boxes appear inside `/amr1`, because nav2 runs each costmap as a
composed node with its own sub-namespace.

**`/amr1/local_costmap`**

| Topic | Meaning |
|---|---|
| `/amr1/local_costmap/costmap` | The 8 × 8 m rolling grid, published at 5 Hz for visualisation |
| `/amr1/local_costmap/costmap_raw` | Raw cost values, consumed by the controller |
| `/amr1/local_costmap/costmap_updates` | Incremental updates rather than full grids |
| `/amr1/local_costmap/published_footprint` | The robot polygon, so RViz and the traffic layer know its extent |

Layers: `obstacle_layer` (from `/amr1/scan`), **`fleet_layer`** (from
`/fleet/trajectories`), `inflation_layer`. Frame `amr1/odom`, rolling.

**`/amr1/global_costmap`**

Same four topics. Layers: `static_layer` (from **`/map`**, the fused grid),
**`slope_layer`** (from the elevation map on disk — *no topic, which is why it
has no incoming edge in this graph*), `obstacle_layer`, `inflation_layer`.
Frame `map`, 48 × 34 m, static.

> **Interview point:** the slope layer reads a PGM/YAML elevation map from the
> filesystem at activation, not from a topic. So one of the project's headline
> features is deliberately *invisible* in rqt_graph. Good thing to point out —
> it shows you understand the graph shows message flow, not all data flow.

### 4.5 The nav2 servers

| Node | Role |
|---|---|
| `/amr1/planner_server` | Global planning. Hosts `SmacPlanner2D`. Serves the `compute_path_to_pose` and `compute_path_through_poses` actions. Owns `global_costmap`. |
| `/amr1/controller_server` | Local control. Hosts DWB at 20 Hz. Serves `follow_path`. Owns `local_costmap`. Publishes **`/amr1/cmd_vel_nav`**. |
| `/amr1/smoother_server` | nav2's *path* smoother (geometric — do not confuse with our velocity smoother). Serves `smooth_path`. |
| `/amr1/behavior_server` | Recovery behaviours: `spin`, `backup`, `drive_on_heading`, `wait`. Also publishes `cmd_vel_nav` while a recovery runs. |
| `/amr1/bt_navigator` | Runs the behaviour tree. Serves `navigate_to_pose` and `navigate_through_poses`, and *calls* the action servers above. |
| `/amr1/waypoint_follower` | Sequences multiple goals through `navigate_to_pose`. |
| `/amr1/lifecycle_manager_navigation` | Configures and activates all of the above in order, and holds a bond with each. If a server dies, the bond breaks and the manager tears the stack down rather than leaving it half-alive. |

**Why `bt_navigator` has two extra client nodes.** On the far right you see
`/amr1/bt_navigator_navigate_to_pose_rclcpp_node` and
`/amr1/bt_navigator_navigate_through_poses_rclcpp_node`. These are **internal
client nodes** that nav2 creates so the behaviour tree can call action servers
without blocking its own executor. They are not something we wrote and not
something misconfigured — they are normal nav2 internals, and every action
edge on the right of the graph terminates in one of them.

### 4.6 The action topic clusters

Each of the eight boxes on the right-hand side is **one action**, which in ROS 2
is really five topics:

```
/amr1/<action>/_action/send_goal        (service)
/amr1/<action>/_action/cancel_goal      (service)
/amr1/<action>/_action/get_result       (service)
/amr1/<action>/_action/feedback         (topic)
/amr1/<action>/_action/status           (topic)
```

rqt_graph only draws the two **topics** — which is why every action box shows a
`feedback` and a `status` rectangle and nothing else.

| Action | Server | Called by |
|---|---|---|
| `navigate_to_pose` | `bt_navigator` | `send_goals.py`, `waypoint_follower`, RViz |
| `navigate_through_poses` | `bt_navigator` | external |
| `compute_path_to_pose` | `planner_server` | `bt_navigator` |
| `compute_path_through_poses` | `planner_server` | `bt_navigator` |
| `follow_path` | `controller_server` | `bt_navigator` |
| `spin` | `behavior_server` | `bt_navigator` (recovery) |
| `backup` | `behavior_server` | `bt_navigator` (recovery) |
| `drive_on_heading` | `behavior_server` | `bt_navigator` (recovery) |
| `wait` | `behavior_server` | `bt_navigator` (recovery) |

### 4.7 The velocity chain — the spine of the whole system

This is the chain to trace first in an interview, left to right:

```
/amr1/controller_server ──┐
                          ├─► /amr1/cmd_vel_nav ─► /amr1/velocity_smoother
/amr1/behavior_server  ───┘                              ▲
                                                         │
                          /fleet/traffic_directives ─────┘
                                                         │
                                                         ▼
                                            /amr1/cmd_vel_smoothed
                                                         │
                                                         ▼
                                             /amr1/safety_override
                                                         │  ▲
                                       /amr1/scan ───────┘  │
                                       /amr1/odom ──────────┘
                                                         │
                                                         ▼
                                                 /amr1/cmd_vel
                                                         │
                                                         ▼
                                          /amr1/amr1_diff_drive (Gazebo)
                                                         │
                                                         ▼
                                                    /amr1/odom
```

| Stage | Node | What it does |
|---|---|---|
| 1 | `controller_server` / `behavior_server` | Produce a *desired* velocity on `cmd_vel_nav` |
| 2 | `velocity_smoother` | Applies acceleration + **jerk** limits, derated by payload and speed, and scales the target by the traffic directive. Publishes `cmd_vel_smoothed`. |
| 3 | `safety_override` | Compares `d_safe = k·v² + d_min` against the nearest return in the ±70° forward cone of `/amr1/scan`. Either passes the command through or replaces it with zero. Publishes **`cmd_vel`**. |
| 4 | `amr1_diff_drive` | Gazebo plugin. Converts `cmd_vel` into wheel velocities, integrates encoder odometry, publishes `odom` and TF `odom → base_footprint`. |

**Three things to notice in the graph itself:**

1. **`/amr1/cmd_vel` has exactly one publisher.** Follow the arrows into that
   rectangle — only `safety_override` writes it. That single-writer topology is
   what makes the override authoritative rather than advisory, and it is
   *visible in the picture*.
2. **`odom` is consumed in three places** — the safety monitor (for the speed
   term in `d_safe`), the trajectory broadcaster (to seed its prediction), and
   nav2. One publisher, several readers.
3. **The traffic directive enters at stage 2, not stage 3.** A yield is
   cooperative and gets shaped by the smoother; a safety halt is unconditional
   and bypasses it. Two different mechanisms on purpose.

### 4.8 The trajectory broadcaster

```
/amr1/odom ─┐
            ├─► /amr1/trajectory_broadcaster ─► /fleet/trajectories
/amr1/plan ─┘
```

Takes the robot's current odometry and its active nav2 plan, projects **4 s
ahead sampled every 0.2 s**, and publishes a `PredictedTrajectory` at 10 Hz
carrying the robot's id, footprint radius and yield priority.

Consumed by two different things, which is the crux of requirement §3.4/§3.5:

- `/traffic_control` — for **arbitration** (should someone yield?)
- the *other* robot's `fleet_layer` inside its local costmap — for **cost**
  (steer around where the peer will be)

### 4.9 Robot state publisher and joint states

```
/amr1/amr1_joint_states (Gazebo) ─► /amr1/joint_states ─► /amr1/robot_state_publisher ─► /tf
```

`robot_state_publisher` reads the URDF and the wheel joint angles and publishes
every link transform — `base_link → lidar_link`, `→ imu_link`, `→ camera_link`,
the wheels, the casters. Without it, nav2 could not transform a scan from
`lidar_link` into `base_footprint`.

### 4.10 The `transform_listener_impl_*` swarm

The dozen ellipses named `/amr1/transform_listener_impl_55a4f0e2b1c0` and
similar are **not** something we wrote. Every C++ node that constructs a
`tf2_ros::TransformListener` gets a hidden internal node to run its own TF
subscription callback group. The hex suffix is the object's memory address.

There is one per TF-consuming node: both costmaps, the controller, the planner,
the behaviour server, the BT navigator, the safety override, the trajectory
broadcaster, the selective mapper. They all subscribe to `/tf` and `/tf_static`
and publish nothing.

**This is why the left side of the graph is a hairball.** It is normal, it is
not a leak, and being able to say so immediately is a good signal.

---

## 5. `/tf` and `/tf_static` — the two busiest topics

Everything with a pose subscribes to `/tf`. Publishers:

| Publisher | Transform |
|---|---|
| `/map_fusion` | `map → amr1/map`, `map → amr2/map` (**static**) |
| `/amr1/slam_toolbox` | `amr1/map → amr1/odom` (the SLAM correction) |
| `/amr1/amr1_diff_drive` | `amr1/odom → amr1/base_footprint` (odometry) |
| `/amr1/robot_state_publisher` | `base_footprint → base_link → every sensor and wheel` |

Resulting chain:

```
map → amr1/map → amr1/odom → amr1/base_footprint → amr1/base_link → amr1/lidar_link
```

**Each link comes from a different node, and each can fail independently.** That
is exactly what produced the `"Could not find a connection between 'map' and
'amr1/base_footprint' — Tf has two or more unconnected trees"` error during
development: `slam_toolbox` had crashed, so the middle link was missing while
both ends were present.

---

## 6. Every chain, end to end

**Perception**
```
Gazebo → scan_raw → bsp_validation → scan → {slam_toolbox, both costmaps, safety_override}
```

**Mapping**
```
scan → slam_toolbox → amr1/map → selective_mapping → amr1/map_contribution
                                                   ↘
                              amr2/map_contribution → map_fusion → /map → global_costmap (static layer)
```

**Global planning**
```
navigate_to_pose → bt_navigator → compute_path_to_pose → planner_server
                                   (global_costmap: static + slope + obstacle + inflation)
                                → /amr1/plan
```

**Local control**
```
/amr1/plan → bt_navigator → follow_path → controller_server
             (local_costmap: obstacle + fleet + inflation)
           → cmd_vel_nav
```

**Arbitration and actuation**
```
cmd_vel_nav ─► velocity_smoother ─► cmd_vel_smoothed ─► safety_override ─► cmd_vel ─► diff_drive
                      ▲                                       ▲
        /fleet/traffic_directives                        /amr1/scan
```

**Cooperation**
```
odom + plan → trajectory_broadcaster → /fleet/trajectories ─┬─► traffic_control → /fleet/traffic_directives
                                                            └─► peer's fleet_layer (local costmap cost)
```

**Recovery**
```
bt_navigator → {spin | backup | drive_on_heading | wait} → behavior_server → cmd_vel_nav
```

---

## 7. What is deliberately NOT in this graph

Being able to list these is as strong as explaining what is there.

| Missing | Why |
|---|---|
| The four **pedestrians** | Gazebo `<actor>` elements — kinematic, scripted in the world SDF, no ROS interface |
| The **slope layer's** data source | Reads a PGM/YAML elevation map from disk at activation, not from a topic |
| **Services** — `set_payload`, `set_safety_override`, `/gazebo/set_entity_state` | rqt_graph draws topics and actions, not services |
| **Parameters** | e.g. the entire `robot_models.yaml` derivation happens at launch, before any node starts |
| **The action `send_goal`/`cancel`/`get_result`** legs | Those are services; only `feedback` and `status` are topics |
| `/clock` | Usually filtered out by rqt_graph's default settings, but every node subscribes to it under `use_sim_time: True` |

---

## 8. The four sentences to say if you are shown this graph

1. *"The two large boxes are structurally identical — that symmetry is the
   scalability requirement made visible; a third robot is another identical box
   and nothing else changes."*
2. *"Everything enters through `bsp_validation`. The `_raw` topics have exactly
   one subscriber each, so no unvalidated data can reach a planner even by
   accident — that's requirement 4.1 satisfied by topology rather than by
   convention."*
3. *"`cmd_vel` has exactly one publisher, the safety override. That
   single-writer arrangement is what makes the override authoritative; anything
   upstream is only ever publishing a proposal."*
4. *"The only two edges that cross between robots are `/fleet/trajectories` and
   `/fleet/traffic_directives`. That is the entire cooperative interface —
   prediction in, arbitration out."*
