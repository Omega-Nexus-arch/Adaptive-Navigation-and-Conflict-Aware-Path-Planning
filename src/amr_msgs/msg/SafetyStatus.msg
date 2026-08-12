# Telemetry from the low-level safety override node. Published at the node's
# full control rate so the halt latency is observable in a rosbag.
std_msgs/Header header
string robot_id

# True while the override is actively suppressing the navigation command.
bool halt_active

# Closest validated obstacle range in the guarded sector [m].
float64 min_obstacle_distance

# Speed-dependent trigger distance d_safe = k * v^2 + d_min [m].
float64 safe_distance

# Speed used to evaluate d_safe [m/s].
float64 current_speed

# Wall-clock microseconds from sensing the violation to publishing the halt.
uint32 reaction_latency_us

uint8 REASON_NONE            = 0
uint8 REASON_OBSTACLE        = 1
uint8 REASON_SENSOR_INVALID  = 2
uint8 REASON_SENSOR_TIMEOUT  = 3
uint8 REASON_MANUAL_OVERRIDE = 4
uint8 reason
