# Forward projection of a single robot's motion, published by every robot so
# that peers can plan around it (conflict-aware local planning, MAPF element).
std_msgs/Header header

# Fleet-unique robot identifier, e.g. "amr1". Matches the ROS namespace.
string robot_id

# Circumscribed radius of the robot footprint [m]. Conflict tests inflate by
# the sum of the two participating radii.
float64 footprint_radius

# Scheduling priority. HIGHER value wins a conflict; the lower-priority robot
# yields. Sourced from the fleet configuration so the policy stays declarative.
int32 yield_priority

# Chronologically ordered samples, poses expressed in `header.frame_id`
# (always the shared global map frame).
TrajectoryPoint[] points
