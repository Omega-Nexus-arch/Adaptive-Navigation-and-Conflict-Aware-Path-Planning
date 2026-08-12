# Current load carried by a robot. Drives the dynamic-state dependent
# acceleration / jerk envelope in the motion smoother.
std_msgs/Header header
string robot_id
float64 payload_kg
float64 capacity_kg
float64 load_ratio           # payload_kg / capacity_kg, clamped to [0, 1]
