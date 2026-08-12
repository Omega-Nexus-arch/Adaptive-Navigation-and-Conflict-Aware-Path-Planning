# Aggregated view of every robot's projection, republished by the traffic
# controller so a late-joining node gets the whole fleet in one message.
std_msgs/Header header
PredictedTrajectory[] trajectories
