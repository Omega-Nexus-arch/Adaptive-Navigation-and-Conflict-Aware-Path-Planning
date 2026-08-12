# A single space-time sample of a robot's projected motion.
geometry_msgs/Pose pose

# Offset from PredictedTrajectory.header.stamp at which the robot is expected
# to occupy `pose`. Seconds.
float64 time_from_start

# Body-frame forward speed expected at this sample [m/s]. Used by the conflict
# detector to size the temporal uncertainty window.
float64 speed
