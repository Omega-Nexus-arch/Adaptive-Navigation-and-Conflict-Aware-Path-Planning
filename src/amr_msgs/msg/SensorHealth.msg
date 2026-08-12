# Output of the Board Support Package (BSP) validation routine for one stream.
std_msgs/Header header
string robot_id
string sensor_name          # "lidar" | "imu" | "camera"

uint8 STATUS_OK       = 0   # Passed every check; forwarded downstream.
uint8 STATUS_DEGRADED = 1   # Forwarded, but at least one soft check failed.
uint8 STATUS_INVALID  = 2   # Dropped; never reaches the navigation stack.
uint8 status

uint64 messages_received
uint64 messages_rejected
uint64 messages_degraded

# Measured publish rate of the raw stream [Hz].
float64 measured_rate_hz

# Names of the checks that failed on the most recent message.
string[] active_faults
