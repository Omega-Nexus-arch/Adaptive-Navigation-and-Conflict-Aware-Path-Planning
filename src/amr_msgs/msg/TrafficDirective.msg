# Command issued by the central traffic controller to one robot.
std_msgs/Header header
string robot_id

# --- action enumeration -----------------------------------------------------
uint8 ACTION_PROCEED = 0   # No conflict: track the navigation velocity as-is.
uint8 ACTION_SLOW    = 1   # Soft conflict: scale the target velocity down.
uint8 ACTION_YIELD   = 2   # Hard conflict: controlled stop until released.
uint8 ACTION_HOLD    = 3   # Administrative hold (e.g. deadlock resolution).
uint8 action

# Multiplier in [0, 1] applied to the navigation velocity BEFORE smoothing, so
# a yield is executed as a jerk-limited controlled stop rather than a hard cut.
float64 speed_scale

# Robot this directive is protecting against ("" when ACTION_PROCEED).
string conflicting_robot

# Predicted seconds until the conflict would occur. NaN when not applicable.
float64 time_to_conflict

# Human-readable justification, surfaced in logs and RViz.
string reason
