# Telemetry proving the selective-iteration mapping policy is actually
# restricting the map update area (System Optimization criterion).
std_msgs/Header header
string robot_id

bool update_published        # False when the whole update was suppressed.
uint32 cells_considered
uint32 cells_written
uint32 cells_suppressed      # Skipped because the region is well-explored.
uint32 frontier_cells        # Cells adjacent to unknown space (always written).
float64 suppression_ratio    # cells_suppressed / cells_considered
float64 effective_rate_hz    # Achieved publish rate after throttling.
string policy_state          # "frontier_burst" | "throttled" | "idle"
