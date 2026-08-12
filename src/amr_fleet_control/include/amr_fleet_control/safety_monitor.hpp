// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_FLEET_CONTROL__SAFETY_MONITOR_HPP_
#define AMR_FLEET_CONTROL__SAFETY_MONITOR_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "amr_core/robot_model.hpp"

namespace amr_fleet_control
{

/// \brief Why the override is (or is not) engaged.
///
/// Values match `amr_msgs/SafetyStatus` so the node can forward them without a
/// translation table that could drift.
enum class HaltReason : std::uint8_t
{
  kNone = 0,
  kObstacle = 1,
  kSensorInvalid = 2,   ///< The BSP layer rejected the scan.
  kSensorTimeout = 3,   ///< No scan arrived within the watchdog window.
  kManualOverride = 4   ///< Engaged over the service interface.
};

const char * HaltReasonToString(HaltReason reason);

/// \brief One evaluation of the safety envelope.
struct SafetyDecision
{
  bool halt = false;
  HaltReason reason = HaltReason::kNone;
  double min_obstacle_distance = 0.0;
  double safe_distance = 0.0;      ///< d_safe = k*v^2 + d_min at the current speed.
  double release_distance = 0.0;   ///< d_safe + hysteresis.
  double speed = 0.0;
  /// True only on the tick the halt engages. The node uses it to emit a single
  /// high-visibility log line instead of one per control cycle.
  bool newly_engaged = false;
  bool newly_released = false;
};

/// \brief A single LiDAR return, reduced to what the monitor needs.
struct RangeSample
{
  double angle = 0.0;   ///< Bearing in the robot's body frame [rad].
  double range = 0.0;   ///< Distance [m].
};

/// \brief Low-level, speed-dependent obstacle guard.
///
/// ### What it enforces
///
/// The robot must never be closer to an obstacle than
///
/// ```
///   d_safe(v) = k * v^2 + d_min
/// ```
///
/// where the quadratic term is the kinematic stopping distance (so k is about
/// 1/(2a) for the model's braking authority) and `d_min` is the standstill
/// buffer. When the constraint is violated the monitor demands an immediate
/// halt, which the node applies by *overriding* whatever the navigation stack
/// most recently commanded rather than by asking it to slow down.
///
/// ### Why the halt latches
///
/// Releasing the instant the measured range crosses back over `d_safe` would
/// chatter: the halt reduces speed, a lower speed shrinks `d_safe`, the
/// constraint is satisfied, the robot accelerates, and the cycle repeats at
/// the loop rate. Release therefore requires `d_safe + hysteresis` *and* a
/// minimum hold time.
///
/// ### Fail-safe posture
///
/// Absent or rejected sensor data halts the robot. A monitor that assumes the
/// world is clear when it cannot see is worse than no monitor, because it
/// still carries the authority to override the planner.
///
/// This class holds no ROS types so the entire policy can be exercised in unit
/// tests at arbitrary timings, including the ones that are hard to stage in
/// simulation.
class SafetyMonitor
{
public:
  struct Options
  {
    /// Minimum time a halt stays engaged once triggered [s].
    double min_hold_seconds = 0.4;
    /// A scan older than this trips kSensorTimeout [s].
    double sensor_timeout_seconds = 0.35;
    /// Returns below this are dropped as self-hits / noise [m].
    double min_valid_range = 0.02;
  };

  SafetyMonitor(const amr_core::RobotProfile & profile, const Options & options);

  /// \brief Evaluate the envelope against a fresh scan.
  ///
  /// \param samples          Validated returns in the body frame. Only those
  ///                         inside the guarded forward cone are considered.
  /// \param speed            Current forward speed [m/s].
  /// \param now              Monotonic time [s].
  /// \param scan_stamp       Acquisition time of \p samples [s].
  /// \param sensor_valid     False when the BSP layer rejected the scan.
  SafetyDecision Evaluate(
    const std::vector<RangeSample> & samples, double speed, double now, double scan_stamp,
    bool sensor_valid);

  /// \brief Evaluate with no new scan available, e.g. from a watchdog tick.
  SafetyDecision EvaluateStale(double speed, double now);

  /// \brief Engage or release the manual override (service interface / e-stop).
  ///
  /// Releasing a manual halt clears the latch outright rather than handing the
  /// robot to the obstacle path's minimum hold timer. That timer exists to
  /// suppress sensor chatter, and an operator who has just released e-stop is
  /// not chatter; making them wait for it would be surprising and would train
  /// people to jab the control. The next `Evaluate` still has to find the way
  /// clear before the robot actually moves.
  void SetManualOverride(bool engaged)
  {
    if (!engaged && manual_override_ && reason_ == HaltReason::kManualOverride) {
      halted_ = false;
      reason_ = HaltReason::kNone;
    }
    manual_override_ = engaged;
  }
  bool ManualOverride() const {return manual_override_;}

  bool Halted() const {return halted_;}

  /// \brief Smallest range inside the guarded cone, or +infinity when empty.
  ///
  /// Exposed for testing and because the node publishes it as telemetry.
  double MinRangeInSector(const std::vector<RangeSample> & samples) const;

private:
  SafetyDecision Decide(double min_range, double speed, double now, HaltReason forced);

  amr_core::RobotProfile profile_;
  Options options_;

  bool halted_ = false;
  bool manual_override_ = false;
  HaltReason reason_ = HaltReason::kNone;
  double halt_started_at_ = 0.0;
  double last_scan_time_ = -1.0;
  bool has_received_scan_ = false;
};

}  // namespace amr_fleet_control

#endif  // AMR_FLEET_CONTROL__SAFETY_MONITOR_HPP_
