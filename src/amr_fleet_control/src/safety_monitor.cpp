// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.

#include "amr_fleet_control/safety_monitor.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "amr_core/geometry.hpp"

namespace amr_fleet_control
{

const char * HaltReasonToString(HaltReason reason)
{
  switch (reason) {
    case HaltReason::kObstacle:
      return "obstacle";
    case HaltReason::kSensorInvalid:
      return "sensor_invalid";
    case HaltReason::kSensorTimeout:
      return "sensor_timeout";
    case HaltReason::kManualOverride:
      return "manual_override";
    case HaltReason::kNone:
    default:
      return "none";
  }
}

SafetyMonitor::SafetyMonitor(const amr_core::RobotProfile & profile, const Options & options)
: profile_(profile), options_(options) {}

double SafetyMonitor::MinRangeInSector(const std::vector<RangeSample> & samples) const
{
  double minimum = std::numeric_limits<double>::infinity();
  const double half_angle = profile_.safety.sector_half_angle;

  for (const RangeSample & sample : samples) {
    // Only the cone the robot is driving into matters. Guarding the full 360
    // degrees would halt the robot every time it passed a rack in a 2.4 m
    // aisle, and a monitor that fires constantly gets switched off.
    if (std::abs(amr_core::NormalizeAngle(sample.angle)) > half_angle) {
      continue;
    }
    // Non-finite returns mean "nothing out there" in the LaserScan convention;
    // sub-minimum returns are self-hits off the chassis.
    if (!std::isfinite(sample.range) || sample.range < options_.min_valid_range) {
      continue;
    }
    if (sample.range < minimum) {
      minimum = sample.range;
    }
  }
  return minimum;
}

SafetyDecision SafetyMonitor::Decide(
  double min_range, double speed, double now, HaltReason forced)
{
  SafetyDecision decision;
  decision.speed = speed;
  decision.min_obstacle_distance = min_range;
  decision.safe_distance = profile_.safety.SafeDistance(speed);
  decision.release_distance = profile_.safety.ReleaseDistance(speed);

  const bool was_halted = halted_;

  // The manual override outranks everything: an operator pressing e-stop must
  // not be argued with by a clear sensor reading.
  if (manual_override_) {
    halted_ = true;
    reason_ = HaltReason::kManualOverride;
    if (!was_halted) {
      halt_started_at_ = now;
      decision.newly_engaged = true;
    }
    decision.halt = true;
    decision.reason = reason_;
    return decision;
  }

  if (forced != HaltReason::kNone) {
    halted_ = true;
    reason_ = forced;
    if (!was_halted) {
      halt_started_at_ = now;
      decision.newly_engaged = true;
    }
    decision.halt = true;
    decision.reason = reason_;
    return decision;
  }

  if (!halted_) {
    // Engage on the *trigger* distance.
    if (min_range < decision.safe_distance) {
      halted_ = true;
      reason_ = HaltReason::kObstacle;
      halt_started_at_ = now;
      decision.newly_engaged = true;
    }
  } else {
    // Release on the *release* distance, and not before the hold time. The two
    // conditions together are what prevent limit-cycle chatter.
    const bool clear = min_range > decision.release_distance;
    const bool held_long_enough = (now - halt_started_at_) >= options_.min_hold_seconds;
    if (clear && held_long_enough) {
      halted_ = false;
      reason_ = HaltReason::kNone;
      decision.newly_released = true;
    }
  }

  decision.halt = halted_;
  decision.reason = reason_;
  return decision;
}

SafetyDecision SafetyMonitor::Evaluate(
  const std::vector<RangeSample> & samples, double speed, double now, double scan_stamp,
  bool sensor_valid)
{
  last_scan_time_ = scan_stamp;
  has_received_scan_ = true;

  if (!sensor_valid) {
    // The BSP layer rejected this scan. Treating "I cannot trust my sensor" as
    // "the way ahead is clear" is the single most dangerous default available.
    return Decide(0.0, speed, now, HaltReason::kSensorInvalid);
  }

  const double age = now - scan_stamp;
  if (age > options_.sensor_timeout_seconds) {
    return Decide(0.0, speed, now, HaltReason::kSensorTimeout);
  }

  return Decide(MinRangeInSector(samples), speed, now, HaltReason::kNone);
}

SafetyDecision SafetyMonitor::EvaluateStale(double speed, double now)
{
  // Before the first scan the robot has never been cleared to move; after the
  // watchdog expires it has stopped being cleared. Both halt.
  if (!has_received_scan_) {
    return Decide(0.0, speed, now, HaltReason::kSensorTimeout);
  }
  const double age = now - last_scan_time_;
  if (age > options_.sensor_timeout_seconds) {
    return Decide(0.0, speed, now, HaltReason::kSensorTimeout);
  }

  // Still inside the watchdog window: hold the previous verdict rather than
  // fabricating a clear reading.
  SafetyDecision decision;
  decision.speed = speed;
  decision.halt = halted_;
  decision.reason = reason_;
  decision.safe_distance = profile_.safety.SafeDistance(speed);
  decision.release_distance = profile_.safety.ReleaseDistance(speed);
  decision.min_obstacle_distance = std::numeric_limits<double>::infinity();
  return decision;
}

}  // namespace amr_fleet_control
