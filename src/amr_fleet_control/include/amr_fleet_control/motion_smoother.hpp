// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_FLEET_CONTROL__MOTION_SMOOTHER_HPP_
#define AMR_FLEET_CONTROL__MOTION_SMOOTHER_HPP_

#include <cstdint>

#include "amr_core/geometry.hpp"
#include "amr_core/robot_model.hpp"

namespace amr_fleet_control
{

/// \brief Which constraint bound the output on this tick.
///
/// Exposed rather than inferred, because "is the robot smooth because the
/// smoother is working, or because nav2 happened to ask for something gentle?"
/// is otherwise unanswerable from a rosbag.
enum class LimitReason : std::uint8_t
{
  kNone = 0,        ///< The target was achievable outright.
  kVelocity = 1,    ///< Clipped by max_vel.
  kAcceleration = 2,
  kJerk = 3
};

const char * LimitReasonToString(LimitReason reason);

/// \brief Per-axis diagnostic of one smoothing step.
struct AxisDiagnostics
{
  double target = 0.0;          ///< Requested velocity after traffic scaling.
  double command = 0.0;         ///< Velocity actually emitted.
  double acceleration = 0.0;    ///< Achieved acceleration [units/s].
  double jerk = 0.0;            ///< Achieved jerk [units/s^2].
  double accel_limit = 0.0;     ///< Limit in force this tick.
  double jerk_limit = 0.0;
  LimitReason reason = LimitReason::kNone;
};

/// \brief What the smoother did on one tick, for telemetry and tests.
struct SmootherDiagnostics
{
  AxisDiagnostics linear;
  AxisDiagnostics angular;
  double load_ratio = 0.0;
  double speed_ratio = 0.0;
  double speed_scale = 1.0;     ///< Traffic-controller multiplier applied.
  bool timed_out = false;       ///< True while ramping down after a lost input.
};

/// \brief The robot's instantaneous dynamic state, which the limits depend on.
struct DynamicState
{
  /// Carried mass as a fraction of capacity, in [0, 1].
  double load_ratio = 0.0;
  /// Multiplier in [0, 1] from the traffic controller. A yield is delivered
  /// here rather than as a direct zero command, which is precisely what makes
  /// it a *controlled* stop: the jerk limiter still shapes the deceleration.
  double speed_scale = 1.0;
};

/// \brief Jerk- and acceleration-limited velocity shaper for one robot.
///
/// ### The rule being enforced
///
/// For each axis independently, with limits that are themselves functions of
/// the dynamic state:
///
/// ```
///   e         = clamp(v_target, min_vel, max_vel) - v_current
///   a_cap     = min(accel_or_decel_limit,          // actuator envelope
///                   approach_cap(e, jerk, dt),     // S-curve terminal law
///                   |e| / dt)                      // no within-tick overshoot
///   a_final   = clamp(sign(e) * a_cap,
///                     a_previous - jerk*dt, a_previous + jerk*dt)
///   v_command = v_current + a_final*dt
/// ```
///
/// The jerk clamp is applied last, so it can only tighten the acceleration
/// bound, never loosen it. Note that there is no clamp of `v_command` to the
/// velocity envelope: `approach_cap` guarantees the envelope is approached
/// asymptotically rather than hit, and a post-hoc velocity clamp would
/// reintroduce exactly the jerk discontinuity the S-curve removes. See the
/// implementation for the derivation.
///
/// ### Why the limits are state-dependent
///
/// A loaded robot has the same actuators and more inertia, so its achievable
/// acceleration falls; at speed, the same acceleration is less stable. Both
/// effects are folded in through `amr_core::DynamicLimits`, which is where the
/// heavy mapper's deliberately lower ceilings come from.
///
/// ### Threading
///
/// Not thread-safe by design. One instance belongs to one control loop; the
/// node serialises access by running the loop in a single callback group.
class MotionSmoother
{
public:
  /// \param profile Physical envelope of the robot this smoother drives.
  explicit MotionSmoother(const amr_core::RobotProfile & profile);

  /// \brief Advance one control tick.
  ///
  /// \param target Velocity requested by the navigation stack.
  /// \param state  Payload and traffic scaling in force.
  /// \param dt     Time since the previous call [s]. Non-positive or absurdly
  ///               large values are rejected: a stale dt would let the
  ///               acceleration clamp authorise an arbitrarily large step.
  /// \return The velocity to command.
  amr_core::Velocity2D Smooth(
    const amr_core::Velocity2D & target, const DynamicState & state, double dt);

  /// \brief Bring the robot to rest under the normal limits.
  ///
  /// Used when the navigation stack goes quiet. This is a *controlled* stop:
  /// an immediate halt is the safety node's job and deliberately does not pass
  /// through here.
  amr_core::Velocity2D SmoothToStop(const DynamicState & state, double dt);

  /// \brief Forget all history. Call after a teleport or a controller restart,
  ///        where continuing to jerk-limit from a stale acceleration would
  ///        fight the new controller.
  void Reset();

  const amr_core::Velocity2D & Command() const {return command_;}
  const SmootherDiagnostics & Diagnostics() const {return diagnostics_;}

  /// \brief Velocities below this magnitude are snapped to zero.
  ///
  /// The terminal-approach law lands the command exactly on the target, so
  /// this only sweeps up floating-point dust. It is deliberately six orders of
  /// magnitude below the threshold a naive saturating limiter would need,
  /// because a large snap threshold is itself a jerk discontinuity.
  static constexpr double kStopThresholdLinear = 1e-6;
  static constexpr double kStopThresholdAngular = 1e-6;

  /// \brief Largest dt accepted before the tick is treated as a dropout.
  static constexpr double kMaxTimeStep = 0.5;

private:
  /// \brief Shape one axis. Returns the new velocity and fills \p diagnostics.
  double SmoothAxis(
    double target, double current, double & previous_accel, double accel_limit,
    double decel_limit, double jerk_limit, double min_vel, double max_vel, double dt,
    double stop_threshold, AxisDiagnostics & diagnostics) const;

  amr_core::RobotProfile profile_;
  amr_core::Velocity2D command_;
  double previous_accel_linear_ = 0.0;
  double previous_accel_angular_ = 0.0;
  SmootherDiagnostics diagnostics_;
};

}  // namespace amr_fleet_control

#endif  // AMR_FLEET_CONTROL__MOTION_SMOOTHER_HPP_
