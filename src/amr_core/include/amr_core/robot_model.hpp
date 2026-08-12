// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_CORE__ROBOT_MODEL_HPP_
#define AMR_CORE__ROBOT_MODEL_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace amr_core
{

/// \brief What a robot is for. Drives defaults, not behaviour.
///
/// Behaviour is always parameter-driven; this enum exists so that logs and
/// diagnostics can say "the mapper yielded to nobody" rather than
/// "robot with priority 100 yielded to nobody".
enum class RobotRole : std::uint8_t
{
  kMapper = 0,   ///< Heavy lead unit. Contributes authoritative map updates.
  kScout = 1,    ///< Light follower. Fast, low payload.
  kUnknown = 2
};

/// \brief Convert a YAML role string to the enum. Unknown strings do not throw;
///        they degrade to kUnknown so a typo cannot silently become "mapper".
RobotRole RoleFromString(const std::string & text);

/// \brief Inverse of RoleFromString, for logging and diagnostics.
const char * RoleToString(RobotRole role);

// ---------------------------------------------------------------------------
// Sensor specifications
// ---------------------------------------------------------------------------

/// \brief LiDAR geometry and the envelope the BSP validator enforces.
struct LidarSpec
{
  double height = 0.35;          ///< Mounting height above base_footprint [m].
  int samples = 720;
  double min_angle = -M_PI;
  double max_angle = M_PI;
  double range_min = 0.10;
  double range_max = 20.0;
  double rate = 20.0;            ///< Expected publish rate [Hz].
  double noise_stddev = 0.01;
};

/// \brief IMU envelope. The angular-velocity limit is the one the assignment
///        calls out explicitly: exceeding it must raise a warning.
struct ImuSpec
{
  double height = 0.20;
  double rate = 100.0;
  /// Largest yaw rate this chassis can physically produce, plus margin.
  /// A reading beyond this is a sensor fault, not a manoeuvre.
  double max_angular_velocity = 3.0;
  /// Largest linear acceleration magnitude excluding gravity [m/s^2].
  double max_linear_acceleration = 10.0;
  double gyro_noise_stddev = 0.0004;
  double accel_noise_stddev = 0.017;
};

/// \brief Camera envelope. Intensity bounds catch a blacked-out or saturated
///        imager, which is the classic silent failure on a real vehicle.
struct CameraSpec
{
  double height = 0.40;
  int width = 640;
  int height_px = 480;
  double rate = 20.0;
  double hfov = 1.4;
  double min_mean_intensity = 6.0;
  double max_mean_intensity = 249.0;
};

// ---------------------------------------------------------------------------
// Motion and safety
// ---------------------------------------------------------------------------

/// \brief The complete dynamic envelope of a robot model.
///
/// Everything the motion smoother needs, and nothing it does not. Kept as a
/// plain struct so it can be copied into a control loop without touching the
/// parameter server.
struct DynamicLimits
{
  double max_vel_x = 1.0;
  double min_vel_x = -0.4;
  double max_vel_theta = 1.5;

  double max_accel_x = 0.8;
  double max_decel_x = 1.2;      ///< Magnitude; applied as -max_decel_x.
  double max_accel_theta = 1.5;

  double max_jerk_x = 1.5;
  double max_jerk_theta = 3.0;

  /// Fraction by which accel/jerk shrink at 100% payload, in [0, 1).
  double payload_derating = 0.3;
  /// Fraction by which accel/jerk shrink at top speed, in [0, 1).
  double speed_derating = 0.2;

  /// \brief Effective longitudinal acceleration limit for a dynamic state.
  ///
  /// Both deratings are multiplicative and independent: a fully loaded robot
  /// at top speed is the most constrained state, which is the physically
  /// sensible ordering.
  ///
  /// \param load_ratio  payload / capacity, clamped to [0, 1].
  /// \param speed_ratio |v| / max_vel_x, clamped to [0, 1].
  double EffectiveAccelX(double load_ratio, double speed_ratio) const
  {
    return max_accel_x * DeratingFactor(load_ratio, speed_ratio);
  }

  double EffectiveDecelX(double load_ratio, double /*speed_ratio*/) const
  {
    // Braking authority is derated by payload (more mass, same brakes) but
    // NOT by speed: a robot must never lose stopping power precisely when it
    // is going fast. This asymmetry is deliberate and safety-relevant. The
    // speed argument is accepted anyway so all limit accessors share a
    // signature and can be called from one generic call site.
    return max_decel_x * (1.0 - payload_derating * Clamp01(load_ratio));
  }

  double EffectiveAccelTheta(double load_ratio, double speed_ratio) const
  {
    return max_accel_theta * DeratingFactor(load_ratio, speed_ratio);
  }

  double EffectiveJerkX(double load_ratio, double speed_ratio) const
  {
    return max_jerk_x * DeratingFactor(load_ratio, speed_ratio);
  }

  double EffectiveJerkTheta(double load_ratio, double speed_ratio) const
  {
    return max_jerk_theta * DeratingFactor(load_ratio, speed_ratio);
  }

  /// \brief Shared derating term, floored so limits never reach zero.
  double DeratingFactor(double load_ratio, double speed_ratio) const
  {
    const double factor = (1.0 - payload_derating * Clamp01(load_ratio)) *
      (1.0 - speed_derating * Clamp01(speed_ratio));
    // A zero limit would freeze the robot permanently; 5% keeps it creeping.
    return std::max(0.05, factor);
  }

  static double Clamp01(double value)
  {
    return std::min(1.0, std::max(0.0, value));
  }
};

/// \brief Parameters of the speed-dependent safety envelope.
///
/// The trigger distance is d_safe = k * v^2 + d_min. The quadratic term is the
/// kinematic stopping distance (v^2 / 2a, so k ~ 1 / 2a) and d_min is the
/// standstill buffer that keeps a stationary robot from being nudged.
struct SafetySpec
{
  double k = 0.5;                    ///< [s^2/m]
  double d_min = 0.35;               ///< [m]
  /// Extra clearance required before the halt releases, preventing chatter at
  /// the boundary.
  double release_hysteresis = 0.15;  ///< [m]
  /// Half-angle of the forward cone the monitor guards [rad]. Returns outside
  /// it cannot be hit while driving forward and would cause false halts in
  /// narrow aisles.
  double sector_half_angle = 1.22;

  /// \brief Trigger distance for a given speed.
  double SafeDistance(double speed) const
  {
    const double v = std::abs(speed);
    return k * v * v + d_min;
  }

  /// \brief Distance at which an engaged halt is allowed to release.
  double ReleaseDistance(double speed) const
  {
    return SafeDistance(speed) + release_hysteresis;
  }
};

// ---------------------------------------------------------------------------
// The model
// ---------------------------------------------------------------------------

/// \brief Everything known about one *model* of robot.
///
/// This struct is the C++ face of a block in
/// `amr_description/config/robot_models.yaml`. Nodes take a copy at startup
/// and never consult the parameter server again on the hot path.
struct RobotProfile
{
  std::string model_name;        ///< Key in the model library, e.g. "heavy_mapper".
  std::string human_name;
  RobotRole role = RobotRole::kUnknown;

  // Geometry
  double chassis_length = 0.6;
  double chassis_width = 0.4;
  double wheel_radius = 0.1;
  double wheel_separation = 0.4;
  double footprint_radius = 0.4;   ///< Circumscribed.
  double inscribed_radius = 0.25;

  // Payload
  double payload_capacity_kg = 50.0;

  DynamicLimits limits;
  SafetySpec safety;
  LidarSpec lidar;
  ImuSpec imu;
  CameraSpec camera;

  /// Traffic priority: higher wins a conflict, lower yields.
  int yield_priority = 0;

  /// \brief Load ratio for a payload, clamped and divide-by-zero safe.
  double LoadRatio(double payload_kg) const
  {
    if (payload_capacity_kg <= 0.0) {
      return 0.0;
    }
    return DynamicLimits::Clamp01(payload_kg / payload_capacity_kg);
  }

  /// \brief Fraction of top speed, clamped and divide-by-zero safe.
  double SpeedRatio(double speed) const
  {
    if (limits.max_vel_x <= 0.0) {
      return 0.0;
    }
    return DynamicLimits::Clamp01(std::abs(speed) / limits.max_vel_x);
  }
};

/// \brief One *instance* of a robot in the fleet.
///
/// Separating instance from model is what lets the fleet grow to ten or more
/// robots by appending roster entries rather than duplicating physics.
struct RobotInstance
{
  std::string name;              ///< ROS namespace, e.g. "amr1".
  std::string model_name;        ///< Key into the model library.
  RobotProfile profile;          ///< Resolved copy of the model block.

  // Spawn pose in the shared map frame. Also seeds the static transform
  // map -> <name>/map used to fuse the per-robot SLAM maps.
  double initial_x = 0.0;
  double initial_y = 0.0;
  double initial_yaw = 0.0;

  /// Overrides the model's priority when set (>= 0). Lets two robots of the
  /// same model be ranked without inventing a new model.
  int priority_override = -1;

  /// \brief Effective traffic priority for this instance.
  int YieldPriority() const
  {
    return priority_override >= 0 ? priority_override : profile.yield_priority;
  }
};

}  // namespace amr_core

#endif  // AMR_CORE__ROBOT_MODEL_HPP_
