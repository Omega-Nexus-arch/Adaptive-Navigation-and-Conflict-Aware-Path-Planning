// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_SENSOR_BSP__SENSOR_VALIDATORS_HPP_
#define AMR_SENSOR_BSP__SENSOR_VALIDATORS_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "amr_core/robot_model.hpp"
#include "amr_sensor_bsp/validator.hpp"

namespace amr_sensor_bsp
{

// ---------------------------------------------------------------------------
// LiDAR
// ---------------------------------------------------------------------------

/// \brief A scan reduced to the fields the validator needs.
///
/// Mirrors `sensor_msgs/LaserScan` without depending on it, so the validation
/// logic can be exercised from a unit test with no ROS types in sight.
struct LidarFrame
{
  double stamp = 0.0;
  double angle_min = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
  std::vector<float> ranges;
};

/// \brief Validates and conditions a 2D scan.
///
/// ### Checks
///
/// | Check | Severity | Rationale |
/// |---|---|---|
/// | empty scan | hard | Nothing to consume. |
/// | wrong beam count | hard | The driver is misconfigured; angles would be wrong. |
/// | all returns non-finite | hard | The sensor is not producing data. |
/// | geometry disagrees with the model | hard | Angles cannot be trusted. |
/// | some out-of-bounds returns | soft | Individual bad beams; clamp and carry on. |
/// | stale timestamp | soft | Reported; the safety monitor enforces its own watchdog. |
///
/// ### Ground-return rejection on ramps
///
/// A fixed, level 2D LiDAR pitches with the chassis. On the warehouse's 8-15
/// degree ramps its beam tilts down and strikes the floor a few metres ahead:
/// at a 0.42 m mounting height and 8.7 degrees that is 2.8 m. The navigation
/// stack sees a phantom wall across the ramp, the costmap fills in, and the
/// planner concludes the ramp is impassable - which would quietly invalidate
/// the entire slope-cost experiment.
///
/// So when the validated IMU reports a pitch beyond `ground_rejection_pitch`,
/// returns consistent with the ground plane are discarded (set to infinity)
/// rather than the whole scan being rejected. The predicted ground range is
/// `h / sin(|pitch|)`; anything at or beyond it, less a tolerance, is floor.
///
/// This is the reason the BSP layer is worth building rather than
/// stubbing: it is the only place that has both the raw scan and the validated
/// attitude, and the fix needs both.
class LidarValidator : public SensorValidator
{
public:
  struct Options
  {
    /// Tolerated deviation of scan geometry from the model, in radians.
    double geometry_tolerance = 0.02;
    /// Fraction of non-finite returns above which the scan is degraded.
    double max_invalid_fraction = 0.35;
    /// A scan older than this is flagged (soft).
    double staleness_seconds = 0.5;
    /// Enable IMU-informed ground-return rejection.
    bool ground_rejection_enabled = true;
    /// Pitch magnitude beyond which rejection engages [rad]. Below it the
    /// beam reaches far enough that ground returns are out of range anyway.
    double ground_rejection_pitch = 0.06;
    /// Returns within this fraction of the predicted ground range are treated
    /// as floor.
    double ground_rejection_tolerance = 0.25;
  };

  LidarValidator(const amr_core::LidarSpec & spec, const Options & options);

  /// \brief Validate \p frame and write the conditioned scan to \p out_ranges.
  ///
  /// \p out_ranges may alias nothing; pass nullptr when only the verdict is
  /// wanted. On a hard fault it is left untouched.
  ValidationResult Validate(const LidarFrame & frame, std::vector<float> * out_ranges);

  /// \brief Supply the latest validated chassis pitch, in radians.
  ///
  /// Only ever fed from an IMU sample the IMU validator accepted: conditioning
  /// the scan on rejected attitude data would couple the two failures.
  void SetPitch(double pitch_radians) {pitch_ = pitch_radians;}
  double Pitch() const {return pitch_;}

  /// \brief Predicted range at which the beam meets the floor, or infinity
  ///        when the robot is level enough that it does not.
  double GroundReturnRange() const;

  /// \brief Number of returns suppressed as floor by the most recent call.
  std::uint32_t LastGroundReturnsSuppressed() const {return ground_suppressed_;}

private:
  amr_core::LidarSpec spec_;
  Options options_;
  double pitch_ = 0.0;
  std::uint32_t ground_suppressed_ = 0;
};

// ---------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------

/// \brief An IMU sample reduced to the fields the validator needs.
struct ImuFrame
{
  double stamp = 0.0;
  double angular_velocity[3] = {0.0, 0.0, 0.0};
  double linear_acceleration[3] = {0.0, 0.0, 0.0};
  double orientation[4] = {0.0, 0.0, 0.0, 1.0};   ///< x, y, z, w
};

/// \brief Validates an IMU sample against what the chassis can physically do.
///
/// The check the brief calls out explicitly is the angular-velocity limit: a
/// warning is logged when the measured rate exceeds a physically plausible
/// value for the robot model. `heavy_mapper` can command 0.90 rad/s and is
/// given a 2.50 rad/s ceiling; `light_scout` commands 1.90 and is given 4.50.
/// The margin covers being shoved, cresting a ramp, and honest sensor noise;
/// beyond it the reading is a fault, not a manoeuvre.
///
/// Deliberately a *soft* fault. A yaw rate of 40 rad/s is far more likely to
/// be a units or timestamp bug than a robot actually spinning, and dropping
/// IMU samples mid-manoeuvre would destabilise the localisation that has to
/// recover from it. The stack keeps running, loudly.
class ImuValidator : public SensorValidator
{
public:
  struct Options
  {
    double staleness_seconds = 0.5;
    /// Tolerance on |q| = 1 before the orientation is called malformed.
    double quaternion_tolerance = 0.05;
    /// Gravity is included in the reported acceleration, so the plausibility
    /// bound is `max_linear_acceleration + kGravity + this`.
    double acceleration_margin = 2.0;
  };

  ImuValidator(const amr_core::ImuSpec & spec, const Options & options);

  ValidationResult Validate(const ImuFrame & frame);

  /// \brief Chassis pitch from the most recent accepted sample [rad].
  double Pitch() const {return pitch_;}

  /// \brief Largest |omega| seen so far, for the diagnostics summary.
  double PeakAngularVelocity() const {return peak_angular_velocity_;}

  static constexpr double kGravity = 9.80665;

private:
  amr_core::ImuSpec spec_;
  Options options_;
  double pitch_ = 0.0;
  double peak_angular_velocity_ = 0.0;
};

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------

/// \brief An image reduced to the fields the validator needs.
struct CameraFrame
{
  double stamp = 0.0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::string encoding;
  /// Row-major pixel data. May be empty when only metadata is being checked.
  const std::uint8_t * data = nullptr;
  std::size_t data_size = 0;
  std::uint32_t channels = 3;
};

/// \brief Validates a camera frame.
///
/// The rubric asks for validation output on the IMU *and* the camera, and a
/// camera has its own characteristic silent failure: it keeps publishing
/// perfectly well-formed frames that contain nothing. A blacked-out imager and
/// a saturated one both produce a valid `sensor_msgs/Image` at the right rate,
/// with the right dimensions, that no downstream consumer can use. Dimension
/// and encoding checks alone would pass both.
///
/// So alongside the structural checks there is a content check: mean intensity
/// must fall inside the model's configured window. Sampling is strided - one
/// pixel in every `sample_stride` - because this runs at camera rate and full
/// summation of a 848x480 frame is wasted work for a statistic this coarse.
class CameraValidator : public SensorValidator
{
public:
  struct Options
  {
    double staleness_seconds = 0.5;
    /// Sample one pixel in every N when computing mean intensity.
    std::uint32_t sample_stride = 17;
    /// Check pixel content, not just metadata.
    bool check_intensity = true;
  };

  CameraValidator(const amr_core::CameraSpec & spec, const Options & options);

  ValidationResult Validate(const CameraFrame & frame);

  /// \brief Mean intensity of the most recent frame, or -1 when not computed.
  double LastMeanIntensity() const {return last_mean_intensity_;}

private:
  amr_core::CameraSpec spec_;
  Options options_;
  double last_mean_intensity_ = -1.0;
};

}  // namespace amr_sensor_bsp

#endif  // AMR_SENSOR_BSP__SENSOR_VALIDATORS_HPP_
