// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.

#include "amr_sensor_bsp/sensor_validators.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "amr_core/geometry.hpp"

namespace amr_sensor_bsp
{

const char * ValidationStatusToString(ValidationStatus status)
{
  switch (status) {
    case ValidationStatus::kDegraded:
      return "DEGRADED";
    case ValidationStatus::kInvalid:
      return "INVALID";
    case ValidationStatus::kValid:
    default:
      return "OK";
  }
}

// ---------------------------------------------------------------------------
// LidarValidator
// ---------------------------------------------------------------------------

LidarValidator::LidarValidator(const amr_core::LidarSpec & spec, const Options & options)
: SensorValidator("lidar"), spec_(spec), options_(options) {}

double LidarValidator::GroundReturnRange() const
{
  const double pitch = std::abs(pitch_);
  if (!options_.ground_rejection_enabled || pitch < options_.ground_rejection_pitch) {
    return std::numeric_limits<double>::infinity();
  }
  // Beam leaves the sensor level with the chassis; the chassis is pitched by
  // `pitch`; it meets the floor at h / sin(pitch).
  return spec_.height / std::sin(pitch);
}

ValidationResult LidarValidator::Validate(
  const LidarFrame & frame, std::vector<float> * out_ranges)
{
  ValidationResult result;
  ground_suppressed_ = 0;
  RecordArrival(frame.stamp);

  // ---- Hard structural checks -------------------------------------------
  if (frame.ranges.empty()) {
    result.AddFault("empty_scan", true);
    RecordResult(result);
    return result;
  }

  if (static_cast<int>(frame.ranges.size()) != spec_.samples) {
    // The beam count determines the angle of every return. If it disagrees
    // with the model, every bearing computed downstream is wrong, and a
    // silently rotated scan is far more dangerous than no scan.
    result.AddFault("beam_count", true);
  }

  if (std::abs(frame.angle_min - spec_.min_angle) > options_.geometry_tolerance) {
    result.AddFault("angle_min", true);
  }

  const double expected_increment =
    (spec_.max_angle - spec_.min_angle) / static_cast<double>(std::max(1, spec_.samples - 1));
  if (frame.angle_increment > 0.0 &&
    std::abs(frame.angle_increment - expected_increment) > options_.geometry_tolerance)
  {
    result.AddFault("angle_increment", true);
  }

  // ---- Content checks ----------------------------------------------------
  std::size_t non_finite = 0;
  std::size_t out_of_bounds = 0;
  const double ground_range = GroundReturnRange();
  const double ground_threshold =
    std::isfinite(ground_range) ? ground_range * (1.0 - options_.ground_rejection_tolerance) :
    std::numeric_limits<double>::infinity();

  std::vector<float> conditioned;
  conditioned.reserve(frame.ranges.size());

  for (const float raw : frame.ranges) {
    const double value = static_cast<double>(raw);

    if (!std::isfinite(value)) {
      // A LaserScan uses inf for "no return", which is normal, and NaN for a
      // dropout, which is not - but both are handled identically downstream,
      // so they are counted together and passed through as inf.
      ++non_finite;
      conditioned.push_back(std::numeric_limits<float>::infinity());
      continue;
    }

    if (value < spec_.range_min || value > spec_.range_max) {
      // Out of the sensor's rated window: the number is meaningless, but one
      // bad beam is no reason to blind the robot.
      ++out_of_bounds;
      conditioned.push_back(std::numeric_limits<float>::infinity());
      continue;
    }

    if (value >= ground_threshold) {
      // Consistent with the floor ahead of a pitched chassis, not an obstacle.
      ++ground_suppressed_;
      conditioned.push_back(std::numeric_limits<float>::infinity());
      continue;
    }

    conditioned.push_back(raw);
  }

  const double invalid_fraction =
    static_cast<double>(non_finite) / static_cast<double>(frame.ranges.size());

  if (non_finite == frame.ranges.size()) {
    // Every single beam missing means the sensor is not producing data at all.
    result.AddFault("no_returns", true);
  } else if (invalid_fraction > options_.max_invalid_fraction) {
    result.AddFault("sparse_returns", false);
  }

  if (out_of_bounds > 0) {
    result.AddFault("out_of_range_returns", false);
  }

  if (result.ShouldForward() && out_ranges != nullptr) {
    *out_ranges = std::move(conditioned);
  }

  RecordResult(result);
  return result;
}

// ---------------------------------------------------------------------------
// ImuValidator
// ---------------------------------------------------------------------------

ImuValidator::ImuValidator(const amr_core::ImuSpec & spec, const Options & options)
: SensorValidator("imu"), spec_(spec), options_(options) {}

ValidationResult ImuValidator::Validate(const ImuFrame & frame)
{
  ValidationResult result;
  RecordArrival(frame.stamp);

  // ---- Finiteness: a NaN here propagates into the whole TF tree ----------
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(frame.angular_velocity[axis]) ||
      !std::isfinite(frame.linear_acceleration[axis]))
    {
      result.AddFault("non_finite", true);
      RecordResult(result);
      return result;
    }
  }

  // ---- Orientation ------------------------------------------------------
  const double norm = std::sqrt(
    frame.orientation[0] * frame.orientation[0] + frame.orientation[1] * frame.orientation[1] +
    frame.orientation[2] * frame.orientation[2] + frame.orientation[3] * frame.orientation[3]);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > options_.quaternion_tolerance) {
    // An all-zero quaternion is the classic uninitialised-driver signature and
    // would rotate everything to nonsense.
    result.AddFault("orientation_not_normalised", true);
  } else {
    pitch_ = amr_core::PitchFromQuaternion(
      frame.orientation[0], frame.orientation[1], frame.orientation[2], frame.orientation[3]);
  }

  // ---- The angular-velocity plausibility check the brief calls out -------
  const double angular_magnitude = std::sqrt(
    frame.angular_velocity[0] * frame.angular_velocity[0] +
    frame.angular_velocity[1] * frame.angular_velocity[1] +
    frame.angular_velocity[2] * frame.angular_velocity[2]);
  peak_angular_velocity_ = std::max(peak_angular_velocity_, angular_magnitude);

  if (angular_magnitude > spec_.max_angular_velocity) {
    // Soft on purpose. See the class comment: this is far more likely to be a
    // units or timestamp bug than a robot genuinely spinning that fast, and
    // dropping IMU data mid-manoeuvre would destabilise the localisation that
    // has to recover from it.
    result.AddFault("angular_velocity", false);
  }

  // ---- Linear acceleration ----------------------------------------------
  const double acceleration_magnitude = std::sqrt(
    frame.linear_acceleration[0] * frame.linear_acceleration[0] +
    frame.linear_acceleration[1] * frame.linear_acceleration[1] +
    frame.linear_acceleration[2] * frame.linear_acceleration[2]);
  // The reported vector includes gravity, so a stationary robot reads ~9.81.
  const double bound = spec_.max_linear_acceleration + kGravity + options_.acceleration_margin;
  if (acceleration_magnitude > bound) {
    result.AddFault("linear_acceleration", false);
  }

  RecordResult(result);
  return result;
}

// ---------------------------------------------------------------------------
// CameraValidator
// ---------------------------------------------------------------------------

CameraValidator::CameraValidator(const amr_core::CameraSpec & spec, const Options & options)
: SensorValidator("camera"), spec_(spec), options_(options) {}

ValidationResult CameraValidator::Validate(const CameraFrame & frame)
{
  ValidationResult result;
  last_mean_intensity_ = -1.0;
  RecordArrival(frame.stamp);

  if (frame.width == 0 || frame.height == 0) {
    result.AddFault("empty_image", true);
    RecordResult(result);
    return result;
  }

  if (static_cast<int>(frame.width) != spec_.width ||
    static_cast<int>(frame.height) != spec_.height_px)
  {
    // A resolution change means the driver was reconfigured behind the stack's
    // back; every intrinsic downstream is now wrong.
    result.AddFault("resolution", true);
  }

  if (!frame.encoding.empty() && frame.encoding != "rgb8" && frame.encoding != "bgr8" &&
    frame.encoding != "mono8")
  {
    result.AddFault("encoding", true);
  }

  const std::size_t expected =
    static_cast<std::size_t>(frame.width) * frame.height * frame.channels;
  if (frame.data != nullptr && frame.data_size != expected) {
    result.AddFault("buffer_size", true);
  }

  // ---- Content: the silent failure a metadata check cannot catch ---------
  if (options_.check_intensity && frame.data != nullptr && result.ShouldForward() &&
    frame.data_size >= expected && expected > 0)
  {
    const std::uint32_t stride = std::max(1u, options_.sample_stride);
    std::uint64_t total = 0;
    std::uint64_t count = 0;
    for (std::size_t i = 0; i < frame.data_size; i += stride) {
      total += frame.data[i];
      ++count;
    }
    if (count > 0) {
      last_mean_intensity_ = static_cast<double>(total) / static_cast<double>(count);
      if (last_mean_intensity_ < spec_.min_mean_intensity) {
        result.AddFault("image_black", false);
      } else if (last_mean_intensity_ > spec_.max_mean_intensity) {
        result.AddFault("image_saturated", false);
      }
    }
  }

  RecordResult(result);
  return result;
}

}  // namespace amr_sensor_bsp
