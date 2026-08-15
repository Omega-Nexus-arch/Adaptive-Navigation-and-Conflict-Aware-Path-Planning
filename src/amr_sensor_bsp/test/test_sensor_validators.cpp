// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// The BSP layer is the gate everything else trusts, so these tests check both
// halves of the contract: that genuine faults are caught and correctly graded
// hard or soft, and - just as important - that ordinary healthy data is not
// rejected. A validator that fails clean data is worse than none, because it
// will be switched off.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "amr_sensor_bsp/sensor_validators.hpp"

namespace
{

using amr_sensor_bsp::CameraFrame;
using amr_sensor_bsp::CameraValidator;
using amr_sensor_bsp::ImuFrame;
using amr_sensor_bsp::ImuValidator;
using amr_sensor_bsp::LidarFrame;
using amr_sensor_bsp::LidarValidator;
using amr_sensor_bsp::ValidationResult;
using amr_sensor_bsp::ValidationStatus;

bool HasFault(const ValidationResult & result, const std::string & name)
{
  return std::find(result.faults.begin(), result.faults.end(), name) != result.faults.end();
}

// ---------------------------------------------------------------------------
// Fixtures matching the heavy_mapper model
// ---------------------------------------------------------------------------

amr_core::LidarSpec HeavyLidarSpec()
{
  amr_core::LidarSpec spec;
  spec.height = 0.42;
  spec.samples = 360;
  spec.min_angle = -M_PI;
  spec.max_angle = M_PI;
  spec.range_min = 0.12;
  spec.range_max = 25.0;
  spec.rate = 15.0;
  return spec;
}

amr_core::ImuSpec HeavyImuSpec()
{
  amr_core::ImuSpec spec;
  spec.rate = 100.0;
  spec.max_angular_velocity = 2.50;
  spec.max_linear_acceleration = 9.0;
  return spec;
}

amr_core::CameraSpec HeavyCameraSpec()
{
  amr_core::CameraSpec spec;
  spec.width = 64;
  spec.height_px = 48;
  spec.rate = 15.0;
  spec.min_mean_intensity = 6.0;
  spec.max_mean_intensity = 249.0;
  return spec;
}

LidarFrame HealthyScan(double range = 5.0)
{
  const amr_core::LidarSpec spec = HeavyLidarSpec();
  LidarFrame frame;
  frame.stamp = 100.0;
  frame.angle_min = spec.min_angle;
  frame.angle_increment = (spec.max_angle - spec.min_angle) / (spec.samples - 1);
  frame.range_min = spec.range_min;
  frame.range_max = spec.range_max;
  frame.ranges.assign(static_cast<std::size_t>(spec.samples), static_cast<float>(range));
  return frame;
}

ImuFrame HealthyImu()
{
  ImuFrame frame;
  frame.stamp = 100.0;
  frame.angular_velocity[2] = 0.4;
  frame.linear_acceleration[2] = ImuValidator::kGravity;
  frame.orientation[3] = 1.0;
  return frame;
}

// ---------------------------------------------------------------------------
// LiDAR
// ---------------------------------------------------------------------------

TEST(LidarValidatorTest, HealthyScanPassesUntouched) {
  LidarValidator validator(HeavyLidarSpec(), {});
  std::vector<float> out;
  const ValidationResult result = validator.Validate(HealthyScan(), &out);

  EXPECT_EQ(result.status, ValidationStatus::kValid);
  EXPECT_TRUE(result.faults.empty());
  EXPECT_EQ(out.size(), 360u);
  EXPECT_NEAR(out[0], 5.0, 1e-6);
  EXPECT_EQ(validator.Statistics().received, 1u);
  EXPECT_EQ(validator.Statistics().rejected, 0u);
}

TEST(LidarValidatorTest, EmptyScanIsAHardFault) {
  LidarValidator validator(HeavyLidarSpec(), {});
  LidarFrame frame = HealthyScan();
  frame.ranges.clear();

  const ValidationResult result = validator.Validate(frame, nullptr);
  EXPECT_EQ(result.status, ValidationStatus::kInvalid);
  EXPECT_FALSE(result.ShouldForward());
  EXPECT_TRUE(HasFault(result, "empty_scan"));
  EXPECT_EQ(validator.Statistics().rejected, 1u);
}

TEST(LidarValidatorTest, WrongBeamCountIsAHardFault) {
  // Every bearing downstream is derived from the beam index. A silently
  // rotated scan is more dangerous than no scan at all.
  LidarValidator validator(HeavyLidarSpec(), {});
  LidarFrame frame = HealthyScan();
  frame.ranges.resize(180);

  const ValidationResult result = validator.Validate(frame, nullptr);
  EXPECT_EQ(result.status, ValidationStatus::kInvalid);
  EXPECT_TRUE(HasFault(result, "beam_count"));
}

TEST(LidarValidatorTest, WrongGeometryIsAHardFault) {
  LidarValidator validator(HeavyLidarSpec(), {});
  LidarFrame frame = HealthyScan();
  frame.angle_min = 0.0;   // Driver reconfigured to a half sweep.

  const ValidationResult result = validator.Validate(frame, nullptr);
  EXPECT_EQ(result.status, ValidationStatus::kInvalid);
  EXPECT_TRUE(HasFault(result, "angle_min"));
}

TEST(LidarValidatorTest, ATotallyBlindScanIsAHardFault) {
  LidarValidator validator(HeavyLidarSpec(), {});
  LidarFrame frame = HealthyScan();
  std::fill(
    frame.ranges.begin(), frame.ranges.end(), std::numeric_limits<float>::quiet_NaN());

  const ValidationResult result = validator.Validate(frame, nullptr);
  EXPECT_EQ(result.status, ValidationStatus::kInvalid);
  EXPECT_TRUE(HasFault(result, "no_returns"));
}

TEST(LidarValidatorTest, AFewBadBeamsAreASoftFaultAndTheScanStillFlows) {
  LidarValidator validator(HeavyLidarSpec(), {});
  LidarFrame frame = HealthyScan();
  frame.ranges[10] = 900.0f;    // Beyond range_max.
  frame.ranges[11] = 0.001f;    // Inside range_min.

  std::vector<float> out;
  const ValidationResult result = validator.Validate(frame, &out);

  EXPECT_EQ(result.status, ValidationStatus::kDegraded);
  EXPECT_TRUE(result.ShouldForward()) << "two bad beams must not blind the robot";
  EXPECT_TRUE(HasFault(result, "out_of_range_returns"));
  EXPECT_TRUE(std::isinf(out[10]));
  EXPECT_TRUE(std::isinf(out[11]));
  EXPECT_NEAR(out[12], 5.0, 1e-6) << "good beams must be left alone";
}

TEST(LidarValidatorTest, MostlyMissingReturnsAreDegradedNotRejected) {
  LidarValidator validator(HeavyLidarSpec(), {});
  LidarFrame frame = HealthyScan();
  for (std::size_t i = 0; i < 300; ++i) {
    frame.ranges[i] = std::numeric_limits<float>::infinity();
  }

  const ValidationResult result = validator.Validate(frame, nullptr);
  EXPECT_EQ(result.status, ValidationStatus::kDegraded);
  EXPECT_TRUE(HasFault(result, "sparse_returns"));
}

TEST(LidarValidatorTest, StatisticsCountEachOutcome) {
  LidarValidator validator(HeavyLidarSpec(), {});
  validator.Validate(HealthyScan(), nullptr);

  LidarFrame degraded = HealthyScan();
  degraded.ranges[0] = 900.0f;
  validator.Validate(degraded, nullptr);

  LidarFrame invalid = HealthyScan();
  invalid.ranges.clear();
  validator.Validate(invalid, nullptr);

  EXPECT_EQ(validator.Statistics().received, 3u);
  EXPECT_EQ(validator.Statistics().degraded, 1u);
  EXPECT_EQ(validator.Statistics().rejected, 1u);
  EXPECT_NEAR(validator.Statistics().RejectRatio(), 1.0 / 3.0, 1e-9);
}

TEST(LidarValidatorTest, MeasuredRateTracksTheArrivalInterval) {
  LidarValidator validator(HeavyLidarSpec(), {});
  for (int i = 0; i < 80; ++i) {
    LidarFrame frame = HealthyScan();
    frame.stamp = 100.0 + i / 15.0;    // 15 Hz, the model's rate.
    validator.Validate(frame, nullptr);
  }
  EXPECT_NEAR(validator.Statistics().measured_rate_hz, 15.0, 1.0);
}

// ---- Ground-return rejection ----------------------------------------------

TEST(LidarValidatorTest, LevelRobotSuppressesNothing) {
  LidarValidator validator(HeavyLidarSpec(), {});
  validator.SetPitch(0.0);

  std::vector<float> out;
  validator.Validate(HealthyScan(20.0), &out);
  EXPECT_EQ(validator.LastGroundReturnsSuppressed(), 0u);
  EXPECT_TRUE(std::isinf(validator.GroundReturnRange()));
  EXPECT_NEAR(out[0], 20.0, 1e-6);
}

TEST(LidarValidatorTest, OnARampFloorReturnsAreSuppressedButObstaclesSurvive) {
  // The scenario that would otherwise make the ramps impassable: a phantom
  // wall across the ramp closes it in the costmap and quietly invalidates the
  // whole slope-cost experiment.
  //
  // Each beam gets ITS OWN predicted floor range. A 360 degree scanner pitched
  // nose-down only aims at the floor ahead; sideways beams are level and rear
  // beams point up, and none of those can produce a ground return.
  LidarValidator validator(HeavyLidarSpec(), {});
  const double pitch = 6.0 * M_PI / 180.0;
  validator.SetPitch(pitch);

  const amr_core::LidarSpec spec = HeavyLidarSpec();
  const double increment =
    (spec.max_angle - spec.min_angle) / static_cast<double>(spec.samples - 1);

  LidarFrame frame = HealthyScan(5.0);
  std::size_t floor_beams = 0;
  for (int i = 0; i < spec.samples; ++i) {
    const double bearing = spec.min_angle + i * increment;
    const double range = validator.GroundReturnRangeAt(bearing);
    if (std::isfinite(range) && range < spec.range_max) {
      frame.ranges[i] = static_cast<float>(range);
      ++floor_beams;
    }
  }
  ASSERT_GT(floor_beams, 50u) << "expected a forward arc of floor returns";

  // One genuine obstacle dead ahead, much closer than the floor line.
  const int ahead = spec.samples / 2;
  frame.ranges[ahead] = 1.0f;

  std::vector<float> out;
  const ValidationResult result = validator.Validate(frame, &out);

  EXPECT_TRUE(result.ShouldForward());
  EXPECT_GT(validator.LastGroundReturnsSuppressed(), 40u);
  EXPECT_NEAR(out[ahead], 1.0, 1e-6) << "a real obstacle on the ramp must survive";

  // Beam 0 is at angle_min, i.e. directly behind: aimed above the horizon, so
  // its 5 m return is a wall and must be left alone.
  EXPECT_NEAR(out[0], 5.0, 1e-6)
    << "a rear beam cannot see the floor; suppressing it erases the far field";
}


TEST(LidarValidatorTest, TheFarFieldSurvivesWhilePitched) {
  // The failure this exists for: the old rule discarded every return beyond
  // the forward ground range, in every direction. On a 6 degree ramp that
  // deleted everything past 4.31 m across the whole scan -- the far walls, the
  // deck ahead, the guard rails -- and the costmap filled with fan-shaped
  // voids where the raytracer then cleared the space.
  //
  // A wall at 15 m is not floor. It is far away, which is a different thing.
  LidarValidator validator(HeavyLidarSpec(), {});
  validator.SetPitch(6.0 * M_PI / 180.0);

  const float wall = 15.0f;
  std::vector<float> out;
  const ValidationResult result = validator.Validate(HealthyScan(wall), &out);

  ASSERT_TRUE(result.ShouldForward());
  for (std::size_t i = 0; i < out.size(); ++i) {
    EXPECT_NEAR(out[i], wall, 1e-6)
      << "beam " << i << " was a 15 m wall and got deleted as floor";
  }
  EXPECT_EQ(validator.LastGroundReturnsSuppressed(), 0u);
}


TEST(LidarValidatorTest, OnlyBeamsAimedAtTheFloorHaveAGroundRange) {
  LidarValidator validator(HeavyLidarSpec(), {});
  validator.SetPitch(6.0 * M_PI / 180.0);

  EXPECT_TRUE(std::isfinite(validator.GroundReturnRangeAt(0.0)))
    << "straight ahead the beam is tilted into the floor";
  EXPECT_TRUE(std::isinf(validator.GroundReturnRangeAt(M_PI / 2.0)))
    << "a sideways beam stays level; it never meets the floor";
  EXPECT_TRUE(std::isinf(validator.GroundReturnRangeAt(M_PI)))
    << "a rear beam is aimed above the horizon";

  // And the range grows as the beam swings away from straight ahead.
  EXPECT_GT(validator.GroundReturnRangeAt(1.0), validator.GroundReturnRangeAt(0.0));
}


TEST(LidarValidatorTest, GroundRejectionCanBeDisabled) {
  LidarValidator::Options options;
  options.ground_rejection_enabled = false;
  LidarValidator validator(HeavyLidarSpec(), options);
  validator.SetPitch(0.5);

  std::vector<float> out;
  validator.Validate(HealthyScan(3.0), &out);
  EXPECT_EQ(validator.LastGroundReturnsSuppressed(), 0u);
  EXPECT_NEAR(out[0], 3.0, 1e-6);
}

TEST(LidarValidatorTest, ShallowPitchDoesNotTriggerRejection) {
  // Ordinary suspension pitch under acceleration must not start deleting
  // distant returns.
  LidarValidator validator(HeavyLidarSpec(), {});
  validator.SetPitch(0.02);

  std::vector<float> out;
  validator.Validate(HealthyScan(20.0), &out);
  EXPECT_EQ(validator.LastGroundReturnsSuppressed(), 0u);
}

// ---------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------

TEST(ImuValidatorTest, HealthySamplePasses) {
  ImuValidator validator(HeavyImuSpec(), {});
  const ValidationResult result = validator.Validate(HealthyImu());
  EXPECT_EQ(result.status, ValidationStatus::kValid);
  EXPECT_TRUE(result.faults.empty());
}

TEST(ImuValidatorTest, ImplausibleAngularVelocityIsFlagged) {
  // The check the brief calls out by name. heavy_mapper commands at most
  // 0.90 rad/s and is allowed up to 2.50 before the reading is called a fault.
  ImuValidator validator(HeavyImuSpec(), {});
  ImuFrame frame = HealthyImu();
  frame.angular_velocity[2] = 40.0;

  const ValidationResult result = validator.Validate(frame);
  EXPECT_TRUE(HasFault(result, "angular_velocity"));
  EXPECT_EQ(result.status, ValidationStatus::kDegraded);
  EXPECT_TRUE(result.ShouldForward())
    << "dropping IMU data mid-manoeuvre would destabilise the localisation "
    "that has to recover from the fault";
  EXPECT_NEAR(validator.PeakAngularVelocity(), 40.0, 1e-9);
}

TEST(ImuValidatorTest, TheLimitIsPerModel) {
  // The same 3.0 rad/s reading is a fault on the heavy mapper and normal on
  // the scout, which is the whole point of a per-model envelope.
  ImuFrame frame = HealthyImu();
  frame.angular_velocity[2] = 3.0;

  ImuValidator heavy(HeavyImuSpec(), {});
  EXPECT_TRUE(HasFault(heavy.Validate(frame), "angular_velocity"));

  amr_core::ImuSpec scout_spec = HeavyImuSpec();
  scout_spec.max_angular_velocity = 4.50;
  ImuValidator scout(scout_spec, {});
  EXPECT_FALSE(HasFault(scout.Validate(frame), "angular_velocity"));
}

TEST(ImuValidatorTest, TheLimitUsesTheVectorMagnitudeNotJustYaw) {
  // A robot tipping while turning can exceed the envelope without any single
  // axis doing so.
  ImuValidator validator(HeavyImuSpec(), {});
  ImuFrame frame = HealthyImu();
  frame.angular_velocity[0] = 1.6;
  frame.angular_velocity[1] = 1.6;
  frame.angular_velocity[2] = 1.6;   // magnitude 2.77 > 2.50

  EXPECT_TRUE(HasFault(validator.Validate(frame), "angular_velocity"));
}

TEST(ImuValidatorTest, NonFiniteDataIsAHardFault) {
  ImuValidator validator(HeavyImuSpec(), {});
  ImuFrame frame = HealthyImu();
  frame.angular_velocity[1] = std::numeric_limits<double>::quiet_NaN();

  const ValidationResult result = validator.Validate(frame);
  EXPECT_EQ(result.status, ValidationStatus::kInvalid);
  EXPECT_FALSE(result.ShouldForward())
    << "a NaN would propagate through the whole TF tree";
}

TEST(ImuValidatorTest, UninitialisedOrientationIsAHardFault) {
  ImuValidator validator(HeavyImuSpec(), {});
  ImuFrame frame = HealthyImu();
  frame.orientation[3] = 0.0;   // all-zero quaternion

  const ValidationResult result = validator.Validate(frame);
  EXPECT_EQ(result.status, ValidationStatus::kInvalid);
  EXPECT_TRUE(HasFault(result, "orientation_not_normalised"));
}

TEST(ImuValidatorTest, GravityAloneIsNotAnAccelerationFault) {
  // The reported vector includes gravity; a stationary robot reads 9.81. A
  // validator that forgets this fires on every single message.
  ImuValidator validator(HeavyImuSpec(), {});
  const ValidationResult result = validator.Validate(HealthyImu());
  EXPECT_FALSE(HasFault(result, "linear_acceleration"));
}

TEST(ImuValidatorTest, ImplausibleAccelerationIsFlagged) {
  ImuValidator validator(HeavyImuSpec(), {});
  ImuFrame frame = HealthyImu();
  frame.linear_acceleration[0] = 60.0;

  const ValidationResult result = validator.Validate(frame);
  EXPECT_TRUE(HasFault(result, "linear_acceleration"));
  EXPECT_EQ(result.status, ValidationStatus::kDegraded);
}

TEST(ImuValidatorTest, PitchIsExtractedForTheLidarValidator) {
  // The coupling that makes ramps navigable: attitude from the IMU is what
  // lets the LiDAR validator recognise a floor return.
  ImuValidator validator(HeavyImuSpec(), {});
  ImuFrame frame = HealthyImu();
  const double pitch = 0.15;
  frame.orientation[1] = std::sin(pitch / 2.0);
  frame.orientation[3] = std::cos(pitch / 2.0);

  validator.Validate(frame);
  EXPECT_NEAR(validator.Pitch(), pitch, 1e-9);
}

TEST(ImuValidatorTest, PitchIsNotUpdatedFromARejectedSample) {
  ImuValidator validator(HeavyImuSpec(), {});
  ImuFrame good = HealthyImu();
  good.orientation[1] = std::sin(0.1 / 2.0);
  good.orientation[3] = std::cos(0.1 / 2.0);
  validator.Validate(good);
  ASSERT_NEAR(validator.Pitch(), 0.1, 1e-9);

  ImuFrame bad = HealthyImu();
  bad.orientation[0] = bad.orientation[1] = bad.orientation[2] = bad.orientation[3] = 0.0;
  validator.Validate(bad);
  EXPECT_NEAR(validator.Pitch(), 0.1, 1e-9)
    << "a malformed quaternion must not overwrite a good attitude estimate";
}

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------

CameraFrame MakeFrame(std::vector<std::uint8_t> * storage, std::uint8_t fill)
{
  const amr_core::CameraSpec spec = HeavyCameraSpec();
  storage->assign(
    static_cast<std::size_t>(spec.width) * static_cast<std::size_t>(spec.height_px) * 3, fill);

  CameraFrame frame;
  frame.stamp = 100.0;
  frame.width = static_cast<std::uint32_t>(spec.width);
  frame.height = static_cast<std::uint32_t>(spec.height_px);
  frame.encoding = "rgb8";
  frame.data = storage->data();
  frame.data_size = storage->size();
  frame.channels = 3;
  return frame;
}

TEST(CameraValidatorTest, HealthyFramePasses) {
  CameraValidator validator(HeavyCameraSpec(), {});
  std::vector<std::uint8_t> storage;
  const ValidationResult result = validator.Validate(MakeFrame(&storage, 128));

  EXPECT_EQ(result.status, ValidationStatus::kValid);
  EXPECT_NEAR(validator.LastMeanIntensity(), 128.0, 1e-9);
}

TEST(CameraValidatorTest, ABlackedOutImagerIsCaught) {
  // The failure mode a metadata check cannot see: perfectly formed frames, at
  // the right rate, containing nothing.
  CameraValidator validator(HeavyCameraSpec(), {});
  std::vector<std::uint8_t> storage;
  const ValidationResult result = validator.Validate(MakeFrame(&storage, 1));

  EXPECT_TRUE(HasFault(result, "image_black"));
  EXPECT_EQ(result.status, ValidationStatus::kDegraded);
}

TEST(CameraValidatorTest, ASaturatedImagerIsCaught) {
  CameraValidator validator(HeavyCameraSpec(), {});
  std::vector<std::uint8_t> storage;
  const ValidationResult result = validator.Validate(MakeFrame(&storage, 255));

  EXPECT_TRUE(HasFault(result, "image_saturated"));
  EXPECT_EQ(result.status, ValidationStatus::kDegraded);
}

TEST(CameraValidatorTest, WrongResolutionIsAHardFault) {
  CameraValidator validator(HeavyCameraSpec(), {});
  std::vector<std::uint8_t> storage;
  CameraFrame frame = MakeFrame(&storage, 128);
  frame.width = 320;

  const ValidationResult result = validator.Validate(frame);
  EXPECT_EQ(result.status, ValidationStatus::kInvalid);
  EXPECT_TRUE(HasFault(result, "resolution"));
}

TEST(CameraValidatorTest, UnsupportedEncodingIsAHardFault) {
  CameraValidator validator(HeavyCameraSpec(), {});
  std::vector<std::uint8_t> storage;
  CameraFrame frame = MakeFrame(&storage, 128);
  frame.encoding = "16UC1";

  EXPECT_EQ(validator.Validate(frame).status, ValidationStatus::kInvalid);
}

TEST(CameraValidatorTest, TruncatedBufferIsAHardFault) {
  CameraValidator validator(HeavyCameraSpec(), {});
  std::vector<std::uint8_t> storage;
  CameraFrame frame = MakeFrame(&storage, 128);
  frame.data_size /= 2;

  const ValidationResult result = validator.Validate(frame);
  EXPECT_EQ(result.status, ValidationStatus::kInvalid);
  EXPECT_TRUE(HasFault(result, "buffer_size"));
}

TEST(CameraValidatorTest, ZeroSizedImageIsAHardFault) {
  CameraValidator validator(HeavyCameraSpec(), {});
  CameraFrame frame;
  frame.stamp = 100.0;
  const ValidationResult result = validator.Validate(frame);
  EXPECT_EQ(result.status, ValidationStatus::kInvalid);
  EXPECT_TRUE(HasFault(result, "empty_image"));
}

TEST(CameraValidatorTest, IntensityCheckingCanBeDisabled) {
  CameraValidator::Options options;
  options.check_intensity = false;
  CameraValidator validator(HeavyCameraSpec(), options);
  std::vector<std::uint8_t> storage;

  EXPECT_EQ(validator.Validate(MakeFrame(&storage, 0)).status, ValidationStatus::kValid);
  EXPECT_DOUBLE_EQ(validator.LastMeanIntensity(), -1.0);
}

}  // namespace

// ---------------------------------------------------------------------------
// Ground rejection is directional
// ---------------------------------------------------------------------------

TEST(LidarValidatorTest, ClimbingARampDoesNotDeleteTheFarField) {
  // The bug this exists for: GroundReturnRange() tested std::abs(pitch), so a
  // robot pitched NOSE-UP -- beam aimed above the floor, which it can never
  // reach -- still had every return past h/sin(pitch) discarded as "floor".
  // On a 6 deg ramp that removed 83% of a 25 m scanner while the robot was on
  // the ramp: the far walls, the deck ahead and the guard rails all left the
  // costmap at the moment they mattered most.
  LidarValidator validator(HeavyLidarSpec(), {});
  validator.SetPitch(-6.0 * M_PI / 180.0);          // nose up, REP-103

  EXPECT_TRUE(std::isinf(validator.GroundReturnRange()))
    << "there is no ground to reject when the beam points above it";

  const float far_wall = 18.0f;
  std::vector<float> out;
  const ValidationResult result = validator.Validate(HealthyScan(far_wall), &out);

  ASSERT_TRUE(result.ShouldForward());
  EXPECT_EQ(validator.LastGroundReturnsSuppressed(), 0u);
  EXPECT_NEAR(out[0], far_wall, 1e-6);
  EXPECT_NEAR(out[out.size() / 2], far_wall, 1e-6);
}

TEST(LidarValidatorTest, DescendingARampStillSuppressesTheFloor) {
  // The other direction has to keep working, or the ramps close up again --
  // but now only for the beams that can actually see the floor.
  LidarValidator validator(HeavyLidarSpec(), {});
  const double descending = 6.0 * M_PI / 180.0;
  validator.SetPitch(descending);

  const amr_core::LidarSpec spec = HeavyLidarSpec();
  EXPECT_NEAR(validator.GroundReturnRangeAt(0.0),
              spec.height / std::sin(descending), 1e-6);

  const double increment =
    (spec.max_angle - spec.min_angle) / static_cast<double>(spec.samples - 1);
  LidarFrame frame = HealthyScan(2.0);
  for (int i = 0; i < spec.samples; ++i) {
    const double bearing = spec.min_angle + i * increment;
    const double range = validator.GroundReturnRangeAt(bearing);
    if (std::isfinite(range) && range < spec.range_max) {
      frame.ranges[i] = static_cast<float>(range);
    }
  }

  std::vector<float> out;
  ASSERT_TRUE(validator.Validate(frame, &out).ShouldForward());
  EXPECT_GT(validator.LastGroundReturnsSuppressed(), 40u)
    << "the forward arc of floor returns must still be removed";
}


TEST(LidarValidatorTest, ARealObstacleSurvivesClimbingAndDescending) {
  for (int sign = -1; sign <= 1; sign += 2) {
    LidarValidator validator(HeavyLidarSpec(), {});
    validator.SetPitch(sign * 9.0 * M_PI / 180.0);

    LidarFrame frame = HealthyScan(1.5);
    std::vector<float> out;
    ASSERT_TRUE(validator.Validate(frame, &out).ShouldForward());
    EXPECT_NEAR(out[0], 1.5, 1e-6)
      << "an obstacle at 1.5 m must survive whichever way the robot is tilted";
  }
}
