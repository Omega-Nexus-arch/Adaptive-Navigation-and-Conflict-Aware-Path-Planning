// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Unit tests for the dynamic-envelope arithmetic. These matter more than they
// look: every acceleration and jerk bound the motion smoother applies comes
// out of these six functions, and a sign or ordering error here would show up
// in simulation only as "the heavy robot feels a bit twitchy".

#include <gtest/gtest.h>

#include "amr_core/robot_model.hpp"

namespace
{

using amr_core::DynamicLimits;
using amr_core::RobotProfile;
using amr_core::RobotRole;
using amr_core::SafetySpec;

/// Mirrors the `heavy_mapper` block of robot_models.yaml.
DynamicLimits HeavyLimits()
{
  DynamicLimits limits;
  limits.max_vel_x = 0.75;
  limits.max_accel_x = 0.35;
  limits.max_decel_x = 0.70;
  limits.max_accel_theta = 0.80;
  limits.max_jerk_x = 0.60;
  limits.max_jerk_theta = 1.20;
  limits.payload_derating = 0.55;
  limits.speed_derating = 0.30;
  return limits;
}

/// Mirrors the `light_scout` block.
DynamicLimits ScoutLimits()
{
  DynamicLimits limits;
  limits.max_vel_x = 1.40;
  limits.max_accel_x = 1.10;
  limits.max_decel_x = 1.60;
  limits.max_accel_theta = 2.40;
  limits.max_jerk_x = 2.50;
  limits.max_jerk_theta = 4.00;
  limits.payload_derating = 0.30;
  limits.speed_derating = 0.20;
  return limits;
}

// ---------------------------------------------------------------------------
// The headline requirement: the heavy unit must be the sluggish one.
// ---------------------------------------------------------------------------

TEST(DynamicLimits, HeavyUnitIsLessAgileThanScoutInEveryState) {
  const DynamicLimits heavy = HeavyLimits();
  const DynamicLimits scout = ScoutLimits();

  for (double load = 0.0; load <= 1.0; load += 0.25) {
    for (double speed = 0.0; speed <= 1.0; speed += 0.25) {
      EXPECT_LT(heavy.EffectiveAccelX(load, speed), scout.EffectiveAccelX(load, speed))
        << "load=" << load << " speed=" << speed;
      EXPECT_LT(heavy.EffectiveJerkX(load, speed), scout.EffectiveJerkX(load, speed))
        << "load=" << load << " speed=" << speed;
      EXPECT_LT(
        heavy.EffectiveAccelTheta(load, speed), scout.EffectiveAccelTheta(load, speed))
        << "load=" << load << " speed=" << speed;
    }
  }
}

TEST(DynamicLimits, PayloadReducesAcceleration) {
  const DynamicLimits limits = HeavyLimits();
  const double empty = limits.EffectiveAccelX(0.0, 0.0);
  const double half = limits.EffectiveAccelX(0.5, 0.0);
  const double full = limits.EffectiveAccelX(1.0, 0.0);

  EXPECT_DOUBLE_EQ(empty, 0.35);
  EXPECT_LT(half, empty);
  EXPECT_LT(full, half);
  // 0.35 * (1 - 0.55) = 0.1575
  EXPECT_NEAR(full, 0.1575, 1e-9);
}

TEST(DynamicLimits, SpeedReducesAcceleration) {
  const DynamicLimits limits = ScoutLimits();
  EXPECT_GT(limits.EffectiveAccelX(0.0, 0.0), limits.EffectiveAccelX(0.0, 1.0));
  // 1.10 * (1 - 0.20) = 0.88
  EXPECT_NEAR(limits.EffectiveAccelX(0.0, 1.0), 0.88, 1e-9);
}

TEST(DynamicLimits, BrakingAuthorityIsNotDeratedBySpeed) {
  // Losing stopping power at speed would invert the safety envelope: the
  // faster the robot travels, the longer it would take to stop, on top of the
  // v^2 term. The decel accessor must ignore its speed argument.
  const DynamicLimits limits = HeavyLimits();
  EXPECT_DOUBLE_EQ(limits.EffectiveDecelX(0.0, 0.0), limits.EffectiveDecelX(0.0, 1.0));
  EXPECT_DOUBLE_EQ(limits.EffectiveDecelX(0.5, 0.0), limits.EffectiveDecelX(0.5, 1.0));
}

TEST(DynamicLimits, BrakingAuthorityIsStillDeratedByPayload) {
  const DynamicLimits limits = HeavyLimits();
  EXPECT_LT(limits.EffectiveDecelX(1.0, 0.0), limits.EffectiveDecelX(0.0, 0.0));
}

TEST(DynamicLimits, LimitsNeverCollapseToZero) {
  DynamicLimits limits = HeavyLimits();
  limits.payload_derating = 0.99;
  limits.speed_derating = 0.99;
  // A zero limit would freeze the robot for good; the floor keeps it creeping.
  EXPECT_GT(limits.EffectiveAccelX(1.0, 1.0), 0.0);
  EXPECT_GE(limits.DeratingFactor(1.0, 1.0), 0.05);
}

TEST(DynamicLimits, RatiosAreClampedAgainstOutOfRangeInput) {
  const DynamicLimits limits = HeavyLimits();
  // Over-loaded or over-speed states must saturate, not extrapolate into
  // negative limits.
  EXPECT_DOUBLE_EQ(limits.EffectiveAccelX(5.0, 0.0), limits.EffectiveAccelX(1.0, 0.0));
  EXPECT_DOUBLE_EQ(limits.EffectiveAccelX(-3.0, 0.0), limits.EffectiveAccelX(0.0, 0.0));
  EXPECT_GT(limits.EffectiveAccelX(5.0, 5.0), 0.0);
}

// ---------------------------------------------------------------------------
// Safety envelope: d_safe = k * v^2 + d_min
// ---------------------------------------------------------------------------

TEST(SafetySpec, SafeDistanceFollowsTheQuadraticLaw) {
  SafetySpec spec;
  spec.k = 0.85;
  spec.d_min = 0.45;

  EXPECT_NEAR(spec.SafeDistance(0.0), 0.45, 1e-12);
  EXPECT_NEAR(spec.SafeDistance(1.0), 1.30, 1e-12);
  EXPECT_NEAR(spec.SafeDistance(0.5), 0.45 + 0.85 * 0.25, 1e-12);
}

TEST(SafetySpec, SafeDistanceIsSymmetricInDirectionOfTravel) {
  // Reversing at 0.4 m/s is exactly as dangerous as advancing at 0.4 m/s.
  SafetySpec spec;
  EXPECT_DOUBLE_EQ(spec.SafeDistance(-0.4), spec.SafeDistance(0.4));
}

TEST(SafetySpec, ReleaseDistanceAlwaysExceedsTriggerDistance) {
  SafetySpec spec;
  spec.release_hysteresis = 0.15;
  for (double v = 0.0; v <= 2.0; v += 0.1) {
    EXPECT_GT(spec.ReleaseDistance(v), spec.SafeDistance(v))
      << "no hysteresis at v=" << v << " means the halt will chatter";
  }
}

TEST(SafetySpec, HeavierRobotDemandsMoreRoomAtEverySpeed) {
  SafetySpec heavy;
  heavy.k = 0.85;
  heavy.d_min = 0.45;
  SafetySpec scout;
  scout.k = 0.42;
  scout.d_min = 0.30;

  for (double v = 0.0; v <= 1.5; v += 0.1) {
    EXPECT_GT(heavy.SafeDistance(v), scout.SafeDistance(v)) << "at v=" << v;
  }
}

// ---------------------------------------------------------------------------
// Profile helpers
// ---------------------------------------------------------------------------

TEST(RobotProfile, LoadRatioClampsAndSurvivesZeroCapacity) {
  RobotProfile profile;
  profile.payload_capacity_kg = 120.0;
  EXPECT_DOUBLE_EQ(profile.LoadRatio(0.0), 0.0);
  EXPECT_DOUBLE_EQ(profile.LoadRatio(60.0), 0.5);
  EXPECT_DOUBLE_EQ(profile.LoadRatio(500.0), 1.0);
  EXPECT_DOUBLE_EQ(profile.LoadRatio(-10.0), 0.0);

  profile.payload_capacity_kg = 0.0;
  EXPECT_DOUBLE_EQ(profile.LoadRatio(10.0), 0.0) << "must not divide by zero";
}

TEST(RobotProfile, SpeedRatioClampsAndIgnoresDirection) {
  RobotProfile profile;
  profile.limits.max_vel_x = 1.4;
  EXPECT_NEAR(profile.SpeedRatio(0.7), 0.5, 1e-12);
  EXPECT_NEAR(profile.SpeedRatio(-0.7), 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(profile.SpeedRatio(9.9), 1.0);

  profile.limits.max_vel_x = 0.0;
  EXPECT_DOUBLE_EQ(profile.SpeedRatio(1.0), 0.0);
}

TEST(RobotRoleEnum, RoundTripsAndDegradesSafely) {
  EXPECT_EQ(amr_core::RoleFromString("mapper"), RobotRole::kMapper);
  EXPECT_EQ(amr_core::RoleFromString("scout"), RobotRole::kScout);
  // A typo must not silently become a real role.
  EXPECT_EQ(amr_core::RoleFromString("Mapper"), RobotRole::kUnknown);
  EXPECT_EQ(amr_core::RoleFromString(""), RobotRole::kUnknown);
  EXPECT_STREQ(amr_core::RoleToString(RobotRole::kScout), "scout");
  EXPECT_STREQ(amr_core::RoleToString(RobotRole::kUnknown), "unknown");
}

TEST(RobotInstance, PriorityOverrideTakesPrecedence) {
  amr_core::RobotInstance instance;
  instance.profile.yield_priority = 50;
  EXPECT_EQ(instance.YieldPriority(), 50);

  instance.priority_override = 90;
  EXPECT_EQ(instance.YieldPriority(), 90);

  instance.priority_override = 0;
  EXPECT_EQ(instance.YieldPriority(), 0) << "zero is a valid priority, not 'unset'";
}

}  // namespace
