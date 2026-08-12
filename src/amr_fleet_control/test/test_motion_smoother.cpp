// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// The smoother is the only thing standing between nav2's step-shaped velocity
// commands and the drive train, so these tests check the guarantees a
// downstream integrator would want to rely on: bounded acceleration, bounded
// jerk, monotone response to payload, and no pathological behaviour when the
// control loop misses a tick.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "amr_fleet_control/motion_smoother.hpp"
#include "test_fixtures.hpp"

namespace
{

using amr_core::Velocity2D;
using amr_fleet_control::DynamicState;
using amr_fleet_control::LimitReason;
using amr_fleet_control::MotionSmoother;
using amr_fleet_control::test::HeavyMapperProfile;
using amr_fleet_control::test::LightScoutProfile;

constexpr double kDt = 0.05;   // 20 Hz control loop.

/// \brief One smoothing run, recorded per tick.
struct Trace
{
  std::vector<double> velocity;
  std::vector<double> acceleration;
  std::vector<double> jerk;

  double MaxAbsAccel() const
  {
    double worst = 0.0;
    for (double a : acceleration) {
      worst = std::max(worst, std::abs(a));
    }
    return worst;
  }
  double MaxAbsJerk() const
  {
    double worst = 0.0;
    for (double j : jerk) {
      worst = std::max(worst, std::abs(j));
    }
    return worst;
  }
};

/// \brief Drive the smoother with a constant target and record the trace.
///
/// Named `Drive`, not `Run`: a `TEST()` body is a member function of a class
/// derived from `testing::Test`, which declares a private `Run()`. Unqualified
/// lookup finds that member first and never reaches namespace scope, so a
/// helper called `Run` fails to compile with a misleading "is private within
/// this context" error.
Trace Drive(
  MotionSmoother * smoother, const Velocity2D & target, const DynamicState & state, int steps,
  double dt = kDt)
{
  Trace trace;
  for (int i = 0; i < steps; ++i) {
    const Velocity2D command = smoother->Smooth(target, state, dt);
    trace.velocity.push_back(command.linear_x);
    trace.acceleration.push_back(smoother->Diagnostics().linear.acceleration);
    trace.jerk.push_back(smoother->Diagnostics().linear.jerk);
  }
  return trace;
}

// ---------------------------------------------------------------------------
// Core guarantees
// ---------------------------------------------------------------------------

TEST(MotionSmootherTest, StepInputDoesNotProduceAStepOutput) {
  MotionSmoother smoother(HeavyMapperProfile());
  const Velocity2D target{0.75, 0.0};
  const Trace trace = Drive(&smoother, target, DynamicState{}, 1);

  // A raw pass-through would emit 0.75 immediately. The whole point of the
  // node is that it does not.
  EXPECT_LT(trace.velocity.front(), 0.05);
  EXPECT_GT(trace.velocity.front(), 0.0);
}

TEST(MotionSmootherTest, NeverExceedsTheAccelerationLimit) {
  const auto profile = HeavyMapperProfile();
  MotionSmoother smoother(profile);
  const Trace trace = Drive(&smoother, Velocity2D{0.75, 0.0}, DynamicState{}, 120);

  // The limit shrinks with speed, so the unloaded, zero-speed value is the
  // loosest bound that must still hold everywhere.
  const double ceiling = profile.limits.EffectiveAccelX(0.0, 0.0);
  EXPECT_LE(trace.MaxAbsAccel(), ceiling + 1e-9);
}

TEST(MotionSmootherTest, NeverExceedsTheJerkLimit) {
  const auto profile = HeavyMapperProfile();
  MotionSmoother smoother(profile);
  const Trace trace = Drive(&smoother, Velocity2D{0.75, 0.0}, DynamicState{}, 120);

  const double ceiling = profile.limits.EffectiveJerkX(0.0, 0.0);
  EXPECT_LE(trace.MaxAbsJerk(), ceiling + 1e-9);
}

TEST(MotionSmootherTest, JerkIsBoundedThroughAnAbruptReversal) {
  // The hardest case for a jerk limiter: full forward, then full reverse with
  // no warning. Nav2 does exactly this when a recovery behaviour kicks in.
  const auto profile = LightScoutProfile();
  MotionSmoother smoother(profile);
  Drive(&smoother, Velocity2D{1.4, 0.0}, DynamicState{}, 200);

  const Trace reversal = Drive(&smoother, Velocity2D{-0.5, 0.0}, DynamicState{}, 200);
  EXPECT_LE(reversal.MaxAbsJerk(), profile.limits.EffectiveJerkX(0.0, 0.0) + 1e-9);
  EXPECT_LE(reversal.MaxAbsAccel(), profile.limits.max_decel_x + 1e-9);
}

TEST(MotionSmootherTest, ConvergesToTheTarget) {
  MotionSmoother smoother(HeavyMapperProfile());
  const Trace trace = Drive(&smoother, Velocity2D{0.5, 0.3}, DynamicState{}, 400);
  EXPECT_NEAR(trace.velocity.back(), 0.5, 1e-3);
  EXPECT_NEAR(smoother.Command().angular_z, 0.3, 1e-3);
}

TEST(MotionSmootherTest, ActuallyReachesZeroRatherThanCreeping) {
  MotionSmoother smoother(LightScoutProfile());
  Drive(&smoother, Velocity2D{1.0, 0.5}, DynamicState{}, 200);
  Drive(&smoother, Velocity2D{0.0, 0.0}, DynamicState{}, 400);

  EXPECT_DOUBLE_EQ(smoother.Command().linear_x, 0.0);
  EXPECT_DOUBLE_EQ(smoother.Command().angular_z, 0.0);
}

TEST(MotionSmootherTest, ClampsTargetsBeyondTheVelocityEnvelope) {
  const auto profile = LightScoutProfile();
  MotionSmoother smoother(profile);
  const Trace trace = Drive(&smoother, Velocity2D{9.0, 0.0}, DynamicState{}, 500);
  EXPECT_NEAR(trace.velocity.back(), profile.limits.max_vel_x, 1e-6);
  EXPECT_LE(
    *std::max_element(trace.velocity.begin(), trace.velocity.end()),
    profile.limits.max_vel_x + 1e-9);
}

TEST(MotionSmootherTest, ReverseSpeedIsClampedToItsOwnLimit) {
  const auto profile = HeavyMapperProfile();
  MotionSmoother smoother(profile);
  const Trace trace = Drive(&smoother, Velocity2D{-5.0, 0.0}, DynamicState{}, 500);
  EXPECT_NEAR(trace.velocity.back(), profile.limits.min_vel_x, 1e-6);
}

// ---------------------------------------------------------------------------
// Dynamic-state dependence: the assignment's explicit requirement
// ---------------------------------------------------------------------------

TEST(MotionSmootherTest, PayloadSlowsTheAccelerationRamp) {
  MotionSmoother empty(HeavyMapperProfile());
  MotionSmoother loaded(HeavyMapperProfile());

  DynamicState no_load;
  DynamicState full_load;
  full_load.load_ratio = 1.0;

  const Trace empty_trace = Drive(&empty, Velocity2D{0.7, 0.0}, no_load, 60);
  const Trace loaded_trace = Drive(&loaded, Velocity2D{0.7, 0.0}, full_load, 60);

  EXPECT_GT(empty_trace.velocity.back(), loaded_trace.velocity.back())
    << "a fully loaded robot must not reach speed as quickly as an empty one";
  EXPECT_GT(empty_trace.MaxAbsAccel(), loaded_trace.MaxAbsAccel());
}

TEST(MotionSmootherTest, HeavyUnitRampsSlowerThanTheScout) {
  // The headline heterogeneity requirement, measured end to end rather than
  // read off the configuration.
  MotionSmoother heavy(HeavyMapperProfile());
  MotionSmoother scout(LightScoutProfile());

  const Trace heavy_trace = Drive(&heavy, Velocity2D{0.7, 0.0}, DynamicState{}, 40);
  const Trace scout_trace = Drive(&scout, Velocity2D{0.7, 0.0}, DynamicState{}, 40);

  EXPECT_LT(heavy_trace.velocity.back(), scout_trace.velocity.back());
  EXPECT_LT(heavy_trace.MaxAbsAccel(), scout_trace.MaxAbsAccel());
  EXPECT_LT(heavy_trace.MaxAbsJerk(), scout_trace.MaxAbsJerk());
}

TEST(MotionSmootherTest, AccelerationTapersAsSpeedRises) {
  const auto profile = LightScoutProfile();
  MotionSmoother smoother(profile);
  const Trace trace = Drive(&smoother, Velocity2D{1.4, 0.0}, DynamicState{}, 300);

  // Find the peak acceleration and the acceleration once nearly up to speed.
  const double peak = trace.MaxAbsAccel();
  double near_top = 0.0;
  for (std::size_t i = 0; i < trace.velocity.size(); ++i) {
    if (trace.velocity[i] > 0.95 * profile.limits.max_vel_x) {
      near_top = std::abs(trace.acceleration[i]);
      break;
    }
  }
  EXPECT_LT(near_top, peak) << "speed derating should be visible in the trace";
}

// ---------------------------------------------------------------------------
// Traffic integration: a yield must be a *controlled* stop
// ---------------------------------------------------------------------------

TEST(MotionSmootherTest, YieldScaleProducesAJerkLimitedStopNotACut) {
  const auto profile = LightScoutProfile();
  MotionSmoother smoother(profile);
  Drive(&smoother, Velocity2D{1.2, 0.0}, DynamicState{}, 300);
  const double cruise = smoother.Command().linear_x;
  EXPECT_GT(cruise, 1.0);

  DynamicState yielding;
  yielding.speed_scale = 0.0;              // Traffic controller says stop.
  const Trace stop = Drive(&smoother, Velocity2D{1.2, 0.0}, yielding, 200);

  EXPECT_DOUBLE_EQ(smoother.Command().linear_x, 0.0) << "the yield must complete";
  EXPECT_LE(stop.MaxAbsJerk(), profile.limits.EffectiveJerkX(0.0, 0.0) + 1e-9)
    << "a yield that violates the jerk limit is a cut, not a controlled stop";
  EXPECT_LE(stop.MaxAbsAccel(), profile.limits.max_decel_x + 1e-9);
  EXPECT_GT(stop.velocity.size(), 5u);
}

TEST(MotionSmootherTest, SlowScaleSettlesAtTheScaledSpeed) {
  MotionSmoother smoother(LightScoutProfile());
  DynamicState slow;
  slow.speed_scale = 0.35;
  const Trace trace = Drive(&smoother, Velocity2D{1.0, 0.0}, slow, 400);
  EXPECT_NEAR(trace.velocity.back(), 0.35, 1e-3);
}

// ---------------------------------------------------------------------------
// Robustness
// ---------------------------------------------------------------------------

TEST(MotionSmootherTest, RejectsNonPositiveAndOversizedTimeSteps) {
  MotionSmoother smoother(HeavyMapperProfile());
  Drive(&smoother, Velocity2D{0.4, 0.0}, DynamicState{}, 50);
  const double before = smoother.Command().linear_x;

  EXPECT_DOUBLE_EQ(smoother.Smooth(Velocity2D{0.75, 0.0}, DynamicState{}, 0.0).linear_x, before);
  EXPECT_DOUBLE_EQ(smoother.Smooth(Velocity2D{0.75, 0.0}, DynamicState{}, -0.1).linear_x, before);
  // A five-second gap means the loop stalled. Honouring it would authorise a
  // velocity step of accel_limit * 5, which no actuator could deliver.
  EXPECT_DOUBLE_EQ(smoother.Smooth(Velocity2D{0.75, 0.0}, DynamicState{}, 5.0).linear_x, before);
  EXPECT_TRUE(smoother.Diagnostics().timed_out);
}

TEST(MotionSmootherTest, ResetClearsAccelerationHistory) {
  MotionSmoother smoother(LightScoutProfile());
  Drive(&smoother, Velocity2D{1.2, 0.0}, DynamicState{}, 100);
  EXPECT_GT(smoother.Command().linear_x, 0.0);

  smoother.Reset();
  EXPECT_DOUBLE_EQ(smoother.Command().linear_x, 0.0);
  // After a reset the first step must be jerk-limited from zero, not from the
  // stale acceleration.
  const Trace trace = Drive(&smoother, Velocity2D{1.2, 0.0}, DynamicState{}, 1);
  EXPECT_LE(
    std::abs(trace.jerk.front()),
    LightScoutProfile().limits.EffectiveJerkX(0.0, 0.0) + 1e-9);
}

TEST(MotionSmootherTest, SmoothToStopBringsTheRobotDownUnderNormalLimits) {
  const auto profile = HeavyMapperProfile();
  MotionSmoother smoother(profile);
  Drive(&smoother, Velocity2D{0.7, 0.4}, DynamicState{}, 300);

  double worst_jerk = 0.0;
  for (int i = 0; i < 300; ++i) {
    smoother.SmoothToStop(DynamicState{}, kDt);
    worst_jerk = std::max(worst_jerk, std::abs(smoother.Diagnostics().linear.jerk));
  }
  EXPECT_DOUBLE_EQ(smoother.Command().linear_x, 0.0);
  EXPECT_DOUBLE_EQ(smoother.Command().angular_z, 0.0);
  EXPECT_LE(worst_jerk, profile.limits.EffectiveJerkX(0.0, 0.0) + 1e-9);
}

TEST(MotionSmootherTest, DiagnosticsIdentifyTheBindingConstraint) {
  MotionSmoother smoother(HeavyMapperProfile());
  // First tick from rest against a large step: jerk is the binding limit.
  smoother.Smooth(Velocity2D{0.75, 0.0}, DynamicState{}, kDt);
  EXPECT_EQ(smoother.Diagnostics().linear.reason, LimitReason::kJerk);

  // Once settled at the target, nothing binds.
  Drive(&smoother, Velocity2D{0.3, 0.0}, DynamicState{}, 500);
  smoother.Smooth(Velocity2D{0.3, 0.0}, DynamicState{}, kDt);
  EXPECT_EQ(smoother.Diagnostics().linear.reason, LimitReason::kNone);
}

TEST(MotionSmootherTest, AngularAxisIsShapedIndependently) {
  const auto profile = LightScoutProfile();
  MotionSmoother smoother(profile);

  double worst_angular_jerk = 0.0;
  for (int i = 0; i < 200; ++i) {
    smoother.Smooth(Velocity2D{0.0, 1.9}, DynamicState{}, kDt);
    worst_angular_jerk =
      std::max(worst_angular_jerk, std::abs(smoother.Diagnostics().angular.jerk));
  }
  EXPECT_NEAR(smoother.Command().angular_z, profile.limits.max_vel_theta, 1e-3);
  EXPECT_DOUBLE_EQ(smoother.Command().linear_x, 0.0);
  EXPECT_LE(worst_angular_jerk, profile.limits.EffectiveJerkTheta(0.0, 0.0) + 1e-9);
}

}  // namespace
