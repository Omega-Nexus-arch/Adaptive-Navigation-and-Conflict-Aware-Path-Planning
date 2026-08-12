// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// The safety monitor is the last line of defence and the only component
// authorised to override the navigation stack, so it is tested against the
// awkward cases rather than the happy path: chatter at the release boundary,
// sensor loss, obstacles outside the guarded cone, and the speed dependence
// that makes the envelope meaningful in the first place.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "amr_fleet_control/safety_monitor.hpp"
#include "test_fixtures.hpp"

namespace
{

using amr_fleet_control::HaltReason;
using amr_fleet_control::RangeSample;
using amr_fleet_control::SafetyDecision;
using amr_fleet_control::SafetyMonitor;
using amr_fleet_control::test::HeavyMapperProfile;
using amr_fleet_control::test::LightScoutProfile;

SafetyMonitor::Options DefaultOptions()
{
  SafetyMonitor::Options options;
  options.min_hold_seconds = 0.4;
  options.sensor_timeout_seconds = 0.35;
  options.min_valid_range = 0.02;
  return options;
}

/// \brief A scan with a single return dead ahead.
std::vector<RangeSample> ObstacleAhead(double range)
{
  return {RangeSample{0.0, range}};
}

/// \brief A full sweep with everything far away.
std::vector<RangeSample> ClearScan(double range = 20.0)
{
  std::vector<RangeSample> scan;
  for (int i = -18; i <= 18; ++i) {
    scan.push_back(RangeSample{i * M_PI / 18.0, range});
  }
  return scan;
}

// ---------------------------------------------------------------------------
// The envelope itself
// ---------------------------------------------------------------------------

TEST(SafetyMonitorTest, StaysClearWhenTheObstacleIsBeyondTheEnvelope) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  // At 0.5 m/s the heavy unit needs 0.85*0.25 + 0.45 = 0.6625 m.
  const SafetyDecision decision = monitor.Evaluate(ObstacleAhead(2.0), 0.5, 1.0, 1.0, true);
  EXPECT_FALSE(decision.halt);
  EXPECT_EQ(decision.reason, HaltReason::kNone);
  EXPECT_NEAR(decision.safe_distance, 0.6625, 1e-9);
}

TEST(SafetyMonitorTest, HaltsWhenTheObstacleViolatesTheEnvelope) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  const SafetyDecision decision = monitor.Evaluate(ObstacleAhead(0.5), 0.5, 1.0, 1.0, true);
  EXPECT_TRUE(decision.halt);
  EXPECT_EQ(decision.reason, HaltReason::kObstacle);
  EXPECT_TRUE(decision.newly_engaged);
  EXPECT_NEAR(decision.min_obstacle_distance, 0.5, 1e-9);
}

TEST(SafetyMonitorTest, TheEnvelopeGrowsWithSpeed) {
  // The same obstacle at the same distance is safe when crawling and unsafe
  // when travelling fast. Without this, `d_safe` is just a constant with extra
  // arithmetic.
  //
  // Heavy unit, k = 0.85, d_min = 0.45:
  //   at 0.20 m/s  d_safe = 0.85*0.04   + 0.45 = 0.484 m
  //   at 0.75 m/s  d_safe = 0.85*0.5625 + 0.45 = 0.928 m
  // A 0.90 m obstacle therefore straddles the two.
  const double range = 0.90;

  SafetyMonitor slow(HeavyMapperProfile(), DefaultOptions());
  EXPECT_FALSE(slow.Evaluate(ObstacleAhead(range), 0.2, 1.0, 1.0, true).halt);

  SafetyMonitor fast(HeavyMapperProfile(), DefaultOptions());
  EXPECT_TRUE(fast.Evaluate(ObstacleAhead(range), 0.75, 1.0, 1.0, true).halt);
}

TEST(SafetyMonitorTest, TheHeavyUnitHaltsEarlierThanTheScout) {
  const double range = 0.75;
  const double speed = 0.7;

  SafetyMonitor heavy(HeavyMapperProfile(), DefaultOptions());
  SafetyMonitor scout(LightScoutProfile(), DefaultOptions());

  EXPECT_TRUE(heavy.Evaluate(ObstacleAhead(range), speed, 1.0, 1.0, true).halt);
  EXPECT_FALSE(scout.Evaluate(ObstacleAhead(range), speed, 1.0, 1.0, true).halt)
    << "the lighter robot stops in less room, so it may keep going here";
}

TEST(SafetyMonitorTest, ReversingIsGuardedToo) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  // Speed enters the envelope as a magnitude, so backing up at 0.5 m/s demands
  // the same clearance as advancing at 0.5 m/s.
  const SafetyDecision decision = monitor.Evaluate(ObstacleAhead(0.5), -0.5, 1.0, 1.0, true);
  EXPECT_TRUE(decision.halt);
}

// ---------------------------------------------------------------------------
// Sector filtering
// ---------------------------------------------------------------------------

TEST(SafetyMonitorTest, IgnoresObstaclesOutsideTheGuardedCone) {
  const auto profile = HeavyMapperProfile();
  SafetyMonitor monitor(profile, DefaultOptions());

  // Directly behind: a rack the robot has already driven past.
  std::vector<RangeSample> behind = {RangeSample{M_PI, 0.10}};
  EXPECT_FALSE(monitor.Evaluate(behind, 0.5, 1.0, 1.0, true).halt)
    << "guarding all 360 degrees would halt the robot in every 2.4 m aisle";

  // Just inside the cone: must be seen.
  std::vector<RangeSample> inside = {
    RangeSample{profile.safety.sector_half_angle - 0.01, 0.10}};
  EXPECT_TRUE(monitor.Evaluate(inside, 0.5, 1.0, 1.0, true).halt);
}

TEST(SafetyMonitorTest, DiscardsSelfHitsAndNonFiniteReturns) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  std::vector<RangeSample> scan = {
    RangeSample{0.0, 0.001},                                        // self-hit
    RangeSample{0.1, std::numeric_limits<double>::infinity()},      // no return
    RangeSample{-0.1, std::numeric_limits<double>::quiet_NaN()},    // dropout
    RangeSample{0.2, 5.0},                                          // real
  };
  const SafetyDecision decision = monitor.Evaluate(scan, 0.5, 1.0, 1.0, true);
  EXPECT_FALSE(decision.halt);
  EXPECT_NEAR(decision.min_obstacle_distance, 5.0, 1e-9)
    << "a 1 mm self-hit must not be mistaken for the nearest obstacle";
}

TEST(SafetyMonitorTest, ReportsInfinityForAnEmptyCone) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  EXPECT_TRUE(std::isinf(monitor.MinRangeInSector({})));
}

// ---------------------------------------------------------------------------
// Latching and hysteresis
// ---------------------------------------------------------------------------

TEST(SafetyMonitorTest, HoldsTheHaltUntilTheReleaseDistanceIsCleared) {
  const auto profile = HeavyMapperProfile();
  SafetyMonitor monitor(profile, DefaultOptions());

  ASSERT_TRUE(monitor.Evaluate(ObstacleAhead(0.4), 0.0, 1.0, 1.0, true).halt);

  // Now stationary: d_safe collapses to d_min = 0.45, release needs 0.60.
  // A range between the two must NOT release, or the robot would accelerate,
  // grow its own envelope, and halt again - the chatter loop.
  const SafetyDecision between = monitor.Evaluate(ObstacleAhead(0.50), 0.0, 2.0, 2.0, true);
  EXPECT_TRUE(between.halt);

  const SafetyDecision clear = monitor.Evaluate(ObstacleAhead(0.70), 0.0, 2.5, 2.5, true);
  EXPECT_FALSE(clear.halt);
  EXPECT_TRUE(clear.newly_released);
}

TEST(SafetyMonitorTest, RespectsTheMinimumHoldTime) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  ASSERT_TRUE(monitor.Evaluate(ObstacleAhead(0.2), 0.0, 10.0, 10.0, true).halt);

  // Obstacle vanishes immediately, but the hold time has not elapsed.
  EXPECT_TRUE(monitor.Evaluate(ClearScan(), 0.0, 10.1, 10.1, true).halt);
  EXPECT_TRUE(monitor.Evaluate(ClearScan(), 0.0, 10.3, 10.3, true).halt);
  EXPECT_FALSE(monitor.Evaluate(ClearScan(), 0.0, 10.5, 10.5, true).halt);
}

TEST(SafetyMonitorTest, DoesNotChatterAtTheBoundary) {
  // Drive the monitor with a range sitting exactly on the trigger distance for
  // many cycles. A monitor without hysteresis toggles every tick.
  const auto profile = HeavyMapperProfile();
  SafetyMonitor monitor(profile, DefaultOptions());
  const double boundary = profile.safety.SafeDistance(0.0);

  int transitions = 0;
  bool previous = false;
  double now = 0.0;
  for (int i = 0; i < 200; ++i) {
    now += 0.05;
    const SafetyDecision decision =
      monitor.Evaluate(ObstacleAhead(boundary - 1e-4), 0.0, now, now, true);
    if (decision.halt != previous) {
      ++transitions;
      previous = decision.halt;
    }
  }
  EXPECT_LE(transitions, 1) << "the halt toggled " << transitions << " times";
}

TEST(SafetyMonitorTest, EngagementAndReleaseEdgesAreReportedOnce) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  EXPECT_TRUE(monitor.Evaluate(ObstacleAhead(0.2), 0.0, 1.0, 1.0, true).newly_engaged);
  EXPECT_FALSE(monitor.Evaluate(ObstacleAhead(0.2), 0.0, 1.1, 1.1, true).newly_engaged);

  EXPECT_TRUE(monitor.Evaluate(ClearScan(), 0.0, 2.0, 2.0, true).newly_released);
  EXPECT_FALSE(monitor.Evaluate(ClearScan(), 0.0, 2.1, 2.1, true).newly_released);
}

// ---------------------------------------------------------------------------
// Fail-safe posture
// ---------------------------------------------------------------------------

TEST(SafetyMonitorTest, RejectedSensorDataHalts) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  // The BSP layer says the scan is untrustworthy. A clear-looking scan must
  // not be believed.
  const SafetyDecision decision = monitor.Evaluate(ClearScan(), 0.6, 1.0, 1.0, false);
  EXPECT_TRUE(decision.halt);
  EXPECT_EQ(decision.reason, HaltReason::kSensorInvalid);
}

TEST(SafetyMonitorTest, StaleSensorDataHalts) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  // Scan is 1 s old against a 0.35 s watchdog.
  const SafetyDecision decision = monitor.Evaluate(ClearScan(), 0.6, 2.0, 1.0, true);
  EXPECT_TRUE(decision.halt);
  EXPECT_EQ(decision.reason, HaltReason::kSensorTimeout);
}

TEST(SafetyMonitorTest, HaltsBeforeTheFirstScanEverArrives) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  // Startup: the robot has never been cleared to move.
  const SafetyDecision decision = monitor.EvaluateStale(0.0, 0.5);
  EXPECT_TRUE(decision.halt);
  EXPECT_EQ(decision.reason, HaltReason::kSensorTimeout);
}

TEST(SafetyMonitorTest, WatchdogTicksHoldTheVerdictInsideTheWindow) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  ASSERT_FALSE(monitor.Evaluate(ClearScan(), 0.4, 1.0, 1.0, true).halt);

  // 0.2 s later, still inside the 0.35 s window: keep the previous verdict
  // rather than fabricating a clear reading.
  EXPECT_FALSE(monitor.EvaluateStale(0.4, 1.2).halt);
  // Past the window: fail safe.
  const SafetyDecision expired = monitor.EvaluateStale(0.4, 1.5);
  EXPECT_TRUE(expired.halt);
  EXPECT_EQ(expired.reason, HaltReason::kSensorTimeout);
}

TEST(SafetyMonitorTest, ManualOverrideOutranksAClearSensor) {
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  monitor.SetManualOverride(true);

  const SafetyDecision decision = monitor.Evaluate(ClearScan(), 0.0, 1.0, 1.0, true);
  EXPECT_TRUE(decision.halt);
  EXPECT_EQ(decision.reason, HaltReason::kManualOverride);

  monitor.SetManualOverride(false);
  EXPECT_FALSE(monitor.Evaluate(ClearScan(), 0.0, 2.0, 2.0, true).halt);
}

TEST(SafetyMonitorTest, ManualOverrideIgnoresTheMinimumHoldTime) {
  // An operator releasing e-stop should not be made to wait for a hold timer
  // that exists to suppress sensor chatter.
  SafetyMonitor monitor(HeavyMapperProfile(), DefaultOptions());
  monitor.SetManualOverride(true);
  ASSERT_TRUE(monitor.Evaluate(ClearScan(), 0.0, 1.0, 1.0, true).halt);
  monitor.SetManualOverride(false);
  EXPECT_FALSE(monitor.Evaluate(ClearScan(), 0.0, 1.01, 1.01, true).halt);
}

// ---------------------------------------------------------------------------
// The scenario the brief actually describes
// ---------------------------------------------------------------------------

TEST(SafetyMonitorTest, PedestrianStepsOutInFrontOfAMovingRobot) {
  // A dynamic obstacle appears 0.8 m ahead of the scout at cruise. The monitor
  // must halt on the very first scan that shows it - one control period of
  // latency, not a planner replan cycle.
  const auto profile = LightScoutProfile();
  SafetyMonitor monitor(profile, DefaultOptions());

  double now = 0.0;
  for (int i = 0; i < 10; ++i) {
    now += 0.05;
    ASSERT_FALSE(monitor.Evaluate(ClearScan(), 1.3, now, now, true).halt);
  }

  now += 0.05;
  const SafetyDecision decision = monitor.Evaluate(ObstacleAhead(0.8), 1.3, now, now, true);
  EXPECT_TRUE(decision.halt);
  EXPECT_TRUE(decision.newly_engaged);
  // 0.42 * 1.69 + 0.30 = 1.0098 m of demanded clearance against 0.8 m actual.
  EXPECT_GT(decision.safe_distance, decision.min_obstacle_distance);
}

}  // namespace
