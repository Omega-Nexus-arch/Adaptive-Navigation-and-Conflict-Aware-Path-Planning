// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Conflict detection and the yielding protocol. The scenarios are built
// directly from the brief: two robots converging on a narrow intersection, the
// lighter one giving way to the heavier mission-critical one, and the whole
// thing continuing to work when the fleet is ten robots rather than two.

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "amr_fleet_control/traffic_manager.hpp"
#include "test_fixtures.hpp"

namespace
{

using amr_core::FleetPolicy;
using amr_core::Pose2D;
using amr_core::Velocity2D;
using amr_fleet_control::Conflict;
using amr_fleet_control::ConflictDetector;
using amr_fleet_control::Directive;
using amr_fleet_control::PredictedPath;
using amr_fleet_control::PredictorOptions;
using amr_fleet_control::TrafficAction;
using amr_fleet_control::TrajectoryPredictor;
using amr_fleet_control::YieldPolicy;
using amr_fleet_control::test::HeavyMapperProfile;
using amr_fleet_control::test::LargeFleet;
using amr_fleet_control::test::LightScoutProfile;
using amr_fleet_control::test::TwoRobotFleet;

FleetPolicy DefaultPolicy()
{
  FleetPolicy policy;
  policy.horizon_seconds = 4.0;
  policy.sample_period = 0.2;
  policy.conflict_margin = 0.35;
  policy.conflict_react_seconds = 3.0;
  policy.hard_yield_seconds = 1.5;
  policy.slow_speed_scale = 0.35;
  policy.max_yield_seconds = 12.0;
  policy.yield_cooldown_seconds = 4.0;
  policy.trajectory_timeout_seconds = 1.0;
  return policy;
}

/// \brief Straight-line projection, built by hand so the geometry is explicit.
PredictedPath StraightPath(
  const std::string & id, int priority, double radius, double x0, double y0, double heading,
  double speed, double stamp, const FleetPolicy & policy)
{
  PredictedPath path;
  path.robot_id = id;
  path.stamp = stamp;
  path.footprint_radius = radius;
  path.yield_priority = priority;

  const int steps = static_cast<int>(std::round(policy.horizon_seconds / policy.sample_period));
  for (int step = 0; step <= steps; ++step) {
    const double t = step * policy.sample_period;
    amr_fleet_control::TrajectorySample sample;
    sample.time = t;
    sample.pose = Pose2D{x0 + speed * t * std::cos(heading),
      y0 + speed * t * std::sin(heading), heading};
    sample.speed = speed;
    path.samples.push_back(sample);
  }
  return path;
}

// ---------------------------------------------------------------------------
// ConflictDetector
// ---------------------------------------------------------------------------

TEST(ConflictDetectorTest, HeadOnApproachIsAConflict) {
  const FleetPolicy policy = DefaultPolicy();
  ConflictDetector detector(policy);

  // Two robots closing on the origin from opposite sides at 1 m/s. They meet
  // in about 2 s.
  const PredictedPath a =
    StraightPath("amr1", 100, 0.55, -2.5, 0.0, 0.0, 1.0, 0.0, policy);
  const PredictedPath b =
    StraightPath("amr2", 50, 0.37, 2.5, 0.0, M_PI, 1.0, 0.0, policy);

  Conflict conflict;
  ASSERT_TRUE(detector.DetectPair(a, b, 0.0, &conflict));
  EXPECT_GT(conflict.time_to_conflict, 0.0);
  EXPECT_LT(conflict.time_to_conflict, 3.0);
  EXPECT_LE(conflict.separation, conflict.required_separation);
  EXPECT_NEAR(conflict.required_separation, 0.55 + 0.37 + 0.35, 1e-9);
  EXPECT_NEAR(conflict.x, 0.0, 0.6) << "the meeting point should be near the origin";
}

TEST(ConflictDetectorTest, ParallelTracksAreNotAConflict) {
  const FleetPolicy policy = DefaultPolicy();
  ConflictDetector detector(policy);

  // Same direction, 3 m apart laterally: they never come close.
  const PredictedPath a = StraightPath("amr1", 100, 0.55, 0.0, 0.0, 0.0, 1.0, 0.0, policy);
  const PredictedPath b = StraightPath("amr2", 50, 0.37, 0.0, 3.0, 0.0, 1.0, 0.0, policy);
  EXPECT_FALSE(detector.DetectPair(a, b, 0.0, nullptr));
}

TEST(ConflictDetectorTest, CrossingPathsAtDifferentTimesAreNotAConflict) {
  // This is the property that makes the detector worth writing. Two paths that
  // intersect geometrically are only a problem if the robots are there
  // together. A purely spatial test would stop the fleet at every junction in
  // a warehouse where every aisle crosses every other one.
  const FleetPolicy policy = DefaultPolicy();
  ConflictDetector detector(policy);

  // A crosses the origin at t = 2 s travelling east.
  const PredictedPath a = StraightPath("amr1", 100, 0.55, -2.0, 0.0, 0.0, 1.0, 0.0, policy);
  // B crosses the origin heading north, but starts 8 m away at 1 m/s, so it
  // arrives long after A has gone.
  const PredictedPath b =
    StraightPath("amr2", 50, 0.37, 0.0, -8.0, M_PI / 2.0, 1.0, 0.0, policy);

  EXPECT_FALSE(detector.DetectPair(a, b, 0.0, nullptr))
    << "paths cross in space but not in time";
}

TEST(ConflictDetectorTest, CrossingPathsAtTheSameTimeAreAConflict) {
  const FleetPolicy policy = DefaultPolicy();
  ConflictDetector detector(policy);

  // Both reach the origin at t = 2 s.
  const PredictedPath a = StraightPath("amr1", 100, 0.55, -2.0, 0.0, 0.0, 1.0, 0.0, policy);
  const PredictedPath b =
    StraightPath("amr2", 50, 0.37, 0.0, -2.0, M_PI / 2.0, 1.0, 0.0, policy);

  Conflict conflict;
  ASSERT_TRUE(detector.DetectPair(a, b, 0.0, &conflict));

  // The conflict is flagged at t = 1.2 s, not at t = 2.0 s when the two
  // centres coincide. That is the point of a footprint-aware test: with a
  // required separation of 0.55 + 0.37 + 0.35 = 1.27 m, the robots are already
  // too close at (-0.8, 0) and (0, -0.8), which are 1.13 m apart. Warning only
  // at the moment of collision would leave nothing to react with.
  EXPECT_GT(conflict.time_to_conflict, 1.0);
  EXPECT_LT(conflict.time_to_conflict, 2.0);
  EXPECT_LT(conflict.separation, conflict.required_separation);
}

TEST(ConflictDetectorTest, IgnoresConflictsBeyondTheReactionHorizon) {
  FleetPolicy policy = DefaultPolicy();
  policy.conflict_react_seconds = 1.0;   // Only look 1 s ahead.
  ConflictDetector detector(policy);

  const PredictedPath a = StraightPath("amr1", 100, 0.55, -2.5, 0.0, 0.0, 1.0, 0.0, policy);
  const PredictedPath b = StraightPath("amr2", 50, 0.37, 2.5, 0.0, M_PI, 1.0, 0.0, policy);

  EXPECT_FALSE(detector.DetectPair(a, b, 0.0, nullptr))
    << "a conflict 2 s out must not be acted on when the policy looks 1 s ahead";
}

TEST(ConflictDetectorTest, DiscardsStaleTrajectories) {
  const FleetPolicy policy = DefaultPolicy();
  ConflictDetector detector(policy);

  std::map<std::string, PredictedPath> paths;
  paths["amr1"] = StraightPath("amr1", 100, 0.55, -2.5, 0.0, 0.0, 1.0, 10.0, policy);
  // Published 5 s ago against a 1 s timeout.
  paths["amr2"] = StraightPath("amr2", 50, 0.37, 2.5, 0.0, M_PI, 1.0, 5.0, policy);

  EXPECT_TRUE(detector.Detect(paths, 10.0).empty())
    << "acting on a 5 s old prediction is worse than acting on none";
}

TEST(ConflictDetectorTest, EmptyAndSingleRobotFleetsProduceNothing) {
  const FleetPolicy policy = DefaultPolicy();
  ConflictDetector detector(policy);

  std::map<std::string, PredictedPath> paths;
  EXPECT_TRUE(detector.Detect(paths, 0.0).empty());

  paths["amr1"] = StraightPath("amr1", 100, 0.55, 0.0, 0.0, 0.0, 1.0, 0.0, policy);
  EXPECT_TRUE(detector.Detect(paths, 0.0).empty());
}

TEST(ConflictDetectorTest, ConflictsAreReturnedMostImminentFirst) {
  const FleetPolicy policy = DefaultPolicy();
  ConflictDetector detector(policy);

  // amr1 drives east from the origin.
  // amr2 comes straight back at it: conflict almost immediately.
  // amr3 drops south onto amr1's track at x = 2, which amr1 reaches at t = 2.
  std::map<std::string, PredictedPath> paths;
  paths["amr1"] = StraightPath("amr1", 100, 0.4, 0.0, 0.0, 0.0, 1.0, 0.0, policy);
  paths["amr2"] = StraightPath("amr2", 90, 0.4, 1.5, 0.0, M_PI, 1.0, 0.0, policy);
  paths["amr3"] = StraightPath("amr3", 80, 0.4, 2.0, 2.0, -M_PI / 2.0, 1.0, 0.0, policy);

  const std::vector<Conflict> conflicts = detector.Detect(paths, 0.0);
  ASSERT_GT(conflicts.size(), 1u);
  for (std::size_t i = 1; i < conflicts.size(); ++i) {
    EXPECT_LE(conflicts[i - 1].time_to_conflict, conflicts[i].time_to_conflict);
  }
}

TEST(ConflictDetectorTest, LargerFootprintsConflictSooner) {
  const FleetPolicy policy = DefaultPolicy();
  ConflictDetector detector(policy);

  const PredictedPath small_a = StraightPath("a", 2, 0.20, -3.0, 0.0, 0.0, 1.0, 0.0, policy);
  const PredictedPath small_b = StraightPath("b", 1, 0.20, 3.0, 0.0, M_PI, 1.0, 0.0, policy);
  const PredictedPath big_a = StraightPath("a", 2, 0.80, -3.0, 0.0, 0.0, 1.0, 0.0, policy);
  const PredictedPath big_b = StraightPath("b", 1, 0.80, 3.0, 0.0, M_PI, 1.0, 0.0, policy);

  Conflict small_conflict;
  Conflict big_conflict;
  ASSERT_TRUE(detector.DetectPair(small_a, small_b, 0.0, &small_conflict));
  ASSERT_TRUE(detector.DetectPair(big_a, big_b, 0.0, &big_conflict));
  EXPECT_LT(big_conflict.time_to_conflict, small_conflict.time_to_conflict);
}

// ---------------------------------------------------------------------------
// YieldPolicy: the protocol from the brief
// ---------------------------------------------------------------------------

TEST(YieldPolicyTest, EveryRobotGetsAnExplicitDirectiveEvenWhenClear) {
  // Silence must be distinguishable from "you may proceed", so a robot can
  // treat a missing directive as a fault and fail safe.
  YieldPolicy policy(TwoRobotFleet(DefaultPolicy()));
  const std::map<std::string, Directive> directives = policy.Resolve({}, 0.0);

  EXPECT_EQ(directives.size(), 2u);
  for (const auto & entry : directives) {
    EXPECT_EQ(entry.second.action, TrafficAction::kProceed);
    EXPECT_DOUBLE_EQ(entry.second.speed_scale, 1.0);
  }
}

TEST(YieldPolicyTest, TheScoutYieldsToTheHeavyMapper) {
  // The brief's headline rule, verified end to end.
  YieldPolicy policy(TwoRobotFleet(DefaultPolicy()));

  Conflict conflict;
  conflict.robot_a = "amr1";     // priority 100, heavy
  conflict.robot_b = "amr2";     // priority 50, scout
  conflict.time_to_conflict = 1.0;

  const std::map<std::string, Directive> directives = policy.Resolve({conflict}, 0.0);
  EXPECT_EQ(directives.at("amr1").action, TrafficAction::kProceed);
  EXPECT_EQ(directives.at("amr2").action, TrafficAction::kYield);
  EXPECT_DOUBLE_EQ(directives.at("amr2").speed_scale, 0.0);
  EXPECT_EQ(directives.at("amr2").conflicting_robot, "amr1");
}

TEST(YieldPolicyTest, TheOutcomeDoesNotDependOnConflictOrdering) {
  // Swapping which robot is listed first must not change who gives way.
  YieldPolicy policy(TwoRobotFleet(DefaultPolicy()));

  Conflict swapped;
  swapped.robot_a = "amr2";
  swapped.robot_b = "amr1";
  swapped.time_to_conflict = 1.0;

  const std::map<std::string, Directive> directives = policy.Resolve({swapped}, 0.0);
  EXPECT_EQ(directives.at("amr1").action, TrafficAction::kProceed);
  EXPECT_EQ(directives.at("amr2").action, TrafficAction::kYield);
}

TEST(YieldPolicyTest, DistantConflictsSlowRatherThanStop) {
  // Bleeding off speed early usually dissolves the conflict without anyone
  // stopping, which is worth much more throughput than stop-and-wait.
  const FleetPolicy fleet_policy = DefaultPolicy();
  YieldPolicy policy(TwoRobotFleet(fleet_policy));

  Conflict conflict;
  conflict.robot_a = "amr1";
  conflict.robot_b = "amr2";
  conflict.time_to_conflict = 2.5;   // beyond hard_yield_seconds = 1.5

  const std::map<std::string, Directive> directives = policy.Resolve({conflict}, 0.0);
  EXPECT_EQ(directives.at("amr2").action, TrafficAction::kSlow);
  EXPECT_DOUBLE_EQ(directives.at("amr2").speed_scale, fleet_policy.slow_speed_scale);
  EXPECT_GT(directives.at("amr2").speed_scale, 0.0);
}

TEST(YieldPolicyTest, AnImminentConflictIsNotDowngradedByADistantOne) {
  YieldPolicy policy(TwoRobotFleet(DefaultPolicy()));

  Conflict imminent;
  imminent.robot_a = "amr1";
  imminent.robot_b = "amr2";
  imminent.time_to_conflict = 0.5;

  Conflict distant;
  distant.robot_a = "amr1";
  distant.robot_b = "amr2";
  distant.time_to_conflict = 2.8;

  const std::map<std::string, Directive> directives =
    policy.Resolve({imminent, distant}, 0.0);
  EXPECT_EQ(directives.at("amr2").action, TrafficAction::kYield);
  EXPECT_DOUBLE_EQ(directives.at("amr2").speed_scale, 0.0);
}

TEST(YieldPolicyTest, ClearingTheConflictReleasesTheYieldImmediately) {
  YieldPolicy policy(TwoRobotFleet(DefaultPolicy()));

  Conflict conflict;
  conflict.robot_a = "amr1";
  conflict.robot_b = "amr2";
  conflict.time_to_conflict = 0.8;

  ASSERT_EQ(policy.Resolve({conflict}, 0.0).at("amr2").action, TrafficAction::kYield);
  EXPECT_EQ(policy.Resolve({}, 1.0).at("amr2").action, TrafficAction::kProceed)
    << "a robot must not inherit last cycle's yield once the conflict is gone";
}

TEST(YieldPolicyTest, UnknownRobotsCannotCommandTheFleet) {
  YieldPolicy policy(TwoRobotFleet(DefaultPolicy()));

  Conflict conflict;
  conflict.robot_a = "amr1";
  conflict.robot_b = "rogue_publisher";
  conflict.time_to_conflict = 0.2;

  const std::map<std::string, Directive> directives = policy.Resolve({conflict}, 0.0);
  EXPECT_EQ(directives.size(), 2u);
  EXPECT_EQ(directives.at("amr1").action, TrafficAction::kProceed);
  EXPECT_EQ(directives.at("amr2").action, TrafficAction::kProceed);
}

// ---------------------------------------------------------------------------
// Starvation
// ---------------------------------------------------------------------------

TEST(YieldPolicyTest, TracksHowLongARobotHasBeenYielding) {
  const FleetPolicy fleet_policy = DefaultPolicy();
  YieldPolicy policy(TwoRobotFleet(fleet_policy));

  Conflict conflict;
  conflict.robot_a = "amr1";
  conflict.robot_b = "amr2";
  conflict.time_to_conflict = 0.5;

  policy.Resolve({conflict}, 100.0);
  policy.Resolve({conflict}, 103.0);
  EXPECT_NEAR(policy.YieldDuration("amr2", 103.0), 3.0, 1e-9);
  EXPECT_DOUBLE_EQ(policy.YieldDuration("amr1", 103.0), 0.0);
}

TEST(YieldPolicyTest, AStarvedRobotIsEventuallyLetThrough) {
  // Strict priority alone starves the lowest-ranked robot forever in a busy
  // aisle. After max_yield_seconds the loser is boosted above the fleet.
  const FleetPolicy fleet_policy = DefaultPolicy();
  YieldPolicy policy(TwoRobotFleet(fleet_policy));

  Conflict conflict;
  conflict.robot_a = "amr1";
  conflict.robot_b = "amr2";
  conflict.time_to_conflict = 0.5;

  double now = 0.0;
  for (int i = 0; i < 100; ++i) {
    now += 0.1;
    policy.Resolve({conflict}, now);
  }
  ASSERT_EQ(policy.Resolve({conflict}, now).at("amr2").action, TrafficAction::kYield)
    << "10 s in, the scout should still be waiting";

  // Push past max_yield_seconds = 12 s.
  now += 3.0;
  const std::map<std::string, Directive> directives = policy.Resolve({conflict}, now);
  EXPECT_TRUE(policy.HasPriorityBoost("amr2", now));
  EXPECT_EQ(directives.at("amr2").action, TrafficAction::kProceed);
  EXPECT_EQ(directives.at("amr1").action, TrafficAction::kYield)
    << "the boost must actually invert the conflict, not merely release it";
}

TEST(YieldPolicyTest, TheStarvationBoostExpires) {
  const FleetPolicy fleet_policy = DefaultPolicy();
  YieldPolicy policy(TwoRobotFleet(fleet_policy));

  Conflict conflict;
  conflict.robot_a = "amr1";
  conflict.robot_b = "amr2";
  conflict.time_to_conflict = 0.5;

  double now = 0.0;
  for (int i = 0; i < 140; ++i) {
    now += 0.1;
    policy.Resolve({conflict}, now);
  }
  ASSERT_TRUE(policy.HasPriorityBoost("amr2", now));

  // Beyond the cooldown, normal priority order must be restored: a temporary
  // inversion that became permanent would be its own bug.
  now += fleet_policy.yield_cooldown_seconds + 1.0;
  EXPECT_FALSE(policy.HasPriorityBoost("amr2", now));
  EXPECT_EQ(policy.Resolve({conflict}, now).at("amr2").action, TrafficAction::kYield);
}

TEST(YieldPolicyTest, ResetClearsAllHistory) {
  YieldPolicy policy(TwoRobotFleet(DefaultPolicy()));
  Conflict conflict;
  conflict.robot_a = "amr1";
  conflict.robot_b = "amr2";
  conflict.time_to_conflict = 0.5;

  policy.Resolve({conflict}, 100.0);
  policy.Reset();
  EXPECT_DOUBLE_EQ(policy.YieldDuration("amr2", 200.0), 0.0);
  EXPECT_FALSE(policy.HasPriorityBoost("amr2", 200.0));
}

// ---------------------------------------------------------------------------
// Scalability
// ---------------------------------------------------------------------------

TEST(YieldPolicyTest, WorksUnchangedForATenRobotFleet) {
  const FleetPolicy fleet_policy = DefaultPolicy();
  YieldPolicy policy(LargeFleet(10, fleet_policy));

  const std::map<std::string, Directive> clear = policy.Resolve({}, 0.0);
  EXPECT_EQ(clear.size(), 10u);

  // A chain of conflicts: 1-2, 2-3, ... 9-10. Priorities descend with index,
  // so every robot except the first should end up giving way.
  std::vector<Conflict> conflicts;
  for (int i = 1; i < 10; ++i) {
    Conflict conflict;
    conflict.robot_a = "amr" + std::to_string(i);
    conflict.robot_b = "amr" + std::to_string(i + 1);
    conflict.time_to_conflict = 0.5;
    conflicts.push_back(conflict);
  }

  const std::map<std::string, Directive> directives = policy.Resolve(conflicts, 0.0);
  EXPECT_EQ(directives.at("amr1").action, TrafficAction::kProceed);
  for (int i = 2; i <= 10; ++i) {
    EXPECT_EQ(directives.at("amr" + std::to_string(i)).action, TrafficAction::kYield)
      << "amr" << i;
  }
}

TEST(ConflictDetectorTest, ScalesToATenRobotConvergence) {
  // Ten robots converging on one point: 45 pairs, all conflicting. Checks the
  // detector does not choke and that every robot appears.
  const FleetPolicy policy = DefaultPolicy();
  ConflictDetector detector(policy);

  std::map<std::string, PredictedPath> paths;
  for (int i = 0; i < 10; ++i) {
    const double angle = i * 2.0 * M_PI / 10.0;
    const std::string id = "amr" + std::to_string(i + 1);
    paths[id] = StraightPath(
      id, 100 - i * 10, 0.4, 2.0 * std::cos(angle), 2.0 * std::sin(angle),
      angle + M_PI, 1.0, 0.0, policy);
  }

  const std::vector<Conflict> conflicts = detector.Detect(paths, 0.0);
  EXPECT_EQ(conflicts.size(), 45u) << "all 10-choose-2 pairs should conflict";
}

// ---------------------------------------------------------------------------
// Predictor integration
// ---------------------------------------------------------------------------

TEST(TrajectoryPredictorTest, TwistRolloutMatchesTheIntegrator) {
  PredictorOptions options;
  options.horizon_seconds = 2.0;
  options.sample_period = 0.5;
  TrajectoryPredictor predictor(LightScoutProfile(), options);

  const PredictedPath path = predictor.PredictFromTwist(
    "amr2", 50, Pose2D{0.0, 0.0, 0.0}, Velocity2D{1.0, 0.0}, 0.0);

  ASSERT_EQ(path.samples.size(), 5u);
  EXPECT_NEAR(path.samples.back().pose.x, 2.0, 1e-9);
  EXPECT_NEAR(path.samples.back().pose.y, 0.0, 1e-9);
  EXPECT_EQ(path.robot_id, "amr2");
  EXPECT_NEAR(path.footprint_radius, LightScoutProfile().footprint_radius, 1e-12);
}

TEST(TrajectoryPredictorTest, PlanFollowingBeatsTwistExtrapolationRoundACorner) {
  // A robot driving straight at a T-junction that is about to turn left. The
  // twist rollout says it continues into the rack; the plan rollout knows
  // better. Getting this right is the difference between a useful prediction
  // and a misleading one.
  PredictorOptions options;
  options.horizon_seconds = 4.0;
  options.sample_period = 0.5;
  TrajectoryPredictor predictor(LightScoutProfile(), options);

  const Pose2D pose{0.0, 0.0, 0.0};
  const Velocity2D velocity{1.0, 0.0};
  const std::vector<Pose2D> plan = {
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0},
    {2.0, 1.0, M_PI / 2.0}, {2.0, 2.0, M_PI / 2.0}, {2.0, 3.0, M_PI / 2.0},
  };

  const PredictedPath along =
    predictor.PredictAlongPlan("amr2", 50, pose, velocity, plan, 0.0);
  const PredictedPath straight =
    predictor.PredictFromTwist("amr2", 50, pose, velocity, 0.0);

  // At t = 3 s the plan has the robot 1 m up the north leg.
  const Pose2D plan_pose = along.PoseAt(3.0);
  EXPECT_NEAR(plan_pose.x, 2.0, 0.15);
  EXPECT_NEAR(plan_pose.y, 1.0, 0.15);

  const Pose2D twist_pose = straight.PoseAt(3.0);
  EXPECT_NEAR(twist_pose.x, 3.0, 1e-6);
  EXPECT_NEAR(twist_pose.y, 0.0, 1e-6);
}

TEST(TrajectoryPredictorTest, FallsBackToTwistWhenThePlanIsDegenerate) {
  PredictorOptions options;
  TrajectoryPredictor predictor(LightScoutProfile(), options);

  const std::vector<Pose2D> single = {{0.0, 0.0, 0.0}};
  const PredictedPath path = predictor.PredictAlongPlan(
    "amr2", 50, Pose2D{0.0, 0.0, 0.0}, Velocity2D{1.0, 0.0}, single, 0.0);
  EXPECT_FALSE(path.Empty());
  EXPECT_NEAR(path.PoseAt(1.0).x, 1.0, 1e-6);
}

TEST(TrajectoryPredictorTest, AStationaryRobotHoldingAPlanIsStillProjectedForward) {
  // Predicting that a robot stopped at a junction will stay there forever
  // hides the conflict until it is too late to resolve it gracefully.
  PredictorOptions options;
  options.horizon_seconds = 4.0;
  options.sample_period = 0.5;
  TrajectoryPredictor predictor(LightScoutProfile(), options);

  const std::vector<Pose2D> plan = {
    {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {4.0, 0.0, 0.0}, {6.0, 0.0, 0.0}};
  const PredictedPath path = predictor.PredictAlongPlan(
    "amr2", 50, Pose2D{0.0, 0.0, 0.0}, Velocity2D{0.0, 0.0}, plan, 0.0);

  EXPECT_GT(path.PoseAt(4.0).x, 0.5) << "a stopped robot with a plan is about to move";
}

TEST(TrajectoryPredictorTest, InterpolationClampsOutsideTheSampledSpan) {
  PredictorOptions options;
  options.horizon_seconds = 1.0;
  options.sample_period = 0.5;
  TrajectoryPredictor predictor(LightScoutProfile(), options);

  const PredictedPath path = predictor.PredictFromTwist(
    "amr2", 50, Pose2D{0.0, 0.0, 0.0}, Velocity2D{1.0, 0.0}, 10.0);

  // Before the start and after the end, hold the endpoints: a robot sits at
  // the end of its projection rather than vanishing, which is the
  // conservative reading for collision checking.
  EXPECT_NEAR(path.PoseAt(5.0).x, 0.0, 1e-9);
  EXPECT_NEAR(path.PoseAt(99.0).x, 1.0, 1e-9);
}

TEST(TrajectoryPredictorTest, HeadingInterpolationTakesTheShortWayAcrossPi) {
  PredictedPath path;
  path.robot_id = "amr1";
  path.stamp = 0.0;
  path.samples.push_back({0.0, Pose2D{0.0, 0.0, 3.0}, 1.0});
  path.samples.push_back({1.0, Pose2D{1.0, 0.0, -3.0}, 1.0});

  // Midway the heading should be near +/-pi, not near zero.
  const double theta = path.PoseAt(0.5).theta;
  EXPECT_GT(std::abs(theta), 3.0) << "interpolated the long way round: " << theta;
}

// ---------------------------------------------------------------------------
// End-to-end: the narrow-intersection scenario from the brief
// ---------------------------------------------------------------------------

TEST(TrafficIntegrationTest, TwoRobotsConvergingOnThePinchResolveByPriority) {
  const FleetPolicy fleet_policy = DefaultPolicy();
  const auto fleet = TwoRobotFleet(fleet_policy);

  PredictorOptions options;
  options.horizon_seconds = fleet_policy.horizon_seconds;
  options.sample_period = fleet_policy.sample_period;

  TrajectoryPredictor heavy(HeavyMapperProfile(), options);
  TrajectoryPredictor scout(LightScoutProfile(), options);

  // Both heading for the doorway at the origin from opposite sides, matching
  // the world's 2.0 m Pinch, which admits one robot at a time.
  std::map<std::string, PredictedPath> paths;
  paths["amr1"] = heavy.PredictFromTwist(
    "amr1", 100, Pose2D{-2.0, 0.0, 0.0}, Velocity2D{0.7, 0.0}, 0.0);
  paths["amr2"] = scout.PredictFromTwist(
    "amr2", 50, Pose2D{2.0, 0.0, M_PI}, Velocity2D{1.2, 0.0}, 0.0);

  ConflictDetector detector(fleet_policy);
  const std::vector<Conflict> conflicts = detector.Detect(paths, 0.0);
  ASSERT_EQ(conflicts.size(), 1u);

  YieldPolicy policy(fleet);
  const std::map<std::string, Directive> directives = policy.Resolve(conflicts, 0.0);

  EXPECT_EQ(directives.at("amr1").action, TrafficAction::kProceed)
    << "the mission-critical heavy unit keeps going";
  EXPECT_NE(directives.at("amr2").action, TrafficAction::kProceed)
    << "the scout must give way";
  EXPECT_LT(directives.at("amr2").speed_scale, 1.0);
  EXPECT_FALSE(directives.at("amr2").reason.empty())
    << "the directive must explain itself for the operator log";
}

}  // namespace
