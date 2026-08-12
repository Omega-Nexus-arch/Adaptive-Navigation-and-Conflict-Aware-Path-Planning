// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// These tests run against the *real* elevation map that amr_gazebo generates
// alongside the world, not a synthetic fixture. That is the point: they verify
// that the terrain the planner prices is the terrain Gazebo simulates, and
// that the shipped ramps come out expensive-but-usable rather than free or
// lethal. A layout change that broke either property fails the build.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "amr_navigation/slope_cost_model.hpp"

namespace
{

using amr_navigation::ElevationMap;
using amr_navigation::SlopeCostModel;
using amr_navigation::SlopeSample;

constexpr double kDegrees = M_PI / 180.0;

std::string WarehouseElevationPath()
{
  return std::string(AMR_NAVIGATION_TEST_DATA_DIR) + "/warehouse_elevation.yaml";
}

/// Mirrors the heavy_mapper configuration in nav2_common.yaml.
SlopeCostModel::Options HeavyOptions()
{
  SlopeCostModel::Options options;
  options.free_angle = 2.0 * kDegrees;
  options.max_traversable_angle = 16.0 * kDegrees;
  options.base_cost = 40;
  options.max_cost = 253;
  options.curve_exponent = 2.0;
  return options;
}

// ---------------------------------------------------------------------------
// The cost curve, independent of any map
// ---------------------------------------------------------------------------

TEST(SlopeCostModelTest, FlatGroundIsFree) {
  const SlopeCostModel model(HeavyOptions());
  EXPECT_EQ(model.CostForAngle(0.0), SlopeCostModel::kFree);
  EXPECT_EQ(model.CostForAngle(1.0 * kDegrees), SlopeCostModel::kFree);
}

TEST(SlopeCostModelTest, TraversableSlopesAreExpensiveButNotLethal) {
  // The behaviour the requirement turns on. A ramp must be *avoidable*, which
  // means costly, and *usable*, which means below LETHAL.
  const SlopeCostModel model(HeavyOptions());
  for (double degrees = 3.0; degrees <= 15.0; degrees += 1.0) {
    const std::uint8_t cost = model.CostForAngle(degrees * kDegrees);
    EXPECT_GT(cost, 0) << degrees << " deg should not be free";
    EXPECT_LT(cost, SlopeCostModel::kLethal)
      << degrees << " deg is traversable and must not be marked lethal";
  }
}

TEST(SlopeCostModelTest, CostMustStayBelowInscribedInflatedObstacle) {
  // nav2 treats 253 as inscribed-inflated and 254 as definitely-in-collision.
  // A ramp priced at either is refused outright rather than merely avoided,
  // which would silently break the "only viable path" case.
  const SlopeCostModel model(HeavyOptions());
  const std::uint8_t worst = model.CostForAngle(15.99 * kDegrees);
  EXPECT_LE(worst, 253);
  EXPECT_LT(worst, SlopeCostModel::kLethal);
}

TEST(SlopeCostModelTest, BeyondTheClimbingLimitIsLethal) {
  const SlopeCostModel model(HeavyOptions());
  EXPECT_EQ(model.CostForAngle(16.0 * kDegrees), SlopeCostModel::kLethal);
  EXPECT_EQ(model.CostForAngle(30.0 * kDegrees), SlopeCostModel::kLethal);
  EXPECT_EQ(model.CostForAngle(89.0 * kDegrees), SlopeCostModel::kLethal);
}

TEST(SlopeCostModelTest, CostIncreasesMonotonicallyWithSlope) {
  const SlopeCostModel model(HeavyOptions());
  int previous = -1;
  for (double degrees = 2.0; degrees < 16.0; degrees += 0.25) {
    const int cost = model.CostForAngle(degrees * kDegrees);
    EXPECT_GE(cost, previous) << "cost fell at " << degrees << " deg";
    previous = cost;
  }
}

TEST(SlopeCostModelTest, TheCurveKeepsGentleRampsCheaperThanSteepOnes) {
  // The behaviour that makes the two mezzanine ramps distinguishable: the
  // 7.8 degree service ramp must be markedly cheaper than the 14.8 degree
  // maintenance ramp, or the planner has no reason to prefer it.
  const SlopeCostModel model(HeavyOptions());
  const std::uint8_t gentle = model.CostForAngle(7.8 * kDegrees);
  const std::uint8_t steep = model.CostForAngle(14.8 * kDegrees);
  EXPECT_LT(gentle, steep);
  EXPECT_LT(gentle * 2, steep)
    << "a near-linear curve would leave the planner nearly indifferent";
}

TEST(SlopeCostModelTest, DescendingCostsTheSameAsClimbing) {
  const SlopeCostModel model(HeavyOptions());
  EXPECT_EQ(model.CostForAngle(9.0 * kDegrees), model.CostForAngle(-9.0 * kDegrees));
}

TEST(SlopeCostModelTest, PerRobotLimitsChangeTheVerdict) {
  // The same physical ramp can be usable for one robot and impassable for
  // another, which is the honest model for a heterogeneous fleet.
  SlopeCostModel::Options capable = HeavyOptions();
  capable.max_traversable_angle = 16.0 * kDegrees;
  SlopeCostModel::Options limited = HeavyOptions();
  limited.max_traversable_angle = 10.0 * kDegrees;

  const double ramp = 14.8 * kDegrees;
  EXPECT_LT(SlopeCostModel(capable).CostForAngle(ramp), SlopeCostModel::kLethal);
  EXPECT_EQ(SlopeCostModel(limited).CostForAngle(ramp), SlopeCostModel::kLethal);
}

// ---------------------------------------------------------------------------
// Against the generated warehouse
// ---------------------------------------------------------------------------

TEST(ElevationMapTest, LoadsTheGeneratedWarehouseMap) {
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  EXPECT_FALSE(map.Empty());
  EXPECT_NEAR(map.resolution, 0.05, 1e-9);
  EXPECT_EQ(map.width, 880u);      // 44 m at 5 cm
  EXPECT_EQ(map.height, 600u);     // 30 m at 5 cm
  EXPECT_NEAR(map.origin_x, -22.0, 1e-9);
  EXPECT_NEAR(map.origin_y, -15.0, 1e-9);
}

TEST(ElevationMapTest, DecodesTheKnownHeightsOfTheWorld) {
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  // Comfortably above one LSB of the height encoding (0.69 m / 255).
  const double tolerance = 0.01;

  EXPECT_NEAR(map.HeightAt(-18.5, 2.2), 0.00, tolerance) << "west dock";
  EXPECT_NEAR(map.HeightAt(17.2, 2.0), 0.00, tolerance) << "packing bay 4";
  EXPECT_NEAR(map.HeightAt(0.0, 3.6), 0.55, tolerance) << "hump bridge deck";
  EXPECT_NEAR(map.HeightAt(7.0, 10.5), 0.45, tolerance) << "mezzanine deck";
}

TEST(ElevationMapTest, OutOfBoundsQueriesFallBack) {
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  EXPECT_NEAR(map.HeightAt(-500.0, 0.0, -7.0), -7.0, 1e-9);
  EXPECT_NEAR(map.HeightAt(0.0, 500.0, -7.0), -7.0, 1e-9);
}

TEST(SlopeCostModelTest, TheWarehouseFloorIsFree) {
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  const SlopeCostModel model(HeavyOptions());

  // Points chosen well away from every ramp and deck edge.
  const double points[][2] = {
    {-18.5, 2.2}, {-17.5, -10.7}, {17.2, 2.0}, {14.0, -6.5}, {-14.0, -1.0},
    {-3.5, 0.0}, {3.5, 0.0},
  };
  for (const auto & point : points) {
    EXPECT_EQ(model.CostAt(map, point[0], point[1]), SlopeCostModel::kFree)
      << "flat floor at (" << point[0] << ", " << point[1] << ") was charged";
  }
}

TEST(SlopeCostModelTest, TheGeneratedRampsMeasureTheirDocumentedGradient) {
  // Closes the loop between the world generator and the planner: the slope the
  // planner measures off the elevation map must be the slope the world
  // builder claims to have built.
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  const SlopeCostModel model(HeavyOptions());

  struct Probe
  {
    const char * name;
    double x;
    double y;
    double expected_degrees;
  };
  const Probe probes[] = {
    {"hump_ramp_west", -3.4, 3.6, 8.7},
    {"hump_ramp_east", 3.4, 3.6, 8.7},
    {"mezz_ramp_gentle", 7.0, 5.85, 7.8},
    {"mezz_ramp_steep", 10.5, 6.65, 14.8},
  };

  for (const Probe & probe : probes) {
    const SlopeSample sample = model.SampleSlope(map, probe.x, probe.y);
    ASSERT_TRUE(sample.valid) << probe.name;
    // Tolerance covers the 1/255 quantisation of the height encoding.
    EXPECT_NEAR(sample.angle / kDegrees, probe.expected_degrees, 1.5)
      << probe.name << " measured " << sample.angle / kDegrees << " deg";
  }
}

TEST(SlopeCostModelTest, EveryShippedRampIsAvoidableButUsable) {
  // The requirement, checked against the actual world rather than in the
  // abstract: each ramp must cost something (so a flat route wins) and must
  // stay below lethal (so it is available when nothing else is).
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  const SlopeCostModel model(HeavyOptions());

  const double ramp_points[][2] = {
    {-3.4, 3.6}, {3.4, 3.6}, {7.0, 5.85}, {10.5, 6.65},
  };
  for (const auto & point : ramp_points) {
    const std::uint8_t cost = model.CostAt(map, point[0], point[1]);
    EXPECT_GT(cost, 0) << "ramp at (" << point[0] << ", " << point[1] << ") is free, so the "
      "planner has no reason to prefer the flat route";
    EXPECT_LT(cost, SlopeCostModel::kLethal)
      << "ramp at (" << point[0] << ", " << point[1] << ") is impassable, so the "
      "mezzanine can never be reached";
  }
}

TEST(SlopeCostModelTest, TheSteepMezzanineRampCostsMoreThanTheGentleOne) {
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  const SlopeCostModel model(HeavyOptions());

  const std::uint8_t gentle = model.CostAt(map, 7.0, 5.85);
  const std::uint8_t steep = model.CostAt(map, 10.5, 6.65);
  EXPECT_LT(gentle, steep)
    << "the planner would have no reason to prefer the service ramp";
}

TEST(SlopeCostModelTest, TheFlatDoorwayIsCheaperThanTheSlopedBridge) {
  // The head-to-head that decides the demo: crossing the firewall through The
  // Pinch must price lower than going over the Hump Bridge.
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  const SlopeCostModel model(HeavyOptions());

  const std::uint8_t doorway = model.CostAt(map, 0.0, 0.0);
  const std::uint8_t bridge_ramp = model.CostAt(map, -3.4, 3.6);

  EXPECT_EQ(doorway, SlopeCostModel::kFree);
  EXPECT_GT(bridge_ramp, doorway)
    << "with the doorway open, the planner must prefer it over the ramp";
}

TEST(SlopeCostModelTest, MissingElevationDataDoesNotBlockTheRobot) {
  // A costmap cell with no height information must not become an obstacle, or
  // the robot is trapped the moment it drives off the surveyed area.
  const ElevationMap empty;
  const SlopeCostModel model(HeavyOptions());
  EXPECT_EQ(model.CostAt(empty, 0.0, 0.0), SlopeCostModel::kFree);

  SlopeCostModel::Options strict = HeavyOptions();
  strict.unknown_is_free = false;
  EXPECT_EQ(SlopeCostModel(strict).CostAt(empty, 0.0, 0.0), SlopeCostModel::kNoInformation);
}

TEST(SlopeCostModelTest, TheGradientDoesNotDependOnWhereInACellYouAsk) {
  // Regression guard. Probing at plus or minus one cell width from an
  // arbitrary point lets a query near a cell boundary round its own probe back
  // into the same cell, halving the measured gradient - silently, and worst on
  // the steep slopes the lethal threshold exists to catch. Sampling anywhere
  // inside one cell must give one answer.
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  const SlopeCostModel model(HeavyOptions());

  const double base_x = -3.4;
  const double base_y = 3.6;
  const double reference = model.SampleSlope(map, base_x, base_y).angle;
  EXPECT_GT(reference / kDegrees, 7.0) << "the probe must actually be on the ramp";

  for (double dx = -0.024; dx <= 0.024; dx += 0.008) {
    for (double dy = -0.024; dy <= 0.024; dy += 0.008) {
      const double angle = model.SampleSlope(map, base_x + dx, base_y + dy).angle;
      EXPECT_NEAR(angle, reference, 1e-9)
        << "offset (" << dx << ", " << dy << ") changed the measured slope";
    }
  }
}

TEST(ElevationMapTest, MissingFilesFailLoudly) {
  EXPECT_THROW(ElevationMap::FromFiles("/nonexistent/path.yaml"), std::runtime_error);
}

}  // namespace
