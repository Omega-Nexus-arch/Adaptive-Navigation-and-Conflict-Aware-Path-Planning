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
#include <sstream>
#include <fstream>
#include <algorithm>
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
  // STRICTLY below 253. The comment above always said "a ramp priced at either
  // is refused outright"; the assertion said `<= 253`, which permits exactly
  // the value it warns about. nav2_params.yaml carried the same off-by-one as
  // `max_cost: 253`. See DESIGN_NOTES 8s.
  EXPECT_LT(worst, 253);
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

/// \brief One ramp or deck, as emitted by world_builder.render_landmarks.
struct Landmark
{
  std::string kind;      ///< "ramp" or "deck"
  std::string name;
  double x = 0.0;
  double y = 0.0;
  double value = 0.0;    ///< degrees for a ramp, height in metres for a deck.
};

/// \brief Read the generated landmark table.
///
/// These figures used to be hardcoded here -- probe points at (7.0, 5.85) and
/// gradients of 8.7 and 14.8 degrees. The world is GENERATED, it was
/// regenerated many times, and the constants were never updated, so five tests
/// in this file had been asserting against a warehouse that no longer existed.
/// The file below is written by the same pass that writes the world, so a ramp
/// cannot move without these tests following it.
std::vector<Landmark> Landmarks()
{
  const std::string path =
    std::string(AMR_NAVIGATION_TEST_DATA_DIR) + "/warehouse_landmarks.txt";
  std::ifstream input(path);
  EXPECT_TRUE(input.is_open())
    << "missing " << path << " -- regenerate the world with "
    << "`ros2 run amr_gazebo generate_world.py`";

  std::vector<Landmark> found;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream fields(line);
    Landmark entry;
    if (fields >> entry.kind >> entry.name >> entry.x >> entry.y >> entry.value) {
      found.push_back(entry);
    }
  }
  return found;
}

std::vector<Landmark> LandmarksOfKind(const std::string & kind)
{
  std::vector<Landmark> chosen;
  for (const Landmark & entry : Landmarks()) {
    if (entry.kind == kind) {
      chosen.push_back(entry);
    }
  }
  return chosen;
}

TEST(ElevationMapTest, DecodesTheKnownHeightsOfTheWorld) {
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  // Comfortably above one LSB of the height encoding (0.69 m / 255).
  const double tolerance = 0.01;

  EXPECT_NEAR(map.HeightAt(-18.5, 2.2), 0.00, tolerance) << "west dock";
  EXPECT_NEAR(map.HeightAt(17.2, 2.0), 0.00, tolerance) << "packing bay 4";

  const std::vector<Landmark> decks = LandmarksOfKind("deck");
  EXPECT_FALSE(decks.empty()) << "the world should declare at least one deck";
  for (const Landmark & deck : decks) {
    EXPECT_NEAR(map.HeightAt(deck.x, deck.y), deck.value, tolerance) << deck.name;
  }
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

  const std::vector<Landmark> ramps = LandmarksOfKind("ramp");
  EXPECT_FALSE(ramps.empty()) << "the world should declare at least one ramp";

  for (const Landmark & probe : ramps) {
    const SlopeSample sample = model.SampleSlope(map, probe.x, probe.y);
    ASSERT_TRUE(sample.valid) << probe.name;
    // Tolerance covers the 1/255 quantisation of the height encoding.
    EXPECT_NEAR(sample.angle / kDegrees, probe.value, 1.5)
      << probe.name << " measured " << sample.angle / kDegrees
      << " deg, world declares " << probe.value;
  }
}

TEST(SlopeCostModelTest, EveryShippedRampIsAvoidableButUsable) {
  // The requirement, checked against the actual world rather than in the
  // abstract: each ramp must cost something (so a flat route wins) and must
  // stay below lethal (so it is available when nothing else is).
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  const SlopeCostModel model(HeavyOptions());

  for (const Landmark & ramp : LandmarksOfKind("ramp")) {
    const std::uint8_t cost = model.CostAt(map, ramp.x, ramp.y);
    EXPECT_GT(cost, 0) << ramp.name << " is free, so the planner has no reason "
      "to prefer the flat route";
    // Below INSCRIBED_INFLATED_OBSTACLE, not merely below LETHAL: Smac's
    // Node2D::isNodeValid refuses 253 outright, so a ramp priced there is
    // impassable rather than expensive and the mezzanine becomes unreachable.
    EXPECT_LT(cost, 253)
      << ramp.name << " is priced at or above the planner's refusal threshold";
  }
}

TEST(SlopeCostModelTest, TheSteepMezzanineRampCostsMoreThanTheGentleOne) {
  const ElevationMap map = ElevationMap::FromFiles(WarehouseElevationPath());
  const SlopeCostModel model(HeavyOptions());

  const std::vector<Landmark> ramps = LandmarksOfKind("ramp");
  ASSERT_GE(ramps.size(), 2u);
  const Landmark & shallowest = *std::min_element(
    ramps.begin(), ramps.end(),
    [](const Landmark & a, const Landmark & b) {return a.value < b.value;});
  const Landmark & steepest = *std::max_element(
    ramps.begin(), ramps.end(),
    [](const Landmark & a, const Landmark & b) {return a.value < b.value;});
  const std::uint8_t gentle = model.CostAt(map, shallowest.x, shallowest.y);
  const std::uint8_t steep = model.CostAt(map, steepest.x, steepest.y);
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

  const std::vector<Landmark> ramps = LandmarksOfKind("ramp");
  ASSERT_FALSE(ramps.empty());
  const Landmark & probe = ramps.front();
  // Snap to the CENTRE of whichever cell the probe lands in. The claim under
  // test is that sampling anywhere inside ONE cell gives one answer, so the
  // base point has to be a cell centre -- a landmark sitting 1 cm off centre
  // puts the +/- 24 mm sweep across a boundary and fails for the wrong reason.
  const auto cell_centre = [&map](double value, double origin) {
      return origin + (std::floor((value - origin) / map.resolution) + 0.5) *
             map.resolution;
    };
  const double base_x = cell_centre(probe.x, map.origin_x);
  const double base_y = cell_centre(probe.y, map.origin_y);
  const double reference = model.SampleSlope(map, base_x, base_y).angle;
  // Half the declared gradient, so the check survives the height quantisation
  // without becoming a restatement of the previous test.
  EXPECT_GT(reference / kDegrees, probe.value * 0.5)
    << "the probe must actually be on " << probe.name;

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
