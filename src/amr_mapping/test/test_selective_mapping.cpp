// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// The System Optimization criterion asks for a demonstrated ability to
// programmatically restrict the map update area according to a defined
// condition. These tests are that demonstration: they measure the suppression
// the policy achieves and check the two properties that make it safe to run -
// frontier cells are never throttled, and a genuine change is never hidden.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "amr_mapping/map_fusion.hpp"
#include "amr_mapping/selective_mapping.hpp"

namespace
{

using amr_mapping::GridInfo;
using amr_mapping::MapContribution;
using amr_mapping::MapFusion;
using amr_mapping::SelectiveMappingOptions;
using amr_mapping::SelectiveMappingPolicy;
using amr_mapping::SelectiveMappingStats;

constexpr std::int8_t kUnknown = -1;
constexpr std::int8_t kFree = 0;
constexpr std::int8_t kOccupied = 100;

/// \brief A 16 x 16 m grid at 10 cm resolution.
///
/// Size matters here. The policy treats the *edge of the known grid* as an
/// exploration frontier, which is correct - a SLAM map grows outwards, so its
/// border really is where the unexplored world begins - but it means a small
/// test grid is mostly border. At 60 x 60 with a 3-cell frontier radius, 19%
/// of the grid is edge band and no amount of throttling can suppress it. A
/// 160 x 160 grid puts the band at 5%, which is representative of a real
/// warehouse map and lets the throttle's own behaviour be measured.
GridInfo TestGrid(std::uint32_t width = 160, std::uint32_t height = 160)
{
  GridInfo info;
  info.resolution = 0.1;
  info.width = width;
  info.height = height;
  info.origin_x = -8.0;
  info.origin_y = -8.0;
  return info;
}

/// \brief Drive over the whole grid enough times to saturate every cell.
void SaturateEntireGrid(SelectiveMappingPolicy * policy)
{
  for (int pass = 0; pass < 6; ++pass) {
    for (double x = -7.9; x <= 7.9; x += 0.4) {
      for (double y = -7.9; y <= 7.9; y += 0.4) {
        policy->RecordTraversal(x, y);
      }
    }
  }
}

SelectiveMappingOptions TestOptions()
{
  SelectiveMappingOptions options;
  options.frontier_radius = 0.2;
  options.saturation_visits = 4;
  options.saturated_period = 5.0;
  options.explored_period = 1.0;
  options.significant_change = 25;
  options.traversal_radius = 0.5;
  options.min_cells_to_publish = 1;
  return options;
}

/// \brief A grid whose central block is known and whose border is unknown.
std::vector<std::int8_t> PartiallyKnownGrid(const GridInfo & info, std::int8_t fill = kFree)
{
  std::vector<std::int8_t> data(info.CellCount(), kUnknown);
  for (std::uint32_t row = 10; row < info.height - 10; ++row) {
    for (std::uint32_t column = 10; column < info.width - 10; ++column) {
      data[info.Index(static_cast<int>(column), static_cast<int>(row))] = fill;
    }
  }
  return data;
}

/// \brief A fully known grid: no frontier anywhere in the interior.
std::vector<std::int8_t> FullyKnownGrid(const GridInfo & info, std::int8_t fill = kFree)
{
  return std::vector<std::int8_t>(info.CellCount(), fill);
}

// ---------------------------------------------------------------------------
// Grid arithmetic
// ---------------------------------------------------------------------------

TEST(GridInfoTest, WorldAndCellRoundTrip) {
  const GridInfo info = TestGrid();
  int column = 0;
  int row = 0;
  ASSERT_TRUE(info.ToCell(-7.95, -7.95, &column, &row));
  EXPECT_EQ(column, 0);
  EXPECT_EQ(row, 0);

  double x = 0.0;
  double y = 0.0;
  info.ToWorld(column, row, &x, &y);
  EXPECT_NEAR(x, -7.95, 1e-9);
  EXPECT_NEAR(y, -7.95, 1e-9);

  // A point mid-grid must land mid-grid, not fold back to the origin.
  ASSERT_TRUE(info.ToCell(0.05, 0.05, &column, &row));
  EXPECT_EQ(column, 80);
  EXPECT_EQ(row, 80);
}

TEST(GridInfoTest, RejectsOutOfBoundsPoints) {
  const GridInfo info = TestGrid();
  int column = 0;
  int row = 0;
  EXPECT_FALSE(info.ToCell(-99.0, 0.0, &column, &row));
  EXPECT_FALSE(info.ToCell(0.0, 99.0, &column, &row));
}

// ---------------------------------------------------------------------------
// First pass and frontier behaviour
// ---------------------------------------------------------------------------

TEST(SelectiveMappingTest, FirstUpdatePublishesEverythingKnown) {
  SelectiveMappingPolicy policy(TestOptions());
  const GridInfo info = TestGrid();
  policy.Configure(info);

  const std::vector<std::int8_t> source = PartiallyKnownGrid(info);
  std::vector<std::int8_t> filtered;
  const SelectiveMappingStats stats = policy.Filter(source, 0.0, &filtered);

  EXPECT_GT(stats.cells_considered, 0u);
  EXPECT_EQ(stats.cells_suppressed, 0u) << "nothing is known yet, so nothing may be withheld";
  EXPECT_EQ(stats.cells_written, stats.cells_considered);
  EXPECT_TRUE(stats.update_published);
}

TEST(SelectiveMappingTest, FrontierCellsAreNeverThrottled) {
  // The core requirement: unexplored boundaries take priority. Even with the
  // region driven to saturation and the clock barely advanced, the cells next
  // to unknown space must still flow.
  SelectiveMappingPolicy policy(TestOptions());
  const GridInfo info = TestGrid();
  policy.Configure(info);

  const std::vector<std::int8_t> source = PartiallyKnownGrid(info);
  std::vector<std::int8_t> filtered;
  policy.Filter(source, 0.0, &filtered);

  // Saturate the whole known region by driving over it repeatedly.
  SaturateEntireGrid(&policy);

  const SelectiveMappingStats stats = policy.Filter(source, 0.05, &filtered);
  EXPECT_GT(stats.frontier_cells, 0u) << "the fixture must actually have a frontier";
  EXPECT_GE(stats.cells_written, stats.frontier_cells)
    << "every frontier cell must be republished regardless of throttling";
}

TEST(SelectiveMappingTest, RepeatedlyTraversedRegionsAreThrottled) {
  // The other half of the requirement, measured rather than asserted.
  SelectiveMappingPolicy policy(TestOptions());
  const GridInfo info = TestGrid();
  policy.Configure(info);

  // A fully known grid has no interior frontier, isolating the throttle.
  const std::vector<std::int8_t> source = FullyKnownGrid(info);
  std::vector<std::int8_t> filtered;
  policy.Filter(source, 0.0, &filtered);

  SaturateEntireGrid(&policy);

  // 0.5 s later: inside both the explored and the saturated period.
  const SelectiveMappingStats stats = policy.Filter(source, 0.5, &filtered);
  EXPECT_GT(stats.suppression_ratio, 0.9)
    << "a saturated, unchanged, frontier-free map should be almost entirely "
    "suppressed; got " << stats.suppression_ratio;
  EXPECT_EQ(stats.policy_state, "throttled");
}

TEST(SelectiveMappingTest, SuppressionRisesWithFamiliarity) {
  // The headline number for the demo: the same map costs less to share the
  // better it is known.
  SelectiveMappingPolicy policy(TestOptions());
  const GridInfo info = TestGrid();
  policy.Configure(info);

  const std::vector<std::int8_t> source = FullyKnownGrid(info);
  std::vector<std::int8_t> filtered;
  policy.Filter(source, 0.0, &filtered);

  const SelectiveMappingStats fresh = policy.Filter(source, 0.1, &filtered);

  SaturateEntireGrid(&policy);
  const SelectiveMappingStats familiar = policy.Filter(source, 2.0, &filtered);

  // At t = 2.0 an unsaturated cell would be due (explored_period = 1.0) but a
  // saturated one is not (saturated_period = 5.0).
  EXPECT_GT(familiar.suppression_ratio, fresh.suppression_ratio - 1e-9);
  EXPECT_GT(familiar.suppression_ratio, 0.9);
}

TEST(SelectiveMappingTest, ThrottledCellsEventuallyRefresh) {
  // Suppression must be a delay, not a deletion: a permanently withheld cell
  // would let the merged map drift away from reality for good.
  SelectiveMappingPolicy policy(TestOptions());
  const GridInfo info = TestGrid();
  policy.Configure(info);

  const std::vector<std::int8_t> source = FullyKnownGrid(info);
  std::vector<std::int8_t> filtered;
  policy.Filter(source, 0.0, &filtered);

  SaturateEntireGrid(&policy);

  EXPECT_GT(policy.Filter(source, 1.0, &filtered).suppression_ratio, 0.9);
  // Past saturated_period = 5.0 s the whole region is due again.
  const SelectiveMappingStats refreshed = policy.Filter(source, 6.0, &filtered);
  EXPECT_LT(refreshed.suppression_ratio, 0.1);
}

TEST(SelectiveMappingTest, ASignificantChangeBypassesThrottling) {
  // A rack that has moved is news even in the most familiar aisle.
  SelectiveMappingPolicy policy(TestOptions());
  const GridInfo info = TestGrid();
  policy.Configure(info);

  std::vector<std::int8_t> source = FullyKnownGrid(info, kFree);
  std::vector<std::int8_t> filtered;
  policy.Filter(source, 0.0, &filtered);

  SaturateEntireGrid(&policy);
  ASSERT_GT(policy.Filter(source, 0.5, &filtered).suppression_ratio, 0.9);

  // Something solid appears in the middle of the well-known region.
  const std::size_t index = info.Index(30, 30);
  source[index] = kOccupied;

  const SelectiveMappingStats stats = policy.Filter(source, 0.6, &filtered);
  EXPECT_EQ(filtered[index], kOccupied)
    << "a new obstacle must reach the merged map immediately, however well "
    "travelled the aisle";
  EXPECT_GT(stats.cells_written, 0u);
}

TEST(SelectiveMappingTest, SmallFluctuationsDoNotBypassThrottling) {
  // The mirror image: sensor noise must not defeat the throttle, or the whole
  // mechanism achieves nothing on a real robot.
  SelectiveMappingPolicy policy(TestOptions());
  const GridInfo info = TestGrid();
  policy.Configure(info);

  std::vector<std::int8_t> source = FullyKnownGrid(info, static_cast<std::int8_t>(40));
  std::vector<std::int8_t> filtered;
  policy.Filter(source, 0.0, &filtered);

  SaturateEntireGrid(&policy);
  policy.Filter(source, 0.5, &filtered);

  // A 10-point wobble, below the 25-point significance threshold.
  for (auto & value : source) {
    value = static_cast<std::int8_t>(50);
  }
  const SelectiveMappingStats stats = policy.Filter(source, 0.6, &filtered);
  EXPECT_GT(stats.suppression_ratio, 0.9) << "noise defeated the throttle";
}

TEST(SelectiveMappingTest, TheEdgeOfTheKnownGridCountsAsFrontier) {
  // A SLAM map grows outwards, so its border is literally where the
  // unexplored world begins. Throttling the border would slow exactly the
  // updates that extend the map, so out-of-bounds probes count as unknown.
  SelectiveMappingPolicy policy(TestOptions());
  const GridInfo info = TestGrid();
  policy.Configure(info);

  const std::vector<std::int8_t> source = FullyKnownGrid(info);
  std::vector<std::int8_t> filtered;
  policy.Filter(source, 0.0, &filtered);
  SaturateEntireGrid(&policy);

  const SelectiveMappingStats stats = policy.Filter(source, 0.5, &filtered);
  EXPECT_GT(stats.frontier_cells, 0u)
    << "a fully known grid still has a frontier: its own edge";
  EXPECT_GT(stats.cells_written, 0u);

  // A corner cell is on the edge band and must always be written.
  EXPECT_NE(filtered[info.Index(0, 0)], kUnknown);
  // A deep interior cell must not be.
  EXPECT_EQ(filtered[info.Index(80, 80)], kUnknown);
}

TEST(SelectiveMappingTest, UnknownCellsAreNeverCounted) {
  SelectiveMappingPolicy policy(TestOptions());
  const GridInfo info = TestGrid();
  policy.Configure(info);

  const std::vector<std::int8_t> empty(info.CellCount(), kUnknown);
  std::vector<std::int8_t> filtered;
  const SelectiveMappingStats stats = policy.Filter(empty, 0.0, &filtered);

  EXPECT_EQ(stats.cells_considered, 0u);
  EXPECT_EQ(stats.cells_written, 0u);
  EXPECT_FALSE(stats.update_published);
  EXPECT_EQ(stats.policy_state, "idle");
}

TEST(SelectiveMappingTest, TraversalSaturatesRatherThanOverflowing) {
  SelectiveMappingPolicy policy(TestOptions());
  const GridInfo info = TestGrid();
  policy.Configure(info);

  for (int i = 0; i < 1000; ++i) {
    policy.RecordTraversal(0.0, 0.0);
  }
  int column = 0;
  int row = 0;
  ASSERT_TRUE(info.ToCell(0.0, 0.0, &column, &row));
  EXPECT_EQ(policy.VisitCount(column, row), TestOptions().saturation_visits)
    << "an overflowing counter would silently reset a region to unexplored";
}

TEST(SelectiveMappingTest, TraversalOnlyMarksTheLocalNeighbourhood) {
  SelectiveMappingPolicy policy(TestOptions());
  const GridInfo info = TestGrid();
  policy.Configure(info);

  policy.RecordTraversal(0.0, 0.0);

  int near_column = 0;
  int near_row = 0;
  ASSERT_TRUE(info.ToCell(0.2, 0.0, &near_column, &near_row));
  EXPECT_GT(policy.VisitCount(near_column, near_row), 0u);

  int far_column = 0;
  int far_row = 0;
  ASSERT_TRUE(info.ToCell(2.0, 2.0, &far_column, &far_row));
  EXPECT_EQ(policy.VisitCount(far_column, far_row), 0u);
}

TEST(SelectiveMappingTest, MismatchedGridSizeIsRejected) {
  SelectiveMappingPolicy policy(TestOptions());
  policy.Configure(TestGrid());

  const std::vector<std::int8_t> wrong_size(10, kFree);
  std::vector<std::int8_t> filtered;
  const SelectiveMappingStats stats = policy.Filter(wrong_size, 0.0, &filtered);
  EXPECT_FALSE(stats.update_published);
  EXPECT_EQ(stats.cells_considered, 0u);
}

// ---------------------------------------------------------------------------
// Map fusion
// ---------------------------------------------------------------------------

MapFusion::Options FusionOptions()
{
  MapFusion::Options options;
  options.resolution = 0.1;
  options.x_min = -5.0;
  options.x_max = 5.0;
  options.y_min = -5.0;
  options.y_max = 5.0;
  return options;
}

MapContribution MakeContribution(
  const std::string & id, std::int8_t fill, double offset_x, double offset_y)
{
  MapContribution contribution;
  contribution.robot_id = id;
  contribution.info.resolution = 0.1;
  contribution.info.width = 20;
  contribution.info.height = 20;
  contribution.info.origin_x = 0.0;
  contribution.info.origin_y = 0.0;
  contribution.data.assign(contribution.info.CellCount(), fill);
  contribution.offset_x = offset_x;
  contribution.offset_y = offset_y;
  return contribution;
}

TEST(MapFusionTest, StartsEntirelyUnknown) {
  MapFusion fusion(FusionOptions());
  EXPECT_EQ(fusion.ObservedCells(), 0u);
  EXPECT_NEAR(fusion.UnknownFraction(), 1.0, 1e-9);

  std::vector<std::int8_t> rendered;
  fusion.Render(&rendered);
  for (const std::int8_t value : rendered) {
    ASSERT_EQ(value, kUnknown);
  }
}

TEST(MapFusionTest, BothRobotsContributeToOneMap) {
  // The cooperative-mapping requirement in its simplest form: two robots
  // mapping disjoint regions produce one map covering both.
  MapFusion fusion(FusionOptions());
  fusion.Integrate(MakeContribution("amr1", kFree, -4.0, -4.0));
  const std::size_t after_first = fusion.ObservedCells();
  EXPECT_GT(after_first, 0u);

  fusion.Integrate(MakeContribution("amr2", kFree, 1.0, 1.0));
  EXPECT_GT(fusion.ObservedCells(), after_first)
    << "the second robot must add coverage, not overwrite the first";
}

TEST(MapFusionTest, SuppressedCellsAreTreatedAsNoEvidence) {
  // What makes fusion compose with selective mapping: a suppressed cell is an
  // absence of evidence, not evidence of absence.
  MapFusion fusion(FusionOptions());
  MapContribution contribution = MakeContribution("amr1", kUnknown, 0.0, 0.0);
  EXPECT_EQ(fusion.Integrate(contribution), 0u);
  EXPECT_EQ(fusion.ObservedCells(), 0u);
}

TEST(MapFusionTest, RepeatedAgreementConvergesToOccupied) {
  MapFusion fusion(FusionOptions());
  for (int i = 0; i < 10; ++i) {
    fusion.Integrate(MakeContribution("amr1", kOccupied, 0.0, 0.0));
  }
  std::vector<std::int8_t> rendered;
  fusion.Render(&rendered);

  int column = 0;
  int row = 0;
  ASSERT_TRUE(fusion.Info().ToCell(1.0, 1.0, &column, &row));
  EXPECT_EQ(rendered[fusion.Info().Index(column, row)], kOccupied);
}

TEST(MapFusionTest, ASingleTransientObservationIsOverturnable) {
  // The reason for log-odds rather than a max: one pedestrian seen once must
  // not close an aisle permanently.
  MapFusion fusion(FusionOptions());
  fusion.Integrate(MakeContribution("amr1", kOccupied, 0.0, 0.0));

  for (int i = 0; i < 5; ++i) {
    fusion.Integrate(MakeContribution("amr2", kFree, 0.0, 0.0));
  }

  std::vector<std::int8_t> rendered;
  fusion.Render(&rendered);
  int column = 0;
  int row = 0;
  ASSERT_TRUE(fusion.Info().ToCell(1.0, 1.0, &column, &row));
  EXPECT_EQ(rendered[fusion.Info().Index(column, row)], kFree)
    << "a max-occupancy fusion would leave this cell blocked for good";
}

TEST(MapFusionTest, ConfidenceIsClampedSoTheMapStaysResponsive) {
  // Without a clamp a long-held belief becomes unfalsifiable and the map stops
  // tracking reality.
  MapFusion fusion(FusionOptions());
  for (int i = 0; i < 500; ++i) {
    fusion.Integrate(MakeContribution("amr1", kOccupied, 0.0, 0.0));
  }
  for (int i = 0; i < 12; ++i) {
    fusion.Integrate(MakeContribution("amr2", kFree, 0.0, 0.0));
  }

  std::vector<std::int8_t> rendered;
  fusion.Render(&rendered);
  int column = 0;
  int row = 0;
  ASSERT_TRUE(fusion.Info().ToCell(1.0, 1.0, &column, &row));
  EXPECT_EQ(rendered[fusion.Info().Index(column, row)], kFree)
    << "500 stale observations must not outvote 12 current ones forever";
}

TEST(MapFusionTest, ContributionsAreTransformedIntoTheGlobalFrame) {
  MapFusion fusion(FusionOptions());
  // A 2x2 m patch placed at (-4, -4) must land there, not at the origin.
  fusion.Integrate(MakeContribution("amr1", kOccupied, -4.0, -4.0));

  std::vector<std::int8_t> rendered;
  fusion.Render(&rendered);

  int inside_column = 0;
  int inside_row = 0;
  ASSERT_TRUE(fusion.Info().ToCell(-3.0, -3.0, &inside_column, &inside_row));
  EXPECT_NE(rendered[fusion.Info().Index(inside_column, inside_row)], kUnknown);

  int outside_column = 0;
  int outside_row = 0;
  ASSERT_TRUE(fusion.Info().ToCell(3.0, 3.0, &outside_column, &outside_row));
  EXPECT_EQ(rendered[fusion.Info().Index(outside_column, outside_row)], kUnknown);
}

TEST(MapFusionTest, ContributionsOutsideTheExtentAreDroppedNotWrapped) {
  MapFusion fusion(FusionOptions());
  const std::size_t updated = fusion.Integrate(MakeContribution("amr1", kFree, 100.0, 100.0));
  EXPECT_EQ(updated, 0u);
  EXPECT_EQ(fusion.ObservedCells(), 0u);
}

TEST(MapFusionTest, MalformedContributionsAreIgnored) {
  MapFusion fusion(FusionOptions());
  MapContribution contribution = MakeContribution("amr1", kFree, 0.0, 0.0);
  contribution.data.resize(5);   // Inconsistent with its own metadata.
  EXPECT_EQ(fusion.Integrate(contribution), 0u);
}

TEST(MapFusionTest, EndToEndSelectiveMappingIntoFusion) {
  // The whole pipeline: SLAM grid -> selective filter -> merged map. The point
  // is that heavy suppression does not degrade the merged result, because the
  // fusion accumulator retains what it was told earlier.
  const GridInfo info = TestGrid();
  SelectiveMappingPolicy policy(TestOptions());
  policy.Configure(info);
  MapFusion fusion(FusionOptions());

  const std::vector<std::int8_t> source = FullyKnownGrid(info, kOccupied);
  std::vector<std::int8_t> filtered;

  // First pass: everything flows.
  policy.Filter(source, 0.0, &filtered);
  MapContribution contribution;
  contribution.robot_id = "amr1";
  contribution.info = info;
  contribution.data = filtered;
  contribution.offset_x = 0.0;
  contribution.offset_y = 0.0;
  const std::size_t first = fusion.Integrate(contribution);
  EXPECT_GT(first, 0u);

  // Saturate, then filter again: almost nothing should flow.
  SaturateEntireGrid(&policy);
  const SelectiveMappingStats stats = policy.Filter(source, 0.5, &filtered);
  contribution.data = filtered;
  const std::size_t second = fusion.Integrate(contribution);

  EXPECT_LT(second, first) << "the second pass should carry far less traffic";
  EXPECT_GT(stats.suppression_ratio, 0.5);

  // The merged map must still know about the region.
  EXPECT_GT(fusion.ObservedCells(), 0u);
}

}  // namespace

// ---------------------------------------------------------------------------
// Diagnosing a zero.
//
// Integrate() returns 0 for three unrelated reasons, and the one that actually
// happened -- a double-applied start pose putting every cell outside the merged
// grid -- was indistinguishable from an idle fleet. The report exists so that
// "0.0% explored" can never again be the most specific thing the system says.
// ---------------------------------------------------------------------------

TEST(MapFusionReportTest, TheReportSeparatesTheThreeWaysOfGettingZero) {
  MapFusion fusion(FusionOptions());

  // 1. Nothing to say: the contribution is entirely unknown.
  MapContribution empty = MakeContribution("amr1", kFree, 0.0, 0.0);
  std::fill(empty.data.begin(), empty.data.end(), static_cast<std::int8_t>(-1));
  EXPECT_EQ(fusion.Integrate(empty), 0u);
  EXPECT_EQ(fusion.LastReport().outside_extent, 0u);
  EXPECT_FALSE(fusion.LastReport().geometry_mismatch);
  EXPECT_GT(fusion.LastReport().skipped_unknown, 0u);

  // 2. Malformed: declared size disagrees with the payload.
  MapContribution malformed = MakeContribution("amr1", kFree, 0.0, 0.0);
  malformed.data.pop_back();
  EXPECT_EQ(fusion.Integrate(malformed), 0u);
  EXPECT_TRUE(fusion.LastReport().geometry_mismatch);

  // 3. The frame bug: real evidence, all of it in the wrong place.
  EXPECT_EQ(fusion.Integrate(MakeContribution("amr1", kFree, 100.0, 100.0)), 0u);
  EXPECT_GT(fusion.LastReport().outside_extent, 0u);
  EXPECT_FALSE(fusion.LastReport().geometry_mismatch);
}

TEST(MapFusionReportTest, TheReportLocatesTheEvidenceSoAnOffsetErrorIsVisible) {
  // The shape of the real bug: odometry already carried the spawn pose, so the
  // roster offset was applied a second time and the contribution landed at
  // twice the distance from the origin. The bounding box makes that readable
  // at a glance instead of requiring a rebuild with printf.
  MapFusion fusion(FusionOptions());
  fusion.Integrate(MakeContribution("amr1", kFree, 37.0, 37.0));

  const MapFusion::IntegrationReport & report = fusion.LastReport();
  ASSERT_TRUE(report.has_bounds);
  EXPECT_GT(
    report.min_x,
    fusion.Info().origin_x + fusion.Info().width * fusion.Info().resolution);
  EXPECT_EQ(report.updated, 0u);
}

TEST(MapFusionReportTest, ASuccessfulIntegrationReportsNoProblem) {
  MapFusion fusion(FusionOptions());
  const std::size_t updated = fusion.Integrate(MakeContribution("amr1", kFree, 0.0, 0.0));
  ASSERT_GT(updated, 0u);
  EXPECT_EQ(fusion.LastReport().updated, updated);
  EXPECT_EQ(fusion.LastReport().outside_extent, 0u);
  EXPECT_FALSE(fusion.LastReport().geometry_mismatch);
  EXPECT_TRUE(fusion.LastReport().has_bounds);
}
