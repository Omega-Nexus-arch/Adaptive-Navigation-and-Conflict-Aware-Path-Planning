// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.

#include "amr_mapping/selective_mapping.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace amr_mapping
{

namespace
{
/// Occupancy grids use -1 for "never observed".
constexpr std::int8_t kUnknown = -1;
}  // namespace

// ---------------------------------------------------------------------------
// GridInfo
// ---------------------------------------------------------------------------

bool GridInfo::ToCell(double x, double y, int * column, int * row) const
{
  if (resolution <= 0.0) {
    return false;
  }
  const int candidate_column = static_cast<int>(std::floor((x - origin_x) / resolution));
  const int candidate_row = static_cast<int>(std::floor((y - origin_y) / resolution));
  if (!InBounds(candidate_column, candidate_row)) {
    return false;
  }
  *column = candidate_column;
  *row = candidate_row;
  return true;
}

void GridInfo::ToWorld(int column, int row, double * x, double * y) const
{
  *x = origin_x + (static_cast<double>(column) + 0.5) * resolution;
  *y = origin_y + (static_cast<double>(row) + 0.5) * resolution;
}

bool GridInfo::SameGeometry(const GridInfo & other) const
{
  return width == other.width && height == other.height &&
         std::abs(resolution - other.resolution) < 1e-9 &&
         std::abs(origin_x - other.origin_x) < 1e-6 &&
         std::abs(origin_y - other.origin_y) < 1e-6;
}

// ---------------------------------------------------------------------------
// SelectiveMappingPolicy
// ---------------------------------------------------------------------------

SelectiveMappingPolicy::SelectiveMappingPolicy(const SelectiveMappingOptions & options)
: options_(options) {}

void SelectiveMappingPolicy::Configure(const GridInfo & info)
{
  info_ = info;
  const std::size_t cells = info.CellCount();

  // SLAM grows its grid as the robot explores. Reallocating discards the visit
  // history, which is the right trade: keeping it would need a resample, and
  // the cost of re-learning "this aisle is well travelled" is a few seconds of
  // extra updates, whereas a misaligned history would suppress updates in the
  // wrong places - a far worse failure.
  visits_.assign(cells, 0u);
  last_published_time_.assign(cells, -1.0e9);
  published_.assign(cells, kUnknown);

  frontier_cells_radius_ =
    std::max(1, static_cast<int>(std::round(options_.frontier_radius / info.resolution)));
  traversal_cells_radius_ =
    std::max(1, static_cast<int>(std::round(options_.traversal_radius / info.resolution)));
  configured_ = true;
}

void SelectiveMappingPolicy::RecordTraversal(double x, double y)
{
  if (!configured_) {
    return;
  }
  int centre_column = 0;
  int centre_row = 0;
  if (!info_.ToCell(x, y, &centre_column, &centre_row)) {
    return;
  }

  const int radius = traversal_cells_radius_;
  const int radius_squared = radius * radius;
  for (int d_row = -radius; d_row <= radius; ++d_row) {
    for (int d_column = -radius; d_column <= radius; ++d_column) {
      if (d_row * d_row + d_column * d_column > radius_squared) {
        continue;
      }
      const int column = centre_column + d_column;
      const int row = centre_row + d_row;
      if (!info_.InBounds(column, row)) {
        continue;
      }
      std::uint32_t & count = visits_[info_.Index(column, row)];
      // Saturate rather than wrap. Beyond the threshold the exact count
      // carries no further information and an overflow would silently reset a
      // region to "unexplored".
      if (count < options_.saturation_visits) {
        ++count;
      }
    }
  }
}

std::uint32_t SelectiveMappingPolicy::VisitCount(int column, int row) const
{
  if (!configured_ || !info_.InBounds(column, row)) {
    return 0u;
  }
  return visits_[info_.Index(column, row)];
}

bool SelectiveMappingPolicy::IsFrontier(
  const std::vector<std::int8_t> & source, int column, int row) const
{
  // Sampling the ring rather than the filled disc: a frontier is a boundary,
  // so an annulus of probes finds it just as reliably at a fraction of the
  // cost. This runs once per known cell per update, so the constant matters.
  const int radius = frontier_cells_radius_;
  static constexpr int kDirections = 8;
  static constexpr double kAngleStep = 2.0 * M_PI / kDirections;

  for (int step = 1; step <= radius; ++step) {
    for (int direction = 0; direction < kDirections; ++direction) {
      const double angle = direction * kAngleStep;
      const int probe_column = column + static_cast<int>(std::round(step * std::cos(angle)));
      const int probe_row = row + static_cast<int>(std::round(step * std::sin(angle)));
      if (!info_.InBounds(probe_column, probe_row)) {
        // The edge of the known grid is itself an exploration boundary.
        return true;
      }
      if (source[info_.Index(probe_column, probe_row)] == kUnknown) {
        return true;
      }
    }
  }
  return false;
}

SelectiveMappingStats SelectiveMappingPolicy::Filter(
  const std::vector<std::int8_t> & source, double now, std::vector<std::int8_t> * out_filtered)
{
  SelectiveMappingStats stats;
  if (!configured_ || source.size() != info_.CellCount() || out_filtered == nullptr) {
    return stats;
  }

  out_filtered->assign(source.size(), kUnknown);

  std::uint32_t frontier_writes = 0;
  std::uint32_t saturated_seen = 0;

  for (std::uint32_t row = 0; row < info_.height; ++row) {
    for (std::uint32_t column = 0; column < info_.width; ++column) {
      const std::size_t index = info_.Index(static_cast<int>(column), static_cast<int>(row));
      const std::int8_t value = source[index];

      // Never-observed cells carry no information to share.
      if (value == kUnknown) {
        continue;
      }
      ++stats.cells_considered;

      const std::uint32_t visits = visits_[index];
      const bool saturated = visits >= options_.saturation_visits;
      if (saturated) {
        ++saturated_seen;
      }

      const bool frontier =
        IsFrontier(source, static_cast<int>(column), static_cast<int>(row));
      if (frontier) {
        ++stats.frontier_cells;
      }

      const std::int8_t previous = published_[index];
      const bool never_published = previous == kUnknown;
      const bool changed_a_lot = !never_published &&
        std::abs(static_cast<int>(value) - static_cast<int>(previous)) >=
        options_.significant_change;

      const double elapsed = now - last_published_time_[index];
      const double period = saturated ? options_.saturated_period : options_.explored_period;

      // The decision. Frontier cells and genuine changes bypass throttling
      // entirely; everything else waits for its period.
      const bool write = frontier || never_published || changed_a_lot || elapsed >= period;

      if (!write) {
        ++stats.cells_suppressed;
        continue;
      }

      (*out_filtered)[index] = value;
      published_[index] = value;
      last_published_time_[index] = now;
      ++stats.cells_written;
      if (frontier) {
        ++frontier_writes;
      }
    }
  }

  stats.suppression_ratio = stats.cells_considered == 0 ?
    0.0 :
    static_cast<double>(stats.cells_suppressed) / static_cast<double>(stats.cells_considered);
  stats.update_published = stats.cells_written >= options_.min_cells_to_publish;

  // A short label for the telemetry topic and the RViz overlay, so the policy
  // state is legible on the demo video without reading numbers.
  // Ordered by which effect dominates. Suppression is checked first: once
  // most of the map is being withheld, "throttled" is the honest summary even
  // though the few cells still flowing are, necessarily, frontier cells.
  if (stats.cells_considered == 0) {
    stats.policy_state = "idle";
  } else if (stats.suppression_ratio >= 0.5) {
    stats.policy_state = "throttled";
  } else if (frontier_writes * 2 >= stats.cells_written) {
    stats.policy_state = "frontier_burst";
  } else if (saturated_seen * 2 >= stats.cells_considered) {
    stats.policy_state = "throttled";
  } else {
    stats.policy_state = "exploring";
  }

  return stats;
}

}  // namespace amr_mapping
