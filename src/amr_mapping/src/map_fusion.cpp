// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.

#include "amr_mapping/map_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace amr_mapping
{

namespace
{
constexpr std::int8_t kUnknown = -1;
constexpr std::int8_t kFree = 0;
constexpr std::int8_t kOccupied = 100;

/// \brief Log-odds back to probability.
double ToProbability(double log_odds)
{
  return 1.0 - 1.0 / (1.0 + std::exp(log_odds));
}
}  // namespace

MapFusion::MapFusion(const Options & options)
: options_(options)
{
  info_.resolution = options.resolution;
  info_.origin_x = options.x_min;
  info_.origin_y = options.y_min;
  info_.width =
    static_cast<std::uint32_t>(std::ceil((options.x_max - options.x_min) / options.resolution));
  info_.height =
    static_cast<std::uint32_t>(std::ceil((options.y_max - options.y_min) / options.resolution));
  Reset();
}

void MapFusion::Reset()
{
  log_odds_.assign(info_.CellCount(), 0.0);
  observed_.assign(info_.CellCount(), 0u);
}

std::size_t MapFusion::Integrate(const MapContribution & contribution)
{
  last_report_ = IntegrationReport();

  if (contribution.data.size() != contribution.info.CellCount()) {
    last_report_.geometry_mismatch = true;
    return 0;
  }

  const double cos_yaw = std::cos(contribution.offset_yaw);
  const double sin_yaw = std::sin(contribution.offset_yaw);
  std::size_t updated = 0;

  for (std::uint32_t row = 0; row < contribution.info.height; ++row) {
    for (std::uint32_t column = 0; column < contribution.info.width; ++column) {
      const std::size_t source_index =
        contribution.info.Index(static_cast<int>(column), static_cast<int>(row));
      const std::int8_t value = contribution.data[source_index];

      // A cell the selective-mapping policy suppressed arrives as unknown.
      // Skipping it is what makes the two components compose: absence of new
      // evidence must not be read as evidence of absence.
      if (value == kUnknown) {
        ++last_report_.skipped_unknown;
        continue;
      }

      double local_x = 0.0;
      double local_y = 0.0;
      contribution.info.ToWorld(
        static_cast<int>(column), static_cast<int>(row), &local_x, &local_y);

      // Rigid transform from the robot's private map frame into the shared one.
      const double global_x =
        contribution.offset_x + local_x * cos_yaw - local_y * sin_yaw;
      const double global_y =
        contribution.offset_y + local_x * sin_yaw + local_y * cos_yaw;

      // Track where this robot's evidence actually lands. When none of it
      // lands inside the grid, this box is what names the reason.
      if (!last_report_.has_bounds) {
        last_report_.has_bounds = true;
        last_report_.min_x = last_report_.max_x = global_x;
        last_report_.min_y = last_report_.max_y = global_y;
      } else {
        last_report_.min_x = std::min(last_report_.min_x, global_x);
        last_report_.max_x = std::max(last_report_.max_x, global_x);
        last_report_.min_y = std::min(last_report_.min_y, global_y);
        last_report_.max_y = std::max(last_report_.max_y, global_y);
      }

      int target_column = 0;
      int target_row = 0;
      if (!info_.ToCell(global_x, global_y, &target_column, &target_row)) {
        ++last_report_.outside_extent;
        continue;
      }

      const std::size_t target_index = info_.Index(target_column, target_row);
      // Occupancy grids are 0-100; treat the midpoint as the decision boundary
      // rather than demanding an exact 0 or 100, so partially-confident SLAM
      // output still contributes.
      const double delta = (value >= 50) ? options_.occupied_delta : options_.free_delta;

      log_odds_[target_index] = std::min(
        options_.log_odds_max, std::max(options_.log_odds_min, log_odds_[target_index] + delta));
      observed_[target_index] = 1u;
      ++updated;
    }
  }

  last_report_.updated = updated;
  return updated;
}

void MapFusion::Render(std::vector<std::int8_t> * out_data) const
{
  if (out_data == nullptr) {
    return;
  }
  out_data->assign(info_.CellCount(), kUnknown);

  for (std::size_t index = 0; index < log_odds_.size(); ++index) {
    if (observed_[index] == 0u) {
      continue;   // Never seen by anyone: genuinely unknown.
    }
    const double probability = ToProbability(log_odds_[index]);
    if (probability >= options_.occupied_threshold) {
      (*out_data)[index] = kOccupied;
    } else if (probability <= options_.free_threshold) {
      (*out_data)[index] = kFree;
    } else {
      // Contested: neither robot has convinced the accumulator. Publishing the
      // scaled probability rather than forcing a verdict lets the costmap's
      // own thresholds make the final call.
      (*out_data)[index] = static_cast<std::int8_t>(std::lround(probability * 100.0));
    }
  }
}

std::size_t MapFusion::ObservedCells() const
{
  return static_cast<std::size_t>(std::count(observed_.begin(), observed_.end(), 1u));
}

double MapFusion::UnknownFraction() const
{
  const std::size_t total = info_.CellCount();
  if (total == 0) {
    return 1.0;
  }
  return 1.0 - static_cast<double>(ObservedCells()) / static_cast<double>(total);
}

}  // namespace amr_mapping
