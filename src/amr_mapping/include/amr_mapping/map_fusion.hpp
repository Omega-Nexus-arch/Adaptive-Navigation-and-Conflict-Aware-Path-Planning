// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_MAPPING__MAP_FUSION_HPP_
#define AMR_MAPPING__MAP_FUSION_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "amr_mapping/selective_mapping.hpp"

namespace amr_mapping
{

/// \brief One robot's contribution, already expressed in the global frame.
struct MapContribution
{
  std::string robot_id;
  GridInfo info;
  std::vector<std::int8_t> data;
  /// Rigid offset from this robot's map frame to the shared global frame.
  double offset_x = 0.0;
  double offset_y = 0.0;
  double offset_yaw = 0.0;
  double stamp = 0.0;
};

/// \brief Fuses per-robot occupancy grids into one warehouse-wide map.
///
/// ### Why a log-odds accumulator
///
/// The naive fusion is to take the maximum occupancy across robots. It has a
/// specific pathology in this warehouse: a moving pedestrian seen once by one
/// robot becomes permanently occupied in the merged map, and the aisle closes
/// for good. Accumulating evidence in log-odds instead means a single
/// observation is a weak claim that later contrary observations can overturn,
/// while a rack seen repeatedly from both robots converges hard to occupied.
///
/// Independence between the two robots' observations is assumed, which is not
/// strictly true - they share a world and sometimes a viewpoint - so the
/// per-observation weight is deliberately modest. Overconfidence here shows up
/// as exactly the stale-obstacle problem the accumulator exists to avoid.
///
/// ### Frames
///
/// Each robot runs its own SLAM and owns a private `<robot>/map` frame. Their
/// initial poses are known from the fleet roster, which fixes the rigid
/// transform into the shared `map` frame. The fusion node publishes those as
/// static transforms, so the whole fleet's TF tree has a single root and the
/// merged grid is what every planner consumes.
class MapFusion
{
public:
  struct Options
  {
    double resolution = 0.05;
    /// Extent of the merged grid in the global frame [m].
    double x_min = -25.0;
    double x_max = 25.0;
    double y_min = -18.0;
    double y_max = 18.0;

    /// Log-odds increment per observation. Modest on purpose; see above.
    double occupied_delta = 0.65;
    double free_delta = -0.45;
    /// Clamp keeps the accumulator responsive: without it a long-standing
    /// belief becomes unfalsifiable and the map stops tracking reality.
    double log_odds_min = -3.5;
    double log_odds_max = 3.5;

    /// Probabilities outside these bounds are reported as free / occupied.
    double free_threshold = 0.25;
    double occupied_threshold = 0.65;

    /// Contributions older than this are ignored [s].
    double contribution_timeout = 30.0;
  };

  /// \brief Why the last Integrate() call produced the count it did.
  ///
  /// Integrate() can return zero for three unrelated reasons, and for a long
  /// time the caller could not tell them apart -- which is how a frame bug
  /// (every cell landing outside the merged extent, because the robot's start
  /// pose was applied twice) presented as nothing more informative than
  /// "0.0% explored". A count is a measurement; this is the diagnosis.
  struct IntegrationReport
  {
    std::size_t updated = 0;
    std::size_t skipped_unknown = 0;
    /// Known cells whose global position fell outside the merged grid. A large
    /// value here is almost always a frame or offset error, not a small map.
    std::size_t outside_extent = 0;
    /// The contribution's declared size disagreed with its payload.
    bool geometry_mismatch = false;
    /// Bounding box of the contribution's known cells, in the shared frame.
    /// Compare it against the merged extent to see the offset directly.
    bool has_bounds = false;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
  };

  explicit MapFusion(const Options & options);

  /// \brief Fold one robot's (already filtered) grid into the accumulator.
  ///
  /// Cells marked unknown in the contribution are skipped, which is what makes
  /// this composable with the selective-mapping policy: a suppressed cell is
  /// simply an absence of new evidence, not evidence of absence.
  ///
  /// \return Number of global cells updated.
  std::size_t Integrate(const MapContribution & contribution);

  /// \brief Diagnosis of the most recent Integrate() call.
  const IntegrationReport & LastReport() const {return last_report_;}

  /// \brief Render the accumulator as an occupancy grid.
  void Render(std::vector<std::int8_t> * out_data) const;

  const GridInfo & Info() const {return info_;}

  /// \brief Cells that have received at least one observation.
  std::size_t ObservedCells() const;

  /// \brief Fraction of the merged grid that is still unknown.
  double UnknownFraction() const;

  void Reset();

private:
  Options options_;
  GridInfo info_;
  std::vector<double> log_odds_;
  std::vector<std::uint8_t> observed_;
  IntegrationReport last_report_;
};

}  // namespace amr_mapping

#endif  // AMR_MAPPING__MAP_FUSION_HPP_
