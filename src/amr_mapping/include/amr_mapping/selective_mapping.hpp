// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_MAPPING__SELECTIVE_MAPPING_HPP_
#define AMR_MAPPING__SELECTIVE_MAPPING_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace amr_mapping
{

/// \brief Geometry of an occupancy grid, mirroring `nav_msgs/MapMetaData`.
struct GridInfo
{
  double resolution = 0.05;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;

  std::size_t CellCount() const
  {
    return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  }

  bool InBounds(int column, int row) const
  {
    return column >= 0 && row >= 0 && column < static_cast<int>(width) &&
           row < static_cast<int>(height);
  }

  std::size_t Index(int column, int row) const
  {
    return static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(column);
  }

  /// \brief World point to cell. Returns false when outside the grid.
  bool ToCell(double x, double y, int * column, int * row) const;

  /// \brief Centre of a cell in world coordinates.
  void ToWorld(int column, int row, double * x, double * y) const;

  bool SameGeometry(const GridInfo & other) const;
};

/// \brief What the policy decided for one incoming map update.
struct SelectiveMappingStats
{
  bool update_published = false;
  std::uint32_t cells_considered = 0;
  std::uint32_t cells_written = 0;
  std::uint32_t cells_suppressed = 0;
  std::uint32_t frontier_cells = 0;
  double suppression_ratio = 0.0;
  std::string policy_state = "idle";
};

/// \brief Configuration of the selective-iteration mapping policy.
struct SelectiveMappingOptions
{
  /// A cell within this distance of unknown space counts as frontier-adjacent
  /// and is always republished [m].
  double frontier_radius = 1.0;

  /// Visit count beyond which a region is considered well explored.
  std::uint32_t saturation_visits = 8;

  /// Minimum republish interval for a saturated region [s]. Frontier cells
  /// ignore this entirely.
  double saturated_period = 5.0;

  /// Minimum republish interval for a partially explored region [s].
  double explored_period = 1.0;

  /// A cell whose occupancy differs from the published value by more than this
  /// (on the 0-100 scale) is republished regardless of throttling: a rack that
  /// has moved is news even in a well-mapped aisle.
  int significant_change = 25;

  /// Radius around the robot's own pose whose visit count is incremented [m].
  double traversal_radius = 1.5;

  /// Publish nothing at all unless at least this many cells changed. Stops the
  /// merged map churning on sensor noise.
  std::uint32_t min_cells_to_publish = 12;
};

/// \brief Decides which parts of a robot's map are worth republishing.
///
/// ### The requirement
///
/// AMR-1's contributions to the merged map must prioritise unexplored
/// boundaries and reduce the update frequency for regions it has repeatedly
/// traversed - "selective iteration" in the brief. The success metric is a
/// demonstrated ability to programmatically restrict the map update area
/// according to a defined condition.
///
/// ### How the decision is made
///
/// Every cell is classified, and the class decides the republish period:
///
/// | Class | Condition | Period |
/// |---|---|---|
/// | Frontier | within `frontier_radius` of unknown space | always |
/// | Changed | differs from published by `significant_change` | always |
/// | Explored | visited fewer than `saturation_visits` times | `explored_period` |
/// | Saturated | visited at least `saturation_visits` times | `saturated_period` |
///
/// The visit count comes from where the robot has actually driven, not from
/// how often a cell has been observed. A robot that has driven an aisle eight
/// times knows that aisle; a rack face it has seen from a distance eight times
/// it does not necessarily know any better than once.
///
/// ### Why bother
///
/// The naive alternative - forward the whole grid every cycle - costs the same
/// whether the robot is exploring virgin warehouse or its tenth lap of a
/// mapped aisle. This policy makes the cost track the information: on a fresh
/// map almost everything is frontier and nearly all of it flows; on a mature
/// map only genuine changes and the exploration edge do. The node reports the
/// achieved suppression ratio so the effect is measured rather than assumed.
///
/// The class holds no ROS types and is driven entirely by an injected clock,
/// so the throttling behaviour can be tested at arbitrary timings.
class SelectiveMappingPolicy
{
public:
  explicit SelectiveMappingPolicy(const SelectiveMappingOptions & options);

  /// \brief Reallocate for a new grid geometry, discarding all history.
  void Configure(const GridInfo & info);

  bool Configured() const {return configured_;}
  const GridInfo & Info() const {return info_;}

  /// \brief Mark the neighbourhood of a pose as traversed.
  ///
  /// Called as the robot drives. Repeated traversal is what eventually
  /// saturates a region and slows its updates.
  void RecordTraversal(double x, double y);

  /// \brief Visit count for a cell. Zero when never driven near.
  std::uint32_t VisitCount(int column, int row) const;

  /// \brief Filter \p source into \p out_filtered, which is set to -1
  ///        (unknown) wherever the update is suppressed.
  ///
  /// \param source     Latest full occupancy grid from SLAM, 0-100 or -1.
  /// \param now        Monotonic time [s].
  /// \param out_filtered Receives the sparse update.
  /// \return Statistics describing what was kept and what was dropped.
  SelectiveMappingStats Filter(
    const std::vector<std::int8_t> & source, double now,
    std::vector<std::int8_t> * out_filtered);

  /// \brief The most recently published value of each cell, for diagnostics.
  const std::vector<std::int8_t> & PublishedGrid() const {return published_;}

private:
  /// \brief True when a cell borders unknown space, i.e. is on the frontier.
  bool IsFrontier(const std::vector<std::int8_t> & source, int column, int row) const;

  SelectiveMappingOptions options_;
  GridInfo info_;
  bool configured_ = false;

  std::vector<std::uint32_t> visits_;
  std::vector<double> last_published_time_;
  std::vector<std::int8_t> published_;
  int frontier_cells_radius_ = 20;
  int traversal_cells_radius_ = 30;
};

}  // namespace amr_mapping

#endif  // AMR_MAPPING__SELECTIVE_MAPPING_HPP_
