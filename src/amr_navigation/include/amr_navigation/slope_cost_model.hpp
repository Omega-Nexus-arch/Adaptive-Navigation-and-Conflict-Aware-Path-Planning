// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_NAVIGATION__SLOPE_COST_MODEL_HPP_
#define AMR_NAVIGATION__SLOPE_COST_MODEL_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace amr_navigation
{

/// \brief A floor-height field sampled on a regular grid.
///
/// Loaded from the PGM/YAML pair that `amr_gazebo` emits alongside the world,
/// so the terrain the planner reasons about is provably the terrain Gazebo
/// simulates. On a real vehicle the same structure would be filled from a
/// survey or an accumulated 3D map; nothing downstream cares which.
struct ElevationMap
{
  double resolution = 0.05;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  /// Row 0 is the *top* of the image, i.e. maximum y, matching the
  /// nav2_map_server convention.
  std::vector<double> heights;

  bool Empty() const {return heights.empty();}

  std::size_t Index(std::uint32_t column, std::uint32_t row) const
  {
    return static_cast<std::size_t>(row) * static_cast<std::size_t>(width) + column;
  }

  /// \brief Height at a world point. Returns \p fallback when out of bounds.
  double HeightAt(double x, double y, double fallback = 0.0) const;

  /// \brief Snap a world point to the centre of the cell containing it.
  ///
  /// Needed because sampling at exactly plus or minus one cell width from an
  /// arbitrary point is not safe in floating point: a query landing near a
  /// cell boundary can have its `+resolution` probe round back into the *same*
  /// cell, which silently halves the measured gradient. Differencing from cell
  /// centres makes the stencil exact by construction.
  ///
  /// \return False when the point lies outside the map.
  bool CellCentre(double x, double y, double * centre_x, double * centre_y) const;

  /// \brief Load from a PGM plus its metadata YAML.
  /// \throws std::runtime_error with a message naming the file on failure.
  static ElevationMap FromFiles(const std::string & yaml_path);
};

/// \brief Terrain gradient at one cell.
struct SlopeSample
{
  double gradient = 0.0;     ///< |grad z|, dimensionless (rise over run).
  double angle = 0.0;        ///< atan(gradient) [rad].
  bool valid = false;
};

/// \brief Converts terrain slope into nav2 costmap cost.
///
/// ### What the requirement asks for
///
/// The global planner must correctly price the traversability of sloped
/// surfaces and minimise their use unless they are the only viable path to the
/// goal. Those are two distinct behaviours and they need different mechanisms:
///
/// * **Minimise their use** is a *cost*, not a prohibition. A ramp is given a
///   high but finite cost, so the planner routes around it when a flat
///   alternative exists and accepts it when one does not. Marking ramps lethal
///   would satisfy the first half of the requirement and break the second.
///
/// * **Correctly calculate traversability** is a *limit*. Beyond the gradient
///   the drive train can actually climb, the surface is not expensive, it is
///   impassable, and it is marked lethal.
///
/// ### The cost curve
///
/// ```
///   cost(theta) = 0                                      theta <= free
///               = base + (253 - base) * f(theta)^gamma   free < theta < max
///               = LETHAL                                 theta >= max
/// ```
///
/// where `f` maps the angle linearly onto [0, 1] across the penalised band.
/// The exponent `gamma` shapes how sharply the penalty bites: above 1 the
/// gentle end of the band stays cheap and the steep end becomes very
/// expensive, which is the behaviour wanted here - an 8 degree service ramp
/// should be mildly discouraged while a 15 degree maintenance ramp is a last
/// resort.
///
/// The maximum penalised cost is 253, one below `INSCRIBED_INFLATED_OBSTACLE`.
/// That single-value gap is deliberate and load-bearing: nav2 treats 254 as
/// definitely-in-collision, so a ramp priced at 254 would be refused outright
/// rather than merely avoided.
///
/// ### Per-model limits
///
/// The heavy mapper and the light scout have different climbing ability, so
/// each robot's costmap is configured with its own `max_traversable_angle`.
/// The same physical ramp can therefore be expensive for one robot and lethal
/// for another, which is the honest model.
class SlopeCostModel
{
public:
  struct Options
  {
    /// Slopes at or below this are free [rad]. Covers survey noise and the
    /// quantisation of the elevation map.
    double free_angle = 0.035;          // ~2 degrees
    /// Slopes at or above this are lethal for this robot [rad].
    double max_traversable_angle = 0.279;   // ~16 degrees
    /// Cost applied the instant a slope stops being free.
    std::uint8_t base_cost = 40;
    /// Highest cost applied to a *traversable* slope. Must stay below
    /// INSCRIBED_INFLATED_OBSTACLE (253) or the planner refuses the ramp.
    std::uint8_t max_cost = 253;
    /// Shape exponent. > 1 keeps gentle slopes cheap and punishes steep ones.
    double curve_exponent = 2.0;
    /// Treat cells with no elevation data as flat rather than impassable.
    bool unknown_is_free = true;
  };

  explicit SlopeCostModel(const Options & options);

  /// \brief Central-difference gradient of \p map at a world point.
  ///
  /// Central differences rather than forward: a forward difference biases the
  /// computed slope half a cell downhill, which at 5 cm resolution shifts the
  /// lethal boundary of a 15 degree ramp by a couple of centimetres. Small,
  /// but it is free to do it correctly.
  SlopeSample SampleSlope(const ElevationMap & map, double x, double y) const;

  /// \brief Cost for a given slope angle.
  std::uint8_t CostForAngle(double angle) const;

  /// \brief Convenience: sample and price in one call.
  std::uint8_t CostAt(const ElevationMap & map, double x, double y) const;

  /// \brief nav2's LETHAL_OBSTACLE.
  static constexpr std::uint8_t kLethal = 254;
  /// \brief nav2's NO_INFORMATION.
  static constexpr std::uint8_t kNoInformation = 255;
  /// \brief nav2's FREE_SPACE.
  static constexpr std::uint8_t kFree = 0;

  const Options & GetOptions() const {return options_;}

private:
  Options options_;
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__SLOPE_COST_MODEL_HPP_
