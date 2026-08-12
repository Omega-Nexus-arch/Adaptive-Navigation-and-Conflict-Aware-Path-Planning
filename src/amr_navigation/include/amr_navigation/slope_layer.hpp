// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_NAVIGATION__SLOPE_LAYER_HPP_
#define AMR_NAVIGATION__SLOPE_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "amr_navigation/slope_cost_model.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_navigation
{

/// \brief nav2 costmap layer that prices terrain gradient.
///
/// Registered as a `nav2_costmap_2d::Layer` plugin, so it drops into the
/// `plugins:` list of any costmap without the planner knowing it exists.
///
/// ### Where the cost comes from
///
/// The layer loads a static elevation field once at configure time, converts
/// it to per-cell cost through :class:`SlopeCostModel`, and thereafter serves
/// that pre-baked grid. Slope is a property of the building; recomputing
/// gradients every costmap cycle would burn CPU to arrive at the same answer.
/// Only the copy into the master costmap runs per cycle, and only over the
/// updated window.
///
/// ### Combination method
///
/// Costs are combined with a maximum against whatever the obstacle and static
/// layers have already written, rather than overwritten. A ramp that is also
/// blocked by a pallet must stay blocked - the slope cost is an additional
/// reason to avoid a cell, never a reason to clear one.
///
/// ### Per-robot configuration
///
/// `max_traversable_angle` is a parameter, so each robot's costmap is
/// configured with its own climbing ability and the same ramp can be merely
/// expensive for the heavy mapper and lethal for a robot that cannot climb it.
class SlopeLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SlopeLayer() = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override;

  void reset() override;

  void onFootprintChanged() override {}

  bool isClearable() override {return false;}

private:
  /// \brief Convert the loaded elevation field into a cost grid.
  void BakeCostGrid();

  /// \brief Publish the baked grid so the cost curve can be inspected in RViz.
  ///
  /// Being able to see the slope cost as a layer, separately from the merged
  /// costmap, is the difference between claiming the ramp is penalised and
  /// showing it.
  void PublishDebugGrid();

  std::unique_ptr<SlopeCostModel> model_;
  ElevationMap elevation_;
  std::vector<std::uint8_t> baked_;   ///< Cost per elevation-map cell.
  bool loaded_ = false;

  std::string elevation_path_;
  bool publish_debug_grid_ = false;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_publisher_;
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__SLOPE_LAYER_HPP_
