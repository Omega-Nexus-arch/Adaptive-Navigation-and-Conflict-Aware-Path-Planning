// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_NAVIGATION__FLEET_TRAJECTORY_LAYER_HPP_
#define AMR_NAVIGATION__FLEET_TRAJECTORY_LAYER_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "amr_msgs/msg/predicted_trajectory.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_navigation
{

/// \brief Stamps peer robots' *predicted* positions into the local costmap.
///
/// ### The requirement
///
/// Each robot's local planner must consume the projected trajectories of the
/// other robots to avoid predicted conflicts. This layer is how that reaches
/// the planner: it subscribes to the fleet-wide `/fleet/trajectories` topic,
/// drops its own, and writes the remaining robots' forward projections into
/// the local costmap as cost.
///
/// The planner needs no modification. It already avoids cost; this layer just
/// gives it cost in the places peers are *about to be*, rather than only where
/// they currently are. Together with the traffic controller the two operate at
/// different levels: this layer nudges trajectories apart continuously, while
/// the controller makes the discrete stop-or-go decision when nudging is not
/// enough.
///
/// ### Time decay
///
/// A peer's position now is certain; its position in four seconds is a guess.
/// Cost therefore decays with prediction time, from `present_cost` at t = 0 to
/// `horizon_cost` at the end of the horizon. Without the decay a distant
/// prediction would repel as hard as a present obstacle and the robots would
/// refuse to share an aisle at all.
///
/// ### Why not simply mark the peer's current footprint
///
/// Because that produces exactly the head-on deadlock this system exists to
/// avoid: two robots approaching a doorway each see the other as a static
/// obstacle that is not yet in the way, commit, and meet in the middle.
/// Marking the projection makes the conflict visible while there is still room
/// to resolve it.
class FleetTrajectoryLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  FleetTrajectoryLayer() = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override;

  void reset() override;

  void onFootprintChanged() override {}

  bool isClearable() override {return true;}

private:
  struct PeerTrajectory
  {
    amr_msgs::msg::PredictedTrajectory message;
    rclcpp::Time received;
  };

  void OnTrajectory(const amr_msgs::msg::PredictedTrajectory::SharedPtr message);

  /// \brief Fill a disc of cost centred on a world point.
  void StampDisc(
    nav2_costmap_2d::Costmap2D & master_grid, double x, double y, double radius,
    std::uint8_t cost, int min_i, int min_j, int max_i, int max_j);

  std::string robot_name_;
  double horizon_seconds_ = 3.0;
  double trajectory_timeout_ = 1.0;
  double inflation_margin_ = 0.15;
  std::uint8_t present_cost_ = 200;
  std::uint8_t horizon_cost_ = 30;

  std::map<std::string, PeerTrajectory> peers_;
  rclcpp::Subscription<amr_msgs::msg::PredictedTrajectory>::SharedPtr subscription_;

  // Bounds touched on the previous cycle, so stale cost is cleared when a peer
  // moves on or its trajectory expires.
  double previous_min_x_ = 0.0;
  double previous_min_y_ = 0.0;
  double previous_max_x_ = 0.0;
  double previous_max_y_ = 0.0;
  bool has_previous_bounds_ = false;
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__FLEET_TRAJECTORY_LAYER_HPP_
