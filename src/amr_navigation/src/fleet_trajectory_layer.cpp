// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.

#include "amr_navigation/fleet_trajectory_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace amr_navigation
{

void FleetTrajectoryLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("FleetTrajectoryLayer: owning node has already expired");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("robot_name", rclcpp::ParameterValue(std::string()));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/fleet/trajectories")));
  declareParameter("horizon_seconds", rclcpp::ParameterValue(3.0));
  declareParameter("trajectory_timeout", rclcpp::ParameterValue(1.0));
  declareParameter("inflation_margin", rclcpp::ParameterValue(0.15));
  declareParameter("present_cost", rclcpp::ParameterValue(200));
  declareParameter("horizon_cost", rclcpp::ParameterValue(30));

  std::string topic = "/fleet/trajectories";
  int present_cost = 200;
  int horizon_cost = 30;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".robot_name", robot_name_);
  node->get_parameter(name_ + ".topic", topic);
  node->get_parameter(name_ + ".horizon_seconds", horizon_seconds_);
  node->get_parameter(name_ + ".trajectory_timeout", trajectory_timeout_);
  node->get_parameter(name_ + ".inflation_margin", inflation_margin_);
  node->get_parameter(name_ + ".present_cost", present_cost);
  node->get_parameter(name_ + ".horizon_cost", horizon_cost);

  // Kept strictly below LETHAL. A predicted peer position is a probability,
  // not a wall: marking it lethal would make the planner declare failure
  // rather than route around, and a robot that gives up in a corridor is worse
  // than one that squeezes past.
  present_cost_ = static_cast<std::uint8_t>(std::clamp(present_cost, 0, 252));
  horizon_cost_ = static_cast<std::uint8_t>(std::clamp(horizon_cost, 0, 252));

  if (robot_name_.empty()) {
    RCLCPP_WARN(
      node->get_logger(),
      "%s: 'robot_name' is unset, so this robot cannot recognise its own trajectory and "
      "will treat itself as an obstacle. Set it to the robot's namespace.", name_.c_str());
  }

  subscription_ = node->create_subscription<amr_msgs::msg::PredictedTrajectory>(
    topic, rclcpp::QoS(20),
    [this](amr_msgs::msg::PredictedTrajectory::SharedPtr message) {OnTrajectory(message);});

  current_ = true;
  default_value_ = nav2_costmap_2d::FREE_SPACE;
  matchSize();

  RCLCPP_INFO(
    node->get_logger(),
    "%s: watching '%s' for peers of '%s'; cost decays %u -> %u over %.1f s",
    name_.c_str(), topic.c_str(), robot_name_.c_str(), present_cost_, horizon_cost_,
    horizon_seconds_);
}

void FleetTrajectoryLayer::OnTrajectory(
  const amr_msgs::msg::PredictedTrajectory::SharedPtr message)
{
  // Ignoring our own projection is the whole reason `robot_name` exists;
  // without it the robot inflates a wall along its own intended path.
  if (message->robot_id == robot_name_) {
    return;
  }
  auto node = node_.lock();
  if (!node) {
    return;
  }
  peers_[message->robot_id] = PeerTrajectory{*message, node->now()};
}

void FleetTrajectoryLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  auto node = node_.lock();
  if (!node) {
    return;
  }
  const rclcpp::Time current = node->now();

  // Expire stale peers. A trajectory that stopped arriving is not evidence the
  // peer stopped moving; holding it would leave a ghost in the costmap.
  for (auto it = peers_.begin(); it != peers_.end(); ) {
    if ((current - it->second.received).seconds() > trajectory_timeout_) {
      it = peers_.erase(it);
    } else {
      ++it;
    }
  }

  double touched_min_x = std::numeric_limits<double>::infinity();
  double touched_min_y = std::numeric_limits<double>::infinity();
  double touched_max_x = -std::numeric_limits<double>::infinity();
  double touched_max_y = -std::numeric_limits<double>::infinity();

  for (const auto & entry : peers_) {
    const auto & message = entry.second.message;
    const double reach = message.footprint_radius + inflation_margin_;
    for (const auto & point : message.points) {
      if (point.time_from_start > horizon_seconds_) {
        break;   // Points are chronological.
      }
      touched_min_x = std::min(touched_min_x, point.pose.position.x - reach);
      touched_min_y = std::min(touched_min_y, point.pose.position.y - reach);
      touched_max_x = std::max(touched_max_x, point.pose.position.x + reach);
      touched_max_y = std::max(touched_max_y, point.pose.position.y + reach);
    }
  }

  // Include last cycle's footprint too, so cost written for a peer that has
  // since moved away is cleared rather than left behind.
  if (has_previous_bounds_) {
    touched_min_x = std::min(touched_min_x, previous_min_x_);
    touched_min_y = std::min(touched_min_y, previous_min_y_);
    touched_max_x = std::max(touched_max_x, previous_max_x_);
    touched_max_y = std::max(touched_max_y, previous_max_y_);
  }

  if (touched_min_x <= touched_max_x) {
    *min_x = std::min(*min_x, touched_min_x);
    *min_y = std::min(*min_y, touched_min_y);
    *max_x = std::max(*max_x, touched_max_x);
    *max_y = std::max(*max_y, touched_max_y);

    previous_min_x_ = touched_min_x;
    previous_min_y_ = touched_min_y;
    previous_max_x_ = touched_max_x;
    previous_max_y_ = touched_max_y;
    has_previous_bounds_ = true;
  } else {
    has_previous_bounds_ = false;
  }
}

void FleetTrajectoryLayer::StampDisc(
  nav2_costmap_2d::Costmap2D & master_grid, double x, double y, double radius,
  std::uint8_t cost, int min_i, int min_j, int max_i, int max_j)
{
  unsigned int centre_i = 0;
  unsigned int centre_j = 0;
  if (!master_grid.worldToMap(x, y, centre_i, centre_j)) {
    return;
  }
  const int cells =
    static_cast<int>(std::ceil(radius / master_grid.getResolution()));
  const int cells_squared = cells * cells;

  for (int dj = -cells; dj <= cells; ++dj) {
    for (int di = -cells; di <= cells; ++di) {
      if (di * di + dj * dj > cells_squared) {
        continue;
      }
      const int i = static_cast<int>(centre_i) + di;
      const int j = static_cast<int>(centre_j) + dj;
      if (i < min_i || j < min_j || i >= max_i || j >= max_j) {
        continue;
      }
      const unsigned char existing =
        master_grid.getCost(static_cast<unsigned int>(i), static_cast<unsigned int>(j));
      if (existing == nav2_costmap_2d::NO_INFORMATION || cost > existing) {
        master_grid.setCost(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j), cost);
      }
    }
  }
}

void FleetTrajectoryLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || peers_.empty()) {
    return;
  }

  for (const auto & entry : peers_) {
    const auto & message = entry.second.message;
    const double radius = message.footprint_radius + inflation_margin_;

    for (const auto & point : message.points) {
      if (point.time_from_start > horizon_seconds_) {
        break;
      }
      // Linear decay from certainty now to a light nudge at the horizon.
      const double fraction =
        horizon_seconds_ > 1e-9 ? point.time_from_start / horizon_seconds_ : 0.0;
      const double cost = static_cast<double>(present_cost_) +
        (static_cast<double>(horizon_cost_) - static_cast<double>(present_cost_)) *
        std::min(1.0, std::max(0.0, fraction));

      StampDisc(
        master_grid, point.pose.position.x, point.pose.position.y, radius,
        static_cast<std::uint8_t>(cost), min_i, min_j, max_i, max_j);
    }
  }
}

void FleetTrajectoryLayer::reset()
{
  peers_.clear();
  has_previous_bounds_ = false;
  current_ = true;
}

}  // namespace amr_navigation

PLUGINLIB_EXPORT_CLASS(amr_navigation::FleetTrajectoryLayer, nav2_costmap_2d::Layer)
