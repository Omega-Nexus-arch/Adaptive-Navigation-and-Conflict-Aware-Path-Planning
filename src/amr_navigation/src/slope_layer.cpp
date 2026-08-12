// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.

#include "amr_navigation/slope_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_events_filter.hpp"

namespace amr_navigation
{

namespace
{
constexpr double kDegrees = M_PI / 180.0;
}  // namespace

void SlopeLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SlopeLayer: owning node has already expired");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("elevation_map", rclcpp::ParameterValue(std::string()));
  declareParameter("free_angle_degrees", rclcpp::ParameterValue(2.0));
  declareParameter("max_traversable_angle_degrees", rclcpp::ParameterValue(16.0));
  declareParameter("base_cost", rclcpp::ParameterValue(40));
  declareParameter("max_cost", rclcpp::ParameterValue(253));
  declareParameter("curve_exponent", rclcpp::ParameterValue(2.0));
  declareParameter("unknown_is_free", rclcpp::ParameterValue(true));
  declareParameter("publish_debug_grid", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".elevation_map", elevation_path_);
  node->get_parameter(name_ + ".publish_debug_grid", publish_debug_grid_);

  SlopeCostModel::Options options;
  double free_degrees = 2.0;
  double max_degrees = 16.0;
  int base_cost = 40;
  int max_cost = 253;
  node->get_parameter(name_ + ".free_angle_degrees", free_degrees);
  node->get_parameter(name_ + ".max_traversable_angle_degrees", max_degrees);
  node->get_parameter(name_ + ".base_cost", base_cost);
  node->get_parameter(name_ + ".max_cost", max_cost);
  node->get_parameter(name_ + ".curve_exponent", options.curve_exponent);
  node->get_parameter(name_ + ".unknown_is_free", options.unknown_is_free);

  options.free_angle = free_degrees * kDegrees;
  options.max_traversable_angle = max_degrees * kDegrees;
  // Clamp below LETHAL here as well as in the model: a configuration typo that
  // set max_cost to 254 would make every ramp impassable, and the symptom -
  // "the planner says the mezzanine is unreachable" - points nowhere near the
  // cause.
  options.base_cost = static_cast<std::uint8_t>(std::clamp(base_cost, 0, 253));
  options.max_cost = static_cast<std::uint8_t>(std::clamp(max_cost, 0, 253));
  model_ = std::make_unique<SlopeCostModel>(options);

  current_ = true;
  default_value_ = nav2_costmap_2d::FREE_SPACE;
  matchSize();

  if (elevation_path_.empty()) {
    RCLCPP_WARN(
      node->get_logger(),
      "%s: no 'elevation_map' configured; the layer will contribute nothing. Set it to the "
      "warehouse_elevation.yaml emitted by amr_gazebo.", name_.c_str());
    return;
  }

  try {
    elevation_ = ElevationMap::FromFiles(elevation_path_);
    BakeCostGrid();
    loaded_ = true;
    RCLCPP_INFO(
      node->get_logger(),
      "%s: loaded %ux%u elevation cells at %.3f m from '%s'; free below %.1f deg, "
      "lethal at or above %.1f deg", name_.c_str(), elevation_.width, elevation_.height,
      elevation_.resolution, elevation_path_.c_str(), free_degrees, max_degrees);
  } catch (const std::exception & error) {
    // Deliberately not fatal. A costmap layer that refuses to initialise takes
    // the whole navigation stack down; one that reports a clear error and
    // contributes nothing leaves the robot navigable on flat ground, which is
    // the safer degradation.
    RCLCPP_ERROR(
      node->get_logger(), "%s: could not load the elevation map (%s). Slope costing is OFF.",
      name_.c_str(), error.what());
    return;
  }

  if (publish_debug_grid_) {
    debug_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/" + name_ + "/slope_cost", rclcpp::QoS(1).transient_local());
    PublishDebugGrid();
  }
}

void SlopeLayer::BakeCostGrid()
{
  baked_.assign(elevation_.heights.size(), nav2_costmap_2d::FREE_SPACE);
  for (std::uint32_t row = 0; row < elevation_.height; ++row) {
    for (std::uint32_t column = 0; column < elevation_.width; ++column) {
      // Cell centre in world coordinates; row 0 is maximum y.
      const double x =
        elevation_.origin_x + (static_cast<double>(column) + 0.5) * elevation_.resolution;
      const double y_max =
        elevation_.origin_y + static_cast<double>(elevation_.height) * elevation_.resolution;
      const double y = y_max - (static_cast<double>(row) + 0.5) * elevation_.resolution;
      baked_[elevation_.Index(column, row)] = model_->CostAt(elevation_, x, y);
    }
  }
}

void SlopeLayer::PublishDebugGrid()
{
  if (!debug_publisher_ || !loaded_) {
    return;
  }
  auto node = node_.lock();
  if (!node) {
    return;
  }

  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = node->now();
  message.header.frame_id = layered_costmap_->getGlobalFrameID();
  message.info.resolution = elevation_.resolution;
  message.info.width = elevation_.width;
  message.info.height = elevation_.height;
  message.info.origin.position.x = elevation_.origin_x;
  message.info.origin.position.y = elevation_.origin_y;
  message.info.origin.orientation.w = 1.0;
  message.data.resize(baked_.size());

  for (std::uint32_t row = 0; row < elevation_.height; ++row) {
    for (std::uint32_t column = 0; column < elevation_.width; ++column) {
      // OccupancyGrid rows run bottom-up; the elevation map runs top-down.
      const std::size_t source = elevation_.Index(column, elevation_.height - 1 - row);
      const std::size_t target = elevation_.Index(column, row);
      message.data[target] = static_cast<std::int8_t>(baked_[source] * 100 / 255);
    }
  }
  debug_publisher_->publish(message);
}

void SlopeLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_ || !loaded_) {
    return;
  }
  // The slope field is static, so the layer's contribution spans the whole
  // elevation map for as long as it is loaded. Claiming only the window around
  // the robot would leave stale costs behind on a rolling costmap.
  const double x_max =
    elevation_.origin_x + static_cast<double>(elevation_.width) * elevation_.resolution;
  const double y_max =
    elevation_.origin_y + static_cast<double>(elevation_.height) * elevation_.resolution;

  *min_x = std::min(*min_x, elevation_.origin_x);
  *min_y = std::min(*min_y, elevation_.origin_y);
  *max_x = std::max(*max_x, x_max);
  *max_y = std::max(*max_y, y_max);
}

void SlopeLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !loaded_) {
    return;
  }

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double world_x = 0.0;
      double world_y = 0.0;
      master_grid.mapToWorld(
        static_cast<unsigned int>(i), static_cast<unsigned int>(j), world_x, world_y);

      // The costmap and the elevation map need not share a resolution or an
      // origin, so go through world coordinates rather than assuming indices
      // line up.
      const int column =
        static_cast<int>(std::floor((world_x - elevation_.origin_x) / elevation_.resolution));
      const double y_max =
        elevation_.origin_y + static_cast<double>(elevation_.height) * elevation_.resolution;
      const int row = static_cast<int>(std::floor((y_max - world_y) / elevation_.resolution));

      if (column < 0 || row < 0 || column >= static_cast<int>(elevation_.width) ||
        row >= static_cast<int>(elevation_.height))
      {
        continue;
      }

      const std::uint8_t slope_cost =
        baked_[elevation_.Index(
            static_cast<std::uint32_t>(column), static_cast<std::uint32_t>(row))];
      if (slope_cost == nav2_costmap_2d::FREE_SPACE) {
        continue;
      }

      // Maximum, never overwrite: an obstacle on a ramp must stay an obstacle.
      const unsigned char existing = master_grid.getCost(
        static_cast<unsigned int>(i), static_cast<unsigned int>(j));
      if (existing == nav2_costmap_2d::NO_INFORMATION || slope_cost > existing) {
        master_grid.setCost(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j), slope_cost);
      }
    }
  }
}

void SlopeLayer::reset()
{
  // Nothing to clear: the layer's content is static terrain, not observations.
  current_ = true;
}

}  // namespace amr_navigation

PLUGINLIB_EXPORT_CLASS(amr_navigation::SlopeLayer, nav2_costmap_2d::Layer)
