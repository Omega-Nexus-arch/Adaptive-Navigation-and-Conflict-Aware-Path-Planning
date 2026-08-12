// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "amr_core/fleet_config.hpp"
#include "amr_mapping/map_fusion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace amr_mapping
{

/// \brief Fuses every robot's filtered map into the single warehouse map.
///
/// ### The unified map requirement
///
/// Both robots run their own SLAM and each owns a private `<robot>/map` frame.
/// This node is what makes them one system: it anchors each private frame into
/// the shared `map` frame using the initial poses from the fleet roster, folds
/// their contributions into one occupancy grid, and publishes it on `/map`.
/// Every planner in the fleet then consumes the same map, which is what the
/// brief means by a single unified global occupancy grid.
///
/// ### Why static transforms rather than map merging by correlation
///
/// Scan-matching two independently built maps is a research problem with an
/// unbounded failure mode; anchoring to known dock poses is a five-line
/// transform that is exactly right in a warehouse where robots start on
/// charging plates whose positions are surveyed. The roster is the survey.
/// If a robot were ever to start somewhere unknown, this is the one place that
/// assumption lives, and the correlation-based approach would drop in here
/// without touching anything else.
class MapFusionNode : public rclcpp::Node
{
public:
  MapFusionNode()
  : rclcpp::Node("map_fusion")
  {
    declare_parameter<std::string>("fleet_config", "");
    MapFusion::Options options;
    declare_parameter<double>("resolution", options.resolution);
    declare_parameter<double>("x_min", options.x_min);
    declare_parameter<double>("x_max", options.x_max);
    declare_parameter<double>("y_min", options.y_min);
    declare_parameter<double>("y_max", options.y_max);
    declare_parameter<double>("occupied_delta", options.occupied_delta);
    declare_parameter<double>("free_delta", options.free_delta);
    declare_parameter<double>("publish_rate", 1.0);

    const std::string fleet_config = get_parameter("fleet_config").as_string();
    if (fleet_config.empty()) {
      throw std::runtime_error("required parameter 'fleet_config' was not set");
    }
    fleet_ = amr_core::FleetConfig::FromFile(fleet_config);

    options.resolution = get_parameter("resolution").as_double();
    options.x_min = get_parameter("x_min").as_double();
    options.x_max = get_parameter("x_max").as_double();
    options.y_min = get_parameter("y_min").as_double();
    options.y_max = get_parameter("y_max").as_double();
    options.occupied_delta = get_parameter("occupied_delta").as_double();
    options.free_delta = get_parameter("free_delta").as_double();
    fusion_ = std::make_unique<MapFusion>(options);

    const auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    PublishAnchors();

    // One subscription per robot, created from the roster. Adding an eleventh
    // robot is a roster edit, not a code change.
    for (const auto & robot : fleet_.Robots()) {
      const std::string topic = "/" + robot.name + "/map_contribution";
      subscriptions_.push_back(
        create_subscription<nav_msgs::msg::OccupancyGrid>(
          topic, map_qos,
          [this, name = robot.name](nav_msgs::msg::OccupancyGrid::SharedPtr message) {
            OnContribution(name, message);
          }));
      RCLCPP_INFO(get_logger(), "listening for contributions on %s", topic.c_str());
    }

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / get_parameter("publish_rate").as_double()),
      [this]() {PublishMap();});

    RCLCPP_INFO(
      get_logger(),
      "map fusion up: %ux%u cells at %.3f m over x[%.1f, %.1f] y[%.1f, %.1f], %zu robots",
      fusion_->Info().width, fusion_->Info().height, fusion_->Info().resolution,
      options.x_min, options.x_max, options.y_min, options.y_max, fleet_.Size());
  }

private:
  /// \brief Anchor every robot's private SLAM frame into the shared one.
  void PublishAnchors()
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (const auto & robot : fleet_.Robots()) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = now();
      transform.header.frame_id = fleet_.GlobalFrame();
      transform.child_frame_id = robot.name + "/map";
      transform.transform.translation.x = robot.initial_x;
      transform.transform.translation.y = robot.initial_y;
      transform.transform.rotation.z = std::sin(robot.initial_yaw / 2.0);
      transform.transform.rotation.w = std::cos(robot.initial_yaw / 2.0);
      transforms.push_back(transform);

      RCLCPP_INFO(
        get_logger(), "anchored %s/map at (%.2f, %.2f, %.1f deg) in '%s'",
        robot.name.c_str(), robot.initial_x, robot.initial_y,
        robot.initial_yaw * 180.0 / M_PI, fleet_.GlobalFrame().c_str());
    }
    static_broadcaster_->sendTransform(transforms);
  }

  void OnContribution(
    const std::string & robot_name, const nav_msgs::msg::OccupancyGrid::SharedPtr & message)
  {
    const amr_core::RobotInstance & robot = fleet_.Robot(robot_name);

    MapContribution contribution;
    contribution.robot_id = robot_name;
    contribution.info.resolution = message->info.resolution;
    contribution.info.width = message->info.width;
    contribution.info.height = message->info.height;
    contribution.info.origin_x = message->info.origin.position.x;
    contribution.info.origin_y = message->info.origin.position.y;
    contribution.data = message->data;
    contribution.offset_x = robot.initial_x;
    contribution.offset_y = robot.initial_y;
    contribution.offset_yaw = robot.initial_yaw;
    contribution.stamp = rclcpp::Time(message->header.stamp).seconds();

    const std::size_t updated = fusion_->Integrate(contribution);
    integrated_cells_[robot_name] += updated;

    RCLCPP_DEBUG(
      get_logger(), "%s contributed %zu cells (%zu cumulative)", robot_name.c_str(), updated,
      integrated_cells_[robot_name]);
  }

  void PublishMap()
  {
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = now();
    message.header.frame_id = fleet_.GlobalFrame();
    message.info.resolution = fusion_->Info().resolution;
    message.info.width = fusion_->Info().width;
    message.info.height = fusion_->Info().height;
    message.info.origin.position.x = fusion_->Info().origin_x;
    message.info.origin.position.y = fusion_->Info().origin_y;
    message.info.origin.orientation.w = 1.0;
    fusion_->Render(&message.data);
    map_publisher_->publish(message);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "merged map: %.1f%% explored (%zu cells observed by %zu robots)",
      100.0 * (1.0 - fusion_->UnknownFraction()), fusion_->ObservedCells(), fleet_.Size());
  }

  amr_core::FleetConfig fleet_;
  std::unique_ptr<MapFusion> fusion_;
  std::map<std::string, std::size_t> integrated_cells_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> subscriptions_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace amr_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<amr_mapping::MapFusionNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("map_fusion"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
