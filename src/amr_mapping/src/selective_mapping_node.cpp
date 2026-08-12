// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "amr_core/fleet_config.hpp"
#include "amr_mapping/selective_mapping.hpp"
#include "amr_msgs/msg/map_update_stats.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_mapping
{

/// \brief Applies the selective-iteration policy to one robot's SLAM output.
///
/// ```
///   slam_toolbox --> map --> [ selective_mapping ] --> map_contribution
///                                     |                      |
///                            odom/TF (traversal)             v
///                                                     [ map_fusion ] --> /map
/// ```
///
/// The filtered grid marks suppressed cells as unknown, which the fusion node
/// reads as "no new evidence" rather than "empty". That is the contract that
/// lets aggressive suppression coexist with a complete merged map.
class SelectiveMappingNode : public rclcpp::Node
{
public:
  SelectiveMappingNode()
  : rclcpp::Node("selective_mapping")
  {
    const std::string robot_name = DeclareRequiredString("robot_name");
    const std::string fleet_config = DeclareRequiredString("fleet_config");

    SelectiveMappingOptions options;
    declare_parameter<double>("frontier_radius", options.frontier_radius);
    declare_parameter<int>(
      "saturation_visits", static_cast<int>(options.saturation_visits));
    declare_parameter<double>("saturated_period", options.saturated_period);
    declare_parameter<double>("explored_period", options.explored_period);
    declare_parameter<int>("significant_change", options.significant_change);
    declare_parameter<double>("traversal_radius", options.traversal_radius);
    declare_parameter<int>(
      "min_cells_to_publish", static_cast<int>(options.min_cells_to_publish));
    declare_parameter<double>("traversal_rate", 5.0);
    // Set false on the follower to show, side by side, the difference the
    // policy makes: AMR-1 filters, AMR-2 forwards everything.
    declare_parameter<bool>("enabled", true);

    options.frontier_radius = get_parameter("frontier_radius").as_double();
    options.saturation_visits =
      static_cast<std::uint32_t>(get_parameter("saturation_visits").as_int());
    options.saturated_period = get_parameter("saturated_period").as_double();
    options.explored_period = get_parameter("explored_period").as_double();
    options.significant_change = static_cast<int>(get_parameter("significant_change").as_int());
    options.traversal_radius = get_parameter("traversal_radius").as_double();
    options.min_cells_to_publish =
      static_cast<std::uint32_t>(get_parameter("min_cells_to_publish").as_int());
    enabled_ = get_parameter("enabled").as_bool();

    const amr_core::FleetConfig fleet = amr_core::FleetConfig::FromFile(fleet_config);
    robot_ = fleet.Robot(robot_name);
    policy_ = std::make_unique<SelectiveMappingPolicy>(options);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Transient-local so the fusion node gets the latest contribution even if
    // it starts after this one; map updates are slow and infrequent.
    const auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    contribution_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("map_contribution", map_qos);
    stats_publisher_ =
      create_publisher<amr_msgs::msg::MapUpdateStats>("map_update_stats", rclcpp::QoS(10));

    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr message) {OnMap(message);});

    traversal_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / get_parameter("traversal_rate").as_double()),
      [this]() {RecordTraversal();});

    RCLCPP_INFO(
      get_logger(),
      "selective mapping %s for %s | frontier radius %.2f m, saturation at %u visits, "
      "throttle %.1f s saturated / %.1f s explored",
      enabled_ ? "ENABLED" : "DISABLED (pass-through)", robot_.name.c_str(),
      options.frontier_radius, options.saturation_visits, options.saturated_period,
      options.explored_period);
  }

private:
  std::string DeclareRequiredString(const std::string & name)
  {
    declare_parameter<std::string>(name, "");
    const std::string value = get_parameter(name).as_string();
    if (value.empty()) {
      throw std::runtime_error("required parameter '" + name + "' was not set");
    }
    return value;
  }

  /// \brief Note where the robot currently is, so its aisles saturate.
  void RecordTraversal()
  {
    if (!policy_->Configured()) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        map_frame_, robot_.name + "/base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;   // Not localised yet; nothing to record.
    }
    policy_->RecordTraversal(
      transform.transform.translation.x, transform.transform.translation.y);
  }

  void OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & message)
  {
    map_frame_ = message->header.frame_id;

    GridInfo info;
    info.resolution = message->info.resolution;
    info.width = message->info.width;
    info.height = message->info.height;
    info.origin_x = message->info.origin.position.x;
    info.origin_y = message->info.origin.position.y;

    if (!policy_->Configured() || !policy_->Info().SameGeometry(info)) {
      // SLAM grew or re-anchored the grid.
      RCLCPP_INFO(
        get_logger(), "map geometry changed to %ux%u at %.3f m/cell; resetting history",
        info.width, info.height, info.resolution);
      policy_->Configure(info);
    }

    std::vector<std::int8_t> filtered;
    SelectiveMappingStats stats;

    if (enabled_) {
      stats = policy_->Filter(message->data, now().seconds(), &filtered);
    } else {
      // Pass-through comparison mode.
      filtered = message->data;
      stats.cells_considered = static_cast<std::uint32_t>(message->data.size());
      stats.cells_written = stats.cells_considered;
      stats.update_published = true;
      stats.policy_state = "disabled";
    }

    if (stats.update_published) {
      nav_msgs::msg::OccupancyGrid contribution = *message;
      contribution.data = std::move(filtered);
      contribution_publisher_->publish(contribution);
    }

    amr_msgs::msg::MapUpdateStats report;
    report.header.stamp = now();
    report.header.frame_id = map_frame_;
    report.robot_id = robot_.name;
    report.update_published = stats.update_published;
    report.cells_considered = stats.cells_considered;
    report.cells_written = stats.cells_written;
    report.cells_suppressed = stats.cells_suppressed;
    report.frontier_cells = stats.frontier_cells;
    report.suppression_ratio = stats.suppression_ratio;
    report.policy_state = stats.policy_state;

    const rclcpp::Time current = now();
    if (has_previous_publish_) {
      const double interval = (current - previous_publish_).seconds();
      report.effective_rate_hz = interval > 1e-6 ? 1.0 / interval : 0.0;
    }
    previous_publish_ = current;
    has_previous_publish_ = true;
    stats_publisher_->publish(report);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[%s] %s: wrote %u / %u cells (%.0f%% suppressed, %u on the frontier)",
      robot_.name.c_str(), stats.policy_state.c_str(), stats.cells_written,
      stats.cells_considered, 100.0 * stats.suppression_ratio, stats.frontier_cells);
  }

  amr_core::RobotInstance robot_;
  std::unique_ptr<SelectiveMappingPolicy> policy_;
  bool enabled_ = true;
  std::string map_frame_ = "map";

  bool has_previous_publish_ = false;
  rclcpp::Time previous_publish_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr contribution_publisher_;
  rclcpp::Publisher<amr_msgs::msg::MapUpdateStats>::SharedPtr stats_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::TimerBase::SharedPtr traversal_timer_;
};

}  // namespace amr_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<amr_mapping::SelectiveMappingNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("selective_mapping"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
