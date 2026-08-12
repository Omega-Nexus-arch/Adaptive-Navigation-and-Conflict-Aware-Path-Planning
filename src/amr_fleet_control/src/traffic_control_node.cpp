// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "amr_core/fleet_config.hpp"
#include "amr_fleet_control/traffic_manager.hpp"
#include "amr_msgs/msg/predicted_trajectory.hpp"
#include "amr_msgs/msg/traffic_directive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace amr_fleet_control
{

/// \brief Fleet-wide traffic controller: one instance for the whole warehouse.
///
/// Collects every robot's projected trajectory, finds predicted space-time
/// conflicts, and issues one directive per robot per cycle according to the
/// priority-based yielding protocol.
///
/// ### Why this is centralised when everything else is per-robot
///
/// Yielding is a decision *about a pair*. Deciding it independently on each
/// robot invites the classic failure where both defer, or neither does,
/// depending on message timing. One arbiter with one view of the fleet makes
/// the outcome deterministic and reproducible from a rosbag.
///
/// The trade is a single point of failure, which is handled by *scoping* its
/// authority rather than by replicating it: this node can only slow robots
/// down, never speed them up or steer them, and each robot's safety override
/// runs independently underneath. If this node dies, the fleet keeps working
/// with collision avoidance intact but with throughput that degrades in
/// congestion - which is the right way round.
class TrafficControlNode : public rclcpp::Node
{
public:
  TrafficControlNode()
  : rclcpp::Node("traffic_control")
  {
    declare_parameter<std::string>("fleet_config", "");
    declare_parameter<bool>("publish_markers", true);

    const std::string fleet_config = get_parameter("fleet_config").as_string();
    if (fleet_config.empty()) {
      throw std::runtime_error("required parameter 'fleet_config' was not set");
    }

    fleet_ = amr_core::FleetConfig::FromFile(fleet_config);
    detector_ = std::make_unique<ConflictDetector>(fleet_.Policy());
    policy_ = std::make_unique<YieldPolicy>(fleet_);
    publish_markers_ = get_parameter("publish_markers").as_bool();

    directive_publisher_ = create_publisher<amr_msgs::msg::TrafficDirective>(
      "/fleet/traffic_directives", rclcpp::QoS(20));
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fleet/conflict_markers", rclcpp::QoS(1));

    trajectory_subscription_ = create_subscription<amr_msgs::msg::PredictedTrajectory>(
      "/fleet/trajectories", rclcpp::QoS(20),
      [this](amr_msgs::msg::PredictedTrajectory::SharedPtr message) {OnTrajectory(message);});

    const double rate = fleet_.Policy().traffic_rate_hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {OnTimer();});

    std::string roster;
    for (const auto & robot : fleet_.Robots()) {
      roster += (roster.empty() ? "" : ", ") + robot.name + "(p" +
        std::to_string(robot.YieldPriority()) + ")";
    }
    RCLCPP_INFO(
      get_logger(), "traffic control up for %zu robots at %.0f Hz: %s",
      fleet_.Size(), rate, roster.c_str());
  }

private:
  void OnTrajectory(const amr_msgs::msg::PredictedTrajectory::SharedPtr & message)
  {
    if (!fleet_.Contains(message->robot_id)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "ignoring a trajectory from '%s', which is not on the roster",
        message->robot_id.c_str());
      return;
    }

    PredictedPath path;
    path.robot_id = message->robot_id;
    path.stamp = rclcpp::Time(message->header.stamp).seconds();
    path.footprint_radius = message->footprint_radius;
    path.yield_priority = message->yield_priority;
    path.samples.reserve(message->points.size());

    for (const auto & point : message->points) {
      TrajectorySample sample;
      sample.time = point.time_from_start;
      sample.pose.x = point.pose.position.x;
      sample.pose.y = point.pose.position.y;
      sample.pose.theta = amr_core::YawFromQuaternion(
        point.pose.orientation.x, point.pose.orientation.y, point.pose.orientation.z,
        point.pose.orientation.w);
      sample.speed = point.speed;
      path.samples.push_back(sample);
    }
    paths_[message->robot_id] = std::move(path);
  }

  void OnTimer()
  {
    const rclcpp::Time current = now();
    const double seconds = current.seconds();

    const std::vector<Conflict> conflicts = detector_->Detect(paths_, seconds);
    const std::map<std::string, Directive> directives = policy_->Resolve(conflicts, seconds);

    for (const auto & entry : directives) {
      const Directive & directive = entry.second;
      amr_msgs::msg::TrafficDirective message;
      message.header.stamp = current;
      message.header.frame_id = fleet_.GlobalFrame();
      message.robot_id = directive.robot_id;
      message.action = static_cast<std::uint8_t>(directive.action);
      message.speed_scale = directive.speed_scale;
      message.conflicting_robot = directive.conflicting_robot;
      message.time_to_conflict = directive.time_to_conflict;
      message.reason = directive.reason;
      directive_publisher_->publish(message);

      // Log only on change: at 10 Hz an unconditional log would bury every
      // other message in the console during the demo.
      const auto previous = last_action_.find(directive.robot_id);
      if (previous == last_action_.end() || previous->second != directive.action) {
        if (directive.action == TrafficAction::kProceed) {
          RCLCPP_INFO(get_logger(), "%s: PROCEED", directive.robot_id.c_str());
        } else {
          RCLCPP_WARN(
            get_logger(), "%s: %s - %s", directive.robot_id.c_str(),
            TrafficActionToString(directive.action), directive.reason.c_str());
        }
        last_action_[directive.robot_id] = directive.action;
      }
    }

    if (publish_markers_) {
      PublishMarkers(conflicts, current);
    }
  }

  /// \brief Draw predicted conflicts in RViz.
  ///
  /// Worth the code: "the robots stopped" and "the robots stopped *for this
  /// predicted conflict, there, in 1.2 s*" are very different claims to make
  /// on a demo video.
  void PublishMarkers(const std::vector<Conflict> & conflicts, const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear_all;
    clear_all.header.frame_id = fleet_.GlobalFrame();
    clear_all.header.stamp = stamp;
    clear_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_all);

    int id = 0;
    for (const Conflict & conflict : conflicts) {
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = fleet_.GlobalFrame();
      sphere.header.stamp = stamp;
      sphere.ns = "conflicts";
      sphere.id = id++;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = conflict.x;
      sphere.pose.position.y = conflict.y;
      sphere.pose.position.z = 0.5;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = conflict.required_separation;
      // Red when imminent, amber when merely predicted.
      const bool imminent = conflict.time_to_conflict <= fleet_.Policy().hard_yield_seconds;
      sphere.color.r = 1.0f;
      sphere.color.g = imminent ? 0.0f : 0.7f;
      sphere.color.b = 0.0f;
      sphere.color.a = 0.45f;
      markers.markers.push_back(sphere);

      visualization_msgs::msg::Marker label;
      label.header = sphere.header;
      label.ns = "conflict_labels";
      label.id = id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = conflict.x;
      label.pose.position.y = conflict.y;
      label.pose.position.z = 1.4;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.35;
      label.color.r = label.color.g = label.color.b = 1.0f;
      label.color.a = 1.0f;
      label.text = conflict.robot_a + " x " + conflict.robot_b + "  t-" +
        std::to_string(conflict.time_to_conflict).substr(0, 4) + "s";
      markers.markers.push_back(label);
    }

    marker_publisher_->publish(markers);
  }

  amr_core::FleetConfig fleet_;
  std::unique_ptr<ConflictDetector> detector_;
  std::unique_ptr<YieldPolicy> policy_;
  std::map<std::string, PredictedPath> paths_;
  std::map<std::string, TrafficAction> last_action_;
  bool publish_markers_ = true;

  rclcpp::Publisher<amr_msgs::msg::TrafficDirective>::SharedPtr directive_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<amr_msgs::msg::PredictedTrajectory>::SharedPtr trajectory_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace amr_fleet_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<amr_fleet_control::TrafficControlNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("traffic_control"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
