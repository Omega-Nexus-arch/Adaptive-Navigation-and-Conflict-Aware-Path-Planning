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
#include "amr_fleet_control/safety_monitor.hpp"
#include "amr_msgs/msg/safety_status.hpp"
#include "amr_msgs/srv/set_safety_override.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace amr_fleet_control
{

/// \brief The last gate before the drive train, and the only node authorised
///        to contradict the navigation stack.
///
/// ### Authority
///
/// This node is the sole publisher of `cmd_vel`. Everything upstream - nav2,
/// the velocity smoother, the traffic controller - can only ever *propose* a
/// velocity. That single-writer arrangement is what makes the override real
/// rather than advisory: there is no path by which a planner command can reach
/// the base without passing through here.
///
/// ### Latency
///
/// The halt must be prompt, so:
///
/// * the scan subscription and the control timer share a single mutually
///   exclusive callback group, which keeps the decision path free of locks
///   while making the state machine race-free;
/// * the scan is evaluated inside its own callback and the halt is published
///   there and then, rather than waiting for the next timer tick. The timer is
///   the watchdog and the steady-state republisher, not the primary path;
/// * `SafetyStatus.reaction_latency_us` records the interval from the scan's
///   own acquisition stamp to the moment the halt was published, so the
///   claimed reaction time is measured rather than asserted.
///
/// ### Failure posture
///
/// Fails closed. No scan, a stale scan, or a scan the BSP layer rejected all
/// halt the robot. Contrast the velocity smoother, which fails *open* when the
/// traffic controller goes quiet: that component optimises throughput, this
/// one prevents collisions, and they should fail in opposite directions.
class SafetyOverrideNode : public rclcpp::Node
{
public:
  SafetyOverrideNode()
  : rclcpp::Node("safety_override")
  {
    const std::string robot_name = DeclareRequiredString("robot_name");
    const std::string fleet_config = DeclareRequiredString("fleet_config");

    declare_parameter<double>("control_rate", 50.0);
    declare_parameter<double>("min_hold_seconds", 0.4);
    declare_parameter<double>("sensor_timeout_seconds", 0.35);
    declare_parameter<double>("min_valid_range", 0.02);
    declare_parameter<double>("command_timeout", 0.5);
    // When true, a halt zeroes the command outright. When false the linear
    // axis is zeroed but rotation in place is still allowed, which lets the
    // recovery behaviours turn away from an obstacle instead of deadlocking
    // against it. Default is the strict interpretation.
    declare_parameter<bool>("halt_blocks_rotation", true);

    const amr_core::FleetConfig fleet = amr_core::FleetConfig::FromFile(fleet_config);
    robot_ = fleet.Robot(robot_name);

    SafetyMonitor::Options options;
    options.min_hold_seconds = get_parameter("min_hold_seconds").as_double();
    options.sensor_timeout_seconds = get_parameter("sensor_timeout_seconds").as_double();
    options.min_valid_range = get_parameter("min_valid_range").as_double();
    monitor_ = std::make_unique<SafetyMonitor>(robot_.profile, options);

    command_timeout_ = get_parameter("command_timeout").as_double();
    halt_blocks_rotation_ = get_parameter("halt_blocks_rotation").as_bool();

    callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;

    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    status_publisher_ = create_publisher<amr_msgs::msg::SafetyStatus>("safety_status", 10);

    // `scan` is the BSP-validated stream. The raw Gazebo topic is `scan_raw`
    // and nothing in the navigation stack subscribes to it.
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr message) {OnScan(message);},
      subscription_options);

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr message) {
        speed_ = message->twist.twist.linear.x;
      },
      subscription_options);

    proposed_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_smoothed", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr message) {
        proposed_ = *message;
        last_command_time_ = now();
        has_command_ = true;
      },
      subscription_options);

    override_service_ = create_service<amr_msgs::srv::SetSafetyOverride>(
      "set_safety_override",
      [this](
        const std::shared_ptr<amr_msgs::srv::SetSafetyOverride::Request> request,
        std::shared_ptr<amr_msgs::srv::SetSafetyOverride::Response> response) {
        monitor_->SetManualOverride(request->engage);
        response->success = true;
        response->message = request->engage ? "override engaged" : "override released";
        RCLCPP_WARN(
          get_logger(), "manual safety override %s: %s",
          request->engage ? "ENGAGED" : "RELEASED", request->reason.c_str());
      },
      rmw_qos_profile_services_default, callback_group_);

    const double rate = get_parameter("control_rate").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {OnTimer();}, callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "safety override armed for %s | d_safe = %.2f*v^2 + %.2f m, guarded cone "
      "+/-%.0f deg, watchdog %.0f ms",
      robot_.name.c_str(), robot_.profile.safety.k, robot_.profile.safety.d_min,
      robot_.profile.safety.sector_half_angle * 180.0 / M_PI,
      options.sensor_timeout_seconds * 1000.0);
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

  /// \brief Convert a LaserScan into the monitor's polar samples.
  static std::vector<RangeSample> ToSamples(const sensor_msgs::msg::LaserScan & scan)
  {
    std::vector<RangeSample> samples;
    samples.reserve(scan.ranges.size());
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      RangeSample sample;
      sample.angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      sample.range = static_cast<double>(scan.ranges[i]);
      samples.push_back(sample);
    }
    return samples;
  }

  /// \brief Primary decision path. Runs on the scan, not on a timer.
  void OnScan(const sensor_msgs::msg::LaserScan::SharedPtr & scan)
  {
    const rclcpp::Time received = now();
    const rclcpp::Time stamp(scan->header.stamp);

    // A LaserScan whose BSP validation failed never reaches this topic, so a
    // message arriving here is trusted. The flag is retained so a future
    // pass-through mode can mark data as suspect without dropping it.
    const bool sensor_valid = !scan->ranges.empty();

    const SafetyDecision decision = monitor_->Evaluate(
      ToSamples(*scan), speed_, received.seconds(), stamp.seconds(), sensor_valid);

    Publish(decision, received, stamp);

    if (decision.newly_engaged) {
      RCLCPP_WARN(
        get_logger(),
        "SAFETY HALT [%s] obstacle at %.2f m, envelope %.2f m at %.2f m/s - "
        "overriding the navigation command",
        HaltReasonToString(decision.reason), decision.min_obstacle_distance,
        decision.safe_distance, decision.speed);
    } else if (decision.newly_released) {
      RCLCPP_INFO(
        get_logger(), "safety halt released, nearest obstacle %.2f m",
        decision.min_obstacle_distance);
    }
  }

  /// \brief Watchdog and steady-state republisher.
  ///
  /// Republishing at a fixed rate matters even when nothing changes: a base
  /// driver with its own command timeout must keep seeing traffic, and a halt
  /// that stopped being asserted would quietly lapse.
  void OnTimer()
  {
    const rclcpp::Time current = now();
    const SafetyDecision decision = monitor_->EvaluateStale(speed_, current.seconds());
    Publish(decision, current, current);
  }

  void Publish(
    const SafetyDecision & decision, const rclcpp::Time & received, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::Twist output;

    const bool command_stale =
      !has_command_ || (received - last_command_time_).seconds() > command_timeout_;

    if (decision.halt) {
      // The override: whatever the navigation stack asked for is discarded.
      output.linear.x = 0.0;
      output.angular.z = halt_blocks_rotation_ ? 0.0 : proposed_.angular.z;
    } else if (command_stale) {
      // Nothing upstream is talking. Publishing zero is the only safe reading.
      output.linear.x = 0.0;
      output.angular.z = 0.0;
    } else {
      output = proposed_;
    }

    command_publisher_->publish(output);

    amr_msgs::msg::SafetyStatus status;
    status.header.stamp = received;
    status.header.frame_id = robot_.name + "/base_link";
    status.robot_id = robot_.name;
    status.halt_active = decision.halt;
    status.min_obstacle_distance = decision.min_obstacle_distance;
    status.safe_distance = decision.safe_distance;
    status.current_speed = decision.speed;
    status.reason = static_cast<std::uint8_t>(decision.reason);

    // Measured, not asserted: sensor acquisition to override publication.
    const double latency = (now() - stamp).seconds();
    status.reaction_latency_us =
      latency > 0.0 ? static_cast<std::uint32_t>(latency * 1e6) : 0u;

    status_publisher_->publish(status);
  }

  amr_core::RobotInstance robot_;
  std::unique_ptr<SafetyMonitor> monitor_;

  geometry_msgs::msg::Twist proposed_;
  double speed_ = 0.0;
  double command_timeout_ = 0.5;
  bool halt_blocks_rotation_ = true;
  bool has_command_ = false;
  rclcpp::Time last_command_time_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<amr_msgs::msg::SafetyStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr proposed_subscription_;
  rclcpp::Service<amr_msgs::srv::SetSafetyOverride>::SharedPtr override_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace amr_fleet_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<amr_fleet_control::SafetyOverrideNode>();
    // A single-threaded executor would serialise the scan callback behind any
    // other work on the node; the multi-threaded executor plus the node's
    // mutually exclusive callback group keeps the safety path prompt without
    // exposing the state machine to concurrent access.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("safety_override"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
