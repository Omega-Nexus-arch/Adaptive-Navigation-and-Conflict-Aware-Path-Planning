// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "amr_core/fleet_config.hpp"
#include "amr_fleet_control/motion_smoother.hpp"
#include "amr_msgs/msg/payload_state.hpp"
#include "amr_msgs/msg/traffic_directive.hpp"
#include "amr_msgs/srv/set_payload.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_fleet_control
{

/// \brief Shapes the navigation stack's velocity command for one robot.
///
/// ### Position in the command chain
///
/// ```
///   nav2 controller --> cmd_vel_nav
///                         |
///                         v
///                  [ velocity_smoother ]   <-- this node
///                    * applies the traffic directive's speed scale
///                    * enforces acceleration and jerk limits for the
///                      robot's model, payload and current speed
///                         |
///                         v
///                     cmd_vel_smoothed
///                         |
///                         v
///                  [ safety_override ]     <-- may bypass everything above
///                         |
///                         v
///                       cmd_vel  --> base
/// ```
///
/// ### Why the traffic directive is applied here
///
/// A yield could be implemented by publishing a zero velocity straight to the
/// base. It is applied as a *scale on the target* instead, so the stop is
/// shaped by the same jerk limiter as any other slow-down. That is what makes
/// it a "temporary, controlled stop" rather than a cut, and it is the reason a
/// yielding robot does not shed its payload. Genuinely unsafe situations are
/// the safety node's business, downstream, and deliberately are not shaped.
class VelocitySmootherNode : public rclcpp::Node
{
public:
  VelocitySmootherNode()
  : rclcpp::Node("velocity_smoother")
  {
    const std::string robot_name = DeclareRequiredString("robot_name");
    const std::string fleet_config = DeclareRequiredString("fleet_config");

    declare_parameter<double>("control_rate", 50.0);
    declare_parameter<double>("command_timeout", 0.5);
    declare_parameter<double>("directive_timeout", 1.5);
    declare_parameter<double>("initial_payload_kg", 0.0);
    declare_parameter<bool>("publish_diagnostics", true);

    // Configuration errors are fatal: a smoother running on default limits
    // would silently permit accelerations the chassis cannot deliver.
    const amr_core::FleetConfig fleet = amr_core::FleetConfig::FromFile(fleet_config);
    robot_ = fleet.Robot(robot_name);
    smoother_ = std::make_unique<MotionSmoother>(robot_.profile);

    payload_kg_ = std::clamp(
      get_parameter("initial_payload_kg").as_double(), 0.0,
      robot_.profile.payload_capacity_kg);
    command_timeout_ = get_parameter("command_timeout").as_double();
    directive_timeout_ = get_parameter("directive_timeout").as_double();

    command_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 10);
    // Transient-local: a late subscriber still learns the current payload.
    payload_publisher_ = create_publisher<amr_msgs::msg::PayloadState>(
      "payload_state", rclcpp::QoS(1).transient_local());

    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr message) {
        target_.linear_x = message->linear.x;
        target_.angular_z = message->angular.z;
        last_command_time_ = now();
        has_command_ = true;
      });

    // The traffic controller publishes for the whole fleet on one topic; each
    // robot filters for itself. One topic keeps the graph flat as the fleet
    // grows, instead of N topics that every new node has to learn about.
    directive_subscription_ = create_subscription<amr_msgs::msg::TrafficDirective>(
      "/fleet/traffic_directives", rclcpp::QoS(20),
      [this](amr_msgs::msg::TrafficDirective::SharedPtr message) {
        if (message->robot_id != robot_.name) {
          return;
        }
        speed_scale_ = std::clamp(message->speed_scale, 0.0, 1.0);
        last_directive_time_ = now();
        has_directive_ = true;

        if (message->action != last_action_) {
          RCLCPP_INFO(
            get_logger(), "traffic directive: %s (scale %.2f) - %s",
            ActionName(message->action), speed_scale_, message->reason.c_str());
          last_action_ = message->action;
        }
      });

    payload_service_ = create_service<amr_msgs::srv::SetPayload>(
      "set_payload",
      [this](
        const std::shared_ptr<amr_msgs::srv::SetPayload::Request> request,
        std::shared_ptr<amr_msgs::srv::SetPayload::Response> response) {
        OnSetPayload(*request, response.get());
      });

    const double rate = get_parameter("control_rate").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {OnTimer();});

    last_tick_time_ = now();
    PublishPayloadState();

    RCLCPP_INFO(
      get_logger(),
      "velocity smoother up for %s (model '%s', role %s) at %.0f Hz | "
      "accel %.2f m/s^2, jerk %.2f m/s^3, capacity %.0f kg",
      robot_.name.c_str(), robot_.profile.model_name.c_str(),
      amr_core::RoleToString(robot_.profile.role), rate,
      robot_.profile.limits.max_accel_x, robot_.profile.limits.max_jerk_x,
      robot_.profile.payload_capacity_kg);
  }

private:
  static const char * ActionName(std::uint8_t action)
  {
    switch (action) {
      case amr_msgs::msg::TrafficDirective::ACTION_SLOW:
        return "SLOW";
      case amr_msgs::msg::TrafficDirective::ACTION_YIELD:
        return "YIELD";
      case amr_msgs::msg::TrafficDirective::ACTION_HOLD:
        return "HOLD";
      default:
        return "PROCEED";
    }
  }

  std::string DeclareRequiredString(const std::string & name)
  {
    declare_parameter<std::string>(name, "");
    const std::string value = get_parameter(name).as_string();
    if (value.empty()) {
      throw std::runtime_error("required parameter '" + name + "' was not set");
    }
    return value;
  }

  void OnSetPayload(
    const amr_msgs::srv::SetPayload::Request & request,
    amr_msgs::srv::SetPayload::Response * response)
  {
    const double capacity = robot_.profile.payload_capacity_kg;
    if (request.payload_kg < 0.0) {
      response->success = false;
      response->message = "payload must not be negative";
    } else if (request.payload_kg > capacity) {
      // Clamp rather than reject: refusing to model an overload would leave
      // the smoother using the *empty* limits for an overloaded robot, which
      // is the more dangerous of the two errors.
      response->success = false;
      response->message = "payload exceeds capacity; clamped to capacity";
      payload_kg_ = capacity;
    } else {
      payload_kg_ = request.payload_kg;
      response->success = true;
      response->message = "accepted";
    }
    response->accepted_payload_kg = payload_kg_;
    response->load_ratio = robot_.profile.LoadRatio(payload_kg_);
    PublishPayloadState();

    RCLCPP_INFO(
      get_logger(), "payload now %.1f kg (%.0f%% of capacity); accel limit %.3f m/s^2",
      payload_kg_, 100.0 * response->load_ratio,
      robot_.profile.limits.EffectiveAccelX(response->load_ratio, 0.0));
  }

  void PublishPayloadState()
  {
    amr_msgs::msg::PayloadState message;
    message.header.stamp = now();
    message.robot_id = robot_.name;
    message.payload_kg = payload_kg_;
    message.capacity_kg = robot_.profile.payload_capacity_kg;
    message.load_ratio = robot_.profile.LoadRatio(payload_kg_);
    payload_publisher_->publish(message);
  }

  void OnTimer()
  {
    const rclcpp::Time current = now();
    const double dt = (current - last_tick_time_).seconds();
    last_tick_time_ = current;

    DynamicState state;
    state.load_ratio = robot_.profile.LoadRatio(payload_kg_);

    // A directive that has stopped arriving is treated as "no restriction"
    // only after a timeout, and the lapse is logged. Failing open here is
    // deliberate: the traffic controller is a throughput optimiser, not a
    // safety device, and a dead controller must not immobilise the fleet. The
    // safety override is the component that fails closed.
    if (has_directive_ && (current - last_directive_time_).seconds() > directive_timeout_) {
      if (speed_scale_ < 1.0) {
        RCLCPP_WARN(
          get_logger(),
          "no traffic directive for %.1f s; releasing speed scale %.2f. Collision "
          "avoidance now rests entirely on the safety override.",
          (current - last_directive_time_).seconds(), speed_scale_);
      }
      speed_scale_ = 1.0;
      has_directive_ = false;
    }
    state.speed_scale = speed_scale_;

    const bool command_stale =
      !has_command_ || (current - last_command_time_).seconds() > command_timeout_;

    const amr_core::Velocity2D command =
      command_stale ? smoother_->SmoothToStop(state, dt) : smoother_->Smooth(target_, state, dt);

    if (command_stale && has_command_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no cmd_vel_nav for %.2f s; ramping to a controlled stop",
        (current - last_command_time_).seconds());
    }

    geometry_msgs::msg::Twist output;
    output.linear.x = command.linear_x;
    output.angular.z = command.angular_z;
    command_publisher_->publish(output);
  }

  amr_core::RobotInstance robot_;
  std::unique_ptr<MotionSmoother> smoother_;

  amr_core::Velocity2D target_;
  double payload_kg_ = 0.0;
  double speed_scale_ = 1.0;
  double command_timeout_ = 0.5;
  double directive_timeout_ = 1.5;

  bool has_command_ = false;
  bool has_directive_ = false;
  std::uint8_t last_action_ = amr_msgs::msg::TrafficDirective::ACTION_PROCEED;

  rclcpp::Time last_command_time_;
  rclcpp::Time last_directive_time_;
  rclcpp::Time last_tick_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<amr_msgs::msg::PayloadState>::SharedPtr payload_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<amr_msgs::msg::TrafficDirective>::SharedPtr directive_subscription_;
  rclcpp::Service<amr_msgs::srv::SetPayload>::SharedPtr payload_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace amr_fleet_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<amr_fleet_control::VelocitySmootherNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("velocity_smoother"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
