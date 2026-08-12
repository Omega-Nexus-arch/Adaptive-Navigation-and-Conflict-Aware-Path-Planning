// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "amr_core/fleet_config.hpp"
#include "amr_core/geometry.hpp"
#include "amr_fleet_control/trajectory.hpp"
#include "amr_msgs/msg/predicted_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_fleet_control
{

/// \brief Publishes this robot's forward projection for its peers to plan
///        around.
///
/// One instance runs per robot. Each publishes to the single fleet-wide topic
/// `/fleet/trajectories`; the traffic controller and every robot's local
/// costmap layer subscribe to that one topic. Keeping it flat is what lets the
/// fleet grow without anybody rewiring the graph.
///
/// Projections are always expressed in the shared global frame. Publishing in
/// a per-robot odom frame would make trajectories from different robots
/// incomparable, which is the whole point of sharing them.
class TrajectoryBroadcasterNode : public rclcpp::Node
{
public:
  TrajectoryBroadcasterNode()
  : rclcpp::Node("trajectory_broadcaster")
  {
    const std::string robot_name = DeclareRequiredString("robot_name");
    const std::string fleet_config = DeclareRequiredString("fleet_config");

    declare_parameter<double>("plan_timeout", 3.0);

    const amr_core::FleetConfig fleet = amr_core::FleetConfig::FromFile(fleet_config);
    robot_ = fleet.Robot(robot_name);
    global_frame_ = fleet.GlobalFrame();
    plan_timeout_ = get_parameter("plan_timeout").as_double();

    PredictorOptions options;
    options.horizon_seconds = fleet.Policy().horizon_seconds;
    options.sample_period = fleet.Policy().sample_period;
    predictor_ = std::make_unique<TrajectoryPredictor>(robot_.profile, options);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = create_publisher<amr_msgs::msg::PredictedTrajectory>(
      "/fleet/trajectories", rclcpp::QoS(20));

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr message) {
        velocity_.linear_x = message->twist.twist.linear.x;
        velocity_.angular_z = message->twist.twist.angular.z;
      });

    // nav2's controller server republishes the portion of the global plan it
    // is currently tracking, which is exactly the horizon worth projecting.
    plan_subscription_ = create_subscription<nav_msgs::msg::Path>(
      "plan", rclcpp::QoS(1),
      [this](nav_msgs::msg::Path::SharedPtr message) {OnPlan(message);});

    const double rate = fleet.Policy().trajectory_publish_rate_hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {OnTimer();});

    RCLCPP_INFO(
      get_logger(), "broadcasting %s trajectories in '%s' at %.0f Hz (%.1f s horizon)",
      robot_.name.c_str(), global_frame_.c_str(), rate, options.horizon_seconds);
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

  void OnPlan(const nav_msgs::msg::Path::SharedPtr & message)
  {
    plan_.clear();
    plan_.reserve(message->poses.size());
    for (const auto & pose : message->poses) {
      amr_core::Pose2D point;
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      point.theta = amr_core::YawFromQuaternion(
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
        pose.pose.orientation.w);
      plan_.push_back(point);
    }
    plan_frame_ = message->header.frame_id;
    last_plan_time_ = now();
    has_plan_ = !plan_.empty();
  }

  /// \brief Current pose in the shared global frame, via TF.
  bool LookupPose(amr_core::Pose2D * pose)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        global_frame_, robot_.name + "/base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "cannot locate %s in '%s': %s",
        robot_.name.c_str(), global_frame_.c_str(), error.what());
      return false;
    }
    pose->x = transform.transform.translation.x;
    pose->y = transform.transform.translation.y;
    pose->theta = amr_core::YawFromQuaternion(
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w);
    return true;
  }

  void OnTimer()
  {
    amr_core::Pose2D pose;
    if (!LookupPose(&pose)) {
      // Publishing nothing is correct: peers treat a missing trajectory as
      // "unknown" and the conflict detector ages it out. Publishing a guessed
      // pose would be worse than silence.
      return;
    }

    const rclcpp::Time current = now();
    const bool plan_fresh =
      has_plan_ && (current - last_plan_time_).seconds() < plan_timeout_;

    // A plan in a frame other than the global one cannot be compared with
    // another robot's projection, so fall back rather than mixing frames.
    const bool plan_usable = plan_fresh && plan_frame_ == global_frame_;
    if (plan_fresh && !plan_usable) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "ignoring a plan published in '%s'; projections must be in '%s'",
        plan_frame_.c_str(), global_frame_.c_str());
    }

    const PredictedPath path = plan_usable ?
      predictor_->PredictAlongPlan(
      robot_.name, robot_.YieldPriority(), pose, velocity_, plan_, current.seconds()) :
      predictor_->PredictFromTwist(
      robot_.name, robot_.YieldPriority(), pose, velocity_, current.seconds());

    amr_msgs::msg::PredictedTrajectory message;
    message.header.stamp = current;
    message.header.frame_id = global_frame_;
    message.robot_id = path.robot_id;
    message.footprint_radius = path.footprint_radius;
    message.yield_priority = path.yield_priority;
    message.points.reserve(path.samples.size());

    for (const TrajectorySample & sample : path.samples) {
      amr_msgs::msg::TrajectoryPoint point;
      point.pose.position.x = sample.pose.x;
      point.pose.position.y = sample.pose.y;
      point.pose.position.z = 0.0;
      point.pose.orientation.z = std::sin(sample.pose.theta / 2.0);
      point.pose.orientation.w = std::cos(sample.pose.theta / 2.0);
      point.time_from_start = sample.time;
      point.speed = sample.speed;
      message.points.push_back(point);
    }

    publisher_->publish(message);
  }

  amr_core::RobotInstance robot_;
  std::string global_frame_ = "map";
  std::unique_ptr<TrajectoryPredictor> predictor_;

  amr_core::Velocity2D velocity_;
  std::vector<amr_core::Pose2D> plan_;
  std::string plan_frame_;
  bool has_plan_ = false;
  double plan_timeout_ = 3.0;
  rclcpp::Time last_plan_time_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<amr_msgs::msg::PredictedTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace amr_fleet_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<amr_fleet_control::TrajectoryBroadcasterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("trajectory_broadcaster"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
