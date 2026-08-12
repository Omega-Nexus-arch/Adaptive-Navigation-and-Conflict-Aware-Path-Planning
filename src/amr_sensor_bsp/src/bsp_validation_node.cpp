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
#include "amr_msgs/msg/sensor_health.hpp"
#include "amr_sensor_bsp/sensor_validators.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace amr_sensor_bsp
{

/// \brief The Board Support Package gate for one robot.
///
/// ### Topic contract
///
/// ```
///   Gazebo / driver              this node              navigation stack
///   ---------------              ---------              ----------------
///   scan_raw                --> [ LidarValidator  ] --> scan
///   imu_raw                 --> [ ImuValidator    ] --> imu
///   camera/image_unvalidated--> [ CameraValidator ] --> camera/image_raw
///                                       |
///                                       +-----------> sensor_health
/// ```
///
/// The navigation stack subscribes only to the right-hand column. Those topics
/// have no other publisher, so unvalidated data cannot reach a planner even by
/// mistake - the gate is structural, not a convention.
///
/// ### Ordering
///
/// The IMU callback runs the attitude estimate into the LiDAR validator before
/// the next scan is processed. That is what lets the LiDAR validator recognise
/// floor returns while the robot is on a ramp; see LidarValidator for why that
/// matters for navigation rather than just for tidiness.
class BspValidationNode : public rclcpp::Node
{
public:
  BspValidationNode()
  : rclcpp::Node("bsp_validation")
  {
    const std::string robot_name = DeclareRequiredString("robot_name");
    const std::string fleet_config = DeclareRequiredString("fleet_config");

    declare_parameter<double>("report_period", 2.0);
    declare_parameter<bool>("ground_rejection_enabled", true);
    declare_parameter<bool>("check_image_intensity", true);
    declare_parameter<double>("fault_log_period_ms", 2000.0);

    const amr_core::FleetConfig fleet = amr_core::FleetConfig::FromFile(fleet_config);
    robot_ = fleet.Robot(robot_name);

    LidarValidator::Options lidar_options;
    lidar_options.ground_rejection_enabled =
      get_parameter("ground_rejection_enabled").as_bool();
    lidar_ = std::make_unique<LidarValidator>(robot_.profile.lidar, lidar_options);

    imu_ = std::make_unique<ImuValidator>(robot_.profile.imu, ImuValidator::Options{});

    CameraValidator::Options camera_options;
    camera_options.check_intensity = get_parameter("check_image_intensity").as_bool();
    camera_ = std::make_unique<CameraValidator>(robot_.profile.camera, camera_options);

    fault_log_period_ms_ =
      static_cast<int>(get_parameter("fault_log_period_ms").as_double());

    scan_publisher_ =
      create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
    image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("camera/image_raw", rclcpp::SensorDataQoS());
    health_publisher_ =
      create_publisher<amr_msgs::msg::SensorHealth>("sensor_health", rclcpp::QoS(10));

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr message) {OnScan(message);});
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr message) {OnImu(message);});
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image_unvalidated", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr message) {OnImage(message);});

    report_timer_ = create_wall_timer(
      std::chrono::duration<double>(get_parameter("report_period").as_double()),
      [this]() {PublishHealth();});

    RCLCPP_INFO(
      get_logger(),
      "BSP validation active for %s | IMU |omega| limit %.2f rad/s, LiDAR %d beams "
      "over [%.2f, %.2f] m, camera %dx%d",
      robot_.name.c_str(), robot_.profile.imu.max_angular_velocity,
      robot_.profile.lidar.samples, robot_.profile.lidar.range_min,
      robot_.profile.lidar.range_max, robot_.profile.camera.width,
      robot_.profile.camera.height_px);
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

  static std::string JoinFaults(const std::vector<std::string> & faults)
  {
    std::string joined;
    for (const std::string & fault : faults) {
      joined += joined.empty() ? fault : ", " + fault;
    }
    return joined;
  }

  /// \brief Log a fault at a severity matching how the message was treated.
  ///
  /// Throttled: a persistently faulty sensor at 30 Hz would otherwise make the
  /// console useless precisely when someone is trying to read it.
  void LogResult(const char * sensor, const ValidationResult & result)
  {
    if (result.faults.empty()) {
      return;
    }
    const std::string faults = JoinFaults(result.faults);
    if (result.status == ValidationStatus::kInvalid) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), fault_log_period_ms_,
        "[%s] %s REJECTED: %s - the navigation stack will not see this message",
        robot_.name.c_str(), sensor, faults.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), fault_log_period_ms_,
        "[%s] %s degraded: %s - forwarding, but the sensor is misbehaving",
        robot_.name.c_str(), sensor, faults.c_str());
    }
  }

  void OnScan(const sensor_msgs::msg::LaserScan::SharedPtr & message)
  {
    LidarFrame frame;
    frame.stamp = rclcpp::Time(message->header.stamp).seconds();
    frame.angle_min = message->angle_min;
    frame.angle_increment = message->angle_increment;
    frame.range_min = message->range_min;
    frame.range_max = message->range_max;
    frame.ranges = message->ranges;

    std::vector<float> conditioned;
    const ValidationResult result = lidar_->Validate(frame, &conditioned);
    LogResult("lidar", result);

    if (!result.ShouldForward()) {
      return;
    }

    // Republish the conditioned scan, preserving the original header so
    // timestamps and TF lookups downstream are unaffected.
    sensor_msgs::msg::LaserScan validated = *message;
    validated.ranges = std::move(conditioned);
    scan_publisher_->publish(validated);

    if (lidar_->LastGroundReturnsSuppressed() > 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "[%s] on a %.1f deg slope: suppressed %u floor returns beyond %.2f m",
        robot_.name.c_str(), lidar_->Pitch() * 180.0 / M_PI,
        lidar_->LastGroundReturnsSuppressed(), lidar_->GroundReturnRange());
    }
  }

  void OnImu(const sensor_msgs::msg::Imu::SharedPtr & message)
  {
    ImuFrame frame;
    frame.stamp = rclcpp::Time(message->header.stamp).seconds();
    frame.angular_velocity[0] = message->angular_velocity.x;
    frame.angular_velocity[1] = message->angular_velocity.y;
    frame.angular_velocity[2] = message->angular_velocity.z;
    frame.linear_acceleration[0] = message->linear_acceleration.x;
    frame.linear_acceleration[1] = message->linear_acceleration.y;
    frame.linear_acceleration[2] = message->linear_acceleration.z;
    frame.orientation[0] = message->orientation.x;
    frame.orientation[1] = message->orientation.y;
    frame.orientation[2] = message->orientation.z;
    frame.orientation[3] = message->orientation.w;

    const ValidationResult result = imu_->Validate(frame);

    // The explicit requirement from the brief, given its own log line rather
    // than being folded into the generic fault report.
    const double magnitude = std::sqrt(
      frame.angular_velocity[0] * frame.angular_velocity[0] +
      frame.angular_velocity[1] * frame.angular_velocity[1] +
      frame.angular_velocity[2] * frame.angular_velocity[2]);
    if (magnitude > robot_.profile.imu.max_angular_velocity) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), fault_log_period_ms_,
        "[%s] IMU angular velocity %.2f rad/s exceeds the %.2f rad/s limit for model "
        "'%s' - physically implausible for this chassis, treating as a sensor fault",
        robot_.name.c_str(), magnitude, robot_.profile.imu.max_angular_velocity,
        robot_.profile.model_name.c_str());
    }
    LogResult("imu", result);

    if (!result.ShouldForward()) {
      return;
    }

    // Feed attitude forward before the next scan arrives, so the LiDAR
    // validator can recognise floor returns while climbing a ramp.
    lidar_->SetPitch(imu_->Pitch());
    imu_publisher_->publish(*message);
  }

  void OnImage(const sensor_msgs::msg::Image::SharedPtr & message)
  {
    CameraFrame frame;
    frame.stamp = rclcpp::Time(message->header.stamp).seconds();
    frame.width = message->width;
    frame.height = message->height;
    frame.encoding = message->encoding;
    frame.data = message->data.data();
    frame.data_size = message->data.size();
    frame.channels = (message->encoding == "mono8") ? 1u : 3u;

    const ValidationResult result = camera_->Validate(frame);
    LogResult("camera", result);

    if (result.ShouldForward()) {
      image_publisher_->publish(*message);
    }
  }

  void PublishOne(const SensorValidator & validator, const std::vector<std::string> & faults)
  {
    amr_msgs::msg::SensorHealth message;
    message.header.stamp = now();
    message.robot_id = robot_.name;
    message.sensor_name = validator.SensorName();
    const ValidationStatistics & statistics = validator.Statistics();
    message.messages_received = statistics.received;
    message.messages_rejected = statistics.rejected;
    message.messages_degraded = statistics.degraded;
    message.measured_rate_hz = statistics.measured_rate_hz;
    message.active_faults = faults;

    if (statistics.received == 0) {
      // Never heard from: not healthy, but not rejected either.
      message.status = amr_msgs::msg::SensorHealth::STATUS_INVALID;
    } else if (statistics.rejected > 0) {
      message.status = amr_msgs::msg::SensorHealth::STATUS_INVALID;
    } else if (statistics.degraded > 0) {
      message.status = amr_msgs::msg::SensorHealth::STATUS_DEGRADED;
    } else {
      message.status = amr_msgs::msg::SensorHealth::STATUS_OK;
    }
    health_publisher_->publish(message);
  }

  void PublishHealth()
  {
    PublishOne(*lidar_, {});
    PublishOne(*imu_, {});
    PublishOne(*camera_, {});

    // A stream that has produced nothing at all is the failure most likely to
    // be missed, because there are no error messages to notice.
    WarnIfSilent(*lidar_, "scan_raw");
    WarnIfSilent(*imu_, "imu_raw");
    WarnIfSilent(*camera_, "camera/image_unvalidated");
  }

  void WarnIfSilent(const SensorValidator & validator, const char * topic)
  {
    if (validator.Statistics().received == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "[%s] no %s messages have ever arrived on '%s'", robot_.name.c_str(),
        validator.SensorName().c_str(), topic);
    }
  }

  amr_core::RobotInstance robot_;
  std::unique_ptr<LidarValidator> lidar_;
  std::unique_ptr<ImuValidator> imu_;
  std::unique_ptr<CameraValidator> camera_;
  int fault_log_period_ms_ = 2000;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<amr_msgs::msg::SensorHealth>::SharedPtr health_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

}  // namespace amr_sensor_bsp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<amr_sensor_bsp::BspValidationNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("bsp_validation"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
