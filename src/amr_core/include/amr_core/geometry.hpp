// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// ROS-free geometry helpers. Keeping these independent of rclcpp is what lets
// the algorithmic cores (motion smoother, conflict detector, safety monitor)
// be unit-tested without spinning a node.
//
// Style: Google C++ Style Guide.

#ifndef AMR_CORE__GEOMETRY_HPP_
#define AMR_CORE__GEOMETRY_HPP_

#include <algorithm>
#include <cmath>

namespace amr_core
{

/// \brief Planar pose. All angles in radians, all lengths in metres (REP-103).
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

/// \brief Planar body velocity for a differential-drive robot.
struct Velocity2D
{
  double linear_x = 0.0;
  double angular_z = 0.0;
};

/// \brief Wrap an angle to (-pi, pi].
inline double NormalizeAngle(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle <= 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

/// \brief Smallest signed rotation carrying \p from onto \p to.
inline double AngleDifference(double from, double to)
{
  return NormalizeAngle(to - from);
}

inline double Clamp(double value, double low, double high)
{
  return std::min(high, std::max(low, value));
}

/// \brief Yaw of a quaternion, without pulling in tf2.
inline double YawFromQuaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

/// \brief Pitch of a quaternion. Used to interpret IMU attitude on ramps.
inline double PitchFromQuaternion(double x, double y, double z, double w)
{
  const double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) {
    return std::copysign(M_PI / 2.0, sinp);
  }
  return std::asin(sinp);
}

inline double Distance(const Pose2D & a, const Pose2D & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

/// \brief Squared distance. Prefer this inside inner loops.
inline double SquaredDistance(const Pose2D & a, const Pose2D & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return dx * dx + dy * dy;
}

/// \brief Advance a differential-drive pose by \p dt under constant velocity.
///
/// Uses the exact arc solution rather than the Euler approximation. At the
/// yaw rates the scout is allowed (1.9 rad/s) and a 20 Hz planning tick, the
/// Euler error is several centimetres per step, which is enough to make a
/// predicted trajectory miss a real conflict.
inline Pose2D IntegrateUnicycle(const Pose2D & pose, const Velocity2D & vel, double dt)
{
  Pose2D next;
  if (std::abs(vel.angular_z) < 1e-6) {
    next.x = pose.x + vel.linear_x * std::cos(pose.theta) * dt;
    next.y = pose.y + vel.linear_x * std::sin(pose.theta) * dt;
    next.theta = pose.theta;
    return next;
  }

  const double radius = vel.linear_x / vel.angular_z;
  const double theta_next = pose.theta + vel.angular_z * dt;
  next.x = pose.x + radius * (std::sin(theta_next) - std::sin(pose.theta));
  next.y = pose.y - radius * (std::cos(theta_next) - std::cos(pose.theta));
  next.theta = NormalizeAngle(theta_next);
  return next;
}

/// \brief Bearing of a point relative to a pose, in the pose's own frame.
inline double RelativeBearing(const Pose2D & observer, double target_x, double target_y)
{
  const double bearing = std::atan2(target_y - observer.y, target_x - observer.x);
  return NormalizeAngle(bearing - observer.theta);
}

}  // namespace amr_core

#endif  // AMR_CORE__GEOMETRY_HPP_
