// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.

#include "amr_fleet_control/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace amr_fleet_control
{

using amr_core::Pose2D;
using amr_core::Velocity2D;

namespace
{

/// \brief Index of the last sample at or before \p relative_time.
std::size_t LowerSampleIndex(
  const std::vector<TrajectorySample> & samples, double relative_time)
{
  if (samples.size() < 2) {
    return 0;
  }
  // Samples are uniformly spaced by construction, so a direct index beats a
  // binary search and keeps the inner loop of the conflict detector cheap.
  const double period = samples[1].time - samples[0].time;
  if (period <= 0.0) {
    return 0;
  }
  const double raw = relative_time / period;
  if (raw <= 0.0) {
    return 0;
  }
  const auto index = static_cast<std::size_t>(raw);
  return std::min(index, samples.size() - 2);
}

}  // namespace

Pose2D PredictedPath::PoseAt(double t) const
{
  if (samples.empty()) {
    return Pose2D{};
  }
  const double relative = t - stamp;
  if (relative <= samples.front().time) {
    return samples.front().pose;
  }
  if (relative >= samples.back().time) {
    return samples.back().pose;
  }

  const std::size_t i = LowerSampleIndex(samples, relative);
  const TrajectorySample & a = samples[i];
  const TrajectorySample & b = samples[i + 1];
  const double span = b.time - a.time;
  const double alpha = (span > 0.0) ? (relative - a.time) / span : 0.0;

  Pose2D pose;
  pose.x = a.pose.x + alpha * (b.pose.x - a.pose.x);
  pose.y = a.pose.y + alpha * (b.pose.y - a.pose.y);
  // Interpolating the heading through the shortest arc, not the raw
  // difference, so a projection crossing +/-pi does not spin the wrong way.
  pose.theta = amr_core::NormalizeAngle(
    a.pose.theta + alpha * amr_core::AngleDifference(a.pose.theta, b.pose.theta));
  return pose;
}

double PredictedPath::SpeedAt(double t) const
{
  if (samples.empty()) {
    return 0.0;
  }
  const double relative = t - stamp;
  if (relative <= samples.front().time) {
    return samples.front().speed;
  }
  if (relative >= samples.back().time) {
    return samples.back().speed;
  }
  const std::size_t i = LowerSampleIndex(samples, relative);
  const TrajectorySample & a = samples[i];
  const TrajectorySample & b = samples[i + 1];
  const double span = b.time - a.time;
  const double alpha = (span > 0.0) ? (relative - a.time) / span : 0.0;
  return a.speed + alpha * (b.speed - a.speed);
}

TrajectoryPredictor::TrajectoryPredictor(
  const amr_core::RobotProfile & profile, const PredictorOptions & options)
: profile_(profile), options_(options) {}

PredictedPath TrajectoryPredictor::PredictFromTwist(
  const std::string & robot_id, int yield_priority, const Pose2D & pose,
  const Velocity2D & velocity, double stamp) const
{
  PredictedPath path;
  path.robot_id = robot_id;
  path.stamp = stamp;
  path.footprint_radius = profile_.footprint_radius;
  path.yield_priority = yield_priority;

  const int steps =
    std::max(1, static_cast<int>(std::round(options_.horizon_seconds / options_.sample_period)));
  path.samples.reserve(static_cast<std::size_t>(steps) + 1);

  Pose2D current = pose;
  path.samples.push_back(TrajectorySample{0.0, current, velocity.linear_x});
  for (int step = 1; step <= steps; ++step) {
    current = amr_core::IntegrateUnicycle(current, velocity, options_.sample_period);
    path.samples.push_back(
      TrajectorySample{step * options_.sample_period, current, velocity.linear_x});
  }
  return path;
}

PredictedPath TrajectoryPredictor::PredictAlongPlan(
  const std::string & robot_id, int yield_priority, const Pose2D & pose,
  const Velocity2D & velocity, const std::vector<Pose2D> & plan, double stamp) const
{
  // With fewer than two poses there is no direction information in the plan,
  // so the twist rollout is strictly better.
  if (plan.size() < 2) {
    return PredictFromTwist(robot_id, yield_priority, pose, velocity, stamp);
  }

  PredictedPath path;
  path.robot_id = robot_id;
  path.stamp = stamp;
  path.footprint_radius = profile_.footprint_radius;
  path.yield_priority = yield_priority;

  // Speed used for the projection. A stationary robot that still holds a plan
  // is about to move, so assume a slow crawl rather than predicting that it
  // stays put forever - that would hide the conflict until it is too late to
  // resolve gracefully.
  const double speed = std::max(std::abs(velocity.linear_x), 0.15 * profile_.limits.max_vel_x);

  // Locate the plan vertex nearest the robot; the projection starts there so a
  // stale prefix of the plan cannot drag the prediction backwards.
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const double distance = amr_core::SquaredDistance(pose, plan[i]);
    if (distance < best) {
      best = distance;
      nearest = i;
    }
  }

  // Arc length from the nearest vertex to each subsequent vertex.
  std::vector<double> cumulative(plan.size(), 0.0);
  for (std::size_t i = nearest + 1; i < plan.size(); ++i) {
    cumulative[i] = cumulative[i - 1] + amr_core::Distance(plan[i - 1], plan[i]);
  }
  const double plan_length = cumulative.back();

  const int steps =
    std::max(1, static_cast<int>(std::round(options_.horizon_seconds / options_.sample_period)));
  path.samples.reserve(static_cast<std::size_t>(steps) + 1);

  std::size_t cursor = nearest;
  for (int step = 0; step <= steps; ++step) {
    const double time = step * options_.sample_period;
    const double travelled = std::min(speed * time, plan_length);

    // Walk forward to the segment containing `travelled`. The cursor only ever
    // advances, so the whole projection is O(plan length), not O(steps * plan).
    while (cursor + 1 < plan.size() && cumulative[cursor + 1] < travelled) {
      ++cursor;
    }

    Pose2D sample_pose;
    if (cursor + 1 >= plan.size()) {
      sample_pose = plan.back();
    } else {
      const double segment = cumulative[cursor + 1] - cumulative[cursor];
      const double alpha = (segment > 1e-9) ? (travelled - cumulative[cursor]) / segment : 0.0;
      const Pose2D & a = plan[cursor];
      const Pose2D & b = plan[cursor + 1];
      sample_pose.x = a.x + alpha * (b.x - a.x);
      sample_pose.y = a.y + alpha * (b.y - a.y);
      // Global plans frequently carry a meaningless orientation on every pose,
      // so the heading is taken from the direction of travel instead.
      sample_pose.theta = std::atan2(b.y - a.y, b.x - a.x);
    }

    // Once the plan runs out the robot has arrived: report it as stationary.
    const double sample_speed = (travelled >= plan_length) ? 0.0 : speed;
    path.samples.push_back(TrajectorySample{time, sample_pose, sample_speed});
  }

  return path;
}

}  // namespace amr_fleet_control
