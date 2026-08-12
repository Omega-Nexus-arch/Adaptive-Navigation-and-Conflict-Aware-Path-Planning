// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_FLEET_CONTROL__TRAJECTORY_HPP_
#define AMR_FLEET_CONTROL__TRAJECTORY_HPP_

#include <string>
#include <vector>

#include "amr_core/geometry.hpp"
#include "amr_core/robot_model.hpp"

namespace amr_fleet_control
{

/// \brief One space-time sample of a projected motion.
struct TrajectorySample
{
  double time = 0.0;              ///< Seconds after the trajectory's stamp.
  amr_core::Pose2D pose;          ///< In the shared global frame.
  double speed = 0.0;             ///< Forward speed at this sample [m/s].
};

/// \brief A robot's forward projection, as consumed by the conflict detector.
///
/// This is the ROS-free twin of `amr_msgs/PredictedTrajectory`. Conversion
/// happens once at the node boundary so the algorithm never touches a message
/// type and stays trivially testable.
struct PredictedPath
{
  std::string robot_id;
  double stamp = 0.0;             ///< Absolute time of sample 0 [s].
  double footprint_radius = 0.4;
  int yield_priority = 0;
  std::vector<TrajectorySample> samples;

  bool Empty() const {return samples.empty();}

  /// \brief Pose at absolute time \p t, linearly interpolated between samples.
  ///
  /// Queries outside the sampled span clamp to the nearest endpoint: a robot
  /// is assumed to sit at the end of its projection rather than to vanish,
  /// which is the conservative reading for collision checking.
  amr_core::Pose2D PoseAt(double t) const;

  /// \brief Speed at absolute time \p t, interpolated the same way.
  double SpeedAt(double t) const;

  /// \brief Absolute time of the first sample.
  double StartTime() const {return stamp;}

  /// \brief Absolute time of the last sample.
  double EndTime() const
  {
    return samples.empty() ? stamp : stamp + samples.back().time;
  }
};

/// \brief Configuration for the forward projection.
struct PredictorOptions
{
  double horizon_seconds = 4.0;
  double sample_period = 0.2;
  /// Look-ahead used when following a global plan [m]. Scaled with speed so a
  /// fast robot looks further down its own path.
  double lookahead_gain = 1.2;
  double min_lookahead = 0.4;
};

/// \brief Projects where a robot will be, so its peers can plan around it.
///
/// Two modes, in order of preference:
///
/// 1. **Plan-following.** When the robot has a global plan, the projection
///    walks that plan at the current speed. This is much more informative than
///    extrapolating the current twist, because it knows the robot is about to
///    turn into an aisle rather than continue straight into a rack.
///
/// 2. **Constant-curvature rollout.** With no plan, the current twist is
///    integrated forward with the exact arc solution. Used while the robot is
///    idle, recovering, or between goals.
///
/// Either way the result is expressed in the shared global frame, which is
/// what makes trajectories from different robots comparable at all.
class TrajectoryPredictor
{
public:
  explicit TrajectoryPredictor(
    const amr_core::RobotProfile & profile, const PredictorOptions & options);

  /// \brief Roll the current twist forward. No plan required.
  PredictedPath PredictFromTwist(
    const std::string & robot_id, int yield_priority, const amr_core::Pose2D & pose,
    const amr_core::Velocity2D & velocity, double stamp) const;

  /// \brief Project along \p plan, which must be in the same frame as \p pose.
  ///
  /// Falls back to :func:`PredictFromTwist` when the plan is too short to be
  /// informative, so callers never have to special-case an empty plan.
  PredictedPath PredictAlongPlan(
    const std::string & robot_id, int yield_priority, const amr_core::Pose2D & pose,
    const amr_core::Velocity2D & velocity, const std::vector<amr_core::Pose2D> & plan,
    double stamp) const;

private:
  amr_core::RobotProfile profile_;
  PredictorOptions options_;
};

}  // namespace amr_fleet_control

#endif  // AMR_FLEET_CONTROL__TRAJECTORY_HPP_
