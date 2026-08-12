// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.

#include "amr_fleet_control/traffic_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace amr_fleet_control
{

const char * TrafficActionToString(TrafficAction action)
{
  switch (action) {
    case TrafficAction::kSlow:
      return "SLOW";
    case TrafficAction::kYield:
      return "YIELD";
    case TrafficAction::kHold:
      return "HOLD";
    case TrafficAction::kProceed:
    default:
      return "PROCEED";
  }
}

// ---------------------------------------------------------------------------
// ConflictDetector
// ---------------------------------------------------------------------------

ConflictDetector::ConflictDetector(const amr_core::FleetPolicy & policy)
: policy_(policy) {}

bool ConflictDetector::DetectPair(
  const PredictedPath & a, const PredictedPath & b, double now, Conflict * conflict) const
{
  if (a.Empty() || b.Empty()) {
    return false;
  }

  const double required = a.footprint_radius + b.footprint_radius + policy_.conflict_margin;

  // Compare only over the window both projections actually cover, and never
  // further ahead than the policy is willing to act on.
  const double start = std::max({now, a.StartTime(), b.StartTime()});
  const double end = std::min(
    {a.EndTime(), b.EndTime(), now + policy_.conflict_react_seconds});
  if (end <= start) {
    return false;
  }

  const int steps =
    std::max(1, static_cast<int>(std::ceil((end - start) / policy_.sample_period)));

  for (int step = 0; step <= steps; ++step) {
    const double t = std::min(start + step * policy_.sample_period, end);
    const amr_core::Pose2D pose_a = a.PoseAt(t);
    const amr_core::Pose2D pose_b = b.PoseAt(t);
    const double separation = amr_core::Distance(pose_a, pose_b);

    if (separation < required) {
      if (conflict != nullptr) {
        conflict->robot_a = a.robot_id;
        conflict->robot_b = b.robot_id;
        conflict->time_to_conflict = t - now;
        conflict->separation = separation;
        conflict->required_separation = required;
        conflict->x = 0.5 * (pose_a.x + pose_b.x);
        conflict->y = 0.5 * (pose_a.y + pose_b.y);
      }
      return true;
    }
  }
  return false;
}

std::vector<Conflict> ConflictDetector::Detect(
  const std::map<std::string, PredictedPath> & paths, double now) const
{
  // Drop stale projections up front so the pairwise loop below never has to
  // reason about freshness.
  std::vector<const PredictedPath *> fresh;
  fresh.reserve(paths.size());
  for (const auto & entry : paths) {
    const PredictedPath & path = entry.second;
    if (path.Empty()) {
      continue;
    }
    if (now - path.stamp > policy_.trajectory_timeout_seconds) {
      continue;
    }
    fresh.push_back(&path);
  }

  std::vector<Conflict> conflicts;
  for (std::size_t i = 0; i < fresh.size(); ++i) {
    for (std::size_t j = i + 1; j < fresh.size(); ++j) {
      Conflict conflict;
      if (DetectPair(*fresh[i], *fresh[j], now, &conflict)) {
        conflicts.push_back(conflict);
      }
    }
  }

  // Most imminent first: the arbitration below resolves in this order, so the
  // urgent conflict decides the directive when a robot has several.
  std::sort(
    conflicts.begin(), conflicts.end(),
    [](const Conflict & lhs, const Conflict & rhs) {
      return lhs.time_to_conflict < rhs.time_to_conflict;
    });
  return conflicts;
}

// ---------------------------------------------------------------------------
// YieldPolicy
// ---------------------------------------------------------------------------

YieldPolicy::YieldPolicy(const amr_core::FleetConfig & fleet)
: fleet_(fleet)
{
  for (const auto & robot : fleet_.Robots()) {
    RobotState state;
    state.base_priority = robot.YieldPriority();
    max_base_priority_ = std::max(max_base_priority_, state.base_priority);
    state_.emplace(robot.name, state);
  }
}

void YieldPolicy::Reset()
{
  for (auto & entry : state_) {
    entry.second.yielding = false;
    entry.second.yield_started_at = 0.0;
    entry.second.boost_until = 0.0;
  }
}

bool YieldPolicy::HasPriorityBoost(const std::string & robot_id, double now) const
{
  const auto it = state_.find(robot_id);
  return it != state_.end() && now < it->second.boost_until;
}

int YieldPolicy::EffectivePriority(const std::string & robot_id, double now) const
{
  const auto it = state_.find(robot_id);
  if (it == state_.end()) {
    return 0;
  }
  if (now < it->second.boost_until) {
    // Lift the starved robot clear of the entire roster for the duration of
    // the boost. +1 is enough because base priorities are unique integers.
    return max_base_priority_ + 1;
  }
  return it->second.base_priority;
}

double YieldPolicy::YieldDuration(const std::string & robot_id, double now) const
{
  const auto it = state_.find(robot_id);
  if (it == state_.end() || !it->second.yielding) {
    return 0.0;
  }
  return now - it->second.yield_started_at;
}

std::map<std::string, Directive> YieldPolicy::Resolve(
  const std::vector<Conflict> & conflicts, double now)
{
  // Start from "everyone proceeds" and let conflicts demote individuals. This
  // ordering matters: a robot that has dropped out of the conflict set gets an
  // explicit release rather than inheriting last cycle's yield.
  std::map<std::string, Directive> directives;
  for (const auto & robot : fleet_.Robots()) {
    Directive directive;
    directive.robot_id = robot.name;
    directive.action = TrafficAction::kProceed;
    directive.speed_scale = 1.0;
    directive.reason = "clear";
    directives.emplace(robot.name, directive);
  }

  // First pass: starvation check. Done before arbitration so a boost earned on
  // a previous cycle takes effect on this one.
  for (auto & entry : state_) {
    RobotState & state = entry.second;
    if (state.yielding && (now - state.yield_started_at) > fleet_.Policy().max_yield_seconds) {
      state.boost_until = now + fleet_.Policy().yield_cooldown_seconds;
      state.yielding = false;
    }
  }

  // Second pass: conflicts, most imminent first.
  for (const Conflict & conflict : conflicts) {
    auto it_a = directives.find(conflict.robot_a);
    auto it_b = directives.find(conflict.robot_b);
    if (it_a == directives.end() || it_b == directives.end()) {
      // A robot that is not on the roster (a manual test publisher, say) is
      // ignored rather than allowed to command the fleet.
      continue;
    }

    const int priority_a = EffectivePriority(conflict.robot_a, now);
    const int priority_b = EffectivePriority(conflict.robot_b, now);

    // Unique priorities are enforced at load time, so there is no tie to break.
    const bool a_yields = priority_a < priority_b;
    Directive & yielder = a_yields ? it_a->second : it_b->second;
    const std::string & winner = a_yields ? conflict.robot_b : conflict.robot_a;

    // A robot already told to stop by a nearer conflict is not downgraded to a
    // slow-down by a more distant one.
    if (yielder.action == TrafficAction::kYield) {
      continue;
    }

    const bool imminent = conflict.time_to_conflict <= fleet_.Policy().hard_yield_seconds;
    std::ostringstream reason;
    reason << "yield to " << winner << " (priority " << (a_yields ? priority_b : priority_a)
           << " > " << (a_yields ? priority_a : priority_b) << "), conflict in "
           << std::fixed << std::setprecision(2) << conflict.time_to_conflict << " s at ("
           << conflict.x << ", " << conflict.y << ")";

    yielder.action = imminent ? TrafficAction::kYield : TrafficAction::kSlow;
    yielder.speed_scale = imminent ? 0.0 : fleet_.Policy().slow_speed_scale;
    yielder.conflicting_robot = winner;
    yielder.time_to_conflict = conflict.time_to_conflict;
    yielder.reason = reason.str();
  }

  // Third pass: fold the outcome back into the per-robot state so the next
  // cycle can measure how long each robot has been held.
  for (auto & entry : directives) {
    auto state_it = state_.find(entry.first);
    if (state_it == state_.end()) {
      continue;
    }
    RobotState & state = state_it->second;
    const bool now_yielding = entry.second.action == TrafficAction::kYield;

    if (now_yielding && !state.yielding) {
      state.yielding = true;
      state.yield_started_at = now;
    } else if (!now_yielding) {
      state.yielding = false;
    }

    if (now < state.boost_until) {
      entry.second.reason += " [anti-starvation boost active]";
    }
  }

  return directives;
}

}  // namespace amr_fleet_control
