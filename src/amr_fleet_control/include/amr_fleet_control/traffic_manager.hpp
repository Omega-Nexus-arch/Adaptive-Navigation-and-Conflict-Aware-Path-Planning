// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_FLEET_CONTROL__TRAFFIC_MANAGER_HPP_
#define AMR_FLEET_CONTROL__TRAFFIC_MANAGER_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "amr_core/fleet_config.hpp"
#include "amr_fleet_control/trajectory.hpp"

namespace amr_fleet_control
{

/// \brief A predicted space-time overlap between two robots.
struct Conflict
{
  std::string robot_a;
  std::string robot_b;
  double time_to_conflict = 0.0;   ///< Seconds from now until first overlap.
  double separation = 0.0;         ///< Centre distance at that moment [m].
  double required_separation = 0.0;
  double x = 0.0;                  ///< Where it would happen, global frame.
  double y = 0.0;
};

/// \brief Action the traffic controller assigns to one robot.
///
/// Values mirror `amr_msgs/TrafficDirective` so the node forwards them without
/// a mapping table that could drift out of step.
enum class TrafficAction : std::uint8_t
{
  kProceed = 0,
  kSlow = 1,
  kYield = 2,
  kHold = 3
};

const char * TrafficActionToString(TrafficAction action);

/// \brief A directive for one robot on one cycle.
struct Directive
{
  std::string robot_id;
  TrafficAction action = TrafficAction::kProceed;
  /// Multiplier applied to the navigation velocity *before* smoothing, so a
  /// yield is executed as a jerk-limited controlled stop.
  double speed_scale = 1.0;
  std::string conflicting_robot;
  double time_to_conflict = 0.0;
  std::string reason;
};

/// \brief Finds predicted space-time overlaps between projected trajectories.
///
/// ### Method
///
/// Trajectories are compared on a shared absolute time grid. For each pair and
/// each sampled instant, the centre distance is measured against
/// `r_a + r_b + margin`. The earliest violating instant is reported.
///
/// Sampling in *time* rather than in space is the whole point: two robots
/// whose paths cross are only in conflict if they are at the crossing
/// together. A purely geometric path intersection test would stop the fleet
/// constantly in a warehouse where every aisle crosses every other one.
///
/// ### Complexity
///
/// O(N^2 * S) for N robots and S samples. At N = 10 and S = 21 that is about
/// 950 distance evaluations per cycle, which at 10 Hz is negligible. The
/// quadratic term only starts to matter well past the fleet sizes this system
/// targets, and the structure (a pure function of the trajectory set) leaves
/// room to drop in spatial hashing without touching any caller.
class ConflictDetector
{
public:
  explicit ConflictDetector(const amr_core::FleetPolicy & policy);

  /// \brief All conflicts predicted at absolute time \p now.
  ///
  /// Paths older than the policy's trajectory timeout are ignored: acting on a
  /// stale prediction is worse than acting on none, because it looks
  /// authoritative.
  std::vector<Conflict> Detect(
    const std::map<std::string, PredictedPath> & paths, double now) const;

  /// \brief Test a single pair. Exposed for unit tests and diagnostics.
  bool DetectPair(
    const PredictedPath & a, const PredictedPath & b, double now,
    Conflict * conflict) const;

private:
  amr_core::FleetPolicy policy_;
};

/// \brief Turns conflicts into directives using a fixed priority order.
///
/// ### The protocol
///
/// Every robot carries a unique integer priority (`FleetConfig` refuses to
/// start otherwise). In a conflict the lower-priority robot yields. With the
/// shipped roster that means the light scout AMR-2 always gives way to the
/// heavy, mission-critical mapper AMR-1 - the rule the brief specifies -
/// but nothing in this class knows about those two robots. Ten robots work the
/// same way.
///
/// Severity depends on urgency: a conflict further out than
/// `hard_yield_seconds` produces a slow-down (`kSlow`), one inside it produces
/// a controlled stop (`kYield`). Slowing early usually dissolves the conflict
/// without anyone stopping, which is worth far more throughput than a
/// stop-and-wait rule.
///
/// ### Starvation
///
/// A strict priority order can starve the lowest-ranked robot forever: in a
/// busy aisle there is always someone more important. After
/// `max_yield_seconds` of continuous yielding the starved robot's effective
/// priority is boosted above the whole fleet for `yield_cooldown_seconds`,
/// which forces the conflict to resolve the other way. The inversion is
/// deliberately time-boxed so it cannot become the new steady state, and it is
/// safe to be aggressive here because the per-robot safety override remains in
/// force underneath and is the actual collision guarantee.
class YieldPolicy
{
public:
  explicit YieldPolicy(const amr_core::FleetConfig & fleet);

  /// \brief Assign one directive per robot in the roster.
  ///
  /// Robots with no conflict receive an explicit `kProceed`. Publishing a
  /// positive "you are clear" every cycle means a robot can treat *silence*
  /// as a fault and fail safe, rather than assuming it may continue.
  std::map<std::string, Directive> Resolve(
    const std::vector<Conflict> & conflicts, double now);

  /// \brief Effective priority, including any active starvation boost.
  int EffectivePriority(const std::string & robot_id, double now) const;

  /// \brief How long \p robot_id has been yielding continuously [s].
  double YieldDuration(const std::string & robot_id, double now) const;

  /// \brief True while \p robot_id holds a starvation-driven priority boost.
  bool HasPriorityBoost(const std::string & robot_id, double now) const;

  void Reset();

private:
  struct RobotState
  {
    int base_priority = 0;
    bool yielding = false;
    double yield_started_at = 0.0;
    double boost_until = 0.0;      ///< Absolute time the boost expires.
  };

  amr_core::FleetConfig fleet_;
  std::map<std::string, RobotState> state_;
  int max_base_priority_ = 0;
};

}  // namespace amr_fleet_control

#endif  // AMR_FLEET_CONTROL__TRAFFIC_MANAGER_HPP_
