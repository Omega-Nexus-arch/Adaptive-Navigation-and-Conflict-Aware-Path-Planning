// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_CORE__FLEET_CONFIG_HPP_
#define AMR_CORE__FLEET_CONFIG_HPP_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "amr_core/model_library.hpp"
#include "amr_core/robot_model.hpp"

namespace amr_core
{

/// \brief Fleet-wide policy knobs that are not properties of any one robot.
struct FleetPolicy
{
  // -- Conflict detection --------------------------------------------------
  /// How far ahead trajectories are projected and compared [s].
  double horizon_seconds = 4.0;
  /// Sample spacing along a projected trajectory [s].
  double sample_period = 0.2;
  /// Extra clearance added to the sum of two footprint radii before a
  /// space-time overlap counts as a conflict [m].
  double conflict_margin = 0.35;
  /// A conflict further away than this in time is ignored: predictions that
  /// far out are not trustworthy enough to act on.
  double conflict_react_seconds = 3.0;
  /// Below this time-to-conflict the yield becomes a full stop rather than a
  /// slow-down [s].
  double hard_yield_seconds = 1.5;
  /// Speed multiplier applied for a soft (SLOW) directive.
  double slow_speed_scale = 0.35;

  // -- Deadlock handling ---------------------------------------------------
  /// A robot held longer than this is force-released to break a livelock [s].
  double max_yield_seconds = 12.0;
  /// After a forced release, the same robot cannot be told to yield again for
  /// this long [s].
  double yield_cooldown_seconds = 4.0;

  // -- Timing --------------------------------------------------------------
  double traffic_rate_hz = 10.0;
  double trajectory_publish_rate_hz = 10.0;
  /// A peer trajectory older than this is discarded; a stale prediction is
  /// worse than no prediction.
  double trajectory_timeout_seconds = 1.0;
};

/// \brief The fleet roster: which robots exist, what model each one is, and
///        the shared policy they operate under.
///
/// Growing the fleet to ten or more robots is an edit to `robots:` in
/// `amr_bringup/config/fleet.yaml`. No node, launch file or message definition
/// changes, because every consumer iterates `Robots()` rather than naming
/// `amr1` and `amr2`.
class FleetConfig
{
public:
  FleetConfig() = default;

  /// \brief Load `fleet.yaml`, resolving each roster entry against \p library.
  ///
  /// \throws ConfigError on a missing file, a duplicate robot name, or a model
  ///         key that the library does not define.
  static FleetConfig FromFile(const std::string & yaml_path, const ModelLibrary & library);

  /// \brief Convenience overload: read the model library path from the fleet
  ///        file's own `model_library:` key, resolving it relative to the
  ///        fleet file when it is not absolute.
  static FleetConfig FromFile(const std::string & yaml_path);

  /// \brief Build a roster in memory, bypassing YAML entirely.
  ///
  /// Two callers need this: unit tests, which should not have to author a file
  /// to exercise the traffic policy, and any future node that receives its
  /// roster over the parameter server rather than from disk. Defined inline so
  /// that neither picks up a dependency on yaml-cpp.
  ///
  /// \throws ConfigError on duplicate names or duplicate priorities, applying
  ///         exactly the same invariants as the file loader.
  static FleetConfig FromRoster(
    std::vector<RobotInstance> robots, const FleetPolicy & policy = FleetPolicy(),
    const std::string & global_frame = "map")
  {
    FleetConfig config;
    config.policy_ = policy;
    config.global_frame_ = global_frame;
    config.source_path_ = "<in-memory roster>";

    for (auto & robot : robots) {
      if (config.index_by_name_.count(robot.name) > 0) {
        throw ConfigError("duplicate robot name '" + robot.name + "' in roster");
      }
      config.index_by_name_.emplace(robot.name, config.robots_.size());
      config.robots_.push_back(std::move(robot));
    }

    for (std::size_t i = 0; i < config.robots_.size(); ++i) {
      for (std::size_t j = i + 1; j < config.robots_.size(); ++j) {
        if (config.robots_[i].YieldPriority() == config.robots_[j].YieldPriority()) {
          throw ConfigError(
                  "robots '" + config.robots_[i].name + "' and '" + config.robots_[j].name +
                  "' share a yield priority; the yielding protocol would be "
                  "non-deterministic");
        }
      }
    }
    return config;
  }

  const std::vector<RobotInstance> & Robots() const {return robots_;}

  /// \throws ConfigError if \p name is not in the roster.
  const RobotInstance & Robot(const std::string & name) const;

  bool Contains(const std::string & name) const;

  std::size_t Size() const {return robots_.size();}

  const FleetPolicy & Policy() const {return policy_;}

  /// \brief Robot names in roster order.
  std::vector<std::string> Names() const;

  /// \brief The frame every robot's plans and trajectories are expressed in.
  const std::string & GlobalFrame() const {return global_frame_;}

  const std::string & SourcePath() const {return source_path_;}

private:
  std::vector<RobotInstance> robots_;
  std::map<std::string, std::size_t> index_by_name_;
  FleetPolicy policy_;
  std::string global_frame_ = "map";
  std::string source_path_;
};

}  // namespace amr_core

#endif  // AMR_CORE__FLEET_CONFIG_HPP_
