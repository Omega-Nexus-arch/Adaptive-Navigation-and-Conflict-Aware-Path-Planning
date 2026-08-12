// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.

#include "amr_core/fleet_config.hpp"

#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace amr_core
{
namespace
{

template<typename T>
T Optional(
  const YAML::Node & node, const std::string & key, const T & fallback,
  const std::string & context)
{
  if (!node[key]) {
    return fallback;
  }
  try {
    return node[key].as<T>();
  } catch (const YAML::Exception & error) {
    throw ConfigError(
            context + ": key '" + key + "' has the wrong type (" + error.what() + ")");
  }
}

template<typename T>
T Required(const YAML::Node & node, const std::string & key, const std::string & context)
{
  if (!node[key]) {
    throw ConfigError(context + ": missing required key '" + key + "'");
  }
  try {
    return node[key].as<T>();
  } catch (const YAML::Exception & error) {
    throw ConfigError(
            context + ": key '" + key + "' has the wrong type (" + error.what() + ")");
  }
}

bool FileExists(const std::string & path)
{
  std::ifstream probe(path);
  return probe.good();
}

/// \brief Walk up from \p anchor_file looking for a sibling `<pkg>/<relative>`.
///
/// Lets a `package://` reference resolve straight from a source checkout with
/// nothing built or sourced, which is what keeps the unit tests independent of
/// a ROS installation.
std::string FindInSourceTree(
  const std::string & anchor_file, const std::string & package, const std::string & relative)
{
  std::size_t slash = anchor_file.find_last_of('/');
  if (slash == std::string::npos) {
    return {};
  }
  std::string directory = anchor_file.substr(0, slash);
  while (!directory.empty()) {
    const std::string candidate = directory + "/" + package + "/" + relative;
    if (FileExists(candidate)) {
      return candidate;
    }
    slash = directory.find_last_of('/');
    if (slash == std::string::npos || slash == 0) {
      return {};
    }
    directory = directory.substr(0, slash);
  }
  return {};
}

/// \brief Resolve a `package://<pkg>/<relative>` URI to an absolute path.
///
/// Two lookups, in order:
///
///   1. **The ament index**, by scanning `AMENT_PREFIX_PATH` for
///      `<prefix>/share/<pkg>/<relative>`. This is the install space, and it is
///      what runs in production. The scan is what
///      `ament_index_cpp::get_package_share_directory` does; reimplementing it
///      in ten lines keeps `amr_core` free of ROS dependencies, which is what
///      lets its 27 tests run without a ROS installation.
///   2. **The surrounding source tree**, walking up from \p anchor_file.
///
/// Both matter. Without (1) the config cannot work once installed; without (2)
/// it cannot load from an unbuilt checkout.
///
/// Returns an empty string when neither lookup succeeds, so the caller can
/// report which places were searched rather than a bare "file not found".
std::string ResolvePackageUri(const std::string & uri, const std::string & anchor_file)
{
  static const std::string kScheme = "package://";
  if (uri.compare(0, kScheme.size(), kScheme) != 0) {
    return {};
  }
  const std::string remainder = uri.substr(kScheme.size());
  const std::size_t slash = remainder.find('/');
  if (slash == std::string::npos) {
    return {};
  }
  const std::string package = remainder.substr(0, slash);
  const std::string relative = remainder.substr(slash + 1);

  // 1. Install space.
  const char * prefix_path = std::getenv("AMENT_PREFIX_PATH");
  if (prefix_path != nullptr) {
    std::istringstream stream(prefix_path);
    std::string prefix;
    while (std::getline(stream, prefix, ':')) {
      if (prefix.empty()) {
        continue;
      }
      const std::string candidate = prefix + "/share/" + package + "/" + relative;
      if (FileExists(candidate)) {
        return candidate;
      }
    }
  }

  // 2. Source tree.
  return FindInSourceTree(anchor_file, package, relative);
}

/// \brief Resolve a model-library reference to an absolute path.
///
/// Three accepted forms, in precedence order:
///
///   1. `package://amr_description/config/robot_models.yaml` -- resolved
///      through AMENT_PREFIX_PATH. **This is the form the shipped configs
///      use**, because it is the only one that works in both the source tree
///      and the install space.
///   2. an absolute path -- used verbatim.
///   3. a path relative to the referring file -- convenient for the test
///      fixtures, which sit beside their model library.
///
/// Form 3 used to be what `fleet.yaml` shipped with, and it was wrong: a
/// relative hop out of `install/amr_bringup/share/amr_bringup/config/` lands in
/// `install/amr_bringup/share/`, not in `amr_description`'s install prefix. It
/// worked from the source tree and failed the moment the workspace was
/// installed, which is the worst way for a path bug to behave.
std::string ResolveLibraryPath(const std::string & anchor_file, const std::string & path)
{
  if (path.compare(0, 10, "package://") == 0) {
    const std::string resolved = ResolvePackageUri(path, anchor_file);
    if (resolved.empty()) {
      throw ConfigError(
              "cannot resolve '" + path + "'. Searched AMENT_PREFIX_PATH (is the "
              "package built and the workspace sourced?) and the source tree above '" +
              anchor_file + "'.");
    }
    return resolved;
  }
  if (path.empty() || path.front() == '/') {
    return path;
  }
  const std::size_t slash = anchor_file.find_last_of('/');
  if (slash == std::string::npos) {
    return path;
  }
  return anchor_file.substr(0, slash + 1) + path;
}

FleetPolicy ParsePolicy(const YAML::Node & node)
{
  FleetPolicy policy;
  if (!node) {
    return policy;
  }
  const std::string where = "fleet policy";
  policy.horizon_seconds =
    Optional<double>(node, "horizon_seconds", policy.horizon_seconds, where);
  policy.sample_period =
    Optional<double>(node, "sample_period", policy.sample_period, where);
  policy.conflict_margin =
    Optional<double>(node, "conflict_margin", policy.conflict_margin, where);
  policy.conflict_react_seconds =
    Optional<double>(node, "conflict_react_seconds", policy.conflict_react_seconds, where);
  policy.hard_yield_seconds =
    Optional<double>(node, "hard_yield_seconds", policy.hard_yield_seconds, where);
  policy.slow_speed_scale =
    Optional<double>(node, "slow_speed_scale", policy.slow_speed_scale, where);
  policy.max_yield_seconds =
    Optional<double>(node, "max_yield_seconds", policy.max_yield_seconds, where);
  policy.yield_cooldown_seconds =
    Optional<double>(node, "yield_cooldown_seconds", policy.yield_cooldown_seconds, where);
  policy.traffic_rate_hz =
    Optional<double>(node, "traffic_rate_hz", policy.traffic_rate_hz, where);
  policy.trajectory_publish_rate_hz = Optional<double>(
    node, "trajectory_publish_rate_hz", policy.trajectory_publish_rate_hz, where);
  policy.trajectory_timeout_seconds = Optional<double>(
    node, "trajectory_timeout_seconds", policy.trajectory_timeout_seconds, where);

  if (policy.sample_period <= 0.0) {
    throw ConfigError(where + ": sample_period must be positive");
  }
  if (policy.horizon_seconds < policy.sample_period) {
    throw ConfigError(where + ": horizon_seconds must be at least one sample_period");
  }
  if (policy.slow_speed_scale <= 0.0 || policy.slow_speed_scale > 1.0) {
    throw ConfigError(where + ": slow_speed_scale must lie in (0, 1]");
  }
  if (policy.hard_yield_seconds > policy.conflict_react_seconds) {
    throw ConfigError(
            where + ": hard_yield_seconds must not exceed conflict_react_seconds, "
            "otherwise a conflict escalates to a stop before it is even acted on");
  }
  return policy;
}

}  // namespace

FleetConfig FleetConfig::FromFile(const std::string & yaml_path, const ModelLibrary & library)
{
  FleetConfig config;
  config.source_path_ = yaml_path;

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw ConfigError("cannot read fleet config '" + yaml_path + "': " + error.what());
  }

  // Tolerate the ROS 2 parameter-file wrapper so the same file can be fed to a
  // node as `--params-file` and parsed directly here.
  YAML::Node fleet = root["fleet"] ? root["fleet"] : root;
  if (fleet["ros__parameters"]) {
    fleet = fleet["ros__parameters"];
  }

  config.global_frame_ = Optional<std::string>(fleet, "global_frame", "map", "fleet");
  config.policy_ = ParsePolicy(fleet["policy"]);

  const YAML::Node robots = fleet["robots"];
  if (!robots || !robots.IsSequence() || robots.size() == 0) {
    throw ConfigError(
            "fleet config '" + yaml_path + "': needs a non-empty `robots:` sequence");
  }

  std::set<std::string> seen;
  for (std::size_t i = 0; i < robots.size(); ++i) {
    const YAML::Node & entry = robots[i];
    const std::string where = "fleet.robots[" + std::to_string(i) + "]";
    if (!entry.IsMap()) {
      throw ConfigError(where + ": expected a mapping");
    }

    RobotInstance instance;
    instance.name = Required<std::string>(entry, "name", where);
    if (!seen.insert(instance.name).second) {
      throw ConfigError(where + ": duplicate robot name '" + instance.name + "'");
    }
    if (instance.name.find('/') != std::string::npos) {
      throw ConfigError(
              where + ": robot name '" + instance.name +
              "' must not contain '/'; it is used verbatim as a ROS namespace");
    }

    instance.model_name = Required<std::string>(entry, "model", where);
    if (!library.Contains(instance.model_name)) {
      throw ConfigError(
              where + ": model '" + instance.model_name + "' is not defined in " +
              library.SourcePath());
    }
    instance.profile = library.Get(instance.model_name);

    instance.initial_x = Optional<double>(entry, "x", 0.0, where);
    instance.initial_y = Optional<double>(entry, "y", 0.0, where);
    instance.initial_yaw = Optional<double>(entry, "yaw", 0.0, where);
    instance.priority_override = Optional<int>(entry, "yield_priority", -1, where);

    config.index_by_name_.emplace(instance.name, config.robots_.size());
    config.robots_.push_back(instance);
  }

  // Equal priorities would make the yield decision depend on message arrival
  // order, which is exactly the kind of non-determinism that turns into an
  // intermittent deadlock in a real warehouse. Force the operator to decide.
  std::set<int> priorities;
  for (const auto & robot : config.robots_) {
    if (!priorities.insert(robot.YieldPriority()).second) {
      throw ConfigError(
              "two robots share yield_priority " + std::to_string(robot.YieldPriority()) +
              " (see '" + robot.name + "'); priorities must be unique so the yielding "
              "protocol is deterministic");
    }
  }

  return config;
}

FleetConfig FleetConfig::FromFile(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw ConfigError("cannot read fleet config '" + yaml_path + "': " + error.what());
  }

  YAML::Node fleet = root["fleet"] ? root["fleet"] : root;
  if (fleet["ros__parameters"]) {
    fleet = fleet["ros__parameters"];
  }
  if (!fleet["model_library"]) {
    throw ConfigError(
            "fleet config '" + yaml_path + "': needs a `model_library:` path, or call the "
            "two-argument FromFile overload");
  }

  const std::string library_path =
    ResolveLibraryPath(yaml_path, fleet["model_library"].as<std::string>());
  return FromFile(yaml_path, ModelLibrary::FromFile(library_path));
}

const RobotInstance & FleetConfig::Robot(const std::string & name) const
{
  const auto it = index_by_name_.find(name);
  if (it == index_by_name_.end()) {
    std::string known;
    for (const auto & robot : robots_) {
      known += known.empty() ? robot.name : ", " + robot.name;
    }
    throw ConfigError("robot '" + name + "' is not in the fleet roster: " + known);
  }
  return robots_[it->second];
}

bool FleetConfig::Contains(const std::string & name) const
{
  return index_by_name_.count(name) > 0;
}

std::vector<std::string> FleetConfig::Names() const
{
  std::vector<std::string> names;
  names.reserve(robots_.size());
  for (const auto & robot : robots_) {
    names.push_back(robot.name);
  }
  return names;
}

}  // namespace amr_core
