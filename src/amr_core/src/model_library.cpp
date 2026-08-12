// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.

#include "amr_core/model_library.hpp"

#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace amr_core
{
namespace
{

/// \brief Fetch a required scalar, reporting *where* it was missing.
///
/// A bare `YAML::BadConversion` tells the operator nothing. Every accessor
/// here carries the model name and key into the message, because the person
/// reading it at launch time has no debugger attached.
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

/// \brief Fetch an optional scalar, leaving \p fallback in place when absent.
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

/// \brief Require a sub-map, e.g. the `lidar:` block.
YAML::Node RequiredMap(
  const YAML::Node & node, const std::string & key, const std::string & context)
{
  if (!node[key] || !node[key].IsMap()) {
    throw ConfigError(context + ": missing required section '" + key + ":'");
  }
  return node[key];
}

LidarSpec ParseLidar(const YAML::Node & node, const std::string & context)
{
  const std::string where = context + ".lidar";
  LidarSpec spec;
  spec.height = Required<double>(node, "height", where);
  spec.samples = Required<int>(node, "samples", where);
  spec.min_angle = Optional<double>(node, "min_angle", spec.min_angle, where);
  spec.max_angle = Optional<double>(node, "max_angle", spec.max_angle, where);
  spec.range_min = Required<double>(node, "range_min", where);
  spec.range_max = Required<double>(node, "range_max", where);
  spec.rate = Required<double>(node, "rate", where);
  spec.noise_stddev = Optional<double>(node, "noise_stddev", spec.noise_stddev, where);

  if (spec.range_min >= spec.range_max) {
    throw ConfigError(where + ": range_min must be below range_max");
  }
  if (spec.samples <= 0) {
    throw ConfigError(where + ": samples must be positive");
  }
  return spec;
}

ImuSpec ParseImu(const YAML::Node & node, const std::string & context)
{
  const std::string where = context + ".imu";
  ImuSpec spec;
  spec.height = Required<double>(node, "height", where);
  spec.rate = Required<double>(node, "rate", where);
  spec.max_angular_velocity = Required<double>(node, "max_angular_velocity", where);
  spec.max_linear_acceleration = Required<double>(node, "max_linear_acceleration", where);
  spec.gyro_noise_stddev =
    Optional<double>(node, "gyro_noise_stddev", spec.gyro_noise_stddev, where);
  spec.accel_noise_stddev =
    Optional<double>(node, "accel_noise_stddev", spec.accel_noise_stddev, where);

  if (spec.max_angular_velocity <= 0.0) {
    throw ConfigError(where + ": max_angular_velocity must be positive");
  }
  return spec;
}

CameraSpec ParseCamera(const YAML::Node & node, const std::string & context)
{
  const std::string where = context + ".camera";
  CameraSpec spec;
  spec.height = Required<double>(node, "height", where);
  spec.width = Required<int>(node, "width", where);
  spec.height_px = Required<int>(node, "height_px", where);
  spec.rate = Required<double>(node, "rate", where);
  spec.hfov = Optional<double>(node, "hfov", spec.hfov, where);
  spec.min_mean_intensity =
    Optional<double>(node, "min_mean_intensity", spec.min_mean_intensity, where);
  spec.max_mean_intensity =
    Optional<double>(node, "max_mean_intensity", spec.max_mean_intensity, where);

  if (spec.min_mean_intensity >= spec.max_mean_intensity) {
    throw ConfigError(where + ": min_mean_intensity must be below max_mean_intensity");
  }
  return spec;
}

RobotProfile ParseProfile(const std::string & name, const YAML::Node & node)
{
  const std::string where = "model '" + name + "'";
  if (!node.IsMap()) {
    throw ConfigError(where + ": expected a mapping");
  }

  RobotProfile profile;
  profile.model_name = name;
  profile.human_name = Optional<std::string>(node, "human_name", name, where);
  profile.role = RoleFromString(Optional<std::string>(node, "role", "", where));

  profile.chassis_length = Required<double>(node, "chassis_length", where);
  profile.chassis_width = Required<double>(node, "chassis_width", where);
  profile.wheel_radius = Required<double>(node, "wheel_radius", where);
  profile.wheel_separation = Required<double>(node, "wheel_separation", where);
  profile.footprint_radius = Required<double>(node, "footprint_radius", where);
  profile.inscribed_radius =
    Optional<double>(node, "inscribed_radius", profile.chassis_width / 2.0, where);

  profile.payload_capacity_kg = Required<double>(node, "payload_capacity_kg", where);

  DynamicLimits & limits = profile.limits;
  limits.max_vel_x = Required<double>(node, "max_vel_x", where);
  limits.min_vel_x = Optional<double>(node, "min_vel_x", -0.4, where);
  limits.max_vel_theta = Required<double>(node, "max_vel_theta", where);
  limits.max_accel_x = Required<double>(node, "max_accel_x", where);
  limits.max_decel_x = Required<double>(node, "max_decel_x", where);
  limits.max_accel_theta = Required<double>(node, "max_accel_theta", where);
  limits.max_jerk_x = Required<double>(node, "max_jerk_x", where);
  limits.max_jerk_theta = Required<double>(node, "max_jerk_theta", where);
  limits.payload_derating = Optional<double>(node, "payload_derating", 0.0, where);
  limits.speed_derating = Optional<double>(node, "speed_derating", 0.0, where);

  SafetySpec & safety = profile.safety;
  safety.k = Required<double>(node, "safety_k", where);
  safety.d_min = Required<double>(node, "safety_d_min", where);
  safety.release_hysteresis =
    Optional<double>(node, "safety_release_hysteresis", safety.release_hysteresis, where);
  safety.sector_half_angle =
    Optional<double>(node, "safety_sector_half_angle", safety.sector_half_angle, where);

  profile.yield_priority = Optional<int>(node, "yield_priority", 0, where);

  profile.lidar = ParseLidar(RequiredMap(node, "lidar", where), where);
  profile.imu = ParseImu(RequiredMap(node, "imu", where), where);
  profile.camera = ParseCamera(RequiredMap(node, "camera", where), where);

  // ---- Cross-field validation --------------------------------------------
  // These are the invariants that make the rest of the stack safe to write.
  // Catching them here converts a class of subtle runtime misbehaviour into a
  // clear message before anything moves.
  if (limits.max_vel_x <= 0.0) {
    throw ConfigError(where + ": max_vel_x must be positive");
  }
  if (limits.max_accel_x <= 0.0 || limits.max_decel_x <= 0.0) {
    throw ConfigError(where + ": acceleration limits must be positive");
  }
  if (limits.max_jerk_x <= 0.0 || limits.max_jerk_theta <= 0.0) {
    throw ConfigError(where + ": jerk limits must be positive");
  }
  if (limits.payload_derating < 0.0 || limits.payload_derating >= 1.0) {
    throw ConfigError(where + ": payload_derating must lie in [0, 1)");
  }
  if (limits.speed_derating < 0.0 || limits.speed_derating >= 1.0) {
    throw ConfigError(where + ": speed_derating must lie in [0, 1)");
  }
  if (profile.payload_capacity_kg <= 0.0) {
    throw ConfigError(where + ": payload_capacity_kg must be positive");
  }
  if (safety.k < 0.0 || safety.d_min <= 0.0) {
    throw ConfigError(where + ": safety_k must be non-negative and safety_d_min positive");
  }
  if (profile.footprint_radius < profile.inscribed_radius) {
    throw ConfigError(where + ": footprint_radius must be at least inscribed_radius");
  }
  // The LiDAR cannot see an obstacle it has already passed: if the safety
  // trigger distance at top speed exceeds the sensor's reach, the halt can
  // never fire in time and the envelope is a fiction.
  const double worst_case_trigger = safety.SafeDistance(limits.max_vel_x);
  if (worst_case_trigger > profile.lidar.range_max) {
    throw ConfigError(
            where + ": safety envelope needs " + std::to_string(worst_case_trigger) +
            " m at top speed but the LiDAR only reaches " +
            std::to_string(profile.lidar.range_max) + " m");
  }

  return profile;
}

}  // namespace

ModelLibrary ModelLibrary::FromFile(const std::string & yaml_path)
{
  ModelLibrary library;
  library.source_path_ = yaml_path;

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw ConfigError("cannot read model library '" + yaml_path + "': " + error.what());
  }
  if (!root.IsMap() || root.size() == 0) {
    throw ConfigError("model library '" + yaml_path + "' is empty or not a mapping");
  }

  for (const auto & entry : root) {
    const std::string name = entry.first.as<std::string>();
    library.profiles_.emplace(name, ParseProfile(name, entry.second));
  }
  return library;
}

const RobotProfile & ModelLibrary::Get(const std::string & model_name) const
{
  const auto it = profiles_.find(model_name);
  if (it == profiles_.end()) {
    std::string known;
    for (const auto & name : Names()) {
      known += known.empty() ? name : ", " + name;
    }
    throw ConfigError(
            "unknown robot model '" + model_name + "'; the library defines: " + known);
  }
  return it->second;
}

bool ModelLibrary::Contains(const std::string & model_name) const
{
  return profiles_.count(model_name) > 0;
}

std::vector<std::string> ModelLibrary::Names() const
{
  std::vector<std::string> names;
  names.reserve(profiles_.size());
  for (const auto & entry : profiles_) {
    names.push_back(entry.first);
  }
  return names;  // std::map already iterates in sorted key order.
}

}  // namespace amr_core
