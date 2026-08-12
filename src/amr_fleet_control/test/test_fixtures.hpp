// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Shared fixtures. The profiles here mirror
// amr_description/config/robot_models.yaml; keeping a copy in the tests means
// a production tuning change cannot silently invalidate a test's premise, and
// test_config_loading.cpp separately asserts that the real file still says
// what these fixtures assume.

#ifndef TEST_FIXTURES_HPP_
#define TEST_FIXTURES_HPP_

#include <string>
#include <vector>

#include "amr_core/fleet_config.hpp"
#include "amr_core/robot_model.hpp"

namespace amr_fleet_control
{
namespace test
{

inline amr_core::RobotProfile HeavyMapperProfile()
{
  amr_core::RobotProfile profile;
  profile.model_name = "heavy_mapper";
  profile.role = amr_core::RobotRole::kMapper;
  profile.chassis_length = 0.90;
  profile.chassis_width = 0.62;
  profile.footprint_radius = 0.55;
  profile.inscribed_radius = 0.31;
  profile.payload_capacity_kg = 120.0;

  profile.limits.max_vel_x = 0.75;
  profile.limits.min_vel_x = -0.35;
  profile.limits.max_vel_theta = 0.90;
  profile.limits.max_accel_x = 0.35;
  profile.limits.max_decel_x = 0.70;
  profile.limits.max_accel_theta = 0.80;
  profile.limits.max_jerk_x = 0.60;
  profile.limits.max_jerk_theta = 1.20;
  profile.limits.payload_derating = 0.55;
  profile.limits.speed_derating = 0.30;

  profile.safety.k = 0.85;
  profile.safety.d_min = 0.45;
  profile.safety.release_hysteresis = 0.15;
  profile.safety.sector_half_angle = 1.2217;

  profile.lidar.range_max = 25.0;
  profile.imu.max_angular_velocity = 2.50;
  profile.yield_priority = 100;
  return profile;
}

inline amr_core::RobotProfile LightScoutProfile()
{
  amr_core::RobotProfile profile;
  profile.model_name = "light_scout";
  profile.role = amr_core::RobotRole::kScout;
  profile.chassis_length = 0.58;
  profile.chassis_width = 0.44;
  profile.footprint_radius = 0.37;
  profile.inscribed_radius = 0.22;
  profile.payload_capacity_kg = 30.0;

  profile.limits.max_vel_x = 1.40;
  profile.limits.min_vel_x = -0.50;
  profile.limits.max_vel_theta = 1.90;
  profile.limits.max_accel_x = 1.10;
  profile.limits.max_decel_x = 1.60;
  profile.limits.max_accel_theta = 2.40;
  profile.limits.max_jerk_x = 2.50;
  profile.limits.max_jerk_theta = 4.00;
  profile.limits.payload_derating = 0.30;
  profile.limits.speed_derating = 0.20;

  profile.safety.k = 0.42;
  profile.safety.d_min = 0.30;
  profile.safety.release_hysteresis = 0.12;
  profile.safety.sector_half_angle = 1.3963;

  profile.lidar.range_max = 16.0;
  profile.imu.max_angular_velocity = 4.50;
  profile.yield_priority = 50;
  return profile;
}

/// \brief The shipped two-robot fleet: AMR-1 heavy lead, AMR-2 light scout.
inline amr_core::FleetConfig TwoRobotFleet(const amr_core::FleetPolicy & policy = {})
{
  amr_core::RobotInstance amr1;
  amr1.name = "amr1";
  amr1.model_name = "heavy_mapper";
  amr1.profile = HeavyMapperProfile();

  amr_core::RobotInstance amr2;
  amr2.name = "amr2";
  amr2.model_name = "light_scout";
  amr2.profile = LightScoutProfile();

  return amr_core::FleetConfig::FromRoster({amr1, amr2}, policy);
}

/// \brief An N-robot fleet used to show the policy is not hard-wired to two.
inline amr_core::FleetConfig LargeFleet(
  std::size_t count, const amr_core::FleetPolicy & policy = {})
{
  std::vector<amr_core::RobotInstance> roster;
  for (std::size_t i = 0; i < count; ++i) {
    amr_core::RobotInstance robot;
    robot.name = "amr" + std::to_string(i + 1);
    const bool heavy = (i % 3 == 0);
    robot.model_name = heavy ? "heavy_mapper" : "light_scout";
    robot.profile = heavy ? HeavyMapperProfile() : LightScoutProfile();
    robot.priority_override = static_cast<int>(count - i) * 10;
    roster.push_back(robot);
  }
  return amr_core::FleetConfig::FromRoster(roster, policy);
}

}  // namespace test
}  // namespace amr_fleet_control

#endif  // TEST_FIXTURES_HPP_
