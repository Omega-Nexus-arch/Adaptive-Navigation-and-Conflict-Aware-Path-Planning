// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Configuration loading is the part of the stack most likely to be edited by
// someone who is not reading the code -- an operator adding a robot, a
// technician tuning a limit. These tests pin down the contract that protects
// them: every malformed input produces a specific, actionable exception at
// startup, and never a silently defaulted value that misbehaves later.

#include <gtest/gtest.h>

#include <string>

#include "amr_core/fleet_config.hpp"
#include "amr_core/model_library.hpp"

namespace
{

using amr_core::ConfigError;
using amr_core::FleetConfig;
using amr_core::ModelLibrary;
using amr_core::RobotRole;

std::string DataPath(const std::string & file)
{
  return std::string(AMR_CORE_TEST_DATA_DIR) + "/" + file;
}

// ---------------------------------------------------------------------------
// Model library
// ---------------------------------------------------------------------------

TEST(ModelLibraryTest, LoadsEveryModelAndItsFields) {
  const ModelLibrary library = ModelLibrary::FromFile(DataPath("models_good.yaml"));
  EXPECT_EQ(library.Size(), 2u);
  EXPECT_TRUE(library.Contains("heavy_test"));
  EXPECT_TRUE(library.Contains("light_test"));

  const auto & heavy = library.Get("heavy_test");
  EXPECT_EQ(heavy.model_name, "heavy_test");
  EXPECT_EQ(heavy.role, RobotRole::kMapper);
  EXPECT_NEAR(heavy.payload_capacity_kg, 120.0, 1e-9);
  EXPECT_NEAR(heavy.limits.max_accel_x, 0.35, 1e-9);
  EXPECT_NEAR(heavy.safety.k, 0.85, 1e-9);
  EXPECT_NEAR(heavy.imu.max_angular_velocity, 2.50, 1e-9);
  EXPECT_EQ(heavy.yield_priority, 100);

  const auto & light = library.Get("light_test");
  EXPECT_EQ(light.role, RobotRole::kScout);
  EXPECT_NEAR(light.limits.max_accel_x, 1.10, 1e-9);
}

TEST(ModelLibraryTest, TheHeavyModelIsTheLessAgileOne) {
  // Guards the assignment's explicit requirement against an accidental swap of
  // two numbers in the YAML.
  const ModelLibrary library = ModelLibrary::FromFile(DataPath("models_good.yaml"));
  const auto & heavy = library.Get("heavy_test");
  const auto & light = library.Get("light_test");

  EXPECT_LT(heavy.limits.max_accel_x, light.limits.max_accel_x);
  EXPECT_LT(heavy.limits.max_jerk_x, light.limits.max_jerk_x);
  EXPECT_LT(heavy.limits.max_vel_x, light.limits.max_vel_x);
  EXPECT_GT(heavy.payload_capacity_kg, light.payload_capacity_kg);
  EXPECT_GT(heavy.yield_priority, light.yield_priority)
    << "the mission-critical heavy unit must outrank the scout";
}

TEST(ModelLibraryTest, OptionalKeysFallBackWithoutThrowing) {
  const ModelLibrary library = ModelLibrary::FromFile(DataPath("models_good.yaml"));
  const auto & light = library.Get("light_test");
  // light_test omits safety_release_hysteresis and safety_sector_half_angle.
  EXPECT_GT(light.safety.release_hysteresis, 0.0);
  EXPECT_GT(light.safety.sector_half_angle, 0.0);
}

TEST(ModelLibraryTest, MissingRequiredKeyIsFatalAndNamesTheKey) {
  try {
    ModelLibrary::FromFile(DataPath("models_missing_key.yaml"));
    ADD_FAILURE() << "expected a ConfigError";
  } catch (const ConfigError & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("max_jerk_x"), std::string::npos)
      << "the message must name the offending key, got: " << message;
    EXPECT_NE(message.find("broken"), std::string::npos)
      << "the message must name the offending model, got: " << message;
  }
}

TEST(ModelLibraryTest, RejectsASafetyEnvelopeTheLidarCannotSee) {
  // If d_safe at top speed exceeds the sensor range, the halt can never fire
  // in time. Silently accepting that would ship a safety system that only
  // looks like one.
  EXPECT_THROW(
    ModelLibrary::FromFile(DataPath("models_unreachable_safety.yaml")), ConfigError);
}

TEST(ModelLibraryTest, UnknownModelLookupNamesTheAlternatives) {
  const ModelLibrary library = ModelLibrary::FromFile(DataPath("models_good.yaml"));
  try {
    library.Get("nope");
    ADD_FAILURE() << "expected a ConfigError";
  } catch (const ConfigError & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("heavy_test"), std::string::npos)
      << "an unknown-model error should list what is available, got: " << message;
  }
}

TEST(ModelLibraryTest, MissingFileIsFatal) {
  EXPECT_THROW(ModelLibrary::FromFile(DataPath("no_such_file.yaml")), ConfigError);
}

// ---------------------------------------------------------------------------
// Fleet roster
// ---------------------------------------------------------------------------

TEST(FleetConfigTest, LoadsRosterAndResolvesProfiles) {
  const FleetConfig fleet = FleetConfig::FromFile(DataPath("fleet_good.yaml"));
  EXPECT_EQ(fleet.Size(), 2u);
  EXPECT_EQ(fleet.GlobalFrame(), "map");

  const auto & amr1 = fleet.Robot("amr1");
  EXPECT_EQ(amr1.model_name, "heavy_test");
  EXPECT_NEAR(amr1.initial_x, -18.5, 1e-9);
  EXPECT_NEAR(amr1.initial_y, 2.2, 1e-9);
  // The instance carries a resolved copy of the model, so downstream code
  // never has to hold both objects.
  EXPECT_NEAR(amr1.profile.limits.max_accel_x, 0.35, 1e-9);
  EXPECT_EQ(amr1.YieldPriority(), 100);

  EXPECT_EQ(fleet.Robot("amr2").YieldPriority(), 50);
  EXPECT_GT(amr1.YieldPriority(), fleet.Robot("amr2").YieldPriority());
}

TEST(FleetConfigTest, ResolvesTheModelLibraryPathRelativeToItself) {
  // fleet_good.yaml refers to "models_good.yaml" with no directory component.
  // Loading must work regardless of the process's working directory.
  EXPECT_NO_THROW(FleetConfig::FromFile(DataPath("fleet_good.yaml")));
}

TEST(FleetConfigTest, PolicyDefaultsApplyWhenTheSectionIsAbsent) {
  const FleetConfig fleet = FleetConfig::FromFile(DataPath("fleet_ten_robots.yaml"));
  const auto & policy = fleet.Policy();
  EXPECT_GT(policy.horizon_seconds, 0.0);
  EXPECT_GT(policy.sample_period, 0.0);
  EXPECT_LE(policy.hard_yield_seconds, policy.conflict_react_seconds);
}

TEST(FleetConfigTest, ScalesToTenRobotsWithNoCodeChange) {
  // The scalability requirement, expressed as a test rather than a claim.
  const FleetConfig fleet = FleetConfig::FromFile(DataPath("fleet_ten_robots.yaml"));
  EXPECT_EQ(fleet.Size(), 10u);
  EXPECT_EQ(fleet.Names().size(), 10u);

  for (const auto & robot : fleet.Robots()) {
    EXPECT_FALSE(robot.name.empty());
    EXPECT_GT(robot.profile.limits.max_vel_x, 0.0)
      << robot.name << " did not resolve a profile";
  }
  EXPECT_TRUE(fleet.Contains("amr10"));
  EXPECT_FALSE(fleet.Contains("amr11"));
}

TEST(FleetConfigTest, RejectsDuplicatePriorities) {
  // Equal priorities make the yielding protocol depend on message ordering,
  // which is how an intermittent deadlock gets shipped.
  EXPECT_THROW(
    FleetConfig::FromFile(DataPath("fleet_duplicate_priority.yaml")), ConfigError);
}

TEST(FleetConfigTest, RejectsDuplicateRobotNames) {
  // Two robots in one namespace would silently cross-wire their cmd_vel.
  EXPECT_THROW(FleetConfig::FromFile(DataPath("fleet_duplicate_name.yaml")), ConfigError);
}

TEST(FleetConfigTest, RejectsAnUnknownModelAndNamesTheLibrary) {
  try {
    FleetConfig::FromFile(DataPath("fleet_unknown_model.yaml"));
    ADD_FAILURE() << "expected a ConfigError";
  } catch (const ConfigError & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("does_not_exist"), std::string::npos) << message;
    EXPECT_NE(message.find("models_good.yaml"), std::string::npos)
      << "the error should point at the library that was searched: " << message;
  }
}

TEST(FleetConfigTest, UnknownRobotLookupListsTheRoster) {
  const FleetConfig fleet = FleetConfig::FromFile(DataPath("fleet_good.yaml"));
  try {
    fleet.Robot("ghost");
    ADD_FAILURE() << "expected a ConfigError";
  } catch (const ConfigError & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("amr1"), std::string::npos) << message;
  }
}

TEST(FleetConfigTest, PrioritiesAreTotallyOrdered) {
  // Every pair must be comparable, which is what lets YieldPolicy resolve any
  // conflict without a tie-break rule.
  const FleetConfig fleet = FleetConfig::FromFile(DataPath("fleet_ten_robots.yaml"));
  const auto & robots = fleet.Robots();
  for (std::size_t i = 0; i < robots.size(); ++i) {
    for (std::size_t j = i + 1; j < robots.size(); ++j) {
      EXPECT_NE(robots[i].YieldPriority(), robots[j].YieldPriority())
        << robots[i].name << " and " << robots[j].name << " tie";
    }
  }
}

}  // namespace
