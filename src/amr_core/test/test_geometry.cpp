// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <cmath>

#include "amr_core/geometry.hpp"

namespace
{

using amr_core::AngleDifference;
using amr_core::IntegrateUnicycle;
using amr_core::NormalizeAngle;
using amr_core::PitchFromQuaternion;
using amr_core::Pose2D;
using amr_core::RelativeBearing;
using amr_core::Velocity2D;
using amr_core::YawFromQuaternion;

constexpr double kEps = 1e-9;

TEST(NormalizeAngleTest, MapsIntoHalfOpenInterval) {
  EXPECT_NEAR(NormalizeAngle(0.0), 0.0, kEps);
  EXPECT_NEAR(NormalizeAngle(M_PI), M_PI, kEps);
  EXPECT_NEAR(NormalizeAngle(-M_PI), M_PI, kEps) << "-pi and +pi are the same heading";
  EXPECT_NEAR(NormalizeAngle(3.0 * M_PI), M_PI, kEps);
  EXPECT_NEAR(NormalizeAngle(1.5 * M_PI), -0.5 * M_PI, kEps);
  EXPECT_NEAR(NormalizeAngle(-1.5 * M_PI), 0.5 * M_PI, kEps);
}

TEST(NormalizeAngleTest, HandlesLargeWindings) {
  for (int turns = -5; turns <= 5; ++turns) {
    EXPECT_NEAR(NormalizeAngle(0.3 + turns * 2.0 * M_PI), 0.3, 1e-9) << "turns=" << turns;
  }
}

TEST(AngleDifferenceTest, TakesTheShortWayRound) {
  // The classic wrap bug: 170 deg to -170 deg is a 20 deg turn, not 340.
  EXPECT_NEAR(AngleDifference(2.967, -2.967), 0.349, 1e-3);
  EXPECT_NEAR(AngleDifference(0.0, 0.5), 0.5, kEps);
  EXPECT_NEAR(AngleDifference(0.5, 0.0), -0.5, kEps);
}

TEST(QuaternionTest, YawRoundTrips) {
  for (double yaw = -3.0; yaw <= 3.0; yaw += 0.25) {
    const double qz = std::sin(yaw / 2.0);
    const double qw = std::cos(yaw / 2.0);
    EXPECT_NEAR(YawFromQuaternion(0.0, 0.0, qz, qw), yaw, 1e-9) << "yaw=" << yaw;
  }
}

TEST(QuaternionTest, PitchRoundTripsAndSaturatesAtGimbalLock) {
  for (double pitch = -1.2; pitch <= 1.2; pitch += 0.2) {
    const double qy = std::sin(pitch / 2.0);
    const double qw = std::cos(pitch / 2.0);
    EXPECT_NEAR(PitchFromQuaternion(0.0, qy, 0.0, qw), pitch, 1e-9) << "pitch=" << pitch;
  }
  // Straight up: asin() must not be handed an argument above 1 by rounding.
  EXPECT_NEAR(PitchFromQuaternion(0.0, std::sin(M_PI / 4.0), 0.0, std::cos(M_PI / 4.0)),
    M_PI / 2.0, 1e-9);
}

TEST(IntegrateUnicycleTest, StraightLineMotion) {
  const Pose2D start{1.0, 2.0, 0.0};
  const Velocity2D velocity{0.5, 0.0};
  const Pose2D after = IntegrateUnicycle(start, velocity, 2.0);
  EXPECT_NEAR(after.x, 2.0, kEps);
  EXPECT_NEAR(after.y, 2.0, kEps);
  EXPECT_NEAR(after.theta, 0.0, kEps);
}

TEST(IntegrateUnicycleTest, PureRotationDoesNotTranslate) {
  const Pose2D start{1.0, 2.0, 0.0};
  const Velocity2D velocity{0.0, 1.0};
  const Pose2D after = IntegrateUnicycle(start, velocity, 1.0);
  EXPECT_NEAR(after.x, 1.0, kEps);
  EXPECT_NEAR(after.y, 2.0, kEps);
  EXPECT_NEAR(after.theta, 1.0, kEps);
}

TEST(IntegrateUnicycleTest, ArcSolutionBeatsEulerOnATightTurn) {
  // Quarter circle of radius 1 m: exact endpoint is (1, 1) from the origin
  // heading +x. The Euler approximation lands measurably short, which is why
  // the exact form is used for trajectory prediction.
  const Pose2D start{0.0, 0.0, 0.0};
  const Velocity2D velocity{1.0, 1.0};
  const double dt = M_PI / 2.0;

  const Pose2D exact = IntegrateUnicycle(start, velocity, dt);
  EXPECT_NEAR(exact.x, 1.0, 1e-9);
  EXPECT_NEAR(exact.y, 1.0, 1e-9);
  EXPECT_NEAR(exact.theta, M_PI / 2.0, 1e-9);

  const double euler_x = start.x + velocity.linear_x * std::cos(start.theta) * dt;
  EXPECT_GT(std::abs(euler_x - 1.0), 0.5) << "sanity: Euler really is that wrong here";
}

TEST(IntegrateUnicycleTest, FullCircleReturnsToStart) {
  // Integrating one exact revolution must close the loop to numerical
  // precision. Any per-step bias accumulates here rather than cancelling.
  Pose2D pose{0.0, 0.0, 0.0};
  const Velocity2D velocity{0.8, 0.4};                  // radius 2 m
  const int steps = 1000;
  const double dt = (2.0 * M_PI / velocity.angular_z) / steps;
  for (int i = 0; i < steps; ++i) {
    pose = IntegrateUnicycle(pose, velocity, dt);
  }
  EXPECT_NEAR(pose.x, 0.0, 1e-9);
  EXPECT_NEAR(pose.y, 0.0, 1e-9);
  EXPECT_NEAR(std::abs(pose.theta), 0.0, 1e-9);
}

TEST(IntegrateUnicycleTest, StepSizeDoesNotChangeTheAnswer) {
  // The arc solution is exact for constant velocity, so subdividing the
  // interval must land in precisely the same place. A predictor whose result
  // depends on the tick rate cannot be compared across robots running at
  // different rates, which is exactly what the conflict detector does.
  const Pose2D start{1.0, -2.0, 0.7};
  const Velocity2D velocity{0.6, 0.35};
  const double total = 1.5;

  const Pose2D one_shot = IntegrateUnicycle(start, velocity, total);

  Pose2D stepped = start;
  const int steps = 150;
  for (int i = 0; i < steps; ++i) {
    stepped = IntegrateUnicycle(stepped, velocity, total / steps);
  }
  EXPECT_NEAR(stepped.x, one_shot.x, 1e-9);
  EXPECT_NEAR(stepped.y, one_shot.y, 1e-9);
  EXPECT_NEAR(AngleDifference(stepped.theta, one_shot.theta), 0.0, 1e-9);
}

TEST(IntegrateUnicycleTest, IsContinuousAcrossTheStraightLineThreshold) {
  // The implementation switches formulas at |omega| = 1e-6. A discontinuity
  // there would inject a jump into every predicted trajectory of a robot
  // driving almost straight.
  const Pose2D start{0.0, 0.0, 0.3};
  const double dt = 0.1;
  const Pose2D below = IntegrateUnicycle(start, Velocity2D{1.0, 9e-7}, dt);
  const Pose2D above = IntegrateUnicycle(start, Velocity2D{1.0, 1.1e-6}, dt);
  EXPECT_NEAR(below.x, above.x, 1e-6);
  EXPECT_NEAR(below.y, above.y, 1e-6);
}

TEST(RelativeBearingTest, ReportsAngleInTheObserverFrame) {
  const Pose2D observer{0.0, 0.0, 0.0};
  EXPECT_NEAR(RelativeBearing(observer, 1.0, 0.0), 0.0, kEps);
  EXPECT_NEAR(RelativeBearing(observer, 0.0, 1.0), M_PI / 2.0, kEps);
  EXPECT_NEAR(RelativeBearing(observer, -1.0, 0.0), M_PI, kEps);

  const Pose2D turned{0.0, 0.0, M_PI / 2.0};
  EXPECT_NEAR(RelativeBearing(turned, 0.0, 1.0), 0.0, kEps)
    << "dead ahead of a robot facing +y is +y";
}

}  // namespace
