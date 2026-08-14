#!/usr/bin/env python3
# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Drive the warehouse's dynamic obstacles along randomised patrol loops.

Gazebo Classic ``<actor>`` elements carry no collision geometry and are
therefore invisible to a LiDAR, which would make any "the robot avoided the
pedestrian" demonstration a fiction. The obstacles here are ordinary models
with collision shapes, steered through ``libgazebo_ros_planar_move``. They
show up in the scan, in the costmap and in the safety monitor exactly as a real
intruder would.

Randomness is seeded, so a run can be reproduced when something interesting
happens. Each obstacle independently:

* follows a closed waypoint loop with a lookahead tolerance,
* jitters its cruise speed by +/- ``speed_jitter``,
* occasionally stops dead for a few seconds (``dwell``), which is the case that
  actually stresses a predictive local planner,
* occasionally cuts a corner by skipping a waypoint.

Style: PEP 8, checked by ament_flake8.
"""

import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def _sensor_qos(depth=10):
    """Best-effort QoS matching what the Gazebo plugins publish with."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=depth,
    )


class ObstacleAgent:
    """Waypoint-following state machine for a single dynamic obstacle.

    Deliberately holds no ROS handles beyond the publisher it is given: the
    control law is a pure function of (pose, clock) and can be unit-tested.
    """

    def __init__(self, name, waypoints, speed, rng, params):
        self.name = name
        self.waypoints = list(waypoints)
        self.base_speed = speed
        self.rng = rng
        self.params = params

        self.index = 0
        self.pose = None            # (x, y, yaw)
        self.dwell_until = 0.0
        self.speed = speed

    # -- state updates -------------------------------------------------------

    def update_pose(self, msg):
        orientation = msg.pose.pose.orientation
        siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            math.atan2(siny, cosy),
        )

    def _advance_waypoint(self, now):
        """Move to the next target, sometimes pausing or skipping ahead."""
        step = 1
        if self.rng.random() < self.params['skip_probability']:
            step = 2
        self.index = (self.index + step) % len(self.waypoints)

        if self.rng.random() < self.params['dwell_probability']:
            self.dwell_until = now + self.rng.uniform(*self.params['dwell_seconds'])

        jitter = self.params['speed_jitter']
        self.speed = self.base_speed * self.rng.uniform(1.0 - jitter, 1.0 + jitter)

    # -- control law ---------------------------------------------------------

    def compute_command(self, now):
        """Return the Twist to publish this tick."""
        cmd = Twist()
        if self.pose is None:
            return cmd
        if now < self.dwell_until:
            return cmd

        x, y, yaw = self.pose
        target_x, target_y = self.waypoints[self.index]
        dx, dy = target_x - x, target_y - y
        distance = math.hypot(dx, dy)

        if distance < self.params['arrival_tolerance']:
            self._advance_waypoint(now)
            target_x, target_y = self.waypoints[self.index]
            dx, dy = target_x - x, target_y - y
            distance = math.hypot(dx, dy)
            if distance < 1e-6:
                return cmd

        # Ease in over the last half metre so the obstacle does not teleport
        # through its waypoint at full speed.
        speed = min(self.speed, self.speed * max(0.25, distance / 0.5))
        # planar_move interprets linear.x/linear.y in the world frame for a
        # holonomic body, so translation is independent of which way the model
        # is pointing.
        cmd.linear.x = speed * dx / distance
        cmd.linear.y = speed * dy / distance

        # Turn to face the direction of travel. Irrelevant to the physics --
        # the body is a cylinder and the motion is holonomic -- but a walking
        # person sliding along sideways looks wrong, and this is the only thing
        # that makes the human mesh read as walking rather than gliding. A
        # proportional term on the heading error, wrapped to (-pi, pi] so the
        # person turns the short way round.
        heading_error = math.atan2(math.sin(math.atan2(dy, dx) - yaw),
                                   math.cos(math.atan2(dy, dx) - yaw))
        cmd.angular.z = max(-self.params['max_turn_rate'],
                            min(self.params['max_turn_rate'],
                                self.params['turn_gain'] * heading_error))
        return cmd


class DynamicObstacleDriver(Node):
    """Owns one :class:`ObstacleAgent` per configured obstacle."""

    def __init__(self):
        super().__init__('dynamic_obstacle_driver')

        self.declare_parameter('obstacle_names', [''])
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('random_seed', 20260811)
        self.declare_parameter('arrival_tolerance', 0.35)
        self.declare_parameter('speed_jitter', 0.25)
        self.declare_parameter('dwell_probability', 0.20)
        self.declare_parameter('dwell_min_seconds', 1.5)
        self.declare_parameter('dwell_max_seconds', 4.0)
        self.declare_parameter('skip_probability', 0.10)
        # Heading control. Purely cosmetic -- motion is holonomic -- but it is
        # what makes the human mesh face where it is walking.
        self.declare_parameter('turn_gain', 2.0)
        self.declare_parameter('max_turn_rate', 2.5)
        self.declare_parameter('enabled', True)

        names = [n for n in self.get_parameter('obstacle_names').value if n]
        seed = self.get_parameter('random_seed').value
        params = {
            'arrival_tolerance': self.get_parameter('arrival_tolerance').value,
            'speed_jitter': self.get_parameter('speed_jitter').value,
            'dwell_probability': self.get_parameter('dwell_probability').value,
            'dwell_seconds': (
                self.get_parameter('dwell_min_seconds').value,
                self.get_parameter('dwell_max_seconds').value,
            ),
            'skip_probability': self.get_parameter('skip_probability').value,
            'turn_gain': self.get_parameter('turn_gain').value,
            'max_turn_rate': self.get_parameter('max_turn_rate').value,
        }

        self.agents = {}
        self.publishers_by_name = {}

        for offset, name in enumerate(names):
            self.declare_parameter(f'{name}.speed', 0.8)
            self.declare_parameter(f'{name}.kind', 'cylinder')
            self.declare_parameter(f'{name}.waypoints', [0.0, 0.0])

            flat = list(self.get_parameter(f'{name}.waypoints').value)
            if len(flat) < 4 or len(flat) % 2 != 0:
                self.get_logger().error(
                    f'{name}: waypoints must be an even-length list of at least '
                    f'two (x, y) pairs; got {len(flat)} values. Skipping.'
                )
                continue
            waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]

            # Per-obstacle RNG stream so adding an obstacle does not perturb
            # the motion of the existing ones for a given seed.
            rng = random.Random(seed + 977 * offset)
            self.agents[name] = ObstacleAgent(
                name,
                waypoints,
                self.get_parameter(f'{name}.speed').value,
                rng,
                params,
            )
            self.publishers_by_name[name] = self.create_publisher(
                Twist, f'/obstacles/{name}/cmd_vel', 10
            )
            self.create_subscription(
                Odometry,
                f'/obstacles/{name}/odom',
                self._make_odom_callback(name),
                _sensor_qos(),
            )

        rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'Driving {len(self.agents)} dynamic obstacle(s) at {rate:.0f} Hz '
            f'(seed {seed}): {", ".join(sorted(self.agents))}'
        )

    def _make_odom_callback(self, name):
        def callback(msg):
            self.agents[name].update_pose(msg)
        return callback

    def _on_timer(self):
        if not self.get_parameter('enabled').value:
            # Publishing zeros rather than nothing so a disabled driver leaves
            # the obstacles stationary instead of coasting.
            for name, publisher in self.publishers_by_name.items():
                publisher.publish(Twist())
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        for name, agent in self.agents.items():
            self.publishers_by_name[name].publish(agent.compute_command(now))


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
