# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""The transform chain must compose to the robot's real pose exactly once.

Why this file exists
--------------------
Four components each contribute one link to `map -> <robot>/base_footprint`,
and each is individually reasonable:

    map ------------------ <robot>/map      map_fusion, from the roster's
                                            initial pose
    <robot>/map ---------- <robot>/odom     slam_toolbox, the SLAM correction
    <robot>/odom --------- <robot>/base_footprint
                                            the Gazebo diff-drive plugin

The chain is only correct if the robot's *start pose* appears in exactly one of
them. It appears in the first, by design: that is what anchoring a private SLAM
frame into the shared one means, and it is what lets ten robots share a map.

So the last link must start at zero. `gazebo_ros_diff_drive` will happily do
the opposite -- `<odometry_source>1</odometry_source>` publishes Gazebo's
ground-truth world pose as odometry, which already contains the spawn pose.
Then the start pose is applied twice, and a robot spawned at (-18.5, 2.2)
reports (-37.0, 4.4) in `map`.

Nothing errors. The symptoms appear in two unrelated-looking places:

* the pose is outside the global costmap, so `planner_server` logs
  "Robot is out of bounds of the costmap!" once per cycle and never plans;
* every map contribution lands outside the merged grid, so map fusion reports
  "0.0% explored" forever while each robot's own SLAM looks perfectly healthy.

See DESIGN_NOTES 7h. These tests assert the invariant on the sources, so they
run without ROS, Gazebo, or a built workspace.
"""

import os
import re

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, os.pardir, os.pardir))

#: `gazebo_ros_diff_drive`'s odometry sources.
ENCODER = '0'
WORLD = '1'


def read(*relative):
    with open(os.path.join(SRC, *relative), 'r', encoding='utf-8') as handle:
        return handle.read()


def strip_xml_comments(text):
    return re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)


def test_odometry_starts_at_zero_not_at_the_spawn_pose():
    """The start pose belongs to map_fusion's anchor, and to nothing else."""
    source = strip_xml_comments(read('amr_description', 'urdf', 'amr.gazebo.xacro'))
    found = re.findall(r'<odometry_source>\s*(\S+?)\s*</odometry_source>', source)

    assert found, (
        'amr.gazebo.xacro must state <odometry_source> explicitly. The plugin '
        'default is WORLD, which publishes ground-truth pose as odometry.')
    for value in found:
        assert value == ENCODER, (
            f'<odometry_source>{value}</odometry_source> is WORLD: odometry '
            f'would already carry the spawn pose, and map_fusion applies the '
            f"roster's initial pose on top of it. Use {ENCODER} (ENCODER).")


def test_only_map_fusion_applies_the_rosters_initial_pose():
    """One consumer of `x:`/`y:`/`yaw:` as a *frame offset*, not several.

    The spawn arguments in robot.launch.py place the model in Gazebo, which is
    a different use of the same numbers and does not compose into TF.
    """
    fusion = read('amr_mapping', 'src', 'map_fusion_node.cpp')
    assert 'initial_x' in fusion and 'sendTransform' in fusion, \
        'map_fusion is expected to be the component that anchors each robot'

    # No other node may broadcast a transform derived from the initial pose.
    for package, filename in (
        ('amr_mapping', 'selective_mapping_node.cpp'),
        ('amr_fleet_control', 'trajectory_broadcaster_node.cpp'),
        ('amr_fleet_control', 'safety_override_node.cpp'),
        ('amr_fleet_control', 'velocity_smoother_node.cpp'),
        ('amr_sensor_bsp', 'bsp_validation_node.cpp'),
    ):
        source = read(package, 'src', filename)
        assert 'initial_x' not in source, (
            f'{filename} reads the roster initial pose. Only map_fusion may '
            f'turn it into a transform, or it gets applied twice.')


def test_the_anchor_is_the_only_link_carrying_a_translation():
    """Documents the intended chain, so a future edit has to disagree loudly."""
    gazebo = strip_xml_comments(read('amr_description', 'urdf', 'amr.gazebo.xacro'))

    # The plugin must publish odom -> base_footprint itself...
    assert '<publish_odom_tf>true</publish_odom_tf>' in gazebo
    # ...and must not also publish the wheel transforms that robot_state_publisher
    # already provides, which would make two publishers for one edge.
    assert '<publish_wheel_tf>false</publish_wheel_tf>' in gazebo


def test_every_roster_spawn_is_reachable_as_an_anchor():
    """The anchor is a rigid transform; it needs finite, sane numbers."""
    config_dir = os.path.join(SRC, 'amr_bringup', 'config')
    rosters = [f for f in os.listdir(config_dir)
               if f.startswith('fleet') and f.endswith('.yaml')]
    assert rosters, 'expected at least one roster'

    for name in rosters:
        with open(os.path.join(config_dir, name), 'r', encoding='utf-8') as handle:
            fleet = yaml.safe_load(handle)['fleet']
        for robot in fleet['robots']:
            for key in ('x', 'y', 'yaw'):
                value = float(robot.get(key, 0.0))
                assert value == value and abs(value) < 1e4, \
                    f"{name}: {robot['name']} has a non-finite {key}"
