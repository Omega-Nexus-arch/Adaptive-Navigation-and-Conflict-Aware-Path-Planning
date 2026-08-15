# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""No sensor may be mounted inside the robot it is supposed to perceive from.

Why this file exists
--------------------
A Gazebo ray sensor collides with its own model's links. A 2D LiDAR mounted at
or below the top of the chassis therefore has every beam terminate on the
chassis wall, a few tens of centimetres out. Those hits are beyond ``range_min``
so nothing filters them, and they arrive at SLAM as perfectly ordinary
obstacles.

The consequences arrive far from the cause, and none of them mentions the
LiDAR:

* slam_toolbox maps a box the size of the chassis and nothing else -- the log
  line is ``map geometry changed to 19x13``, which reads like a startup detail;
* the merged map stalls at a fraction of a percent explored;
* the inflation layer turns that ring of self-hits into
  ``INSCRIBED_INFLATED_OBSTACLE`` at the robot's own cell, so the planner
  rejects the *start pose*: ``Starting point in lethal space! Cannot create
  feasible plan.``;
* nav2 runs its recovery behaviours, they also fail, and the goal aborts with
  status 6.

The robot never moves, and every message points somewhere other than the
mounting height. See DESIGN_NOTES 8d.

The arithmetic is simple enough to assert directly, and it scales: the checks
below are derived from ``robot_models.yaml``, so a new model is covered the
moment it is added.
"""

import math
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, os.pardir, os.pardir))

#: Thickness of the cargo deck slab in amr_base.xacro.
DECK_THICKNESS = 0.024

#: Minimum air gap between the top of the robot and the scan plane [m].
#: Small, but it has to be more than the mesh tolerance Gazebo works to.
MIN_CLEARANCE = 0.03


def models():
    path = os.path.join(SRC, 'amr_description', 'config', 'robot_models.yaml')
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def deck_top(model):
    """Height of the highest collision surface on the robot [m]."""
    return (float(model['base_z_offset']) + float(model['chassis_height']) +
            DECK_THICKNESS)


def test_the_scan_plane_clears_the_robots_own_body():
    for name, model in models().items():
        top = deck_top(model)
        lidar_z = float(model['lidar']['height'])
        assert lidar_z >= top + MIN_CLEARANCE, (
            f'{name}: LiDAR at {lidar_z:.3f} m is inside its own chassis, whose '
            f'collision geometry reaches {top:.3f} m. Every beam would hit the '
            f'chassis wall and SLAM would map a '
            f"{model['chassis_length']} x {model['chassis_width']} m box. "
            f'Raise lidar.height above {top + MIN_CLEARANCE:.3f} m.')


def test_a_self_hit_would_not_be_filtered_by_range_min():
    """Explains *why* the clearance matters, and would catch a partial fix.

    Moving the scanner up but leaving it inside the body would still be broken.
    A self-hit is only harmless if it falls inside ``range_min``, and the
    nearest chassis wall is always much further out than that.
    """
    for name, model in models().items():
        nearest_wall = float(model['chassis_width']) / 2.0
        range_min = float(model['lidar']['range_min'])
        assert nearest_wall > range_min, (
            f'{name}: the chassis wall at {nearest_wall:.2f} m is beyond '
            f'range_min ({range_min} m), so a self-hit arrives as a real '
            f'obstacle rather than being discarded. Clearance, not range_min, '
            f'is what keeps the scan clean.')


def test_the_lidar_has_no_collision_geometry_at_the_beam_origin():
    """The scanner housing sits exactly where the rays start."""
    path = os.path.join(SRC, 'amr_description', 'urdf', 'sensors.xacro')
    with open(path, 'r', encoding='utf-8') as handle:
        source = handle.read()

    start = source.index('name="${prefix}lidar_link"')
    end = source.index('</link>', start)
    assert '<collision>' not in source[start:end], (
        'lidar_link has collision geometry. Its own housing would be struck by '
        'every beam of any model whose range_min is below the housing radius.')


def test_the_mast_has_no_collision_geometry_either():
    path = os.path.join(SRC, 'amr_description', 'urdf', 'sensors.xacro')
    with open(path, 'r', encoding='utf-8') as handle:
        source = handle.read()

    assert 'lidar_mast' in source, 'expected a mast raising the scanner'
    start = source.index('name="${prefix}lidar_mast"')
    end = source.index('</link>', start)
    assert '<collision>' not in source[start:end], (
        'the mast sits directly under the scan plane; give it no collision '
        'geometry rather than relying on range_min to discard the returns.')


def test_the_camera_sits_proud_of_the_front_face():
    path = os.path.join(SRC, 'amr_description', 'urdf', 'sensors.xacro')
    with open(path, 'r', encoding='utf-8') as handle:
        source = handle.read()
    assert 'body_length / 2.0 + 0.02' in source, (
        'the camera is flush with the chassis face, so its near plane starts '
        'inside solid geometry')


def test_slam_uses_the_scanners_real_range():
    """The misspelling that cost the heavy mapper a fifth of its range.

    ``minimum_laser_range`` is not a slam_toolbox parameter. It was accepted
    silently and the default 0.0 stayed in force.
    """
    path = os.path.join(SRC, 'amr_bringup', 'config', 'slam_toolbox.yaml')
    with open(path, 'r', encoding='utf-8') as handle:
        text = handle.read()
    body = '\n'.join(
        line.split('#', 1)[0] for line in text.splitlines())

    assert 'minimum_laser_range:' not in body, \
        "slam_toolbox's parameter is `min_laser_range`; the longer spelling " \
        'is ignored and the default 0.0 applies'
    assert 'min_laser_range:' in body and 'max_laser_range:' in body

    launch = os.path.join(SRC, 'amr_bringup', 'launch', 'slam.launch.py')
    with open(launch, 'r', encoding='utf-8') as handle:
        source = handle.read()
    for key in ('min_laser_range', 'max_laser_range'):
        assert f"'{key}'" in source, (
            f'{key} must be rewritten per robot from robot_models.yaml, or the '
            f'two scanner models share one hardcoded range')


#: Gravity [m/s^2].
GRAVITY = 9.81


def steepest_ramp_degrees():
    """The steepest gradient the fleet is expected to climb."""
    import sys as _sys
    _sys.path.insert(0, os.path.join(SRC, 'amr_gazebo'))
    from amr_gazebo import world_builder as wb
    return max(r.slope_deg for r in wb.build_warehouse().ramps)


def test_the_plant_can_out_accelerate_the_steepest_ramp():
    """`<max_wheel_acceleration>` is angular; `max_accel_x` is linear.

    The URDF used to set the Gazebo diff-drive's ceiling to
    ``max_accel_x * 4.0`` with no division by wheel radius. rad/s^2 and m/s^2
    are not the same units, and the mistake is invisible because the number
    still looks plausible: the heavy unit's ceiling came out at 1.40 rad/s^2,
    which is **0.16 m/s^2** of linear acceleration -- less than half the
    0.35 m/s^2 the model itself declares, and well under the 1.03 m/s^2 that
    gravity pulls a robot down a 6 degree ramp.

    Neither robot could out-accelerate the slope it was standing on, so both
    stalled on the ramps. The scout stalled hardest because its wheels are
    smaller, which makes the missing division bite harder.
    """
    path = os.path.join(SRC, 'amr_description', 'urdf', 'amr.gazebo.xacro')
    with open(path, 'r', encoding='utf-8') as handle:
        source = handle.read()
    assert "m['wheel_radius']" in source.split('<max_wheel_acceleration>')[1], (
        'max_wheel_acceleration is in rad/s^2; deriving it from a linear '
        'acceleration without dividing by the wheel radius understates it by '
        'a factor of 1/r'
    )

    slope = math.radians(steepest_ramp_degrees())
    demanded = GRAVITY * math.sin(slope)

    for name, model in models().items():
        linear = float(model['plant_accel_limit'])
        assert linear > demanded, (
            f'{name}: the plant tops out at {linear:.2f} m/s^2 but gravity '
            f'pulls it down the steepest ramp at {demanded:.2f} m/s^2. It '
            f'cannot climb; it will stall and slide back.'
        )
        assert linear >= model['max_accel_x'], (
            f'{name}: the plant ceiling {linear:.2f} m/s^2 is below the '
            f"model's own {model['max_accel_x']} m/s^2, so the simulator -- "
            f'not the motion smoother -- would be shaping the acceleration '
            f'profile, and the smoothing demo would be measuring Gazebo'
        )


def test_a_human_is_solid_across_both_scan_planes():
    """The AMRs were driving through the pedestrians.

    An actor's collisions are per-bone spheres. A 2D LiDAR samples exactly one
    height, so with only knees and hips declared, whether a person was seen at
    all came down to whether that single plane happened to clip a sphere. The
    column now overlaps continuously from the feet to the chest.
    """
    import sys as _sys
    _sys.path.insert(0, os.path.join(SRC, 'amr_gazebo'))
    from amr_gazebo import world_builder as wb

    #: Approximate bone heights on the 1.8 m skeleton [m].
    heights = {
        'LeftFoot': 0.20, 'RightFoot': 0.20,
        'LeftLeg': 0.55, 'RightLeg': 0.55,
        'LeftUpLeg': 0.95, 'RightUpLeg': 0.95,
        'Hips': 1.00, 'LowerBack': 1.10,
    }
    bones = wb.HUMAN_BONE_COLLISIONS
    assert set(bones) <= set(heights), f'unknown bone in {set(bones) - set(heights)}'

    # Every scan plane either robot uses must fall inside some sphere.
    for name, model in models().items():
        plane = float(model['lidar']['height'])
        hit = [b for b, r in bones.items() if abs(plane - heights[b]) <= r]
        assert hit, (
            f'{name} scans at {plane} m and no collision sphere spans that '
            f'height, so its LiDAR passes straight through a pedestrian'
        )

    # And the column must be continuous, or a taller robot added later drops
    # into a gap between two spheres.
    spans = sorted((heights[b] - r, heights[b] + r) for b, r in bones.items())
    reach = spans[0][1]
    for low, high in spans[1:]:
        assert low <= reach + 1e-9, (
            f'the collision column has a gap between {reach:.2f} m and '
            f'{low:.2f} m; a scan plane there would see nothing'
        )
        reach = max(reach, high)
    assert spans[0][0] <= 0.05, 'the column should start at the floor'
    assert reach >= 1.4, 'the column should reach at least chest height'


def test_no_xacro_comment_contains_a_double_hyphen():
    """`--` is illegal inside an XML comment, and xacro will not load the file.

    Easy to write by accident when a comment explains a subtraction or uses a
    dash as punctuation, and the failure is a parser error a long way from the
    prose that caused it.
    """
    import glob
    import re

    urdf = os.path.join(SRC, 'amr_description', 'urdf')
    files = glob.glob(os.path.join(urdf, '*.xacro'))
    assert files, 'expected some xacro files'

    for path in files:
        with open(path, 'r', encoding='utf-8') as handle:
            text = handle.read()
        for comment in re.findall(r'<!--.*?-->', text, re.S):
            inner = comment[4:-3]
            assert '--' not in inner, (
                f'{os.path.basename(path)}: an XML comment contains "--", which '
                f'makes the file unparseable:\n    ...'
                f'{inner[max(0, inner.find("--") - 40):inner.find("--") + 40]}...'
            )


def test_every_model_is_covered_by_these_checks():
    """A new model must not silently escape the placement rules."""
    library = models()
    assert len(library) >= 2
    for name, model in library.items():
        for key in ('base_z_offset', 'chassis_height', 'chassis_width',
                    'chassis_length', 'lidar'):
            assert key in model, f'{name} is missing {key}'
