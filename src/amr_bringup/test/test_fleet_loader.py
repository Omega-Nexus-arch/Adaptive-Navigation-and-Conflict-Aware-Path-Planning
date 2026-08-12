# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Tests for the launch-time roster loader.

These matter because the loader runs *before* any node exists. A roster error
it fails to catch does not produce one clear exception; it produces a
half-started fleet whose nodes then throw one by one, which is a much worse
thing to debug. So the loader duplicates the C++ validation deliberately, and
these tests hold it to that.

The scaling tests also serve as the machine-checkable form of the
"expand to ten or more robots by changing a minimal number of configuration
parameters" requirement.
"""

import os
import sys

import pytest
import yaml

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))

from amr_bringup.fleet_loader import (  # noqa: E402
    FleetConfigError, load_fleet, load_model_library, nav2_substitutions)

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(HERE, os.pardir, 'config')
MODELS = os.path.normpath(
    os.path.join(HERE, os.pardir, os.pardir, 'amr_description', 'config',
                 'robot_models.yaml'))


def config(name):
    return os.path.normpath(os.path.join(CONFIG_DIR, name))


def write_roster(tmp_path, robots, name='roster.yaml'):
    """Write a roster referring to the real model library."""
    path = tmp_path / name
    document = {
        'fleet': {
            'model_library': MODELS,
            'global_frame': 'map',
            'robots': robots,
        }
    }
    path.write_text(yaml.safe_dump(document))
    return str(path)


# ---------------------------------------------------------------------------
# The shipped rosters
# ---------------------------------------------------------------------------


def test_the_shipped_two_robot_roster_loads():
    fleet = load_fleet(config('fleet.yaml'))
    assert len(fleet['robots']) == 2
    assert [r['name'] for r in fleet['robots']] == ['amr1', 'amr2']
    assert fleet['global_frame'] == 'map'


def test_the_shipped_roster_matches_the_brief():
    """AMR-1 is the heavy lead; AMR-2 is the light scout that gives way."""
    fleet = load_fleet(config('fleet.yaml'))
    by_name = {r['name']: r for r in fleet['robots']}

    heavy = by_name['amr1']['model_properties']
    light = by_name['amr2']['model_properties']

    assert heavy['payload_capacity_kg'] > light['payload_capacity_kg']
    assert heavy['max_accel_x'] < light['max_accel_x'], \
        'the heavy unit must have the lower acceleration limit'
    assert heavy['max_vel_x'] < light['max_vel_x']
    assert by_name['amr1']['yield_priority'] > by_name['amr2']['yield_priority'], \
        'the scout must yield to the mission-critical heavy unit'


def test_every_model_block_resolves():
    fleet = load_fleet(config('fleet.yaml'))
    for robot in fleet['robots']:
        model = robot['model_properties']
        for key in ('max_vel_x', 'max_accel_x', 'max_jerk_x', 'footprint_radius',
                    'safety_k', 'safety_d_min', 'lidar', 'imu', 'camera'):
            assert key in model, f"{robot['name']} is missing {key}"


def test_policy_is_exposed():
    fleet = load_fleet(config('fleet.yaml'))
    policy = fleet['policy']
    assert policy['horizon_seconds'] > 0
    assert policy['hard_yield_seconds'] <= policy['conflict_react_seconds'], \
        'a conflict must be actionable before it escalates to a stop'


# ---------------------------------------------------------------------------
# Scalability
# ---------------------------------------------------------------------------


def test_the_ten_robot_roster_loads_unchanged():
    """The scalability requirement, as a test rather than a claim."""
    fleet = load_fleet(config('fleet_ten_robots.yaml'))
    assert len(fleet['robots']) == 10
    names = [r['name'] for r in fleet['robots']]
    assert len(set(names)) == 10
    priorities = [r['yield_priority'] for r in fleet['robots']]
    assert len(set(priorities)) == 10, 'priorities must stay unique at scale'


def test_scaling_needs_no_new_model():
    """Ten robots come out of the same two models: only the roster grew."""
    two = load_fleet(config('fleet.yaml'))
    ten = load_fleet(config('fleet_ten_robots.yaml'))
    assert set(r['model'] for r in ten['robots']) <= set(two['models'].keys())
    assert two['model_library_path'] == ten['model_library_path']


def test_an_arbitrary_fleet_size_loads(tmp_path):
    robots = [
        {'name': f'amr{i}', 'model': 'light_scout', 'yield_priority': i}
        for i in range(1, 26)
    ]
    fleet = load_fleet(write_roster(tmp_path, robots))
    assert len(fleet['robots']) == 25


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


def test_duplicate_names_are_rejected(tmp_path):
    # The name is used verbatim as a ROS namespace, so a duplicate would
    # silently cross-wire two robots' cmd_vel.
    robots = [
        {'name': 'amr1', 'model': 'heavy_mapper', 'yield_priority': 10},
        {'name': 'amr1', 'model': 'light_scout', 'yield_priority': 20},
    ]
    with pytest.raises(FleetConfigError, match='duplicate robot name'):
        load_fleet(write_roster(tmp_path, robots))


def test_duplicate_priorities_are_rejected(tmp_path):
    robots = [
        {'name': 'amr1', 'model': 'heavy_mapper', 'yield_priority': 7},
        {'name': 'amr2', 'model': 'light_scout', 'yield_priority': 7},
    ]
    with pytest.raises(FleetConfigError, match='unique'):
        load_fleet(write_roster(tmp_path, robots))


def test_unknown_models_are_rejected_and_alternatives_listed(tmp_path):
    robots = [{'name': 'amr1', 'model': 'hovercraft', 'yield_priority': 1}]
    with pytest.raises(FleetConfigError, match='heavy_mapper'):
        load_fleet(write_roster(tmp_path, robots))


def test_a_name_with_a_slash_is_rejected(tmp_path):
    robots = [{'name': 'fleet/amr1', 'model': 'light_scout', 'yield_priority': 1}]
    with pytest.raises(FleetConfigError, match="must not contain"):
        load_fleet(write_roster(tmp_path, robots))


def test_an_empty_roster_is_rejected(tmp_path):
    with pytest.raises(FleetConfigError, match='robots'):
        load_fleet(write_roster(tmp_path, []))


def test_a_missing_file_is_rejected():
    with pytest.raises(FleetConfigError, match='not found'):
        load_fleet('/nonexistent/fleet.yaml')


def test_a_missing_model_library_is_rejected(tmp_path):
    path = tmp_path / 'no_library.yaml'
    path.write_text(yaml.safe_dump({'fleet': {'robots': [{'name': 'a', 'model': 'b'}]}}))
    with pytest.raises(FleetConfigError, match='model_library'):
        load_fleet(str(path))


def test_the_model_library_path_resolves_relative_to_the_roster():
    # The shipped roster uses a relative path, so loading must not depend on
    # the working directory.
    fleet = load_fleet(config('fleet.yaml'))
    assert os.path.isabs(fleet['model_library_path'])
    assert os.path.isfile(fleet['model_library_path'])


def test_model_library_loads_standalone():
    models = load_model_library(MODELS)
    assert 'heavy_mapper' in models
    assert 'light_scout' in models


# ---------------------------------------------------------------------------
# nav2 parameter derivation
# ---------------------------------------------------------------------------


def test_nav2_substitutions_come_from_the_model_library():
    """Planner limits must be derived, not duplicated."""
    fleet = load_fleet(config('fleet.yaml'))
    by_name = {r['name']: r for r in fleet['robots']}

    heavy = nav2_substitutions(by_name['amr1'], 'map', '/tmp/elevation.yaml')
    light = nav2_substitutions(by_name['amr2'], 'map', '/tmp/elevation.yaml')

    assert heavy['robot_base_frame'] == 'amr1/base_footprint'
    assert light['robot_base_frame'] == 'amr2/base_footprint'
    assert heavy['odom_topic'] == '/amr1/odom'

    # The values must match the library rather than being independently typed.
    assert float(heavy['max_vel_x']) == \
        by_name['amr1']['model_properties']['max_vel_x']
    assert float(heavy['acc_lim_x']) == \
        by_name['amr1']['model_properties']['max_accel_x']
    assert float(heavy['robot_radius']) == \
        by_name['amr1']['model_properties']['footprint_radius']

    # And the heterogeneity has to survive the derivation.
    assert float(heavy['max_vel_x']) < float(light['max_vel_x'])
    assert float(heavy['acc_lim_x']) < float(light['acc_lim_x'])
    assert float(heavy['robot_radius']) > float(light['robot_radius'])


def test_deceleration_limits_are_negative():
    # nav2 expects decel_lim_* as a negative number. Getting the sign wrong
    # here makes the controller refuse to brake, which is not obvious from the
    # symptom.
    fleet = load_fleet(config('fleet.yaml'))
    for robot in fleet['robots']:
        subs = nav2_substitutions(robot, 'map', '/tmp/elevation.yaml')
        assert float(subs['decel_lim_x']) < 0.0
        assert float(subs['decel_lim_theta']) < 0.0


def test_inflation_radius_exceeds_the_footprint():
    fleet = load_fleet(config('fleet.yaml'))
    for robot in fleet['robots']:
        subs = nav2_substitutions(robot, 'map', '/tmp/elevation.yaml')
        assert float(subs['inflation_radius']) > float(subs['robot_radius'])


def test_every_robot_gets_its_own_elevation_and_slope_limit():
    fleet = load_fleet(config('fleet.yaml'))
    for robot in fleet['robots']:
        subs = nav2_substitutions(robot, 'map', '/opt/elevation.yaml')
        assert subs['elevation_map'] == '/opt/elevation.yaml'
        assert float(subs['max_traversable_angle_degrees']) > 0.0
        assert subs['robot_name'] == robot['name']
