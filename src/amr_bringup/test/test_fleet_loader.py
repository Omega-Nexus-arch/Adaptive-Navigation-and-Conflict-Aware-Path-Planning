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
    DEFAULT_MAP_EXTENTS, FleetConfigError, load_fleet, load_map_extents,
    load_model_library, map_substitutions, nav2_substitutions,
    resolve_library_path, resolve_package_uri)

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


# ---------------------------------------------------------------------------
# Model-library path resolution
#
# The shipped rosters reference the model library with a package:// URI. They
# originally used a *relative* path, which worked from the source tree and broke
# the moment the workspace was installed: a relative hop out of
# install/amr_bringup/share/amr_bringup/config/ lands in
# install/amr_bringup/share/, not in amr_description's install prefix. These
# tests exist so that regression cannot come back quietly.
# ---------------------------------------------------------------------------


def test_the_shipped_rosters_use_a_package_uri_not_a_relative_path():
    """A relative path here works in the source tree and fails once installed."""
    for name in ('fleet.yaml', 'fleet_ten_robots.yaml'):
        with open(config(name), 'r', encoding='utf-8') as handle:
            raw = yaml.safe_load(handle)['fleet']['model_library']
        assert raw.startswith('package://'), (
            f'{name} references its model library as {raw!r}; a relative path '
            f'resolves differently in the install space')


def test_package_uris_resolve_via_the_ament_index(tmp_path, monkeypatch=None):
    """A package:// URI must resolve through AMENT_PREFIX_PATH."""
    # Build a fake install prefix: <prefix>/share/<pkg>/<file>
    share = tmp_path / 'share' / 'fake_pkg' / 'config'
    share.mkdir(parents=True)
    target = share / 'models.yaml'
    target.write_text('x: 1')

    previous = os.environ.get('AMENT_PREFIX_PATH', '')
    os.environ['AMENT_PREFIX_PATH'] = str(tmp_path)
    try:
        resolved = resolve_package_uri('package://fake_pkg/config/models.yaml')
        assert resolved is not None
        assert os.path.isfile(resolved)
        assert os.path.samefile(resolved, target)
    finally:
        os.environ['AMENT_PREFIX_PATH'] = previous


def test_a_non_package_path_is_left_to_the_other_forms():
    assert resolve_package_uri('/abs/path.yaml') is None
    assert resolve_package_uri('relative.yaml') is None


def test_a_package_uri_falls_back_to_the_source_tree():
    """Loadable from a checkout with nothing built or sourced.

    Without this the unit tests would depend on a sourced workspace, and anyone
    reading the repository could not run the loader before building.
    """
    previous = os.environ.get('AMENT_PREFIX_PATH', '')
    os.environ['AMENT_PREFIX_PATH'] = ''
    try:
        resolved = resolve_package_uri(
            'package://amr_description/config/robot_models.yaml',
            anchor_file=config('fleet.yaml'))
        assert resolved is not None and os.path.isfile(resolved)
        assert os.path.samefile(resolved, MODELS)
    finally:
        os.environ['AMENT_PREFIX_PATH'] = previous


def test_a_malformed_package_uri_is_rejected():
    with pytest.raises(FleetConfigError, match='malformed package URI'):
        resolve_package_uri('package://no_slash_here')


def test_an_unresolvable_package_uri_explains_both_lookups():
    previous = os.environ.get('AMENT_PREFIX_PATH', '')
    os.environ['AMENT_PREFIX_PATH'] = ''
    try:
        with pytest.raises(FleetConfigError, match='definitely_not_a_package'):
            resolve_package_uri('package://definitely_not_a_package/x.yaml',
                                anchor_file=config('fleet.yaml'))
    finally:
        os.environ['AMENT_PREFIX_PATH'] = previous


def test_absolute_paths_are_used_verbatim():
    assert resolve_library_path('/anywhere/fleet.yaml', MODELS) == MODELS


def test_relative_paths_resolve_against_the_referring_file():
    """The form the test fixtures rely on, kept working."""
    anchor = os.path.join(os.path.dirname(MODELS), 'anchor.yaml')
    assert resolve_library_path(anchor, 'robot_models.yaml') == MODELS


def test_the_model_library_path_resolves_relative_to_the_roster():
    # However it is written, loading must not depend on the working directory
    # and must land on a real file.
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


def test_frames_are_deliberately_absent_from_the_key_rewrites():
    """Frames must not be rewritten by key name.

    RewrittenYaml replaces every occurrence of a key with one value, but
    `global_frame` is `map` in the global costmap and `<ns>/odom` in the local
    one. Including it here would silently anchor the rolling local costmap to
    the map frame. Frames go through `<ns>` placeholders instead.
    """
    fleet = load_fleet(config('fleet.yaml'))
    subs = nav2_substitutions(fleet['robots'][0], 'map', '/tmp/elevation.yaml')
    for key in ('global_frame', 'robot_base_frame', 'odom_topic', 'local_frame'):
        assert key not in subs, (
            f"'{key}' is a frame and must not be a blanket key rewrite; "
            f"use a <ns> placeholder in nav2_params.yaml instead")


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


def _expand_nav2_params(robot, fleet):
    """Reproduce the two-stage substitution navigation.launch.py performs."""
    def rewrite(node, subs):
        # Mimics nav2_common RewrittenYaml: match by key name, recursively.
        if isinstance(node, dict):
            return {k: (subs[k] if k in subs else rewrite(v, subs))
                    for k, v in node.items()}
        if isinstance(node, list):
            return [rewrite(v, subs) for v in node]
        return node

    subs = nav2_substitutions(robot, fleet['global_frame'], '/opt/elev.yaml')
    with open(config('nav2_params.yaml'), 'r', encoding='utf-8') as handle:
        text = handle.read()

    textual = {'<ns>': robot['name']}
    textual.update(map_substitutions(fleet['map_extents']))
    for placeholder, value in textual.items():
        text = text.replace(placeholder, value)
    return rewrite(yaml.safe_load(text), subs)


def test_no_ns_placeholder_survives_expansion():
    fleet = load_fleet(config('fleet.yaml'))
    for robot in fleet['robots']:
        dumped = yaml.dump(_expand_nav2_params(robot, fleet))
        assert '<ns>' in open(config('nav2_params.yaml')).read(), \
            'the template should still contain placeholders'
        assert '<ns>' not in dumped, \
            f"unsubstituted <ns> left in {robot['name']}'s parameters"


def test_the_rolling_local_costmap_stays_anchored_to_odom():
    """The bug a key-based rewrite silently introduces.

    RewrittenYaml matches on key name, so a single `global_frame` rewrite sets
    *every* occurrence to the same value -- anchoring the rolling local costmap
    to `map`, where it lurches on each SLAM correction. Frames therefore go
    through `<ns>` placeholders, which keep their per-section values.
    """
    fleet = load_fleet(config('fleet.yaml'))
    for robot in fleet['robots']:
        params = _expand_nav2_params(robot, fleet)
        name = robot['name']

        local = params['local_costmap']['local_costmap']['ros__parameters']
        assert local['global_frame'] == f'{name}/odom', \
            f"local costmap anchored to {local['global_frame']}, not {name}/odom"
        assert local['rolling_window'] is True

        glob = params['global_costmap']['global_costmap']['ros__parameters']
        assert glob['global_frame'] == 'map', \
            'the global costmap must stay in the shared map frame'


def test_every_frame_is_namespace_prefixed():
    """An unprefixed frame does not resolve in a namespaced TF tree."""
    fleet = load_fleet(config('fleet.yaml'))
    for robot in fleet['robots']:
        params = _expand_nav2_params(robot, fleet)
        name = robot['name']

        expected = {
            ('bt_navigator', 'robot_base_frame'): f'{name}/base_footprint',
            ('bt_navigator', 'odom_topic'): f'/{name}/odom',
            ('behavior_server', 'local_frame'): f'{name}/odom',
            ('behavior_server', 'robot_base_frame'): f'{name}/base_footprint',
        }
        for (section, key), want in expected.items():
            got = params[section]['ros__parameters'][key]
            assert got == want, f'{section}.{key} is {got!r}, expected {want!r}'

        for costmap in ('local_costmap', 'global_costmap'):
            got = params[costmap][costmap]['ros__parameters']['robot_base_frame']
            assert got == f'{name}/base_footprint', f'{costmap}: {got}'


def test_per_model_values_survive_the_expansion():
    """The heterogeneity must still be there once nav2 has its parameters."""
    fleet = load_fleet(config('fleet.yaml'))
    by_name = {r['name']: _expand_nav2_params(r, fleet) for r in fleet['robots']}

    def follow(name, key):
        return float(by_name[name]['controller_server']['ros__parameters']
                     ['FollowPath'][key])

    assert follow('amr1', 'max_vel_x') < follow('amr2', 'max_vel_x')
    assert follow('amr1', 'acc_lim_x') < follow('amr2', 'acc_lim_x')
    assert follow('amr1', 'decel_lim_x') < 0.0

    heavy = by_name['amr1']['global_costmap']['global_costmap']['ros__parameters']
    light = by_name['amr2']['global_costmap']['global_costmap']['ros__parameters']
    assert float(heavy['robot_radius']) > float(light['robot_radius'])


def test_the_fleet_layer_knows_which_robot_it_is():
    """Without this the robot treats its own predicted path as an obstacle."""
    fleet = load_fleet(config('fleet.yaml'))
    for robot in fleet['robots']:
        params = _expand_nav2_params(robot, fleet)
        layer = (params['local_costmap']['local_costmap']['ros__parameters']
                 ['fleet_layer'])
        assert layer['robot_name'] == robot['name']


def test_every_robot_gets_its_own_elevation_and_slope_limit():
    fleet = load_fleet(config('fleet.yaml'))
    for robot in fleet['robots']:
        subs = nav2_substitutions(robot, 'map', '/opt/elevation.yaml')
        assert subs['elevation_map'] == '/opt/elevation.yaml'
        assert float(subs['max_traversable_angle_degrees']) > 0.0
        assert subs['robot_name'] == robot['name']


# ---------------------------------------------------------------------------
# Shared map extents
#
# nav2's global costmap defaults to 5 x 5 m at the origin and only grows once
# its static layer receives a map. Robots here spawn near x = -18.5, so with
# the defaults the planner sits outside its own costmap and logs
#
#     [nav2_costmap_2d]: Robot is out of bounds of the costmap!
#
# every update cycle -- indefinitely if map fusion is not running, which is the
# case whenever someone launches a single robot with `fleet_services:=false`.
# Nothing errors; the robot simply never plans.
# ---------------------------------------------------------------------------


def test_the_shipped_rosters_declare_extents_that_contain_every_spawn():
    for roster in ('fleet.yaml', 'fleet_ten_robots.yaml'):
        fleet = load_fleet(config(roster))
        extents = fleet['map_extents']
        for robot in fleet['robots']:
            radius = float(robot['model_properties']['footprint_radius'])
            assert extents['x_min'] + radius <= robot['x'] <= extents['x_max'] - radius
            assert extents['y_min'] + radius <= robot['y'] <= extents['y_max'] - radius


def test_the_global_costmap_contains_every_spawn_pose():
    """The property that matters, asserted on the expanded YAML.

    Reaching into the parameters nav2 would really receive -- rather than into
    the loader -- is deliberate: the bug lived in the gap between the extents
    being known and them reaching the costmap.
    """
    for roster in ('fleet.yaml', 'fleet_ten_robots.yaml'):
        fleet = load_fleet(config(roster))
        for robot in fleet['robots']:
            params = _expand_nav2_params(robot, fleet)
            grid = params['global_costmap']['global_costmap']['ros__parameters']

            origin_x = float(grid['origin_x'])
            origin_y = float(grid['origin_y'])
            radius = float(grid['robot_radius'])

            assert origin_x + radius <= robot['x'] <= origin_x + grid['width'] - radius, (
                f"{robot['name']} spawns at x={robot['x']} but its global costmap "
                f"covers x[{origin_x}, {origin_x + grid['width']}]; nav2 would log "
                f"'Robot is out of bounds of the costmap!' and never plan")
            assert origin_y + radius <= robot['y'] <= origin_y + grid['height'] - radius


def test_the_global_costmap_is_not_left_at_the_nav2_default():
    """5 x 5 m at (0, 0) is the shape of the bug. Refuse it explicitly."""
    fleet = load_fleet(config('fleet.yaml'))
    grid = (_expand_nav2_params(fleet['robots'][0], fleet)
            ['global_costmap']['global_costmap']['ros__parameters'])
    assert grid['rolling_window'] is False
    assert grid['width'] > 5 and grid['height'] > 5
    assert float(grid['origin_x']) < 0.0 and float(grid['origin_y']) < 0.0


def test_the_local_costmap_keeps_its_own_geometry():
    """Why the extents are textual placeholders and not key rewrites.

    ``RewrittenYaml`` matches on key name, so a single `width` rewrite would
    give every robot a 48-metre rolling local costmap updated at 10 Hz.
    """
    fleet = load_fleet(config('fleet.yaml'))
    for robot in fleet['robots']:
        params = _expand_nav2_params(robot, fleet)
        local = params['local_costmap']['local_costmap']['ros__parameters']
        assert local['width'] == 8 and local['height'] == 8
        assert local['rolling_window'] is True

    subs = nav2_substitutions(fleet['robots'][0], 'map', '/opt/elev.yaml')
    assert 'width' not in subs and 'origin_x' not in subs


def test_no_placeholder_of_any_kind_survives_expansion():
    fleet = load_fleet(config('fleet.yaml'))
    for robot in fleet['robots']:
        dumped = yaml.dump(_expand_nav2_params(robot, fleet))
        for placeholder in ('<ns>', '<map_width>', '<map_height>',
                            '<map_origin_x>', '<map_origin_y>', '<map_resolution>'):
            assert placeholder not in dumped, \
                f"unsubstituted {placeholder} left in {robot['name']}'s parameters"


def test_map_fusion_and_the_global_costmap_describe_the_same_rectangle():
    """Two consumers, one block. This is what stops them drifting apart."""
    fleet = load_fleet(config('fleet.yaml'))
    extents = fleet['map_extents']
    grid = (_expand_nav2_params(fleet['robots'][0], fleet)
            ['global_costmap']['global_costmap']['ros__parameters'])

    assert float(grid['origin_x']) == pytest.approx(extents['x_min'])
    assert float(grid['origin_y']) == pytest.approx(extents['y_min'])
    assert float(grid['resolution']) == pytest.approx(extents['resolution'])
    assert grid['width'] >= extents['x_max'] - extents['x_min']
    assert grid['height'] >= extents['y_max'] - extents['y_min']


def test_a_spawn_outside_the_extents_is_rejected_at_load(tmp_path):
    path = write_roster(tmp_path, [
        {'name': 'amr1', 'model': 'heavy_mapper', 'x': -18.5, 'y': 2.2,
         'yield_priority': 100},
        {'name': 'amr2', 'model': 'light_scout', 'x': -400.0, 'y': 0.0,
         'yield_priority': 50},
    ])
    with pytest.raises(FleetConfigError) as error:
        load_fleet(path)
    assert 'amr2' in str(error.value)
    assert 'never plan' in str(error.value)


def test_a_spawn_that_only_just_overhangs_the_edge_is_rejected():
    """The centre is inside; the body is not. nav2 rejects every plan from
    such a pose, and says nothing about why."""
    extents = {'x_min': -10.0, 'x_max': 10.0, 'y_min': -10.0, 'y_max': 10.0,
               'resolution': 0.05}
    robot = {'name': 'edge', 'x': 9.8, 'y': 0.0,
             'model_properties': {'footprint_radius': 0.55}}
    with pytest.raises(FleetConfigError):
        load_map_extents({'map_extents': extents}, [robot])

    robot['x'] = 9.0
    assert load_map_extents({'map_extents': extents}, [robot])['x_max'] == 10.0


def test_inverted_or_degenerate_extents_are_rejected():
    for bad in ({'x_min': 5.0, 'x_max': -5.0},
                {'y_min': 1.0, 'y_max': 1.0},
                {'resolution': 0.0},
                {'resolution': -0.05}):
        with pytest.raises(FleetConfigError):
            load_map_extents({'map_extents': dict(DEFAULT_MAP_EXTENTS, **bad)}, [])


def test_a_misspelled_extent_key_is_rejected_not_ignored():
    """Silently defaulting is how the costmap ends up the wrong size."""
    with pytest.raises(FleetConfigError) as error:
        load_map_extents({'map_extents': {'xmin': -24.0}}, [])
    assert 'xmin' in str(error.value)


def test_a_roster_without_extents_still_loads(tmp_path):
    """An older roster must not fail to launch; it gets the world's bounds."""
    path = write_roster(tmp_path, [
        {'name': 'amr1', 'model': 'heavy_mapper', 'x': -18.5, 'y': 2.2,
         'yield_priority': 100},
    ])
    assert load_fleet(path)['map_extents'] == DEFAULT_MAP_EXTENTS


def test_extents_are_wide_enough_for_the_generated_world():
    """A grid smaller than the warehouse makes the far aisles unreachable."""
    elevation = os.path.normpath(os.path.join(
        HERE, os.pardir, os.pardir, 'amr_gazebo', 'maps',
        'warehouse_elevation.yaml'))
    with open(elevation, 'r', encoding='utf-8') as handle:
        world = yaml.safe_load(handle)
    cell = float(world['resolution'])
    extents = load_fleet(config('fleet.yaml'))['map_extents']

    assert extents['x_min'] <= float(world['origin'][0])
    assert extents['y_min'] <= float(world['origin'][1])
    assert extents['x_max'] >= float(world['origin'][0]) + world['width'] * cell
    assert extents['y_max'] >= float(world['origin'][1]) + world['height'] * cell


def test_map_substitutions_round_the_grid_up_never_down():
    subs = map_substitutions(
        {'x_min': -1.2, 'x_max': 1.9, 'y_min': -0.4, 'y_max': 0.1,
         'resolution': 0.05})
    assert int(subs['<map_width>']) >= 1.9 - (-1.2)
    assert int(subs['<map_height>']) >= 0.1 - (-0.4)
    assert float(subs['<map_origin_x>']) == -1.2
