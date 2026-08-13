# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Geometric acceptance tests for the generated warehouse.

These are not smoke tests. Each one asserts a property the navigation
requirements depend on, so a careless edit to the layout fails the build rather
than quietly producing an unsolvable world:

* ramp gradients are what the documentation claims;
* the SDF pose of each ramp slab really does put its top face where the
  elevation map says the floor is (this is the map/world agreement that the
  slope planner's correctness rests on);
* every named goal is reachable from the docks under a step-height
  traversability model, and the mezzanine is reachable *only* via its ramps;
* The Pinch is wide enough for one robot and too narrow for two.
"""

import math
import os
import sys

import pytest

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
)

from amr_gazebo import world_builder as wb  # noqa: E402

RESOLUTION = 0.05
# Maximum height discontinuity a wheeled AMR can climb in one 5 cm step.
MAX_STEP = 0.05


@pytest.fixture(scope='module')
def spec():
    return wb.build_warehouse()


# ---------------------------------------------------------------------------
# Rasterisation helpers
# ---------------------------------------------------------------------------


def _obstacle_grid(spec, resolution=RESOLUTION):
    """Rasterise static bodies into a boolean occupancy grid.

    Guard rails and door jambs are included: they are genuine obstacles, and a
    layout edit that seals a ramp mouth with a rail must fail loudly.
    """
    import numpy as np

    width = int(round((spec.x_max - spec.x_min) / resolution))
    height = int(round((spec.y_max - spec.y_min) / resolution))
    grid = np.zeros((height, width), dtype=bool)

    for box in spec.boxes:
        # Yaw is zero throughout the current layout; assert rather than
        # silently mis-rasterise if that ever changes.
        assert abs(box.yaw) < 1e-9, f'{box.name}: rotated boxes not rasterised'
        x0 = box.x - box.size_x / 2.0
        x1 = box.x + box.size_x / 2.0
        y0 = box.y - box.size_y / 2.0
        y1 = box.y + box.size_y / 2.0

        col0 = max(0, int(math.floor((x0 - spec.x_min) / resolution)))
        col1 = min(width, int(math.ceil((x1 - spec.x_min) / resolution)))
        # Row 0 is maximum y.
        row0 = max(0, int(math.floor((spec.y_max - y1) / resolution)))
        row1 = min(height, int(math.ceil((spec.y_max - y0) / resolution)))
        grid[row0:row1, col0:col1] = True

    return grid


def _elevation_grid(spec, resolution=RESOLUTION):
    import numpy as np

    width = int(round((spec.x_max - spec.x_min) / resolution))
    height = int(round((spec.y_max - spec.y_min) / resolution))
    elev = np.zeros((height, width), dtype=float)
    for row in range(height):
        y = spec.y_max - (row + 0.5) * resolution
        for col in range(width):
            x = spec.x_min + (col + 0.5) * resolution
            elev[row, col] = wb.sample_elevation(spec, x, y)
    return elev


def _to_cell(spec, x, y, resolution=RESOLUTION):
    col = int((x - spec.x_min) / resolution)
    row = int((spec.y_max - y) / resolution)
    return row, col


def _reachable(spec, occupancy, elevation, start_xy, extra_blocked=None):
    """Flood fill from ``start_xy`` under a step-height traversability model.

    Two neighbouring free cells are connected only when their floor heights
    differ by at most ``MAX_STEP``. That single rule is what makes a deck edge
    a cliff and a ramp a road, and it is the same assumption the slope layer
    encodes as cost.
    """
    import numpy as np

    height, width = occupancy.shape
    blocked = occupancy.copy()
    if extra_blocked is not None:
        blocked |= extra_blocked

    visited = np.zeros_like(blocked)
    start = _to_cell(spec, *start_xy)
    assert not blocked[start], f'start {start_xy} is inside an obstacle'

    stack = [start]
    visited[start] = True
    while stack:
        row, col = stack.pop()
        z_here = elevation[row, col]
        for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n_row, n_col = row + d_row, col + d_col
            if not (0 <= n_row < height and 0 <= n_col < width):
                continue
            if visited[n_row, n_col] or blocked[n_row, n_col]:
                continue
            if abs(elevation[n_row, n_col] - z_here) > MAX_STEP:
                continue
            visited[n_row, n_col] = True
            stack.append((n_row, n_col))
    return visited


def _rect_block(spec, occupancy, x0, x1, y0, y1, resolution=RESOLUTION):
    """Return a mask blocking an axis-aligned rectangle (scenario helper)."""
    import numpy as np

    mask = np.zeros_like(occupancy)
    row0, col0 = _to_cell(spec, x0, y1, resolution)
    row1, col1 = _to_cell(spec, x1, y0, resolution)
    mask[row0:row1 + 1, col0:col1 + 1] = True
    return mask


# ---------------------------------------------------------------------------
# Ramp geometry
# ---------------------------------------------------------------------------


EXPECTED_SLOPES_DEG = {
    'hump_ramp_west': 8.7,
    'hump_ramp_east': 8.7,
    'mezz_ramp_gentle': 7.8,
    'mezz_ramp_steep': 14.8,
}


def test_ramp_gradients_match_documentation(spec):
    actual = {r.name: r.slope_deg for r in spec.ramps}
    assert set(actual) == set(EXPECTED_SLOPES_DEG)
    for name, expected in EXPECTED_SLOPES_DEG.items():
        assert actual[name] == pytest.approx(expected, abs=0.15), (
            f'{name}: {actual[name]:.2f} deg, documentation claims {expected} deg'
        )


def test_every_ramp_is_climbable(spec):
    """No ramp exceeds the 15 degree limit the drive train is sized for."""
    for ramp in spec.ramps:
        assert ramp.slope_deg <= 15.0, f'{ramp.name} is {ramp.slope_deg:.1f} deg'


def test_ramp_slab_pose_matches_elevation_model(spec):
    """The SDF slab's top face must coincide with ``Ramp.height_at``.

    This is the load-bearing consistency check: if it fails, Gazebo's physical
    ramp and the elevation map handed to the slope planner describe different
    terrain, and every downstream claim about slope cost is void.
    """
    for ramp in spec.ramps:
        px, py, pz, roll, pitch, yaw = ramp.pose()
        assert roll == pytest.approx(0.0)

        # Unit vector along the slab's local +x, expressed in world axes.
        along = (
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            -math.sin(pitch),
        )
        # Offset from slab centre to top-face centre.
        up = (
            math.sin(pitch) * math.cos(yaw),
            math.sin(pitch) * math.sin(yaw),
            math.cos(pitch),
        )
        half_t = ramp.thickness / 2.0
        top_centre = (
            px + up[0] * half_t,
            py + up[1] * half_t,
            pz + up[2] * half_t,
        )

        for frac in (-0.5, -0.25, 0.0, 0.25, 0.5):
            offset = frac * ramp.length
            wx = top_centre[0] + along[0] * offset
            wy = top_centre[1] + along[1] * offset
            wz = top_centre[2] + along[2] * offset
            expected = ramp.height_at(wx, wy)
            assert wz == pytest.approx(expected, abs=1e-3), (
                f'{ramp.name}: slab top is {wz:.4f} m at ({wx:.2f}, {wy:.2f}) '
                f'but the elevation model says {expected:.4f} m'
            )


def test_ramps_meet_their_decks(spec):
    """Each ramp's high end must land flush with the deck it serves."""
    deck_by_height = {round(d.height, 3): d for d in spec.decks}
    for ramp in spec.ramps:
        top = max(ramp.z_start, ramp.z_end)
        assert round(top, 3) in deck_by_height, (
            f'{ramp.name} rises to {top} m, which matches no deck'
        )


def _overlap_area(a, b):
    """Horizontal intersection area of two (x_min, x_max, y_min, y_max) boxes."""
    dx = min(a[1], b[1]) - max(a[0], b[0])
    dy = min(a[3], b[3]) - max(a[2], b[2])
    return max(0.0, dx) * max(0.0, dy)


def test_floor_surfaces_do_not_overlap(spec):
    """No point on the floor may belong to two different surfaces.

    ``sample_elevation`` resolves overlaps by declaration order, so an overlap
    silently gives the elevation map a height that Gazebo does not simulate.
    Ramps and decks are allowed to *touch* (that is how a ramp joins its deck);
    they are not allowed to intersect.
    """
    surfaces = [(r.name, r.bounds()) for r in spec.ramps]
    surfaces += [
        (d.name, (d.x_min, d.x_max, d.y_min, d.y_max)) for d in spec.decks
    ]
    for i, (name_a, box_a) in enumerate(surfaces):
        for name_b, box_b in surfaces[i + 1:]:
            area = _overlap_area(box_a, box_b)
            assert area < 1e-6, (
                f'{name_a} and {name_b} overlap over {area:.3f} m^2; the '
                'elevation map and the simulated geometry cannot both be right'
            )


# ---------------------------------------------------------------------------
# Elevation map
# ---------------------------------------------------------------------------


def test_elevation_map_round_trips(spec):
    """Decoding the PGM must reproduce the sampled heights within one LSB."""
    pgm, meta, stats = wb.render_elevation_map(spec, RESOLUTION)
    import yaml

    body = pgm.split(b'\n', 3)[3]
    metadata = yaml.safe_load(meta.replace('#', '# '))
    width = int(stats['width'])
    encode_max = metadata['elevation_max']
    lsb = encode_max / 255.0

    # Two independent error sources, and the tolerance has to cover both:
    #   * the 8-bit height quantisation (one LSB), and
    #   * the fact that a probe lands somewhere inside a cell while the decoded
    #     value belongs to that cell's centre. On the steepest 15 degree ramp,
    #     half a cell of horizontal offset is 0.5 * 0.05 * tan(15 deg) of height.
    tolerance = lsb + 0.5 * RESOLUTION * math.tan(math.radians(15.0))

    probes = [
        # (x, y, expected height, description)
        (-18.0, 2.2, 0.0, 'west dock, ground level'),
        (0.0, 3.6, 0.55, 'hump bridge deck'),
        (-3.4, 3.6, 0.275, 'mid-point of the west hump ramp'),
        (7.0, 10.5, 0.45, 'mezzanine deck'),
        (7.0, 5.85, 0.225, 'mid-point of the gentle mezzanine ramp'),
        (17.0, 2.0, 0.0, 'packing bay 4, ground level'),
    ]
    for x, y, expected, description in probes:
        row, col = _to_cell(spec, x, y)
        value = body[row * width + col]
        decoded = value * lsb
        assert decoded == pytest.approx(expected, abs=tolerance), (
            f'{description}: decoded {decoded:.4f} m, expected {expected:.4f} m'
        )


def test_elevation_map_dimensions_match_metadata(spec):
    pgm, meta, stats = wb.render_elevation_map(spec, RESOLUTION)
    header, dims, _maxval, body = pgm.split(b'\n', 3)
    assert header == b'P5'
    width, height = (int(v) for v in dims.split())
    assert width == int(stats['width'])
    assert height == int(stats['height'])
    assert len(body) == width * height
    assert f'width: {width}' in meta
    assert f'height: {height}' in meta


# ---------------------------------------------------------------------------
# Connectivity
# ---------------------------------------------------------------------------


def test_primary_goals_are_reachable_from_the_docks(spec):
    occupancy = _obstacle_grid(spec)
    elevation = _elevation_grid(spec)
    named = {w.name: w for w in spec.waypoints}
    visited = _reachable(spec, occupancy, elevation, (named['dock_a'].x, named['dock_a'].y))

    for goal in ('heavy_storage', 'packing_bay_4', 'east_staging',
                 'west_staging', 'pinch_west', 'pinch_east', 'dock_b',
                 'mezzanine_storage'):
        wp = named[goal]
        assert visited[_to_cell(spec, wp.x, wp.y)], f'{goal} is unreachable'


def test_mezzanine_is_reachable_only_by_ramp(spec):
    """Sealing both mezzanine ramp mouths must isolate the deck.

    If this passes, ``mezzanine_storage`` genuinely exercises the "ramps are
    the only viable path" branch of the planner requirement.
    """
    occupancy = _obstacle_grid(spec)
    elevation = _elevation_grid(spec)
    named = {w.name: w for w in spec.waypoints}

    sealed = _rect_block(spec, occupancy, 5.3, 8.7, 4.0, 7.6)
    sealed |= _rect_block(spec, occupancy, 9.1, 11.9, 5.6, 7.6)

    visited = _reachable(
        spec, occupancy, elevation,
        (named['dock_a'].x, named['dock_a'].y), extra_blocked=sealed,
    )
    goal = named['mezzanine_storage']
    assert not visited[_to_cell(spec, goal.x, goal.y)], (
        'the mezzanine is reachable without using a ramp, so the '
        '"only viable path" scenario proves nothing'
    )


def test_blocking_the_pinch_leaves_the_bridge_as_the_only_crossing(spec):
    """The scripted slope demo depends on this being the actual topology."""
    occupancy = _obstacle_grid(spec)
    elevation = _elevation_grid(spec)
    named = {w.name: w for w in spec.waypoints}

    # Seal the flat doorway only.
    blocked_pinch = _rect_block(spec, occupancy, -0.6, 0.6, -1.1, 1.1)
    visited = _reachable(
        spec, occupancy, elevation,
        (named['dock_a'].x, named['dock_a'].y), extra_blocked=blocked_pinch,
    )
    east = named['east_staging']
    assert visited[_to_cell(spec, east.x, east.y)], (
        'with the doorway shut the east half must still be reachable over '
        'the Hump Bridge'
    )

    # Seal the doorway *and* both bridge ramps: the halves must separate.
    blocked_both = blocked_pinch | _rect_block(spec, occupancy, -5.4, -1.4, 1.9, 5.3)
    blocked_both |= _rect_block(spec, occupancy, 1.4, 5.4, 1.9, 5.3)
    visited = _reachable(
        spec, occupancy, elevation,
        (named['dock_a'].x, named['dock_a'].y), extra_blocked=blocked_both,
    )
    assert not visited[_to_cell(spec, east.x, east.y)], (
        'a third west-east crossing exists; the flat-vs-sloped comparison is '
        'not controlled'
    )


def test_pinch_admits_one_robot_but_not_two(spec):
    """Doorway clearance must sit between one and two AMR-1 widths."""
    occupancy = _obstacle_grid(spec)
    col = _to_cell(spec, 0.0, 0.0)[1]
    column = occupancy[:, col]

    # Walk outwards from y = 0 to find the free span at x = 0.
    row_centre = _to_cell(spec, 0.0, 0.0)[0]
    assert not column[row_centre], 'the doorway centre is blocked'
    row_lo = row_centre
    while row_lo > 0 and not column[row_lo - 1]:
        row_lo -= 1
    row_hi = row_centre
    while row_hi < column.size - 1 and not column[row_hi + 1]:
        row_hi += 1
    clearance = (row_hi - row_lo + 1) * RESOLUTION

    amr1_width = 0.62
    assert clearance >= amr1_width + 0.8, (
        f'doorway is {clearance:.2f} m; too tight for AMR-1 to pass safely'
    )
    assert clearance < 2 * (amr1_width + 0.45), (
        f'doorway is {clearance:.2f} m; two robots could pass abreast and the '
        'yielding protocol would never be exercised'
    )


def test_no_waypoint_sits_inside_an_obstacle(spec):
    occupancy = _obstacle_grid(spec)
    for wp in spec.waypoints:
        assert not occupancy[_to_cell(spec, wp.x, wp.y)], (
            f'waypoint "{wp.name}" is inside a static body'
        )


def test_no_dynamic_obstacle_spawns_inside_geometry(spec):
    occupancy = _obstacle_grid(spec)
    elevation = _elevation_grid(spec)
    for obs in spec.dynamics:
        for x, y in obs.waypoints:
            cell = _to_cell(spec, x, y)
            assert not occupancy[cell], (
                f'{obs.name} waypoint ({x}, {y}) is inside a static body'
            )
            assert elevation[cell] == pytest.approx(0.0, abs=1e-6), (
                f'{obs.name} waypoint ({x}, {y}) is not on the ground floor; '
                'the planar-move plugin cannot climb'
            )


def test_every_dynamic_obstacle_starts_on_its_own_loop(spec):
    """Spawn pose must be the first waypoint.

    When it is not, the obstacle's first move is an unplanned dash from
    wherever it was spawned to wherever the loop begins -- through whatever
    happens to be in between. `ped_0` shipped with its first waypoint's sign
    dropped (`14.0` for `-14.0`), which sent it 28 m across the warehouse and
    straight through the central firewall on its very first leg.
    """
    for obs in spec.dynamics:
        first_x, first_y = obs.waypoints[0]
        assert (obs.start_x, obs.start_y) == pytest.approx((first_x, first_y)), (
            f'{obs.name} spawns at ({obs.start_x}, {obs.start_y}) but its loop '
            f'begins at ({first_x}, {first_y}); its first move would be a dash '
            f'between the two, ignoring anything in the way'
        )


def test_no_patrol_leg_passes_through_a_static_body(spec):
    """The waypoints being clear is not enough; the legs between them must be.

    ``test_no_dynamic_obstacle_spawns_inside_geometry`` already checks the
    waypoints, and it passed with the sign error above, because (14.0, -7.3)
    is perfectly good floor. What was wrong was the *path* to it. Sampling the
    legs is what turns a set of valid points into a valid route.
    """
    def clearance(x, y):
        """Distance from a point to the nearest static body, 0 if inside one."""
        nearest = float('inf')
        for box in spec.boxes:
            if box.yaw:
                continue          # yawed bodies are handled by the grid check
            dx = max(abs(x - box.x) - box.size_x / 2.0, 0.0)
            dy = max(abs(y - box.y) - box.size_y / 2.0, 0.0)
            nearest = min(nearest, math.hypot(dx, dy))
        return nearest

    for obs in spec.dynamics:
        loop = [tuple(point) for point in obs.waypoints]
        for start, end in zip(loop, loop[1:] + loop[:1]):
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            steps = max(2, int(length / 0.05))
            for step in range(steps + 1):
                t = step / steps
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                # The body has width. Checking the centre line only would pass
                # a loop that scrapes along a rack face.
                assert clearance(x, y) >= obs.radius, (
                    f'{obs.name}: the leg {start} -> {end} comes within '
                    f'{clearance(x, y):.2f} m of a static body at '
                    f'({x:.2f}, {y:.2f}), closer than its own {obs.radius} m '
                    f'radius. The obstacle would grind against it, or tunnel '
                    f'through it and present the fleet an obstacle that no '
                    f'sensor ever saw arrive.'
                )


def test_patrol_loops_are_closed_and_non_degenerate(spec):
    """A loop needs at least a triangle, and no repeated points."""
    for obs in spec.dynamics:
        loop = [tuple(w) for w in obs.waypoints]
        assert len(loop) >= 3, f'{obs.name} has only {len(loop)} waypoints'
        assert len(set(loop)) == len(loop), \
            f'{obs.name} repeats a waypoint, so it would stall there'
        assert obs.speed > 0.0, f'{obs.name} has a non-positive speed'


def test_the_generated_config_matches_the_spec(spec):
    """dynamic_obstacles.yaml is generated; hand-editing it would be lost.

    This asserts the round trip, so a change made in the YAML instead of in
    world_builder.py is caught the next time the world is regenerated rather
    than silently reverted.
    """
    import yaml as yaml_module
    rendered = yaml_module.safe_load(
        wb.render_dynamic_obstacles(spec))
    params = rendered['dynamic_obstacle_driver']['ros__parameters']

    assert params['obstacle_names'] == [o.name for o in spec.dynamics]
    for obs in spec.dynamics:
        entry = params[obs.name]
        assert entry['speed'] == pytest.approx(obs.speed)
        assert entry['kind'] == obs.shape
        flat = [coordinate for point in obs.waypoints for coordinate in point]
        assert entry['waypoints'] == pytest.approx(flat)


# ---------------------------------------------------------------------------
# SDF sanity
# ---------------------------------------------------------------------------


def test_generated_sdf_is_well_formed(spec):
    import xml.etree.ElementTree as ET

    root = ET.fromstring(wb.render_sdf(spec))
    assert root.tag == 'sdf'
    world = root.find('world')
    assert world is not None

    names = [m.get('name') for m in world.findall('model')]
    assert len(names) == len(set(names)), 'duplicate model names in the SDF'
    for ramp in spec.ramps:
        assert ramp.name in names
    for obs in spec.dynamics:
        assert obs.name in names


def test_dynamic_obstacles_expose_a_cmd_vel_interface(spec):
    sdf = wb.render_sdf(spec)
    for obs in spec.dynamics:
        assert f'/obstacles/{obs.name}' in sdf, (
            f'{obs.name} has no namespaced planar-move interface, so the '
            'driver node cannot move it'
        )


def test_waypoint_yaml_covers_the_documented_goals(spec):
    import yaml

    parsed = yaml.safe_load(wb.render_waypoints(spec))
    assert set(parsed['waypoints']) == {w.name for w in spec.waypoints}
    for name in ('heavy_storage', 'packing_bay_4', 'mezzanine_storage'):
        assert name in parsed['waypoints']
