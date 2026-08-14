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

import itertools
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


#: Gradients the documentation quotes. Warehouse AMR ramps are built at 5-8%;
#: 10.5% is already the aggressive end of what a differential-drive unit with
#: trailing casters holds under load, and the steep maintenance ramp at 15.9%
#: is deliberately near the limit so the slope cost has something to price.
#: The earlier 26.5% maintenance ramp was not climbable at all, which made the
#: "ramps are the only viable path" scenario untestable. See DESIGN_NOTES 8e.
EXPECTED_SLOPES_DEG = {
    'hump_ramp_west': 6.0,
    'hump_ramp_east': 6.0,
    'mezz_ramp_gentle': 5.0,
    'mezz_ramp_steep': 9.0,
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
    #     value belongs to that cell's centre. On the steepest 9 degree ramp,
    #     half a cell of horizontal offset is 0.5 * 0.05 * tan(9 deg) of height.
    tolerance = lsb + 0.5 * RESOLUTION * math.tan(math.radians(9.0))

    probes = [
        # (x, y, expected height, description)
        (-18.0, 2.2, 0.0, 'west dock, ground level'),
        (0.0, 3.6, 0.38, 'hump bridge deck'),
        (-3.41, 3.6, 0.19, 'mid-point of the west hump ramp'),
        (8.0, 11.7, 0.40, 'mezzanine deck'),
        (8.0, 6.365, 0.20, 'mid-point of the gentle mezzanine ramp'),
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

    # Seal both ramp mouths. Gentle: x in [6.5, 9.5] rising from y = 4.08 to
    # the deck edge at y = 8.65. Steep: x in [10.8, 13.2] from y = 6.12.
    sealed = _rect_block(spec, occupancy, 6.3, 9.7, 3.9, 8.75)
    sealed |= _rect_block(spec, occupancy, 10.6, 13.4, 5.9, 8.75)

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
        """Distance to the nearest thing these obstacles cannot drive over.

        That includes the ramps and raised decks, not just the racks. These
        models are driven by ``libgazebo_ros_planar_move``, which has no
        ability to climb: a leg that clips a ramp toe wedges the obstacle there
        for the rest of the run. Two loops used to do exactly that.
        """
        nearest = float('inf')
        for box in spec.boxes:
            if box.yaw:
                continue          # yawed bodies are handled by the grid check
            dx = max(abs(x - box.x) - box.size_x / 2.0, 0.0)
            dy = max(abs(y - box.y) - box.size_y / 2.0, 0.0)
            nearest = min(nearest, math.hypot(dx, dy))
        for deck in spec.decks:
            dx = max(deck.x_min - x, x - deck.x_max, 0.0)
            dy = max(deck.y_min - y, y - deck.y_max, 0.0)
            nearest = min(nearest, math.hypot(dx, dy))
        for ramp in spec.ramps:
            low, high = sorted((ramp.start, ramp.end))
            if ramp.axis == 'x':
                dx = max(low - x, x - high, 0.0)
                dy = max(abs(y - ramp.cross) - ramp.width / 2.0, 0.0)
            else:
                dy = max(low - y, y - high, 0.0)
                dx = max(abs(x - ramp.cross) - ramp.width / 2.0, 0.0)
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

    assert params['obstacle_names'] == [
        o.name for o in spec.dynamics if o.shape != 'human'
    ], 'the driver must not be handed the scripted actors'
    for obs in spec.dynamics:
        entry = params[obs.name]
        assert entry['speed'] == pytest.approx(obs.speed)
        assert entry['kind'] == obs.shape
        flat = [coordinate for point in obs.waypoints for coordinate in point]
        assert entry['waypoints'] == pytest.approx(flat)


def _loop_separation(first, second, step=0.10):
    """Smallest distance between two polylines. `second` may be an open path."""
    def samples(points, closed=True):
        pts = [tuple(p) for p in points]
        pairs = zip(pts, pts[1:] + pts[:1]) if closed else zip(pts, pts[1:])
        out = []
        for a, b in pairs:
            count = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
            out += [(a[0] + i / count * (b[0] - a[0]),
                     a[1] + i / count * (b[1] - a[1])) for i in range(count + 1)]
        return out

    left = samples(first)
    right = samples(second, closed=len(second) > 2 and second[0] != second[-1]
                    and not isinstance(second, list))
    right = samples(second, closed=False) if len(second) == 2 else samples(second)
    return min(math.hypot(p[0] - q[0], p[1] - q[1]) for p in left for q in right)


def test_no_two_patrol_loops_can_ever_meet(spec):
    """Obstacles must not collide with each other.

    Enforced geometrically, not by schedule. Two loops that merely miss each
    other on the current timing will eventually meet, because the driver
    applies per-agent speed jitter and dwell at corners -- so any argument that
    depends on them staying in step is an argument with a half-life. Requiring
    the loops to be spatially disjoint by the sum of the two radii plus a
    margin removes the timing question entirely.
    """
    margin = 0.30
    for first, second in itertools.combinations(spec.dynamics, 2):
        separation = _loop_separation(first.waypoints, second.waypoints)
        required = first.radius + second.radius + margin
        assert separation >= required, (
            f'{first.name} and {second.name} come within {separation:.2f} m of '
            f'each other, closer than their combined radii plus {margin} m '
            f'({required:.2f} m). They will collide sooner or later, and two '
            f'obstacles locked together is not a dynamic obstacle.'
        )


def test_every_obstacle_obstructs_at_least_one_amr_route(spec):
    """A dynamic obstacle nobody has to avoid is scenery.

    The brief wants the fleet negotiating traffic, so each loop has to cross or
    graze a route the AMRs actually drive.
    """
    named = {w.name: w for w in spec.waypoints}
    routes = [
        # AMR-1's laden run south to Heavy Storage.
        [(named['dock_a'].x, named['dock_a'].y),
         (named['heavy_storage'].x, named['heavy_storage'].y)],
        # AMR-2 crossing the whole warehouse through The Pinch.
        [(named['dock_b'].x, named['dock_b'].y), (named['pinch_west'].x, 0.0),
         (named['pinch_east'].x, 0.0),
         (named['packing_bay_4'].x, named['packing_bay_4'].y)],
        # The slope demo's west-east transit.
        [(named['west_staging'].x, named['west_staging'].y),
         (named['east_staging'].x, named['east_staging'].y)],
        # The north-west mapping sweep. This is a route, not an invention: the
        # brief requires the aisle layout to be *initially unknown*, so the
        # fleet has to drive those aisles to build the map at all. An obstacle
        # patrolling there is traffic the mapper meets, which is the whole
        # point of putting one in a rack block rather than in open floor.
        [(named['dock_a'].x, named['dock_a'].y), (-16.5, 9.0), (-12.0, 9.0)],
        # The north-south corridor west of the firewall -- the only route
        # between the south aisles and the mezzanine approach, and part of the
        # same initially-unknown layout the fleet has to map.
        [(named['dock_a'].x, named['dock_a'].y), (-4.0, 3.0), (-4.0, 12.0)],
    ]
    reach = 3.0
    for obs in spec.dynamics:
        best = min(_loop_separation(obs.waypoints, route) for route in routes)
        assert best <= reach, (
            f'{obs.name} never comes within {reach} m of any AMR route '
            f'(closest {best:.2f} m); it would never be sensed or avoided'
        )


def test_pedestrians_are_animated_actors_the_lidar_can_still_see(spec):
    """Both halves matter, and they pull against each other.

    An `<actor>` is what gives a running animation, and it is *kinematic*: it
    follows its script regardless of contacts, so it cannot wedge itself
    against a rack the way a physics-driven model can. That is the fix for
    obstacles getting stuck.

    What an actor does not give you is a body a LiDAR returns from. The
    reference warehouse world shows the trap: its nine actors carry nothing but
    Gazebo's auto-generated 2 cm bone spheres, so a scan sweeps straight
    through them. Collisions therefore have to be declared, and declared at
    *scan height* -- both robot models scan between 0.48 m and 0.60 m, which is
    knee height on a 1.8 m skeleton.
    """
    sdf = wb.render_sdf(spec)
    humans = [o for o in spec.dynamics if o.shape == 'human']
    assert humans, 'expected the pedestrians to be human actors'

    scan_heights = [0.48, 0.60]
    for obs in humans:
        block = sdf[sdf.index(f'<actor name="{obs.name}">'):]
        block = block[:block.index('</actor>')]

        assert 'run.dae' in block, f'{obs.name} has no running animation'
        assert '<skin>' in block and '<animation' in block
        assert '<trajectory' in block, f'{obs.name} has no scripted trajectory'
        assert '<collision' in block, (
            f'{obs.name} is a bare actor, so the LiDAR would sweep through it'
        )

    # Every declared bone collision must straddle the scan plane, or the
    # pedestrians are invisible to exactly the sensor that matters.
    knees = {'LeftLeg', 'RightLeg'}
    assert knees <= set(wb.HUMAN_BONE_COLLISIONS), (
        'the knees are the bones that sit in the scan plane; collisions on the '
        'torso alone would be swept under'
    )
    for height in scan_heights:
        radius = wb.HUMAN_BONE_COLLISIONS['LeftLeg']
        assert abs(height - wb.HUMAN_KNEE_HEIGHT) <= radius, (
            f'a scan at {height} m misses a {radius} m sphere centred on a knee '
            f'at {wb.HUMAN_KNEE_HEIGHT} m'
        )


def test_actor_trajectories_have_strictly_increasing_timestamps(spec):
    """A repeated timestamp is a zero-length interval: the actor stalls there.

    Easy to produce, because the arrival at the end of one leg and the start of
    the next leg's turn are the same instant, and emitting both is the obvious
    way to write the loop.
    """
    import re as _re

    sdf = wb.render_sdf(spec)
    for obs in (o for o in spec.dynamics if o.shape == 'human'):
        block = sdf[sdf.index(f'<actor name="{obs.name}">'):]
        block = block[:block.index('</actor>')]
        times = [float(t) for t in _re.findall(r'<time>([\d.]+)</time>', block)]
        assert len(times) >= 8, f'{obs.name} has too few waypoints to be a loop'
        assert all(b > a for a, b in zip(times, times[1:])), (
            f'{obs.name} repeats a timestamp; Gazebo would stall or jump there'
        )


def test_actors_run_at_the_speed_they_declare(spec):
    """The script is the only thing setting their pace, so it has to be right."""
    import re as _re

    sdf = wb.render_sdf(spec)
    for obs in (o for o in spec.dynamics if o.shape == 'human'):
        block = sdf[sdf.index(f'<actor name="{obs.name}">'):]
        block = block[:block.index('</actor>')]
        entries = _re.findall(
            r'<time>([\d.]+)</time><pose>([-\d.]+) ([-\d.]+)', block)

        travelled = 0.0
        for (t0, x0, y0), (t1, x1, y1) in zip(entries, entries[1:]):
            step = math.hypot(float(x1) - float(x0), float(y1) - float(y0))
            if step < 1e-6:
                continue                       # a pivot on the spot
            travelled += step
            measured = step / (float(t1) - float(t0))
            assert measured == pytest.approx(obs.speed, abs=1e-3), (
                f'{obs.name} covers a leg at {measured:.2f} m/s, not its '
                f'declared {obs.speed} m/s'
            )

        perimeter = sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(list(obs.waypoints),
                            list(obs.waypoints)[1:] + list(obs.waypoints)[:1]))
        assert travelled == pytest.approx(perimeter, abs=1e-3)


#: How close a patrol loop may come to a robot's dock [m]. An obstacle loitering
#: on a spawn point does not test avoidance, it tests whether the robot can get
#: out of bed. One pedestrian used to pass 0.50 m from dock_b.
DOCK_STANDOFF = 3.5

#: How close a patrol loop may come to the centre of The Pinch [m]. The doorway
#: is 2.0 m wide and both AMRs have to negotiate it in opposite directions --
#: that is already the hardest moment in the scenario. Parking traffic in it
#: converts a negotiation into a blockage, and a blockage proves nothing.
PINCH_STANDOFF = 4.5


def test_no_obstacle_loiters_at_a_dock(spec):
    named = {w.name: w for w in spec.waypoints}
    docks = [named['dock_a'], named['dock_b']]
    for obs in spec.dynamics:
        for dock in docks:
            distance = _loop_separation(obs.waypoints, [(dock.x, dock.y),
                                                        (dock.x, dock.y)])
            assert distance >= DOCK_STANDOFF, (
                f'{obs.name} passes {distance:.2f} m from {dock.name}, inside '
                f'the {DOCK_STANDOFF} m standoff. A robot should not have to '
                f'negotiate traffic before it has left its charging plate.'
            )


def test_no_obstacle_blocks_the_pinch(spec):
    """The doorway has to stay negotiable, not merely passable."""
    for obs in spec.dynamics:
        distance = _loop_separation(obs.waypoints, [(0.0, 0.0), (0.0, 0.0)])
        assert distance >= PINCH_STANDOFF, (
            f'{obs.name} comes within {distance:.2f} m of The Pinch, inside the '
            f'{PINCH_STANDOFF} m standoff. Two AMRs meeting in a 2.0 m doorway '
            f'is the scenario; a third body in it is just a wall.'
        )


def test_the_pinch_doorway_is_unobstructed(spec):
    """Nothing may protrude into the gap the firewall leaves.

    The doorway used to carry two 1.2 m yellow jambs as legibility markers.
    They stuck out 0.1 m past each face of the wall, so the LiDAR returned them
    as obstacles sitting at the mouth of the gap and the inflation layer
    narrowed the corridor below the 2.0 m the firewall actually leaves.
    """
    walls = [b for b in spec.boxes if not b.yaw and abs(b.x) < 1.0]
    for box in walls:
        spans_gap = (box.y - box.size_y / 2.0) < 1.0 and (box.y + box.size_y / 2.0) > -1.0
        assert not spans_gap, (
            f'{box.name} intrudes into the doorway between y = -1.0 and y = 1.0'
        )

    # And the gap really is 2.0 m of clear floor.
    for y in (-0.9, -0.45, 0.0, 0.45, 0.9):
        assert not _obstacle_grid(spec)[_to_cell(spec, 0.0, y)], (
            f'the doorway is blocked at (0.0, {y})'
        )


#: Width of the widest AMR [m], from robot_models.yaml's footprint_radius.
AMR_WIDTH = 1.10


def test_rack_rows_have_no_gaps_a_robot_would_try_to_enter(spec):
    """A hole narrower than the robot is a trap, not a cross-aisle.

    The rows used to be built from 3.6 m segments with 0.9 m gaps. The AMRs are
    1.10 m wide: too wide to fit, but the gap is wide enough that the inflated
    costmap does not close it outright, so the planner would route through one,
    the robot would commit, and it would wedge between two racks.

    Either a gap is comfortably drivable or it should not exist. These rows are
    now continuous, so an aisle is entered around the end of a row.
    """
    from collections import defaultdict

    rows = defaultdict(list)
    for box in spec.boxes:
        if '_rack_' in box.name and not box.yaw:
            rows[(box.name.split('_rack_')[0], round(box.y, 3))].append(box)

    assert rows, 'expected some rack rows'
    for (prefix, y), boxes in rows.items():
        boxes.sort(key=lambda b: b.x)
        for left, right in zip(boxes, boxes[1:]):
            gap = (right.x - right.size_x / 2.0) - (left.x + left.size_x / 2.0)
            assert gap <= 1e-6 or gap >= AMR_WIDTH + 0.8, (
                f'{prefix} row at y={y} has a {gap:.2f} m gap between segments. '
                f'That is impassable for a {AMR_WIDTH} m robot but not obviously '
                f'closed to the planner, which is how a robot ends up wedged '
                f'between two racks.'
            )


def test_the_heavy_storage_goal_sits_between_exactly_two_rows(spec):
    """The goal is in an aisle, flanked by one row on each side."""
    goal = {w.name: w for w in spec.waypoints}['heavy_storage']
    rows = sorted({round(b.y, 3) for b in spec.boxes
                   if b.name.startswith('heavy_rack_')})

    assert len(rows) == 2, (
        f'the heavy block has {len(rows)} rows; the goal is supposed to lie '
        f'between two of them, not inside a deeper block'
    )
    below = [y for y in rows if y < goal.y]
    above = [y for y in rows if y > goal.y]
    assert below and above, (
        f'heavy_storage at y={goal.y} is not between the rows at {rows}'
    )

    # And the aisle is actually drivable.
    occupancy = _obstacle_grid(spec)
    assert not occupancy[_to_cell(spec, goal.x, goal.y)]
    half = (min(above) - max(below)) / 2.0 - 0.5      # 0.5 = half a rack depth
    assert half >= AMR_WIDTH / 2.0 + 0.3, (
        f'the aisle is {2 * half:.2f} m wide, too tight for a {AMR_WIDTH} m robot'
    )


def test_the_heavy_aisle_has_one_mouth(spec):
    """Sealed at the west wall, so reaching the goal means going round.

    That is the point of the change: the route choice becomes *which end of the
    block*, rather than a false choice between a dozen gaps that cannot be
    driven.
    """
    goal = {w.name: w for w in spec.waypoints}['heavy_storage']
    occupancy = _obstacle_grid(spec)

    west_end = min(b.x - b.size_x / 2.0 for b in spec.boxes
                   if b.name.startswith('heavy_rack_'))
    wall_face = spec.x_min + spec.wall_thickness
    assert west_end - wall_face < AMR_WIDTH, (
        f'the west end of the heavy block leaves {west_end - wall_face:.2f} m '
        f'to the wall, which a {AMR_WIDTH} m robot could squeeze through'
    )

    east_end = max(b.x + b.size_x / 2.0 for b in spec.boxes
                   if b.name.startswith('heavy_rack_'))
    for offset in (0.6, 1.2, 2.0):
        assert not occupancy[_to_cell(spec, east_end + offset, goal.y)], (
            f'the east mouth of the heavy aisle is blocked {offset} m out'
        )


#: nav2's inflation radius [m], from nav2_params.yaml. Cost is applied this far
#: out from every obstacle face, so an aisle narrower than
#: 2 * INFLATION_RADIUS + robot width has no cost-free lane down the middle.
INFLATION_RADIUS = 0.75

#: Minimum clear space two ramps must leave between them [m]. A robot leaving
#: one slope has to be able to stand somewhere level before committing to the
#: next.
RAMP_SEPARATION = 1.0


def test_aisles_have_a_cost_free_lane_for_the_widest_robot(spec):
    """A 2.4 m aisle was wide enough to fit and too narrow to plan through.

    The robot fits geometrically -- 1.10 m in 2.40 m -- so every clearance
    check passed. But nav2 inflates 0.75 m from each rack face, which left a
    0.90 m band of zero-cost floor for a 1.10 m robot. There was no position in
    the aisle that did not cost something, so the planner treated the whole
    aisle as expensive and the robot wandered into the shelving looking for
    somewhere cheaper.

    Geometric fit is not the test. A cost-free lane is.
    """
    from collections import defaultdict

    widest = 1.10                                     # heavy_mapper, 2 * 0.55
    depth = 1.0                                       # rack depth

    blocks = defaultdict(set)
    for box in spec.boxes:
        if '_rack_' in box.name and not box.yaw:
            blocks[box.name.split('_rack_')[0]].add(round(box.y, 3))

    assert blocks, 'expected some rack blocks'
    for prefix, rows in blocks.items():
        ordered = sorted(rows)
        for lower, upper in zip(ordered, ordered[1:]):
            aisle = (upper - lower) - depth
            lane = aisle - 2 * INFLATION_RADIUS
            assert lane >= widest, (
                f'{prefix}: rows at y={lower} and y={upper} leave a {aisle:.2f} m '
                f'aisle, so after {INFLATION_RADIUS} m of inflation from each '
                f'face the cost-free lane is {lane:.2f} m. The widest robot is '
                f'{widest} m: it would never find a zero-cost pose in there.'
            )


def test_no_two_ramps_run_close_enough_to_trap_a_robot(spec):
    """A robot coming off one slope needs level ground before the next.

    The mezzanine's gentle ramp and the Hump Bridge's east ramp used to be
    0.28 m apart -- two slopes falling in different directions with a gap
    narrower than a wheel between them. A robot could climb one and then have
    nowhere to put itself to turn, which is exactly how AMR-2 got up the
    mezzanine and could not come back down.
    """
    def extent(ramp):
        low, high = sorted((ramp.start, ramp.end))
        if ramp.axis == 'x':
            return (low, high,
                    ramp.cross - ramp.width / 2.0, ramp.cross + ramp.width / 2.0)
        return (ramp.cross - ramp.width / 2.0, ramp.cross + ramp.width / 2.0,
                low, high)

    for first, second in itertools.combinations(spec.ramps, 2):
        ax0, ax1, ay0, ay1 = extent(first)
        bx0, bx1, by0, by1 = extent(second)
        dx = max(bx0 - ax1, ax0 - bx1, 0.0)
        dy = max(by0 - ay1, ay0 - by1, 0.0)
        gap = math.hypot(dx, dy)
        assert gap >= RAMP_SEPARATION, (
            f'{first.name} and {second.name} are {gap:.2f} m apart, less than '
            f'the {RAMP_SEPARATION} m a robot needs to stand level between '
            f'them. Two slopes that close form a trap, not a junction.'
        )


def test_the_mezzanine_backs_onto_the_north_wall(spec):
    """No dead strip behind the deck, and the ramps sit as far north as they can.

    Every metre the deck is short of the wall pushes both of its ramp feet a
    metre further south, into the Hump Bridge's approaches.
    """
    deck = next(d for d in spec.decks if d.name == 'mezzanine_deck')
    inner_face = spec.y_max - spec.wall_thickness
    assert abs(deck.y_max - inner_face) < 1e-6, (
        f'the mezzanine ends at y={deck.y_max} but the wall\'s inner face is at '
        f'y={inner_face}; the {inner_face - deck.y_max:.2f} m strip behind it is '
        f'floor no robot can reach and it costs the ramps the same distance'
    )


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
    names += [a.get('name') for a in world.findall('actor')]
    assert len(names) == len(set(names)), 'duplicate entity names in the SDF'
    for ramp in spec.ramps:
        assert ramp.name in names
    for obs in spec.dynamics:
        assert obs.name in names


def test_dynamic_obstacles_expose_a_cmd_vel_interface(spec):
    sdf = wb.render_sdf(spec)
    driven = [o for o in spec.dynamics if o.shape != 'human']
    assert driven, 'expected at least one driven obstacle'
    for obs in driven:
        assert f'/obstacles/{obs.name}' in sdf, (
            f'{obs.name} has no namespaced planar-move interface, so the '
            'driver node cannot move it'
        )
    # Pedestrians are scripted actors. They must NOT carry a velocity
    # interface: something steering them at runtime is what let them jam.
    for obs in spec.dynamics:
        if obs.shape == 'human':
            assert f'/obstacles/{obs.name}' not in sdf, (
                f'{obs.name} is an animated actor; it is scripted, not driven'
            )


def test_waypoint_yaml_covers_the_documented_goals(spec):
    import yaml

    parsed = yaml.safe_load(wb.render_waypoints(spec))
    assert set(parsed['waypoints']) == {w.name for w in spec.waypoints}
    for name in ('heavy_storage', 'packing_bay_4', 'mezzanine_storage'):
        assert name in parsed['waypoints']
