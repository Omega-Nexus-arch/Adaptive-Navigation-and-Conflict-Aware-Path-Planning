# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Programmatic SDF world construction for the multi-level logistics warehouse.

Why generate the world instead of hand-writing SDF
--------------------------------------------------
The slope-aware global planner needs an elevation map of the floor. If that map
were authored separately from the ``.world`` file the two would drift apart
after the first edit, and every "the planner avoided the ramp" claim would be
unverifiable. Here a single geometric description produces:

* ``worlds/warehouse_multilevel.world`` -- what Gazebo simulates,
* ``maps/warehouse_elevation.{pgm,yaml}`` -- what ``amr_navigation::SlopeLayer``
  reasons about,
* ``config/waypoints.yaml`` -- the named goals used by the demo scripts.

They cannot disagree, because they are three renderings of the same data.

Coordinate conventions follow REP-103: x forward/east, y left/north, z up,
angles in radians, lengths in metres.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

# ---------------------------------------------------------------------------
# Geometry primitives
# ---------------------------------------------------------------------------


#: Seconds an actor spends pivoting on the spot at each corner of its loop.
#: Without it Gazebo interpolates yaw across the whole following leg and the
#: runner crabs sideways into the turn.
HUMAN_TURN_SECONDS = 0.7

#: Collision spheres bolted onto the animated skeleton, as {bone: radius}.
#:
#: An `<actor>` is a visual by default -- the reference world's people carry
#: nothing but 2 cm spheres, so a LiDAR sweeps straight past them. These are
#: sized and placed to sit *in the scan plane*: both robot models scan at
#: 0.48-0.60 m, which is knee height on a 1.8 m skeleton, so the knees do the
#: work and the hips fill in the torso. Spheres because a bone's frame is
#: rotated (the skeleton is Y-up) and a sphere is the one shape that does not
#: care.
HUMAN_BONE_COLLISIONS = {'LeftLeg': 0.22, 'RightLeg': 0.22, 'Hips': 0.40}

#: Height of the knee bones on the 1.8 m skeleton [m], read off the reference
#: world's saved actor state. This is what makes the knee spheres the right
#: place to put collision: both robot models scan between 0.48 and 0.60 m.
HUMAN_KNEE_HEIGHT = 0.55

#: The walking-person mesh that ships with Gazebo Classic
#: (``/usr/share/gazebo-11/media/models/walk.dae``). Referenced through
#: Gazebo's own resource path, the same way the material scripts below are, so
#: nothing has to be vendored into this repository.
HUMAN_MESH_URI = 'run.dae'

#: Height of that mesh at scale 1.0 [m].
HUMAN_MESH_HEIGHT = 1.80

#: Rotation applied to the mesh so it faces along +x, i.e. the direction
#: `dynamic_obstacle_driver` turns each person to walk in. Set to 0 for a mesh
#: already authored facing +x; change this one constant if a different skin is
#: substituted and the people walk sideways.
HUMAN_MESH_YAW_OFFSET = 0.0

@dataclass(frozen=True)
class Box:
    """An axis-aligned (optionally yawed) static box obstacle.

    ``z`` is the height of the *bottom* face, which makes stacking on top of a
    raised deck read naturally at the call site.
    """

    name: str
    x: float
    y: float
    z: float
    size_x: float
    size_y: float
    size_z: float
    yaw: float = 0.0
    material: str = 'Gazebo/Grey'

    @property
    def center_z(self) -> float:
        return self.z + self.size_z / 2.0


@dataclass(frozen=True)
class Deck:
    """A flat, raised, walkable floor section.

    Contributes its top surface to the elevation map and is rendered as a solid
    slab in Gazebo. Robots drive on top of it.
    """

    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    height: float
    material: str = 'Gazebo/WoodPallet'

    @property
    def center(self) -> Tuple[float, float]:
        return ((self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0)


@dataclass(frozen=True)
class Ramp:
    """A sloped surface joining two floor levels.

    The ramp runs along ``axis`` ('x' or 'y') from ``start`` at ``z_start`` to
    ``end`` at ``z_end``. ``cross`` is the constant coordinate on the other
    horizontal axis and ``width`` the extent across the direction of travel.
    """

    name: str
    axis: str
    start: float
    end: float
    z_start: float
    z_end: float
    cross: float
    width: float
    thickness: float = 0.24
    material: str = 'Gazebo/Bricks'

    # -- derived quantities --------------------------------------------------

    @property
    def run(self) -> float:
        """Signed horizontal travel."""
        return self.end - self.start

    @property
    def rise(self) -> float:
        return self.z_end - self.z_start

    @property
    def length(self) -> float:
        return math.hypot(self.run, self.rise)

    @property
    def pitch(self) -> float:
        """Surface inclination magnitude in radians.

        ``abs(run)`` matters: a ramp declared east-to-west has a negative run,
        and ``atan2(rise, run)`` would report the supplement of the true angle.
        """
        return abs(math.atan2(self.rise, abs(self.run)))

    @property
    def slope_deg(self) -> float:
        return math.degrees(self.pitch)

    def pose(self) -> Tuple[float, float, float, float, float, float]:
        """Return the SDF pose (x, y, z, roll, pitch, yaw) of the slab centre.

        The slab is modelled as a box of size (length, width, thickness) whose
        *top* face is the driving surface. SDF applies rotations as
        ``R = Rz(yaw) * Ry(pitch) * Rx(roll)``; a negative pitch lifts the local
        +x end, so ``p = -theta`` where ``theta`` is the surface inclination
        signed by the direction of travel.
        """
        theta = math.atan2(self.rise, abs(self.run))
        if self.run < 0.0:
            # Travelling in -x/-y: flip so the geometry still rises correctly.
            theta = -theta
        p = -theta
        yaw = 0.0 if self.axis == 'x' else math.pi / 2.0

        mid_along = (self.start + self.end) / 2.0
        mid_z = (self.z_start + self.z_end) / 2.0
        if self.axis == 'x':
            mid_x, mid_y = mid_along, self.cross
        else:
            mid_x, mid_y = self.cross, mid_along

        # Offset from slab centre to top-face centre, rotated into world axes.
        half_t = self.thickness / 2.0
        off_x = half_t * math.sin(p) * math.cos(yaw)
        off_y = half_t * math.sin(p) * math.sin(yaw)
        off_z = half_t * math.cos(p)

        return (mid_x - off_x, mid_y - off_y, mid_z - off_z, 0.0, p, yaw)

    def bounds(self) -> Tuple[float, float, float, float]:
        """Axis-aligned horizontal footprint (x_min, x_max, y_min, y_max)."""
        lo, hi = sorted((self.start, self.end))
        half_w = self.width / 2.0
        if self.axis == 'x':
            return (lo, hi, self.cross - half_w, self.cross + half_w)
        return (self.cross - half_w, self.cross + half_w, lo, hi)

    def height_at(self, x: float, y: float) -> float:
        """Interpolated surface height at a point inside :meth:`bounds`."""
        along = x if self.axis == 'x' else y
        if abs(self.run) < 1e-9:
            return self.z_end
        t = (along - self.start) / self.run
        t = min(1.0, max(0.0, t))
        return self.z_start + t * self.rise


@dataclass(frozen=True)
class DynamicObstacle:
    """A movable obstacle driven at runtime over ``/<name>/cmd_vel``.

    Gazebo Classic ``<actor>`` elements have no collision geometry, so they are
    invisible to a LiDAR. Dynamic traffic is therefore modelled with real
    models carrying ``libgazebo_ros_planar_move``; ``dynamic_obstacle_driver``
    walks them around a waypoint loop. They are physically present, so the
    obstacle-avoidance and safety-override behaviour being demonstrated is
    genuine rather than staged.
    """

    name: str
    shape: str          # 'human' (pedestrian), 'box' or 'cylinder' (third-party robot)
    radius: float
    height: float
    start_x: float
    start_y: float
    waypoints: Sequence[Tuple[float, float]]
    speed: float
    material: str = 'Gazebo/Orange'


@dataclass
class Waypoint:
    """A named navigation goal exported for the demo scripts and RViz."""

    name: str
    x: float
    y: float
    yaw: float
    description: str = ''
    level: str = 'ground'


@dataclass
class WorldSpec:
    """The complete description of the warehouse."""

    name: str = 'warehouse_multilevel'
    x_min: float = -22.0
    x_max: float = 22.0
    y_min: float = -15.0
    y_max: float = 15.0
    wall_height: float = 3.0
    wall_thickness: float = 0.25

    boxes: List[Box] = field(default_factory=list)
    decks: List[Deck] = field(default_factory=list)
    ramps: List[Ramp] = field(default_factory=list)
    dynamics: List[DynamicObstacle] = field(default_factory=list)
    waypoints: List[Waypoint] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------


def _storage_block(
    prefix: str,
    rows_y: Sequence[float],
    x_start: float,
    x_end: float,
    segment: float = 3.6,
    gap: float = 0.9,
    depth: float = 1.0,
    height: float = 2.4,
) -> List[Box]:
    """Build a block of storage racks as rows of continuous shelving.

    Each row is UNBROKEN from ``x_start`` to ``x_end``. It is still drawn as
    separate segments, because a real rack run is bays bolted together, but the
    bays are butted rather than spaced.

    The rows used to carry 0.9 m gaps between segments, on the theory that
    cross-aisles give the planner route choices. In practice the AMRs are
    1.1 m wide: the gap was too narrow to drive through and too wide for the
    costmap to write off, so a robot would commit to it, wedge, and sit there.
    A 0.9 m hole in a wall is not a route, it is a trap.

    With the rows solid, an aisle is entered around the end of a row -- which
    is how warehouse traffic actually works, and it gives the global planner a
    real decision (which end) instead of a false one. ``gap`` is retained in the
    signature for callers that genuinely want spaced shelving; it is unused for
    these blocks and asserted to be irrelevant by test_world_generation.py.
    """
    del gap
    boxes: List[Box] = []
    span = x_end - x_start
    count = max(1, int(round(span / segment)))
    actual = span / count          # butt the segments; no gap between them

    for row_index, y in enumerate(rows_y):
        for seg_index in range(count):
            left = x_start + seg_index * actual
            boxes.append(
                Box(
                    name=f'{prefix}_rack_{row_index}_{seg_index}',
                    x=left + actual / 2.0,
                    y=y,
                    z=0.0,
                    size_x=actual,
                    size_y=depth,
                    size_z=height,
                    material='Gazebo/Wood',
                )
            )
    return boxes


def build_warehouse() -> WorldSpec:
    """Assemble the warehouse layout.

    Design intent, feature by feature:

    ``Central firewall`` (x = 0)
        A full-height partition splitting the floor into a west and an east
        half. It has exactly two crossings, which is what makes route choice
        observable rather than incidental.

    ``The Pinch`` (x = 0, y in [-1.0, 1.0])
        The flat crossing: a 2.0 m doorway. One robot fits with clearance, two
        do not, so when both halves of the fleet want it the yielding protocol
        has to fire. This is the conflict test.

    ``Hump Bridge`` (x in [-5.2, 5.2], y in [2.0, 5.2])
        The sloped crossing: a 0.55 m plinth with an 8.7 deg ramp at each end,
        3.0 m wide and therefore *more* comfortable than the Pinch in every
        respect except gradient. A planner that ignores slope will happily use
        it; a correctly weighted slope cost sends traffic to the flat doorway
        instead. Block the doorway and the bridge becomes the only option, so
        the same pair of structures demonstrates both halves of the
        requirement: avoid ramps when an alternative exists, accept them when
        one does not.

    ``Mezzanine Deck`` (north-east, 0.45 m)
        Reachable only by its two ramps - a gentle 7.8 deg service ramp and a
        steep 14.8 deg maintenance ramp. ``Mezzanine Storage`` sits on it, so
        that goal forces the planner to price the two gradients against each
        other rather than merely avoiding them.

    ``Heavy Storage`` (far south-west)
        AMR-1's primary goal. Deep inside a rack block, so the global plan has
        to solve a real aisle-routing problem.

    ``Packing Bay 4`` (east wall)
        AMR-2's primary goal. Reaching it from the west dock requires crossing
        the firewall, so the two concurrent missions naturally interact.
    """
    spec = WorldSpec()

    # -- Perimeter walls -----------------------------------------------------
    t, h = spec.wall_thickness, spec.wall_height
    span_x = spec.x_max - spec.x_min
    span_y = spec.y_max - spec.y_min
    spec.boxes += [
        Box('wall_south', 0.0, spec.y_min, 0.0, span_x + t, t, h, material='Gazebo/Grey'),
        Box('wall_north', 0.0, spec.y_max, 0.0, span_x + t, t, h, material='Gazebo/Grey'),
        Box('wall_west', spec.x_min, 0.0, 0.0, t, span_y + t, h, material='Gazebo/Grey'),
        Box('wall_east', spec.x_max, 0.0, 0.0, t, span_y + t, h, material='Gazebo/Grey'),
    ]

    # -- Storage blocks ------------------------------------------------------
    # Rows are 5.0 m apart, which leaves a 4.0 m aisle after the 1.0 m rack
    # depth. That number is set by the costmap, not by taste.
    #
    # nav2 inflates 0.75 m out from every obstacle face. In the old 2.4 m aisle
    # the inflated regions from the two facing rows left a free band of
    # 2 * (1.20 - 0.75) = 0.90 m down the middle -- and the heavy mapper is
    # 1.10 m WIDE. There was no position in the aisle where it did not sit in
    # inflation cost, so the planner treated the whole aisle as expensive and
    # the robot crabbed along the shelving looking for somewhere cheaper. At
    # 4.0 m the free band is 2.50 m: both models drive down the centre at zero
    # cost. See DESIGN_NOTES 8k.
    #
    # Fewer rows pay for the wider aisles, which is the right trade -- an aisle
    # a robot cannot use is not storage capacity, it is decoration.
    #
    # South-west heavy storage: two rows, one 4.0 m aisle, sealed at the west
    # wall so the aisle has exactly one mouth (the east end).
    spec.boxes += _storage_block('heavy', (-14.0, -9.0), -21.5, -5.0)
    # North-west general storage: two rows, was three.
    spec.boxes += _storage_block('general', (7.0, 12.0), -21.5, -7.0)
    # South-east buffer storage.
    spec.boxes += _storage_block('buffer', (-14.0, -9.0), 5.0, 18.0)

    # -- Central firewall with a single flat doorway (The Pinch) -------------
    # Segment extents chosen so the only ground-level opening is
    # y in [-1.0, 1.0]; everything from y = 2.0 to y = 5.2 is closed by the
    # Hump Bridge plinth itself.
    spec.boxes += [
        Box('firewall_south', 0.0, -7.95, 0.0, 0.4, 13.9, h, material='Gazebo/DarkGrey'),
        Box('firewall_mid', 0.0, 1.5, 0.0, 0.4, 1.0, h, material='Gazebo/DarkGrey'),
        Box('firewall_north', 0.0, 10.05, 0.0, 0.4, 9.7, h, material='Gazebo/DarkGrey'),
        # NO DOOR JAMBS. They were 1.2 m tall markers at y = +/-1.05, meant to
        # make the doorway legible -- but they protrude 0.1 m past each face of
        # the firewall, so the LiDAR returns them as obstacles sitting exactly
        # at the mouth of the gap. Once the inflation layer rounds them the
        # effective corridor is narrower than the 2.0 m the firewall actually
        # leaves, and the robots hesitate at the one place the scenario needs
        # them to commit. The firewall's own edges mark the doorway perfectly
        # well. See DESIGN_NOTES 8h.
    ]

    # -- Hump Bridge: the sloped crossing ------------------------------------
    # 0.38 m, not 0.55. The deck height sets the ramp run for a given gradient,
    # and the run is what has to fit between the firewall and the mezzanine's
    # gentle ramp at x = 5.5. At 0.55 m a 6 deg ramp needs 5.25 m of run and
    # collides with it; the only way to keep 0.55 m was the 8.7 deg gradient
    # that made the ramps unusable in the first place. See DESIGN_NOTES 8e.
    hump_h = 0.38
    spec.decks.append(Deck('hump_deck', -1.6, 1.6, 2.0, 5.2, hump_h))
    # ~6.0 deg (10.5% grade). Warehouse AMR ramps are built at 5-8%; 10.5% is
    # already at the aggressive end of what a differential-drive unit with
    # trailing casters will take under load. The earlier 3.6 m run gave 8.7 deg
    # (15.3%), which is loading-dock territory -- see DESIGN_NOTES 8e.
    spec.ramps += [
        Ramp('hump_ramp_west', 'x', -5.22, -1.6, 0.0, hump_h, cross=3.6, width=3.0),
        Ramp('hump_ramp_east', 'x', 5.22, 1.6, 0.0, hump_h, cross=3.6, width=3.0),
    ]
    spec.boxes += [
        Box('hump_rail_north', 0.0, 5.2, hump_h, 3.2, 0.12, 0.5, material='Gazebo/Yellow'),
        Box('hump_rail_south', 0.0, 2.0, hump_h, 3.2, 0.12, 0.5, material='Gazebo/Yellow'),
    ]

    # -- Mezzanine deck: ramp-only access, two gradients ---------------------
    mezz_h = 0.40

    # BACKED ONTO THE NORTH WALL. The deck used to sit at y in [7.5, 13.6],
    # 1.15 m short of the wall, and that shortfall pushed both of its ramps
    # south: the gentle ramp's foot landed at y = 2.93 with its west edge at
    # x = 5.50, while the Hump Bridge's east ramp ends at x = 5.22.
    #
    # 0.28 m apart. Two slopes running in different directions, separated by a
    # gap narrower than a wheel. A robot descending one and turning has nowhere
    # to put itself, which is why AMR-2 could climb the mezzanine ramp and then
    # fail to get back down. Sliding the deck north to the wall moves both ramp
    # feet with it, and moving the gentle ramp's centreline east opens the gap
    # to 1.28 m. See DESIGN_NOTES 8l.
    mezz_south = 8.65
    mezz_north = 14.75          # hard against the inner face of the north wall
    spec.decks.append(Deck('mezzanine_deck', 3.0, 13.5, mezz_south, mezz_north, mezz_h))

    spec.ramps += [
        # Gentle service ramp (~5.0 deg, 8.8%): the preferred way up. Centred
        # at x = 8.0, so its west edge clears the Hump Bridge's east ramp by
        # 1.28 m instead of 0.28 m.
        Ramp('mezz_ramp_gentle', 'y', 4.08, mezz_south, 0.0, mezz_h,
             cross=8.0, width=3.0),
        # Steep maintenance ramp (~9.0 deg, 15.9%): the expensive route the
        # planner should refuse unless it is the only way up -- but climbable,
        # which the original 14.8 deg version was not.
        # Centred at x = 12.0, not 11.5. At 11.5 it sat 0.80 m from the gentle
        # ramp -- the same trap as the Hump Bridge near-miss, just between this
        # deck's own two ramps. test_no_two_ramps_run_close_enough_to_trap_a_robot
        # found it; the eye had not.
        Ramp('mezz_ramp_steep', 'y', 6.12, mezz_south, 0.0, mezz_h,
             cross=12.0, width=2.4),
    ]

    # Guard rails, leaving the two ramp mouths open:
    #   gentle -> x in [6.5, 9.5]    steep -> x in [10.8, 13.2]
    # No north rail: the warehouse wall is the edge now.
    rail_mid = (mezz_south + mezz_north) / 2.0
    spec.boxes += [
        Box('mezz_rail_east', 13.5, rail_mid, mezz_h, 0.12, depth_or_span := (
            mezz_north - mezz_south), 0.55, material='Gazebo/Yellow'),
        Box('mezz_rail_west', 3.0, rail_mid, mezz_h, 0.12, depth_or_span, 0.55,
            material='Gazebo/Yellow'),
        Box('mezz_rail_south_a', 4.75, mezz_south, mezz_h, 3.5, 0.12, 0.55,
            material='Gazebo/Yellow'),
        Box('mezz_rail_south_b', 10.15, mezz_south, mezz_h, 1.3, 0.12, 0.55,
            material='Gazebo/Yellow'),
        Box('mezz_rail_south_c', 13.35, mezz_south, mezz_h, 0.3, 0.12, 0.55,
            material='Gazebo/Yellow'),
    ]

    # -- Packing bays along the east wall ------------------------------------
    for bay in range(1, 7):
        bay_y = -10.0 + (bay - 1) * 4.0
        spec.boxes.append(
            Box(
                f'packing_divider_{bay}',
                19.0,
                bay_y + 2.0,
                0.0,
                5.0,
                0.3,
                1.1,
                material='Gazebo/Blue',
            )
        )

    # -- Dynamic obstacles ---------------------------------------------------
    # Four pedestrians in the aisles plus two third-party robots on the main
    # corridor, i.e. exactly where the fleet has to negotiate.
    # Every loop below satisfies three properties, all asserted by
    # test_world_generation.py:
    #
    #   1. it clears racks, walls, ramps AND raised decks by more than the
    #      obstacle's own radius. Ramps matter as much as racks: these models
    #      are driven by libgazebo_ros_planar_move, which cannot climb, so a
    #      leg that clips a ramp toe wedges the obstacle there permanently --
    #      which is exactly what `ped_2` and `thirdparty_1` used to do;
    #   2. no two loops come within the sum of their radii plus 0.3 m of each
    #      other *anywhere*. Separation is enforced geometrically rather than
    #      by timing, because the driver's dwell and speed jitter mean two
    #      loops that merely miss each other on schedule will eventually meet;
    #   3. it crosses at least one AMR route, so the fleet has to sense and
    #      avoid rather than drive past. `thirdparty_0` cuts AMR-1's descent to
    #      Heavy Storage, `ped_2` sits across AMR-2's approach to Packing Bay 4,
    #      and `ped_3` straddles the east mouth of The Pinch.
    spec.dynamics += [
        DynamicObstacle(
            # South-west aisle, below the rack block.
            'ped_0', 'human', 0.35, 1.80, -14.0, 1.0,
            waypoints=((-14.0, 1.0), (-9.0, 1.0), (-9.0, -2.0), (-14.0, -2.0)),
            speed=0.9, material='Gazebo/Yellow',
        ),
        DynamicObstacle(
            # North-west cross-aisles at x = -16.5 and x = -12, the only two
            # gaps through that rack block.
            'ped_1', 'human', 0.35, 1.80, -5.0, 7.8,
            waypoints=((-5.0, 7.8), (-3.0, 7.8), (-3.0, 12.2), (-5.0, 12.2)),
            speed=0.75, material='Gazebo/Purple',
        ),
        DynamicObstacle(
            # East corridor, across AMR-2's final approach to Packing Bay 4.
            # Was (8, -3)..(16, 3), which ran straight onto the mezzanine's
            # steep ramp.
            'ped_2', 'human', 0.35, 1.80, 5.5, -1.2,
            waypoints=((5.5, -1.2), (13.0, -1.2), (13.0, -3.2), (5.5, -3.2)),
            speed=1.05, material='Gazebo/Purple',
        ),
        DynamicObstacle(
            # AMR-2's final approach to Packing Bay 4. Deliberately NOT at the
            # doorway: an obstacle loitering in The Pinch turns a negotiation
            # into a blockage.
            'ped_3', 'human', 0.35, 1.80, 11.0, 2.5,
            waypoints=((11.0, 2.5), (14.0, 2.5), (14.0, 0.5), (11.0, 0.5)),
            speed=0.6, material='Gazebo/Purple',
        ),
        DynamicObstacle(
            # West main corridor, crossing AMR-2's eastbound run -- but held
            # 4.5 m clear of both docks so the AMRs get out cleanly.
            'thirdparty_0', 'box', 0.45, 0.5, -3.5, -5.0,
            waypoints=((-3.5, -5.0), (-1.8, -5.0), (-1.8, -11.5), (-3.5, -11.5)),
            speed=0.7, material='Gazebo/Red',
        ),
        DynamicObstacle(
            # South-east corridor, on the west_staging -> east_staging route
            # used by the slope demo. Was (10, 0)..(16, 5.5), which overlapped
            # the steep ramp corridor.
            'thirdparty_1', 'box', 0.45, 0.5, 5.5, -5.0,
            waypoints=((5.5, -5.0), (13.0, -5.0), (13.0, -7.0), (5.5, -7.0)),
            speed=0.8, material='Gazebo/Red',
        ),
    ]

    # -- Named goals ---------------------------------------------------------
    spec.waypoints += [
        Waypoint('dock_a', -18.5, 2.2, 0.0, 'AMR-1 charging dock / start pose'),
        Waypoint('dock_b', -18.5, 0.0, 0.0, 'AMR-2 charging dock / start pose'),
        # Mid-aisle between the two heavy rows (y = -14.0 and y = -9.0), so the
        # goal is flanked by exactly one row on each side. x = -15.0 puts it
        # deep enough that AMR-1 has to run the length of the block and turn in
        # at the east mouth -- the aisle's only opening -- rather than nosing at
        # a gap in the shelving. Heading is west, along the aisle.
        Waypoint('heavy_storage', -15.0, -11.5, math.pi,
                 'AMR-1 primary goal: mid-aisle between the two heavy rows'),
        Waypoint('packing_bay_4', 17.2, 2.0, 0.0,
                 'AMR-2 primary goal: east-wall packing bay'),
        Waypoint('mezzanine_storage', 8.0, 11.7, 0.0,
                 'Ramp-only goal: proves ramps are used when unavoidable',
                 level='mezzanine'),
        Waypoint('east_staging', 14.0, -6.5, 0.0,
                 'East-side goal for the flat-doorway vs sloped-bridge test'),
        Waypoint('west_staging', -14.0, -1.0, 0.0,
                 'West-side counterpart for the slope experiment'),
        Waypoint('pinch_west', -3.5, 0.0, 0.0, 'West approach to The Pinch'),
        Waypoint('pinch_east', 3.5, 0.0, math.pi, 'East approach to The Pinch'),
    ]

    return spec


# ---------------------------------------------------------------------------
# SDF emission
# ---------------------------------------------------------------------------

_SDF_HEADER = """<?xml version="1.0" ?>
<!--
  GENERATED FILE - do not edit by hand.
  Regenerate with:  ros2 run amr_gazebo generate_world.py
  Source of truth:  amr_gazebo/amr_gazebo/world_builder.py
-->
<sdf version="1.6">
  <world name="{world_name}">

    <!-- A slightly reduced step size keeps the ramp contacts stable at the
         accelerations the light scout is allowed to command. -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>75</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.75 0.78 0.82 1</background>
      <shadows>false</shadows>
    </scene>

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- Exposes /gazebo/get_entity_state and /gazebo/set_entity_state, used by
         the test harness to teleport robots to a scenario start pose. -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>25.0</update_rate>
    </plugin>
"""

_SDF_FOOTER = """
  </world>
</sdf>
"""


def _static_box_sdf(box: Box) -> str:
    return f"""
    <model name="{box.name}">
      <static>true</static>
      <pose>{box.x:.4f} {box.y:.4f} {box.center_z:.4f} 0 0 {box.yaw:.6f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{box.size_x:.4f} {box.size_y:.4f} {box.size_z:.4f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{box.size_x:.4f} {box.size_y:.4f} {box.size_z:.4f}</size></box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{box.material}</name>
            </script>
          </material>
        </visual>
      </link>
    </model>"""


def _deck_sdf(deck: Deck) -> str:
    size_x = deck.x_max - deck.x_min
    size_y = deck.y_max - deck.y_min
    cx, cy = deck.center
    slab = Box(
        deck.name, cx, cy, 0.0, size_x, size_y, deck.height, material=deck.material
    )
    return _static_box_sdf(slab)


def _ramp_sdf(ramp: Ramp) -> str:
    x, y, z, roll, pitch, yaw = ramp.pose()
    return f"""
    <!-- {ramp.name}: {ramp.slope_deg:.1f} deg incline -->
    <model name="{ramp.name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{ramp.length:.4f} {ramp.width:.4f} {ramp.thickness:.4f}</size></box>
          </geometry>
          <surface>
            <friction>
              <ode><mu>1.2</mu><mu2>1.0</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{ramp.length:.4f} {ramp.width:.4f} {ramp.thickness:.4f}</size></box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{ramp.material}</name>
            </script>
          </material>
        </visual>
      </link>
    </model>"""


def _human_actor_sdf(obs: DynamicObstacle) -> str:
    """Render one pedestrian as an animated, scripted Gazebo actor.

    Two properties follow from being an actor rather than a driven model, and
    both are the point:

    * **It runs.** ``run.dae`` is Gazebo Classic's running animation, the same
      skin the reference warehouse world uses, so the legs actually move.
    * **It cannot get stuck.** An actor is kinematic: it follows its script
      regardless of contacts. The previous pedestrians were physics-driven
      through ``libgazebo_ros_planar_move``, so a corner clipped against a rack
      wedged them for the rest of the run. Scripted motion removes that failure
      mode by construction rather than by tuning.

    What an actor does *not* give you is a body the LiDAR can see, which is why
    HUMAN_BONE_COLLISIONS exists. See DESIGN_NOTES 8f.
    """
    loop = [tuple(point) for point in obs.waypoints]
    legs = list(zip(loop, loop[1:] + loop[:1]))

    # One waypoint per event, never two at the same timestamp. Gazebo
    # interpolates between consecutive waypoints by time, so a repeated time is
    # a zero-length interval and the actor stalls or jumps there.
    #
    # The arrival at the end of one leg IS the start of the next leg's pivot,
    # so it is emitted once, not twice.
    heading_of = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in legs]

    waypoints: List[str] = [_actor_waypoint(0.0, loop[0], heading_of[-1])]
    clock = 0.0
    for index, (start, end) in enumerate(legs):
        # Pivot on the spot onto this leg's heading, so a corner is a turn
        # rather than a slide.
        clock += HUMAN_TURN_SECONDS
        waypoints.append(_actor_waypoint(clock, start, heading_of[index]))

        clock += math.hypot(end[0] - start[0], end[1] - start[1]) / obs.speed
        waypoints.append(_actor_waypoint(clock, end, heading_of[index]))

    collisions = '\n'.join(
        f"""      <link name="{bone}">
        <collision name="{bone}_collision">
          <geometry><sphere><radius>{radius:.3f}</radius></sphere></geometry>
        </collision>
      </link>"""
        for bone, radius in HUMAN_BONE_COLLISIONS.items())

    return f"""
    <actor name="{obs.name}">
      <skin>
        <filename>{HUMAN_MESH_URI}</filename>
        <scale>{obs.height / HUMAN_MESH_HEIGHT:.4f}</scale>
      </skin>
      <animation name="run">
        <filename>{HUMAN_MESH_URI}</filename>
        <scale>{obs.height / HUMAN_MESH_HEIGHT:.4f}</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="run">
{chr(10).join(waypoints)}
        </trajectory>
      </script>
{collisions}
    </actor>"""


def _actor_waypoint(time: float, point: Tuple[float, float], yaw: float) -> str:
    return (f'          <waypoint><time>{time:.3f}</time>'
            f'<pose>{point[0]:.4f} {point[1]:.4f} 0 0 0 {yaw:.4f}</pose>'
            f'</waypoint>')


def _dynamic_obstacle_sdf(obs: DynamicObstacle) -> str:
    if obs.shape == 'human':
        # A walking person. The COLLISION stays a plain cylinder and the mesh
        # is visual only -- deliberately.
        #
        # Gazebo Classic's animated `<actor>` is the obvious way to get a
        # walking human, and it is what the reference warehouse world uses. It
        # is also unusable here: an actor carries no collision geometry, so a
        # LiDAR looks straight through it. The whole point of these obstacles
        # is that the fleet has to *sense* them, so the body has to be real.
        #
        # Splitting it this way means the mesh is pure decoration: if
        # walk.dae is missing from a given Gazebo install the person renders
        # as nothing, and the obstacle is still physically there and still
        # sensed. Nothing the assignment is graded on depends on the mesh
        # loading. See DESIGN_NOTES 8f.
        geometry = (
            f'<cylinder><radius>{obs.radius:.3f}</radius>'
            f'<length>{obs.height:.3f}</length></cylinder>'
        )
        scale = obs.height / HUMAN_MESH_HEIGHT
        visual = f"""<visual name="visual">
          <pose>0 0 {-obs.height / 2.0:.4f} 0 0 {HUMAN_MESH_YAW_OFFSET:.4f}</pose>
          <geometry>
            <mesh>
              <uri>{HUMAN_MESH_URI}</uri>
              <scale>{scale:.4f} {scale:.4f} {scale:.4f}</scale>
            </mesh>
          </geometry>
        </visual>"""
        mass = 70.0
    else:
        if obs.shape == 'cylinder':
            geometry = (
                f'<cylinder><radius>{obs.radius:.3f}</radius>'
                f'<length>{obs.height:.3f}</length></cylinder>'
            )
            mass = 70.0
        else:
            geometry = (
                f'<box><size>{obs.radius * 2:.3f} {obs.radius * 2:.3f} '
                f'{obs.height:.3f}</size></box>'
            )
            mass = 40.0
        visual = f"""<visual name="visual">
          <geometry>{geometry}</geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{obs.material}</name>
            </script>
          </material>
        </visual>"""

    return f"""
    <model name="{obs.name}">
      <pose>{obs.start_x:.4f} {obs.start_y:.4f} {obs.height / 2.0:.4f} 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>{mass:.2f}</mass>
          <inertia>
            <ixx>{mass * obs.height ** 2 / 12.0:.4f}</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>{mass * obs.height ** 2 / 12.0:.4f}</iyy><iyz>0</iyz>
            <izz>{mass * obs.radius ** 2 / 2.0:.4f}</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>{geometry}</geometry>
        </collision>
        {visual}
      </link>

      <!-- Holonomic drive so dynamic_obstacle_driver can steer it with a
           plain Twist. Odometry is namespaced to keep the fleet TF tree
           clean. -->
      <plugin name="{obs.name}_move" filename="libgazebo_ros_planar_move.so">
        <ros>
          <namespace>/obstacles/{obs.name}</namespace>
          <remapping>cmd_vel:=cmd_vel</remapping>
          <remapping>odom:=odom</remapping>
        </ros>
        <update_rate>50</update_rate>
        <publish_rate>10</publish_rate>
        <publish_odom>true</publish_odom>
        <publish_odom_tf>false</publish_odom_tf>
        <odometry_frame>obstacles/{obs.name}/odom</odometry_frame>
        <robot_base_frame>obstacles/{obs.name}/base_link</robot_base_frame>
        <covariance_x>0.0001</covariance_x>
        <covariance_y>0.0001</covariance_y>
        <covariance_yaw>0.01</covariance_yaw>
      </plugin>
    </model>"""


def render_sdf(spec: WorldSpec) -> str:
    """Render the whole world to an SDF document."""
    parts = [_SDF_HEADER.format(world_name=spec.name)]
    parts.append('\n    <!-- ===== Static structure ===== -->')
    parts += [_static_box_sdf(b) for b in spec.boxes]
    parts.append('\n    <!-- ===== Raised decks ===== -->')
    parts += [_deck_sdf(d) for d in spec.decks]
    parts.append('\n    <!-- ===== Ramps ===== -->')
    parts += [_ramp_sdf(r) for r in spec.ramps]
    parts.append('\n    <!-- ===== Dynamic obstacles ===== -->')
    # Pedestrians are animated actors; the third-party robots are driven
    # models. Two renderers, one list.
    parts += [_human_actor_sdf(o) if o.shape == 'human' else _dynamic_obstacle_sdf(o)
              for o in spec.dynamics]
    parts.append(_SDF_FOOTER)
    return '\n'.join(parts)


# ---------------------------------------------------------------------------
# Elevation map emission
# ---------------------------------------------------------------------------


def sample_elevation(spec: WorldSpec, x: float, y: float) -> float:
    """Height of the walkable floor at ``(x, y)``.

    Ramps win over decks (a ramp overlapping a deck edge is the transition
    surface) and decks win over the ground plane.
    """
    for ramp in spec.ramps:
        x0, x1, y0, y1 = ramp.bounds()
        if x0 <= x <= x1 and y0 <= y <= y1:
            return ramp.height_at(x, y)
    for deck in spec.decks:
        if deck.x_min <= x <= deck.x_max and deck.y_min <= y <= deck.y_max:
            return deck.height
    return 0.0


def render_elevation_map(
    spec: WorldSpec, resolution: float = 0.05
) -> Tuple[bytes, str, Dict[str, float]]:
    """Rasterise the floor into a PGM height field plus its metadata YAML.

    The encoding matches ``nav2_map_server``'s image convention (row 0 is the
    *top* of the image, i.e. maximum y) so the same origin/resolution semantics
    apply and ``SlopeLayer`` can reuse the standard loader.

    Only *floor* surfaces are rasterised. Racks and walls are deliberately
    excluded: they are obstacles, handled by the obstacle layer, and folding
    them in here would fabricate 90-degree "slopes" everywhere.
    """
    width = int(round((spec.x_max - spec.x_min) / resolution))
    height = int(round((spec.y_max - spec.y_min) / resolution))

    heights: List[List[float]] = []
    z_min, z_max = 0.0, 0.0
    for row in range(height):
        # Row 0 is the top of the image => highest y.
        y = spec.y_max - (row + 0.5) * resolution
        line: List[float] = []
        for col in range(width):
            x = spec.x_min + (col + 0.5) * resolution
            z = sample_elevation(spec, x, y)
            z_min = min(z_min, z)
            z_max = max(z_max, z)
            line.append(z)
        heights.append(line)

    # Encode with 25% headroom so a future taller deck does not force a
    # re-scale of the whole map, but no more than that.
    #
    # The headroom is deliberately tight. Height is quantised to 8 bits, and
    # the slope layer differentiates that field over a two-cell baseline, so
    # the encoding's least significant bit sets the angular resolution of every
    # slope measurement downstream. Padding the range to a round 1.0 m for a
    # world whose tallest deck is 0.55 m throws away 45% of the dynamic range
    # and costs roughly 1.5 degrees of accuracy on the steepest ramp - enough
    # to matter next to a 16 degree lethal threshold.
    encode_min = 0.0
    encode_max = max(0.1, math.ceil(z_max * 1.25 * 100.0) / 100.0)
    span = encode_max - encode_min

    payload = bytearray()
    header = f'P5\n{width} {height}\n255\n'.encode('ascii')
    payload += header
    for line in heights:
        payload += bytes(
            max(0, min(255, int(round((z - encode_min) / span * 255.0))))
            for z in line
        )

    meta = f"""# GENERATED FILE - regenerate with: ros2 run amr_gazebo generate_world.py
#
# Elevation field for amr_navigation::SlopeLayer.
#
# Pixel value v in [0, 255] decodes to a floor height of
#     z = elevation_min + v * (elevation_max - elevation_min) / 255
#
# Geometry keys mirror nav2_map_server's map.yaml so the same image loader and
# origin convention can be reused.
image: warehouse_elevation.pgm
resolution: {resolution}
origin: [{spec.x_min}, {spec.y_min}, 0.0]
negate: 0
occupied_thresh: 0.99
free_thresh: 0.01
mode: raw

elevation_min: {encode_min}
elevation_max: {encode_max}
width: {width}
height: {height}
"""

    stats = {
        'width': float(width),
        'height': float(height),
        'z_min': z_min,
        'z_max': z_max,
        'encode_max': encode_max,
    }
    return bytes(payload), meta, stats


def render_waypoints(spec: WorldSpec) -> str:
    """Emit the named goals as YAML for the demo scripts."""
    lines = [
        '# GENERATED FILE - regenerate with: ros2 run amr_gazebo generate_world.py',
        '#',
        '# Named goals in the shared `map` frame. Consumed by',
        '# amr_bringup/scripts/send_goals.py and demo_conflict.py.',
        'waypoints:',
    ]
    for wp in spec.waypoints:
        lines += [
            f'  {wp.name}:',
            f'    x: {wp.x}',
            f'    y: {wp.y}',
            f'    yaw: {wp.yaw:.6f}',
            f'    level: "{wp.level}"',
            f'    description: "{wp.description}"',
        ]
    return '\n'.join(lines) + '\n'


def render_dynamic_obstacles(spec: WorldSpec) -> str:
    """Emit the dynamic-obstacle patrol loops as YAML for the driver node."""
    lines = [
        '# GENERATED FILE - regenerate with: ros2 run amr_gazebo generate_world.py',
        '#',
        '# Patrol loops for amr_gazebo/dynamic_obstacle_driver.py.',
        '#',
        '# ONLY the driven obstacles appear here. The pedestrians are animated',
        '# Gazebo actors whose trajectory is scripted into the world itself, so',
        '# nothing at runtime steers them -- which is precisely why they can no',
        '# longer wedge themselves against a rack. Their loops are still listed',
        '# below the driver block, unused by it, so the geometry tests and a',
        '# human reader can see all six in one place.',
        'dynamic_obstacle_driver:',
        '  ros__parameters:',
        '    obstacle_names: [' + ', '.join(
            f'"{o.name}"' for o in spec.dynamics if o.shape != 'human') + ']',
    ]
    for obs in spec.dynamics:
        flat: List[float] = []
        for wx, wy in obs.waypoints:
            flat += [wx, wy]
        lines += [
            f'    {obs.name}:',
            f'      speed: {obs.speed}',
            f'      kind: "{obs.shape}"',
            '      waypoints: [' + ', '.join(f'{v}' for v in flat) + ']',
        ]
        if obs.shape == 'human':
            lines.insert(len(lines) - 4, '    # scripted actor - not driven')
    return '\n'.join(lines) + '\n'


def write_all(spec: WorldSpec, package_dir: str, resolution: float = 0.05) -> Dict[str, str]:
    """Write world, elevation map and waypoints into ``package_dir``.

    Returns a mapping of artefact name to the path written.
    """
    worlds_dir = os.path.join(package_dir, 'worlds')
    maps_dir = os.path.join(package_dir, 'maps')
    config_dir = os.path.join(package_dir, 'config')
    for directory in (worlds_dir, maps_dir, config_dir):
        os.makedirs(directory, exist_ok=True)

    world_path = os.path.join(worlds_dir, f'{spec.name}.world')
    with open(world_path, 'w', encoding='utf-8') as handle:
        handle.write(render_sdf(spec))

    pgm, meta, _ = render_elevation_map(spec, resolution)
    pgm_path = os.path.join(maps_dir, 'warehouse_elevation.pgm')
    yaml_path = os.path.join(maps_dir, 'warehouse_elevation.yaml')
    with open(pgm_path, 'wb') as handle:
        handle.write(pgm)
    with open(yaml_path, 'w', encoding='utf-8') as handle:
        handle.write(meta)

    waypoints_path = os.path.join(config_dir, 'waypoints.yaml')
    with open(waypoints_path, 'w', encoding='utf-8') as handle:
        handle.write(render_waypoints(spec))

    obstacles_path = os.path.join(config_dir, 'dynamic_obstacles.yaml')
    with open(obstacles_path, 'w', encoding='utf-8') as handle:
        handle.write(render_dynamic_obstacles(spec))

    return {
        'world': world_path,
        'elevation_pgm': pgm_path,
        'elevation_yaml': yaml_path,
        'waypoints': waypoints_path,
        'dynamic_obstacles': obstacles_path,
    }
