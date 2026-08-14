#!/usr/bin/env python3
# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Render an annotated plan view of the warehouse.

    ros2 run amr_gazebo render_layout_map.py

Everything on the drawing is read from ``world_builder.build_warehouse()``, the
same function that emits the SDF, the elevation map and the waypoint file. It
cannot drift from what Gazebo simulates, because there is nothing to drift
from -- if the picture is wrong, the world is wrong.

Written because a coordinate list is unreadable at this size. Six patrol loops,
nine named goals, four ramps and three rack blocks are hard to hold in your head
and trivial to see.

Style: PEP 8, checked by ament_flake8.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib                                                # noqa: E402
matplotlib.use('Agg')
import matplotlib.patches as patches                             # noqa: E402
import matplotlib.pyplot as plt                                  # noqa: E402

from amr_gazebo import world_builder as wb                       # noqa: E402

RACK = '#c8a165'
WALL = '#4a4a4a'
RAMP = '#c0714f'
DECK = '#e2c48a'
HUMAN = '#1f77b4'
ROBOT = '#d62728'
GOAL = '#2ca02c'
DOCK = '#7f3fbf'


def draw(spec, path, dpi=170):
    fig, ax = plt.subplots(figsize=(17, 12))

    # -- floor, walls, racks -------------------------------------------------
    ax.add_patch(patches.Rectangle(
        (spec.x_min, spec.y_min), spec.x_max - spec.x_min,
        spec.y_max - spec.y_min, facecolor='#f4f4f4', edgecolor='none', zorder=0))

    for box in spec.boxes:
        if box.yaw:
            continue
        lower = (box.x - box.size_x / 2.0, box.y - box.size_y / 2.0)
        if box.name.startswith('wall_'):
            colour, z = WALL, 3
        elif '_rack_' in box.name:
            colour, z = RACK, 2
        else:
            colour, z = '#8f8f8f', 3
        ax.add_patch(patches.Rectangle(
            lower, box.size_x, box.size_y, facecolor=colour,
            edgecolor='#00000030', linewidth=0.4, zorder=z))

    # -- raised decks and ramps ---------------------------------------------
    for deck in spec.decks:
        ax.add_patch(patches.Rectangle(
            (deck.x_min, deck.y_min), deck.x_max - deck.x_min,
            deck.y_max - deck.y_min, facecolor=DECK, edgecolor='#8a6d2f',
            linewidth=1.0, zorder=1))
        ax.text((deck.x_min + deck.x_max) / 2.0, (deck.y_min + deck.y_max) / 2.0,
                f"{deck.name}\n{deck.height:.2f} m", ha='center', va='center',
                fontsize=7, color='#5c4715', zorder=6)

    for ramp in spec.ramps:
        low, high = sorted((ramp.start, ramp.end))
        if ramp.axis == 'x':
            lower = (low, ramp.cross - ramp.width / 2.0)
            size = (high - low, ramp.width)
        else:
            lower = (ramp.cross - ramp.width / 2.0, low)
            size = (ramp.width, high - low)
        ax.add_patch(patches.Rectangle(
            lower, size[0], size[1], facecolor=RAMP, edgecolor='#7a3d22',
            linewidth=1.0, alpha=0.85, zorder=1))
        ax.text(lower[0] + size[0] / 2.0, lower[1] + size[1] / 2.0,
                f'{ramp.slope_deg:.1f}°', ha='center', va='center',
                fontsize=7, color='white', weight='bold', zorder=6)

    # -- patrol loops --------------------------------------------------------
    for obs in spec.dynamics:
        points = [tuple(p) for p in obs.waypoints]
        closed = points + points[:1]
        colour = HUMAN if obs.shape == 'human' else ROBOT
        ax.plot([p[0] for p in closed], [p[1] for p in closed],
                color=colour, linewidth=1.8, linestyle='--', zorder=5)
        ax.add_patch(patches.Circle(
            (obs.start_x, obs.start_y), obs.radius, facecolor=colour,
            edgecolor='black', linewidth=0.5, zorder=7))
        kind = 'human' if obs.shape == 'human' else 'robot'
        ax.annotate(f'{obs.name}\n{kind}, {obs.speed} m/s',
                    (obs.start_x, obs.start_y), textcoords='offset points',
                    xytext=(8, 8), fontsize=7, color=colour, weight='bold',
                    zorder=8)

    # -- named goals ---------------------------------------------------------
    for way in spec.waypoints:
        is_dock = way.name.startswith('dock_')
        colour = DOCK if is_dock else GOAL
        ax.plot(way.x, way.y, marker='*' if not is_dock else 's',
                markersize=13 if not is_dock else 9, color=colour,
                markeredgecolor='black', markeredgewidth=0.6, zorder=9)
        ax.annotate(f'{way.name}\n({way.x:.1f}, {way.y:.1f})', (way.x, way.y),
                    textcoords='offset points', xytext=(9, -12), fontsize=7,
                    color=colour, weight='bold', zorder=9)

    # -- frame ---------------------------------------------------------------
    ax.set_xlim(spec.x_min - 1.0, spec.x_max + 1.0)
    ax.set_ylim(spec.y_min - 1.0, spec.y_max + 1.0)
    ax.set_aspect('equal')
    ax.set_xticks(range(int(spec.x_min), int(spec.x_max) + 1, 2))
    ax.set_yticks(range(int(spec.y_min), int(spec.y_max) + 1, 2))
    ax.grid(True, linewidth=0.3, color='#00000018')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Warehouse layout — racks, ramps, named goals and '
                 'dynamic-obstacle patrol loops\n'
                 'generated from world_builder.build_warehouse()', fontsize=11)

    legend = [
        patches.Patch(facecolor=RACK, label='storage racks (continuous rows)'),
        patches.Patch(facecolor=WALL, label='walls / firewall'),
        patches.Patch(facecolor=RAMP, label='ramps (gradient labelled)'),
        patches.Patch(facecolor=DECK, label='raised decks'),
        plt.Line2D([], [], color=HUMAN, linestyle='--',
                   label='human patrol loop (scripted actor)'),
        plt.Line2D([], [], color=ROBOT, linestyle='--',
                   label='third-party robot loop (driven)'),
        plt.Line2D([], [], color=GOAL, marker='*', linestyle='none',
                   label='named goal'),
        plt.Line2D([], [], color=DOCK, marker='s', linestyle='none',
                   label='dock / spawn'),
    ]
    ax.legend(handles=legend, loc='lower left', fontsize=8, framealpha=0.92)

    fig.tight_layout()
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output', default=None,
        help='PNG to write (default: <package>/maps/warehouse_layout.png)')
    args = parser.parse_args(argv)

    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    output = args.output or os.path.join(
        package_dir, 'maps', 'warehouse_layout.png')
    os.makedirs(os.path.dirname(output), exist_ok=True)

    spec = wb.build_warehouse()
    draw(spec, output)

    print(f'wrote {output}')
    print(f'  {len(spec.boxes)} static bodies, {len(spec.ramps)} ramps, '
          f'{len(spec.decks)} decks')
    print(f'  {len(spec.dynamics)} dynamic obstacles, '
          f'{len(spec.waypoints)} named goals')
    return 0


if __name__ == '__main__':
    sys.exit(main())
