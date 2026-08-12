#!/usr/bin/env python3
# Copyright 2026 RSE Candidate
# Licensed under the Apache License, Version 2.0.
"""Regenerate the warehouse world, its elevation map and its waypoint list.

    ros2 run amr_gazebo generate_world.py            # writes into the install space
    ./scripts/generate_world.py --package-dir .      # writes into the source tree

Always regenerate into the *source* tree and rebuild; writing into the install
space leaves the repository and the running system disagreeing.
"""

import argparse
import os
import sys


def _resolve_package_dir(explicit):
    """Locate the package root, preferring an explicit override."""
    if explicit:
        return os.path.abspath(explicit)
    # scripts/ -> package root
    return os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--package-dir',
        default=None,
        help='Package root to write into (default: parent of this script).',
    )
    parser.add_argument(
        '--resolution',
        type=float,
        default=0.05,
        help='Elevation map resolution in metres per pixel (default: 0.05).',
    )
    args = parser.parse_args(argv)

    package_dir = _resolve_package_dir(args.package_dir)
    # Allow running straight from a source checkout without sourcing the ws.
    sys.path.insert(0, package_dir)
    from amr_gazebo import world_builder  # noqa: E402  (deliberate late import)

    spec = world_builder.build_warehouse()
    written = world_builder.write_all(spec, package_dir, args.resolution)

    print('Generated warehouse:')
    print(f'  static bodies    : {len(spec.boxes)}')
    print(f'  raised decks     : {len(spec.decks)}')
    print(f'  ramps            : {len(spec.ramps)}')
    for ramp in spec.ramps:
        print(f'      - {ramp.name:<20s} {ramp.slope_deg:5.1f} deg '
              f'({ramp.length:.2f} m run)')
    print(f'  dynamic obstacles: {len(spec.dynamics)}')
    print(f'  named waypoints  : {len(spec.waypoints)}')
    print('Artefacts:')
    for key, path in written.items():
        print(f'  {key:<15s} -> {path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
