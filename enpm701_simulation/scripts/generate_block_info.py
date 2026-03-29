#!/usr/bin/env python3
"""Generate random block positions outside the construction and buffer zones."""

import math
import os
import random

import yaml

# Arena bounds: 3.048 x 3.048 centered at origin, minus wall margin
_WALL_MARGIN = 0.1
_X_MIN = -1.524 + _WALL_MARGIN
_X_MAX = 1.524 - _WALL_MARGIN
_Y_MIN = -1.524 + _WALL_MARGIN
_Y_MAX = 1.524 - _WALL_MARGIN

# Extra clearance added to each zone boundary so blocks aren't right on the edge
_ZONE_CLEARANCE = 0.1

# Construction zone: center (-0.9144, 0.9144), size 1.2192 x 1.2192
_CONSTRUCTION_ZONE = {
    'x_min': -1.5240 - _ZONE_CLEARANCE,
    'x_max': -0.3048 + _ZONE_CLEARANCE,
    'y_min': 0.3048 - _ZONE_CLEARANCE,
    'y_max': 1.5240 + _ZONE_CLEARANCE,
}

# Buffer/landing zone: center (-1.2192, -1.2192), size 0.6096 x 0.6096
_BUFFER_ZONE = {
    'x_min': -1.5240 - _ZONE_CLEARANCE,
    'x_max': -0.9144 + _ZONE_CLEARANCE,
    'y_min': -1.5240 - _ZONE_CLEARANCE,
    'y_max': -0.9144 + _ZONE_CLEARANCE,
}

_MIN_SEPARATION = 0.6096
_Z = 0.005
_COLORS = ['1 0 0', '0 1 0', '0 0 1']  # red, green, blue


def _in_zone(x, y, zone):
    return (zone['x_min'] <= x <= zone['x_max'] and
            zone['y_min'] <= y <= zone['y_max'])


def _is_valid(x, y, placed):
    if not (_X_MIN <= x <= _X_MAX and _Y_MIN <= y <= _Y_MAX):
        return False
    if _in_zone(x, y, _CONSTRUCTION_ZONE):
        return False
    if _in_zone(x, y, _BUFFER_ZONE):
        return False
    for px, py in placed:
        if math.hypot(x - px, y - py) < _MIN_SEPARATION:
            return False
    return True


def _generate_positions(n=9, max_attempts=20000):
    placed = []
    for i in range(n):
        for _ in range(max_attempts):
            x = random.uniform(_X_MIN, _X_MAX)
            y = random.uniform(_Y_MIN, _Y_MAX)
            if _is_valid(x, y, placed):
                placed.append((x, y))
                break
        else:
            raise RuntimeError(
                f'Could not place block {i + 1} after {max_attempts} attempts. '
                'Try increasing max_attempts or reducing MIN_SEPARATION.')
    return placed


def generate(output_path):
    """Generate 9 block positions and write to output_path as YAML."""
    positions = _generate_positions(9)
    data = {}
    for i, (x, y) in enumerate(positions):
        name = f'block_{i + 1}'
        data[name] = {
            'color': _COLORS[i % 3],
            'translation': f'{x:.4f} {y:.4f} {_Z}',
        }
    with open(output_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=True)
    print(f'[generate_block_info] Wrote {output_path}')


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.normpath(
        os.path.join(script_dir, '..', 'worlds', 'block_info.yaml'))
    generate(output_path)


if __name__ == '__main__':
    main()
