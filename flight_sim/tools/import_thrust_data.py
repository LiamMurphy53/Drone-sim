#!/usr/bin/env python3
"""Import a static, single-motor thrust sweep; never invent missing measurements."""
import argparse
import csv
import json
import math
from pathlib import Path

DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / 'config/propulsion_curve.json'
MODEL_OUTPUTS = {'gopro_drone': DEFAULT_OUTPUT,
                 'dji_fpv': DEFAULT_OUTPUT.parent / 'aircraft/dji_fpv_propulsion_curve.json'}


def read_measurements(path):
    path = Path(path)
    with path.open(newline='', encoding='utf-8-sig') as source:
        reader = csv.DictReader(source)
        fields = set(reader.fieldnames or [])
        thrust_fields = fields & {'thrust_N', 'thrust_g'}
        if 'motor_command_0_1' not in fields or len(thrust_fields) != 1:
            raise ValueError('Use motor_command_0_1 and exactly one of thrust_N or thrust_g. Torque is optional as torque_Nm.')
        thrust_field = next(iter(thrust_fields))
        has_torque = 'torque_Nm' in fields
        points = []
        for row_number, row in enumerate(reader, start=2):
            if not any(value and value.strip() for value in row.values() if isinstance(value, str)):
                continue
            try:
                command = float(row['motor_command_0_1'])
                thrust = float(row[thrust_field]) * (0.00980665 if thrust_field == 'thrust_g' else 1)
                point = {'command': command, 'thrust_N': thrust}
                if has_torque:
                    point['torque_Nm'] = float(row['torque_Nm'])
            except (ValueError, TypeError):
                raise ValueError(f'Row {row_number}: missing or invalid numeric value.') from None
            if not all(math.isfinite(v) for v in point.values()):
                raise ValueError(f'Row {row_number}: all values must be finite.')
            if not 0 <= command <= 1 or thrust < 0 or point.get('torque_Nm', 0) < 0:
                raise ValueError(f'Row {row_number}: command must be 0–1; thrust and torque must be nonnegative.')
            points.append(point)
    if len(points) < 2:
        raise ValueError('At least two measured points are required.')
    points.sort(key=lambda p: p['command'])
    for a, b in zip(points, points[1:]):
        if a['command'] >= b['command']:
            raise ValueError('Duplicate motor command; supply one averaged static measurement per command.')
        if a['thrust_N'] > b['thrust_N']:
            raise ValueError('Thrust must be monotonic. Check noisy/out-of-order readings; no automatic smoothing is applied.')
        if has_torque and a['thrust_N'] == b['thrust_N'] and a['torque_Nm'] != b['torque_Nm']:
            raise ValueError('A thrust plateau must have equal torque at both ends for this model.')
    if points[0]['command'] != 0 or points[0]['thrust_N'] != 0 or points[-1]['command'] != 1 or points[-1]['thrust_N'] <= 0:
        raise ValueError('Include zero command/zero thrust and a full-command measurement. No extrapolation is performed.')
    if has_torque and points[0]['torque_Nm'] != 0:
        raise ValueError('Zero thrust must have zero torque.')
    return {'schema_version': 1, 'source': path.name, 'interpolation': 'piecewise_linear',
            'notes': 'Static single-motor measurements at one test condition. No voltage correction or aerodynamic inflow model.',
            'points': points}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_file', type=Path)
    parser.add_argument('--drone', choices=MODEL_OUTPUTS, default='gopro_drone',
                        help='Aircraft receiving these measurements (default: GoPro Drone).')
    parser.add_argument('--output', type=Path, help='Override the selected aircraft output path.')
    parser.add_argument('--force', action='store_true', help='Explicitly replace an existing imported curve.')
    args = parser.parse_args()
    args.output = args.output or MODEL_OUTPUTS[args.drone]
    try:
        curve = read_measurements(args.csv_file)
        if args.output.exists() and not args.force:
            raise ValueError(f'{args.output} already exists. Use --force to replace it.')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        # Exclusive creation protects an existing curve from accidental overwrites.
        with args.output.open('w' if args.force else 'x') as output:
            json.dump(curve, output, indent=2, allow_nan=False)
            output.write('\n')
    except (OSError, ValueError) as exc:
        parser.exit(2, f'Import failed: {exc}\n')
    print(f'Imported {len(curve["points"])} points; full-command thrust {curve["points"][-1]["thrust_N"]:.3f} N per motor.')
    print('Restart the simulator to use the measurements. Motor response time is unchanged.')


if __name__ == '__main__':
    main()
