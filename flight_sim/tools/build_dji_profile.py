#!/usr/bin/env python3
"""Rebuild the DJI FPV approximation. See docs/AIRCRAFT_MODELS.md for provenance.

Published dimensions constrain a deliberately simple model. Unpublished dynamics
are explicit engineering estimates, not recovered DJI internals or calibration.
"""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def build():
    mass = 0.795
    length, width, height = 0.178, 0.232, 0.127
    diagonal = 0.245
    # Approximate motor rectangle: published diagonal, envelope aspect ratio.
    x = diagonal / 2 / math.sqrt(1 + (length / width) ** 2)
    y = x * length / width
    # Uniform-box inertia using the complete propeller-free envelope.
    inertia = [mass * (length**2 + height**2) / 12,
               mass * (width**2 + height**2) / 12,
               mass * (width**2 + length**2) / 12]
    config = dict(
        id='dji_fpv', name='DJI FPV',
        source='DJI official specifications + explicit estimates; docs/AIRCRAFT_MODELS.md',
        axes='x left, y backward, z up; SI units below',
        mass_kg=mass, cg_m=[0, 0, 0],
        motors_m=[[x, -y, 0], [-x, -y, 0], [x, y, 0], [-x, y, 0]],
        motor_names=['FL', 'FR', 'RL', 'RR'], spin_sign=[1, -1, -1, 1],
        inertia_kg_m2=[[inertia[0], 0, 0], [0, inertia[1], 0], [0, 0, inertia[2]]],
        max_thrust_N=mass * 9.80665 * 3.8 / 4,
        torque_per_thrust_m=0.013, motor_tau_s=0.05,
        timestep_s=0.002, gravity=9.80665, propeller_radius_m=0.1346 / 2,
        ground_clearance_m=height / 2,
        visual={'style': 'dji_fpv', 'body_size_m': [0.09, 0.08, 0.15]},
        published={'diagonal_m': diagonal, 'dimensions_without_props_m': [length, width, height],
                   'propeller_pitch_m': 0.0711, 'propeller_mass_kg': 0.0052,
                   'manual_mode_max_speed_m_s': 39, 'zero_to_100_kph_s': 2,
                   'note': 'Speed and acceleration are reference specs only, not hard limits or validated outputs.'},
        sources=[{'url': 'https://www.dji.com/dji-fpv', 'retrieved': '2026-09-18',
                  'fields': ['mass', 'dimensions', 'diagonal', 'reference speed and acceleration']},
                 {'url': 'https://store.dji.com/product/dji-fpv-propellers', 'retrieved': '2026-09-18',
                  'fields': ['propeller diameter', 'pitch', 'propeller mass']}],
        assumptions=[
            'Motor rectangle uses envelope aspect ratio and the published diagonal; CG centered and rotors coplanar.',
            'Inertia is a uniform-box estimate, not a measured mass distribution.',
            '3.8:1 maximum thrust/weight is an unverified prior; linear static curve, 50 ms response and Q/T=0.013 m are provisional.',
            'Spin signs match the simulator Betaflight mixer; DJI motor mapping is not reproduced.',
            'Contact geometry, body shape and drag are approximate.',
            'Uses Betaflight Acro, not DJI firmware, Normal/Sport modes, stabilization, braking or GPS.'])
    physics = json.loads((ROOT / 'config/physics.json').read_text())
    physics['aerodynamics']['body_cd_area_m2'] = [0.75 * length * height,
                                                           0.75 * width * height,
                                                           0.75 * width * length]
    physics['aerodynamics']['note'] = (
        'Unmeasured estimate: CdA = 0.75 times each projected propeller-free envelope area. '
        'Rotor drag and angular damping inherit the shared provisional model; not fitted to DJI performance.')
    directory = ROOT / 'config/aircraft'
    directory.mkdir(exist_ok=True)
    for name, data in [('dji_fpv.json', config), ('dji_fpv_physics.json', physics)]:
        (directory / name).write_text(json.dumps(data, indent=2) + '\n')
    print('Rebuilt DJI FPV approximation; measured curves are untouched.')


if __name__ == '__main__':
    build()
