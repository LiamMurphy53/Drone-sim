#!/usr/bin/env python3
"""Export the existing MATLAB input values without requiring MATLAB at runtime."""
import json, re
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
text = (ROOT / 'matlab_sim/inputs/drone_config_inputs.m').read_text()
dynamics = (ROOT / 'matlab_sim/inputs/simulation_inputs.m').read_text()
def scalar(name, source=text):
    return float(re.search(r'\b' + re.escape(name) + r'\s*=\s*([\d.eE+-]+)', source)[1])
def matrix(name):
    body = re.search(re.escape(name) + r'\s*=\s*\[(.*?)\]', text, re.S)[1]
    body = re.sub(r'%[^\n]*', '', body)
    def number(s):
        # Only the numeric grammar used in the source configuration; never eval MATLAB.
        match = re.fullmatch(r'([\d.eE+-]+)(?:\*10\^([+-]?\d+))?', s.strip())
        if not match: raise ValueError('Unsupported numeric input: ' + s)
        return float(match[1]) * 10 ** int(match[2] or 0)
    return [[number(x) for x in row.strip().split(',')] for row in body.split(';') if row.strip()]
config = dict(id='gopro_drone', name='GoPro Drone', source='matlab_sim/inputs/drone_config_inputs.m',
    axes='x left, y backward, z up; SI units below',
    mass_kg=scalar('droneInputs.totalMass_g')/1000,
    cg_m=[x/1000 for x in matrix('droneInputs.centerOfMass_mm')[0]],
    motors_m=[[x/1000 for x in row] for row in matrix('droneInputs.motorLocations_mm')],
    motor_names=['FL','FR','RL','RR'], spin_sign=[1,-1,-1,1],
    inertia_kg_m2=[[x*1e-9 for x in row] for row in matrix('droneInputs.inertiaTensor_g_mm2')],
    max_thrust_N=scalar('droneInputs.maxThrust_N'),
    torque_per_thrust_m=scalar('droneInputs.yawTorquePerThrust_m'),
    motor_tau_s=scalar('simulationInputs.dynamics.motorTimeConstant_s', dynamics),
    timestep_s=scalar('simulationInputs.dynamics.timeStep_s', dynamics), gravity=9.80665,
    propeller_radius_m=scalar('droneInputs.propellerDiameter_in') * 0.0254 / 2,
    ground_clearance_m=0.12, visual={'style':'gopro_drone','body_size_m':[0.055,0.038,0.145]},
    assumptions=['Original linear steady thrust is retained unless config/propulsion_curve.json exists.',
        'Response times are inherited estimates; aerodynamics and contact assumptions live separately in config/physics.json.',
        'No measured aerodynamic calibration, battery sag, propwash, or rotor inertia.'])
path=ROOT/'flight_sim/config/aircraft.json'
path.write_text(json.dumps(config, indent=2)+'\n')
print('Exported',path)
