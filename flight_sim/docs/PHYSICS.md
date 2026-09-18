# Physics model

The model uses SI units. Body/world axes inherit the MATLAB convention: x left, y backward, z up. Rendering and Betaflight coordinate conversions are separate. Mass properties belong to each aircraft profile; estimated aerodynamic settings live in that profile's physics JSON.

## Rigid body and integration

Translation obeys m dv/dt = R Fbody + m g. Rotation obeys I dω/dt = Mbody − ω × (Iω), retaining the full inertia tensor. Orientation uses normalized quaternion increments. Force and angular acceleration are evaluated at an intermediate state for a midpoint step, normally 0.002 s (500 Hz). Contact impulses are applied after free motion; virtual specific force includes their velocity change.

The test suite checks torque-free rotational energy and world angular momentum over 10 seconds, and convergence against smaller timesteps. Those checks establish numerical consistency for the tested cases, not agreement with a real aircraft.

## Propulsion

The static map T(command) is the original linear estimate by default, or a piecewise-linear measured curve. Define normalized rotor state s_target = sqrt(T(command)/Tmax). Rotor state follows a first-order response, using separate rise/fall time constants; integration uses the exact exponential response for constant command during the step. Thrust before ground effect is Tmax s². This represents squared-speed thrust but s is a proxy, not calibrated RPM. Original 50 ms response estimates are retained until measured.

Reaction torque uses measured Q versus static T when supplied, otherwise the inherited Q/T ratio. Curve endpoints and monotonic thrust are required; torque must be supplied at every point or omitted entirely. Ground effect multiplies lift, not reaction torque. The optional `legacy_thrust` response setting preserves first-order thrust dynamics for regression comparisons.

No rotor angular momentum, voltage dependence, current consumption or blade-element aerodynamics is modeled.

## Aerodynamic loads

Air-relative velocity is vehicle velocity minus wind, transformed into body coordinates. Body force uses F_i = −0.5 ρ CdA_i |v| v_i, with v evaluated at the configured center of pressure. CdA is positive along each axis, so drag changes with orientation. The pressure-center moment is r × F.

Each rotor sees v_local = v_body + ω × r_rotor. Its in-plane force is −k s [v_local,x, v_local,y, 0], applied at that rotor. This creates both translational drag and rotational damping. A separate diagonal quadratic damping term is −kω ω |ω|. Together, drag forces and their moments remove mechanical energy in still air; the tests exercise this property even with a displaced pressure center. Wind defaults to zero.

The model form is informed by [RotorPy's multirotor implementation](https://github.com/spencerfolk/rotorpy/blob/main/rotorpy/vehicles/multirotor.py), its [simulation paper](https://arxiv.org/abs/2306.04485), and the [Gazebo multicopter motor model](https://github.com/gazebosim/gz-sim/blob/gz-sim9/src/systems/multicopter_motor_model/MulticopterMotorModel.cc). This implementation was written for this project; aerodynamic coefficients have not been identified from this drone. No source's numerical coefficients should be interpreted as measurements of these aircraft.

## Ground effect and contact

A simplified flat-ground factor is 1 / (1 − (Rprop/(4h))²), using each rotor's height. Height is bounded below by half the propeller radius and the multiplier is capped at 1.20. A smooth tilt fade disables the correction below 0.7 upward alignment. This approximation only modifies lift. It does not reproduce propwash, vortex-ring descent, recirculation or lateral wall/ceiling effects.

Four approximate support points use an iterative inelastic contact solve, with nonnegative accumulated normal impulses and friction limited by μ times normal impulse. A 0.5 mm contact skin avoids alternating support points at rest; 48 iterations limit residual motion without artificial leveling. Ground penetration is corrected vertically. The hard-impact threshold uses the closing speed of a contact point, including rotation. Rotor-disk contact also registers a crash.

Terrain is a flat plane, impacts freeze the aircraft until reset, and obstacle collisions remain approximate. No deformable frame, bouncing propellers, ground material variation or damage physics is present.

## Calibration priorities

1. Replace static thrust and optional torque with measured sweeps at recorded voltage and motor/propeller conditions.
2. Identify rise/fall motor response from time-series measurements, and validate actual inertia/CG if the hardware changes.
3. Fit drag and other aerodynamic loads against flight logs, then match the real Betaflight version, rates and tune.
4. Add voltage/RPM-dependent behavior and descent-flow effects only when there is evidence to constrain them.

A more detailed equation is not by itself a more accurate model. All unmeasured coefficients remain adjustable and explicitly provisional.
