# Validation — 2026-09-18

Tested on this Apple M4 Mac with native arm64 Betaflight 4.5.2 and Godot 4.4.1 using the OpenGL compatibility renderer.

## Automated checks

- **15 legacy physics assertions passed:** imported mass/thrust/inertia, free-fall velocity and position, zero-g accelerometer, asymmetric hover trim, hover altitude, quaternion normalization, single-motor torque signs, inherited thrust-lag mode, rendering handedness, ground support, supported 1-g acceleration, and hard impact. New aero/ground effect are disabled for this baseline comparison.
- **33 new dynamics assertions passed:** rotor-speed response and coast-down, independent response times, measured thrust/torque interpolation, invalid-curve rejection, dissipative aerodynamic forces/moments, directional and quadratic drag, rotor-local motion, relative wind, bounded/tilt-dependent ground effect, torque-free energy/angular momentum, timestep convergence, resting/sliding contact, impact energy and rotor-disk strikes.
- **15 aircraft/profile assertions passed:** named profiles, separate physics/measurement paths, DJI published dimensions/mass, positive inertia, trimmed DJI hover and ground rest, unknown-profile rejection, and actual scene selection/reset behavior. Repeated switching clears flight/motor state and does not accumulate meshes.
- **6 Python importer tests passed**, including invalid data and units, gram-force conversion, optional torque, independent model destinations, and protection against accidentally overwriting an existing measured curve.
- **11 live Betaflight assertions passed for each aircraft:** packet exchange, RC arming, takeoff, initial roll/pitch/yaw directions, no unexpected disarming, bounded small-input response, settling after centering sticks, no crash, and requested disarming. Both tests use the same bridge as keyboard/radio inputs.
- Godot script import, Python syntax, and launcher shell syntax checks passed.

The live flight test uses a higher fixed throttle for DJI's lower estimated maximum thrust. It leaves altitude margin for attitude maneuvers and the final motor-off interval; it is not an altitude-hold test. Direction checks integrate the signed response while each stick command is held, so a later rebound cannot conceal a reversed axis.

The expanded test exposed a yaw convention mismatch in the earlier bridge. This revision delivers internally consistent FLU gyro/attitude axes, uses standard QUADX yaw mixing, and reduces the provisional yaw integral gain to prevent prolonged oscillation. The private controller configuration is versioned so existing simulator installations receive the correction. Measured maximum body rates during the final settling window were approximately 0.012 rad/s for GoPro Drone and 0.069 rad/s for DJI FPV, below the 0.2 rad/s test threshold.

## Limits

These checks validate code behavior and numerical consistency, not agreement with measured flight dynamics. Neither aircraft is hardware-validated. DJI inertia, CG, motor layout, thrust/torque, response time and aerodynamic coefficients are estimates; DJI firmware/assisted flight modes are not reproduced. Sources and assumptions are recorded in [AIRCRAFT_MODELS.md](docs/AIRCRAFT_MODELS.md) and [PHYSICS.md](docs/PHYSICS.md).

A physical RadioMaster Pocket is now detected and its live stick inputs are confirmed (see the follow-up below). Channel calibration and a physical-radio flight check remain required. The MATLAB project remains in `matlab_sim/`; its full test suite was not rerun as part of the native simulator work.

## Native UI and relocation

The simulator launched successfully from `Desktop/Github/Drone-sim`. The native window was inspected with both selector entries, the DJI approximation label, remembered selection after restart, and the DJI procedural model in chase view. The launch-pad markings were flattened to match the ground plane for the shorter DJI landing footprint. Source, model data, tests, documentation and the MATLAB project are tracked; downloaded engines, compiled dependencies, caches and local runtime files are excluded from Git.

## Pocket connection follow-up

macOS detected USB vendor 0x1209 / product 0x4f54 as Radiomaster Pocket Joystick, while the original Godot 4.4.1 engine returned an empty controller list even after restarting. Godot 4.5.2, whose desktop input uses SDL, detected `EdgeTX Radiomaster Pocket Joystick`; the user confirmed that moving the physical sticks changes the simulator's raw input values. The official macOS engine archive was verified against its release SHA-256 before installation. Previous local engines were retained as backups.

The setup script now pins and upgrades to 4.5.2. The simulator rescans after startup and periodically, and provides a manual rescan button that disarms. All 15 legacy physics and 33 dynamics checks passed again on 4.5.2. The earlier live Betaflight and profile tests above were run on 4.4.1; they were not rerun as part of this controller-detection fix. Physical channel mapping and flight are not yet claimed as validated.

References: [Godot desktop controller support](https://docs.godotengine.org/en/stable/tutorials/inputs/controllers_gamepads_joysticks.html), [Godot 4.5.2 release](https://github.com/godotengine/godot-builds/releases/tag/4.5.2-stable).
