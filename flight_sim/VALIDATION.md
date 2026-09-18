# Validation — 2026-09-18

The current default is configuration **v5: Acro with stick-driven zero-throttle steering**. The earlier AirMode and unconditional motor-cutoff configurations are superseded; see the v5 follow-up for the current contract.

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

## Airborne throttle-cut follow-up (v3, superseded)

The reported throttle-cut instability was reproduced with native Betaflight. The previous configuration reduced roll/pitch P and I for the inherited 50 ms motor response but retained the stock anti-gravity gain of 80. Betaflight adds this transient integral boost independently of the configured I gain, and also boosts P during throttle changes. The stock boost was excessive for this provisional plant/tune. AirMode was already enabled.

Configuration v3 reduces `anti_gravity_gain` to 8, retains the original base P/I/D/rate settings and stock `anti_gravity_p_gain = 100`, and explicitly enables AirMode and disables MOTOR_STOP. This is a simulator tune, not a hardware-validated tune. Existing installations receive it on launch, without changing saved Pocket calibration or aircraft measurements.

The new `--test --scenario throttle` regression checks connection, arming, airborne flight, bounded rates, settling during three low-throttle intervals, settling after restored throttle, commanded roll at zero throttle, and disarming. It uses the default unmeasured aircraft models. An automated pilot steers upright through ordinary RC inputs between maneuvers, then stops steering before each maneuver and throughout the measurement windows. It never resets orientation/position in flight, clamps angular velocity, or adds auto-leveling to the application. Settling means less than 0.2 rad/s across a half-second window approximately two seconds after the transient; it does not imply instantaneous response.

Both aircraft passed all 10 throttle assertions on Godot 4.5.2 at 60 FPS with 500 Hz physics. Maximum rates within the three low-throttle settling windows were approximately 0.052 rad/s for GoPro Drone and 0.010 rad/s for DJI FPV; final throttle-recovery settling rates were 0.058 and 0.0013 rad/s respectively. The GoPro's takeoff and collective changes still require pilot correction in Acro, and its response remains slower than a tuned real quad. These checks establish bounded behavior for the tested maneuvers, not flight-data accuracy or freedom from all wobble.

The existing 11-assertion normal-flight integration test also passed again for each aircraft at 60 FPS on Godot 4.5.2: takeoff, all three initial rotation directions, settling, continued arming, and disarming. In total, this follow-up passed 42 live-controller assertions. Python syntax and Git whitespace checks passed. The underlying physics equations were not modified in this follow-up.

As a negative control, the exact new GoPro throttle scenario was rerun at 60 FPS with the original saved controller configuration in a temporary runtime directory. It failed four assertions: staying airborne, bounded rotation, and the first two low-throttle settling windows. The v3 configuration passed the same scenario. This verifies that the regression catches the original fault; the production runtime retained v3 throughout this comparison.

## Motor-cutoff follow-up (v4, superseded)

The requested behavior now explicitly gives up off-throttle stabilization. AirMode, anti-gravity, and throttle boost are off; MOTOR_STOP is on and `pid_at_min_throttle` is off. The bottom 1% of calibrated throttle sends zero to Betaflight and immediately cuts the bridge's applied motor commands, including when an older powered motor packet is still in flight. Betaflight also stops its own motor outputs and clears integral correction while throttle is low. Power returns through a 0.4-second throttle ramp, then follows the stick normally. Cuts bypass the ramp. Powered Acro rate control remains active; this is not direct stick-to-motor control.

The throttle regression now checks this contract instead of v3's zero-throttle steering/settling expectations. It exercises zero, 0.5%, and the 1% boundary with roll/pitch/yaw sticks deflected. A second physical plant starts with the exact aircraft state and rotor speeds at each cutoff and evolves with zero motor commands. Both motion and attitude must match this passive reference, with nonzero travel confirming that motion is not frozen. The test separately checks the raw Betaflight outputs, so masking controller outputs in the application alone cannot pass. Takeoff and between-maneuver steering use ordinary RC channels only.

Both aircraft passed all 12 motor-cutoff assertions at 60 FPS/500 Hz physics on Godot 4.5.2: immediate zero commands, zero native controller outputs, natural rotor coast-down, exact passive-reference motion, continued arming, airborne flight, bounded power return, resumed powered steering, settling, and requested disarming. The GoPro's brief power-return response is still more pronounced than the DJI approximation's; the provisional mass properties, motor response, and powered tune remain subject to measurement and flight-data validation. No mass, thrust, torque, inertia, or motor-response numbers were changed for this behavior change.

Both 11-assertion powered-flight direction tests and all 15 aircraft/profile/UI checks also passed (61 assertions total for this follow-up). The axis test now uses pilot RC corrections to establish upright flight during seconds 6–9, then stops them before every direction and settling check. This avoids assuming a hands-off launch from the asymmetric GoPro frame with the reduced assistance. The application does not include this test pilot. Script syntax and Git whitespace checks passed; Pocket calibration and aircraft selection are preserved separately from the v4 controller update.

## Zero-throttle steering follow-up (v5, current)

The unconditional v4 cutoff also blocked intentional pitch, roll, and yaw. The new configuration distinguishes centered-stick coasting from pilot-commanded steering. It disables MOTOR_STOP, enables PID at minimum throttle, and selects Betaflight's existing EZLANDING mixer. Extra collective authority comes from attitude-stick deflection (`ez_landing_threshold=100`), with zero baseline and speed-based allowances. AirMode, anti-gravity, and throttle boost remain off. Low throttle resets integral correction. The bridge masks stale motor packets only when throttle and attitude sticks are centered; it no longer blocks steering solely because throttle is low. The 0.4-second collective power-return ramp remains, and steering is available during it.

All forces and rotations still come from individual motors and the rigid-body plant. No physical model parameters, thrust data, PID gains, or attitude integration were changed. This is a custom control configuration: centering attitude sticks at zero throttle releases control rather than braking rotation as standard AirMode would. Countersteering supplies braking torque, and steering can produce some lift despite zero collective input. The HUD and pilot instructions explain this behavior.

The new `--test --scenario steering` test runs actual Betaflight with six independent airborne initial conditions: both roll directions, pitch up/down, and both yaw directions. Each maneuver applies moderate stick input, releases to coast, countersteers against existing rotation, then releases again. It exercises 0%, 0.5%, and 1% throttle, integrates the initial signed response, checks motor commands, checks continued arming/connection, and compares every release with a passive plant initialized from the same state. There is no state reset within a maneuver. The separate throttle regression covers continuous takeoff, coasting, and power return.

Both aircraft passed all 31 zero-throttle steering assertions on native Betaflight 4.5.2 and Godot 4.5.2 at 60 FPS/500 Hz physics. All six directions and countersteering worked. Applied and native motor outputs returned to zero in the release checks, and passive-reference motion matched exactly. Maximum body-rate magnitude during these moderate steering/countersteering cases was approximately 1.79 rad/s for GoPro Drone and 1.11 rad/s for DJI FPV. These are software regression results with provisional plant parameters, not measured flight-dynamics validation.

Both 12-assertion continuous throttle/coast tests and both 11-assertion powered-flight tests also passed. The GoPro and DJI maximum rates in the throttle scenario were approximately 3.28 and 0.63 rad/s, with final settling rates of 0.013 and 0.0045 rad/s. Both coast trajectories matched their passive reference exactly. The GoPro remains sensitive to collective changes because of its asymmetric mass/motor geometry and provisional tune. Physical Pocket feel still needs the user's flight check; saved calibration was preserved.

All 15 aircraft/profile/UI assertions passed, for 123 assertions across the final v5 regression runs. Python syntax and Git whitespace checks passed. The native window was reopened and visually checked: DJI remained selected, the Pocket was detected as calibrated, the new pilot instructions fit, and returning to the flight view worked. No radio calibration or measured-parameter files were modified.
