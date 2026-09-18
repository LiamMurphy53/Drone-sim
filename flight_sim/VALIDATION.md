# Validation — 2026-09-18

The current default is configuration **v7g: damped control with steady flight updates**. It preserves v6's stick curve and v5's coast/steering behavior. Earlier AirMode and unconditional motor-cutoff configurations are superseded; see the follow-ups below for the current contract.

Current checks run on this Apple M4 Mac with native arm64 Betaflight 4.5.2 and Godot 4.5.2 using the OpenGL compatibility renderer. The early checks below used Godot 4.4.1 where noted.

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

## Zero-throttle steering follow-up (v5)

The unconditional v4 cutoff also blocked intentional pitch, roll, and yaw. The new configuration distinguishes centered-stick coasting from pilot-commanded steering. It disables MOTOR_STOP, enables PID at minimum throttle, and selects Betaflight's existing EZLANDING mixer. Extra collective authority comes from attitude-stick deflection (`ez_landing_threshold=100`), with zero baseline and speed-based allowances. AirMode, anti-gravity, and throttle boost remain off. Low throttle resets integral correction. The bridge masks stale motor packets only when throttle and attitude sticks are centered; it no longer blocks steering solely because throttle is low. The 0.4-second collective power-return ramp remains, and steering is available during it.

All forces and rotations still come from individual motors and the rigid-body plant. No physical model parameters, thrust data, PID gains, or attitude integration were changed. This is a custom control configuration: centering attitude sticks at zero throttle releases control rather than braking rotation as standard AirMode would. Countersteering supplies braking torque, and steering can produce some lift despite zero collective input. The HUD and pilot instructions explain this behavior.

The new `--test --scenario steering` test runs actual Betaflight with six independent airborne initial conditions: both roll directions, pitch up/down, and both yaw directions. Each maneuver applies moderate stick input, releases to coast, countersteers against existing rotation, then releases again. It exercises 0%, 0.5%, and 1% throttle, integrates the initial signed response, checks motor commands, checks continued arming/connection, and compares every release with a passive plant initialized from the same state. There is no state reset within a maneuver. The separate throttle regression covers continuous takeoff, coasting, and power return.

Both aircraft passed all 31 zero-throttle steering assertions on native Betaflight 4.5.2 and Godot 4.5.2 at 60 FPS/500 Hz physics. All six directions and countersteering worked. Applied and native motor outputs returned to zero in the release checks, and passive-reference motion matched exactly. Maximum body-rate magnitude during these moderate steering/countersteering cases was approximately 1.79 rad/s for GoPro Drone and 1.11 rad/s for DJI FPV. These are software regression results with provisional plant parameters, not measured flight-dynamics validation.

Both 12-assertion continuous throttle/coast tests and both 11-assertion powered-flight tests also passed. The GoPro and DJI maximum rates in the throttle scenario were approximately 3.28 and 0.63 rad/s, with final settling rates of 0.013 and 0.0045 rad/s. Both coast trajectories matched their passive reference exactly. The GoPro remains sensitive to collective changes because of its asymmetric mass/motor geometry and provisional tune. Physical Pocket feel still needs the user's flight check; saved calibration was preserved.

All 15 aircraft/profile/UI assertions passed, for 123 assertions across the final v5 regression runs. Python syntax and Git whitespace checks passed. The native window was reopened and visually checked: DJI remained selected, the Pocket was detected as calibrated, the new pilot instructions fit, and returning to the flight view worked. No radio calibration or measured-parameter files were modified.

## Stick-response follow-up (v6)

The user identified slow response to stick movement as the remaining floatiness. This change raises roll/pitch RC Rate from 0.80 to 1.00 and yaw from 0.70 to 0.90, reduces expo from 0.20/0.20/0.15 to 0.05 on all axes, and disables adaptive RC smoothing. Super Rate remains 0.60/0.60/0.50, so the response is still curved. PID gains, motor lag, mass, thrust, torque, inertia, the 0.4-second collective ramp, and v5's zero-throttle coast/steering behavior are unchanged.

Before modifying the configuration, both aircraft ran the same instrumented 35%-stick direction/countersteering scenario under v5. A repeat with v6 showed earlier rotation and more angular travel in the first 150 ms on every direction. "Response delay" below means time from stick movement to 0.2 rad/s rotation in the commanded direction; it includes the plant's response and is not an isolated USB or transport latency measurement.

| Aircraft | Mean response delay across six directions, v5 → v6 | Extra rotation in the first 150 ms, range across directions |
|---|---|---|
| GoPro Drone | 76.7 → 65.3 ms | 37–54% |
| DJI FPV | 118.3 → 97.0 ms | 43–65% |

These measurements use native Betaflight 4.5.2 and Godot 4.5.2 at 60 FPS with 500 Hz plant integration and the default estimated aircraft parameters. The steering test now records response delay, angular rate at 100 ms, and rotation at 150 ms, and enforces onset/early-motion bounds alongside direction, countersteering, coasting, and stability checks. This demonstrates a stronger early response in the tested conditions; real hardware calibration still requires measured data and a pilot check.

Final regression runs passed all **132 assertions**: 43 steering/response checks, 12 continuous throttle/coast checks, and 11 powered-flight checks per aircraft. The repeat response runs passed the new early-motion bounds in every direction. There is normal run-to-run variation from asynchronous native-controller scheduling; the table records the first before/after comparison rather than a guaranteed latency. The saved v5 baseline measurements fall below the new early-motion requirement on all twelve model/direction combinations. Python syntax and Git whitespace checks also passed.


## Large-input control and timing follow-up (v7g, current)

Large commands exposed problems that the earlier small-input regressions missed. The original v6 DJI stress run peaked near 17.4 rad/s during full roll/pitch reversals despite a requested 8.73 rad/s; powered yaw still reached 2.17 rad/s in the late settling window. The controller connection remained active. These are control-tracking measurements, not evidence that USB packets were being dropped.

The native SITL motor transport also used a mutex as a cross-thread semaphore and skipped individual motor writes while its publication gate was closed. A sensor update could therefore expose a mixture of old and new motor values. The patch always computes a complete motor frame and uses a mutex-protected pending flag only to gate publication. A native C harness extracts and exercises the actual patched motor functions for 100 frames. It passes with the patch and fails on the pinned upstream source. The launcher now rebuilds the controller when the tracked patch changes.

The new tune uses roll/pitch P/I/D = 20/3/50 and yaw = 40/1/30, with no feedforward. Integral relief covers all axes during rapid stick changes. D-min and throttle PID attenuation are disabled so damping does not disappear at high power. Gyro and derivative filters have fixed cutoffs. The v6 rate/expo curve, all physical model parameters, and the low-throttle coast/steering contract are preserved. There is no rate-request slew limiter or clamp on physical rotation.

A 30 FPS stress run still destabilized the tuned controller when physics ran in render-frame batches: GoPro roll reached approximately 24.8 rad/s. The final implementation moves custom rigid-body integration and sensor/RC/motor exchange to a paced 500 Hz worker shared by the application and live tests. UI and input sampling remain on the main thread, with shared state protected by a mutex. Worker stalls do not cause bursts of catch-up packets. Stale UI input disarms after 250 ms. This is a best-effort desktop schedule, not hard real-time operation.

The new aggressive regression establishes normal takeoff, then uses independent airborne fixtures for full-stick holds/reversals on each axis, combined-axis commands, and rapid alternating commands. It covers zero, cruise and full collective power. Each maneuver evolves continuously without resetting physical state. Checks require bounded overshoot, relative tracking RMS below 30% in the hold windows, both rotation directions during rapid reversal, and powered settling below 0.5 rad/s at 1.2–1.7 seconds after release. Off-throttle release intentionally coasts and is not required to stop rotation. Fixture altitude keeps ground impacts from concealing control failures.

The existing steering regression retains all response-delay and early-rotation thresholds. Motor authority is now measured over the entire stick hold, including its initial 100 ms impulse; excluding that impulse incorrectly failed fast, well-damped pitch responses. Tests begin arming at 4.5 seconds instead of 4 seconds to provide margin beyond Betaflight's five-second boot grace period plus the launcher's one-second startup interval.

Final runs passed **242 live-controller assertions**: 55 aggressive checks per aircraft at 30 FPS, plus 43 steering, 12 continuous throttle/coast, and 11 normal-flight checks per aircraft at 60 FPS. There was no controller-link or arming loss in these runs. Both passive coast comparisons matched exactly. The physical model constants were unchanged.

| Final measurement | GoPro Drone | DJI FPV |
|---|---:|---:|
| Worst relative tracking RMS in aggressive hold windows | 25.9% | 17.6% |
| Largest powered settling rate across aggressive cases | 0.342 rad/s | 0.312 rad/s |
| Mean 35%-stick onset delay over six directions | 44.0 ms | 63.3 ms |
| Continuous throttle-test peak body rate | 1.872 rad/s | 0.569 rad/s |
| Continuous throttle-test final settling rate | 0.0147 rad/s | 0.0027 rad/s |

Onset uses the same 0.2 rad/s threshold as v6; that earlier run measured means of 65.3 ms and 97.0 ms. These measurements include controller and modeled motor response, not physical USB latency. DJI's powered full-stick roll/pitch peaks are now about 8.71/8.70 rad/s for an 8.73 rad/s request. The asymmetric GoPro model still overshoots during full-power pitch (approximately 12.14 rad/s), so tracking is not exact. Its tune and physical estimates still need measured data and the user's Pocket flight check.

The native scene/profile suite passed 16 checks, including physical motion while the main thread was deliberately stalled. Five flight-clock checks and the native motor-frame harness also passed, for **264 assertions/tests in the final validation set**. Changed Godot scripts passed parse checks; Python syntax, launcher shell syntax and Git whitespace checks passed. The native flight window was reopened and showed 500 Hz physics with DJI selected. No radio was detected at that final check; saved calibration files were not changed. The setup overlay was closed for the next flight.
