# Flight Lab

A native macOS flight simulator using **Godot 4.5.2** and a local **Betaflight 4.5.2 SITL** process. Both programs run locally; there is no server or account.

## Start flying

Double-click **Launch Flight Sim.command**. The launcher starts Betaflight and the flight window, and stops its Betaflight process when the window closes. The downloaded engine and compiled controller are already present on this Mac.

Use the **DRONE** selector at the top left to choose **GoPro Drone** or **DJI FPV**. Switching disarms, clears motor state, and returns to the launch pad; your selection is remembered. DJI's mass, envelope, diagonal, and propeller size use published specifications, but its inertia, thrust, drag and motor response are estimates. Both aircraft use **Betaflight Acro**. DJI's proprietary controller, Normal/Sport modes, GPS, emergency braking and camera stabilization are not reproduced. Sources and derivations are in [AIRCRAFT_MODELS.md](docs/AIRCRAFT_MODELS.md).

1. Connect the RadioMaster Pocket using its **USB data port** and a data-capable cable. Select **USB Joystick** on the Pocket. Use a dedicated simulator model with unmixed stick channels.
2. Open **Pilot setup** with **Tab**, select the radio, and click **Calibrate Pocket**. Follow the six guided steps, including sweeping both sticks through their full travel. Axis order and direction are detected rather than assumed.
3. Optionally bind arm and reset switches. Begin with the switch OFF, click its binding button, then move it ON. Return it OFF before flying. If a switch does not register, assign it to an output channel in the radio's simulator model.
4. Return to flight, lower throttle fully, wait for Betaflight's startup checks, and press **Space** or toggle the bound arm switch.
5. Raise throttle gradually. With power on, **acro/rate mode** uses your sticks to command rotation; centered sticks ask it to stop rotating, without leveling the aircraft or stopping its travel.
6. With throttle fully down, **pitch, roll, and yaw still steer**. The motors supply the torque needed for your stick command, including pitching up during a descent. Center the attitude sticks to **coast**: motor commands drop to zero, the rotors spin down naturally, and existing motion, rotation, drag, and gravity continue. Countersteer to stop a turn while throttle is off. The bottom 1% of throttle accommodates endpoint jitter. The HUD distinguishes **LOW-THROTTLE STEERING** from **COAST**.

Calibration is saved per reported device name in Godot's application user-data folder. Recalibrate after changing the Pocket's channel setup. Pocket detection and changing stick inputs have been verified on this Mac. Complete the guided calibration before flying; a calibrated physical-radio flight check is still required.

The engine must be **Godot 4.5 or newer** for the SDL controller path used here. Earlier versions can miss a generic EdgeTX USB joystick even when macOS detects it. The pinned setup script installs 4.5.2 and upgrades an older local engine. Controller detection is rescanned after startup and periodically; **Rescan USB controllers** also retries manually and disarms. If macOS cannot see the radio at all, reconnect with the Mac unlocked, allow the accessory if prompted, and verify the data cable/top USB port before calibrating.

No radio connected? Select **Keyboard practice**. W/S increases/decreases throttle; arrow keys control pitch and roll; A/D controls yaw. Keyboard throttle holds its value when released.

| Control | Action |
|---|---|
| Space | Arm/disarm; throttle must be low to arm |
| R | Disarm and reset to the launch pad |
| C | FPV / chase camera |
| Tab | Show/hide pilot setup; opening disarms |
| Escape | Pause/resume; disarms |
| F | Fullscreen/windowed |

Losing window focus pauses aircraft motion and disarms. Disconnecting the radio disarms. Resuming or reconnecting never automatically arms. A hard ground impact or contact with a gate/building requires a reset. The field has five numbered gates and a circuit counter; trees and distant mountains are scenery.

## What is simulated

The independent `scripts/aircraft.gd` model integrates position, velocity, quaternion attitude, and body angular velocity at **500 Hz**, using midpoint force evaluation. It uses the full inertia tensor and each motor's position relative to the CG. Betaflight receives virtual sensor packets and RC commands and returns individual motor outputs; there is no substitute PID controller in the game.

`config/aircraft.json` contains the imported **GoPro Drone** model:

- Mass: 0.808819 kg.
- Original deadcat geometry, CG, inertia tensor, and FL/FR/RL/RR spin directions.
- Maximum thrust: 13.5 N per motor.
- Inherited motor response-time estimate: 0.05 s.
- Reaction torque/thrust ratio: 0.013 m.
- Linear normalized motor-command-to-thrust relationship, as in the legacy plant.

The physics model now includes:

- **Directional body drag**, calculated in body axes from air-relative velocity. Wind is configurable, with zero wind by default.
- **Rotor-local airflow forces**, including the motion of each rotor when the aircraft rotates, and rotational damping.
- **Rotor spin-up/coast-down dynamics** and squared-speed thrust. The steady command-to-thrust curve remains the original linear estimate until measurements are imported. Spin-up and spin-down response times can be set independently.
- **Bounded ground effect** using each rotor's height, fading out with tilt.
- **Four-point ground contact**, inelastic normal impulses, friction, and rotor-disk ground strikes. Field obstacle collisions remain simplified.

The added coefficients are provisional, not measured for either aircraft. `config/physics.json` belongs to GoPro Drone; `config/aircraft/dji_fpv_physics.json` belongs to DJI FPV. The profile registry in `config/aircraft_catalog.json` keeps physical models and measurements separate. [PHYSICS.md](docs/PHYSICS.md) describes the equations, assumptions and references.

Propwash/vortex-ring effects, battery sag, rotor gyroscopic inertia, sensor noise and a hardware-validated tune remain future work. Throttle percentage is the radio input through Betaflight's throttle processing, not a direct motor-command or thrust percentage. A static thrust sweep improves propulsion calibration but does not validate aerodynamic behavior or the controller tune by itself.

The starter Betaflight tune remains gentle and provisional for the inherited 50 ms motor lag. The default uses **Acro with stick-driven low-throttle authority**. This is a custom simulator configuration, not standard AirMode behavior: at zero throttle, centered sticks coast instead of requesting a stop to rotation. AirMode and MOTOR_STOP are off, while PID at minimum throttle is on so deflected sticks can steer. Betaflight's EZLANDING mixer sets the extra collective allowance from attitude-stick deflection, with no baseline or speed-based allowance (`ez_landing_limit=0`, `ez_landing_speed=0`, `ez_landing_threshold=100`). Betaflight still supplies the motor mix; the game does not rotate the aircraft directly. Motor-driven steering can add some lift even at zero throttle.

Stick response uses Betaflight's curved rate mapping: **RC Rate 1.00 / 1.00 / 0.90**, **expo 0.05 on all axes**, and **Super Rate 0.60 / 0.60 / 0.50** for roll/pitch/yaw. Expo softens the center and Super Rate steepens the outer stick response. Compared with v5's 0.80/0.80/0.70 base rates and 0.20/0.20/0.15 expo, the new curve gives approximately 48% more roll/pitch and 44% more yaw sensitivity near center. Full-stick setpoints are 500°/s roll/pitch and 360°/s yaw; physical response can lag those commands. Adaptive RC smoothing is off to avoid additional filtering of the frame-batched simulator input. The calibrated 2.5% stick deadband still rejects center jitter. PID gains, physical motor response, and the low-throttle coast/steering behavior are unchanged.

Betaflight clears integral correction below the throttle threshold. Anti-gravity and throttle boost stay disabled to avoid the earlier throttle-change kicks. The bridge sends true zero throttle within the 1% band and immediately stops stale motor packets only when attitude sticks are also centered. The native mixer likewise produces zero outputs with all sticks centered at zero throttle. Rotor spin-down and rigid-body motion remain physical. Raising throttle needs no arm-switch toggle and uses a 0.4-second collective ramp to soften power return; steering remains available during that ramp. Afterward, throttle follows the stick normally. Lowering throttle bypasses the ramp.

The stock QUADX mixer uses standard yaw direction, matched to the plant's spin signs and the bridge's FLU rate convention. The physical deadcat asymmetry is in the plant; Betaflight's controller must compensate for it. Only Acro is exposed. Actual hardware firmware version, tune, rates and mixer should be matched later. The versioned private simulator configuration updates automatically on launch; Pocket calibration is stored separately and is preserved.

Import updated legacy inputs with:

```sh
python3 flight_sim/tools/import_config.py
```

Run that command from the repository root. It only overwrites GoPro Drone's `config/aircraft.json`; physics settings, DJI files and measured curves are preserved. This exporter supports the numeric format currently used by the MATLAB inputs, not arbitrary MATLAB expressions. `python3 flight_sim/tools/build_dji_profile.py` regenerates the documented DJI approximation and its estimated physics settings, preserving measured curves.

## Measured thrust data

Supply a CSV with **`motor_command_0_1`** and exactly one of **`thrust_N`** or **`thrust_g`** (grams-force). Add **`torque_Nm`** only if torque was measured at every point. The motor command is normalized ESC command, not radio-stick throttle. Include zero command/zero thrust and a measured full-command endpoint; intermediate readings must be monotonic. No missing endpoint is extrapolated, and noisy data is not silently smoothed.

From the repository root:

```sh
python3 flight_sim/tools/import_thrust_data.py /path/to/measurements.csv
```

This targets **GoPro Drone** by default and writes `config/propulsion_curve.json`. Restart the simulator to load it. An existing curve is protected unless `--force` is supplied. For DJI-specific measurements, add `--drone dji_fpv`; its curve is stored separately. Invalid loaded curves block arming and display an error. Remove/rename the selected aircraft's curve to return to the provisional mapping.

Keep voltage, motor/propeller identity, and the test conditions with the readings. RPM and time-series step-response measurements would also help later. This version models one static test condition; importing a thrust curve does not supply battery-voltage behavior, true rotor RPM, torque (if omitted), or motor response time.

## Development and verification

From `flight_sim`:

```sh
.tools/Godot.app/Contents/MacOS/Godot --headless --path . --script res://tests/test_physics.gd
.tools/Godot.app/Contents/MacOS/Godot --headless --path . --script res://tests/test_dynamics.gd
.tools/Godot.app/Contents/MacOS/Godot --headless --path . --script res://tests/test_aircraft_profiles.gd
python3 tests/test_thrust_import.py
python3 tools/launch.py --test
python3 tools/launch.py --test --aircraft dji_fpv
python3 tools/launch.py --test --scenario throttle
python3 tools/launch.py --test --scenario throttle --aircraft dji_fpv
python3 tools/launch.py --test --scenario steering
python3 tools/launch.py --test --scenario steering --aircraft dji_fpv
```

The suites cover imported units, free fall, hover trim, rotor response, dissipative drag, angular momentum/energy conservation, timestep convergence, ground contact, measurements, and aircraft switching. Live tests run actual Betaflight. The flight scenario checks arming, takeoff, all three rotation directions, controlled response and disarming. The throttle scenario checks uninterrupted takeoff, centered-stick coasting, endpoint jitter, natural rotor spin-down, and powered recovery. Its coast is compared to a second physical model with identical initial state and zero motor commands. An automated pilot steers upright through ordinary RC inputs between maneuvers, never during coasting or the final settling window. The steering scenario separately tests both directions on each axis at zero throttle, including pitch up, countersteering against existing rotation, and passive coasting after release. Each direction starts from an independent airborne initial condition; no state reset occurs within a maneuver. Both low-throttle scenarios use the unmeasured default aircraft parameters and check the native controller outputs as well as the applied commands.

Live tests default to 60 rendered frames per second with 500 Hz physics; `--test-fps 30` or `--test-fps 120` exercises different frame batching. Run profile/UI and live integration tests with other simulator instances closed. The legacy comparisons explicitly disable newer aerodynamic effects and select the old thrust-lag model; the dynamics suite tests the new defaults.

Open `project.godot` with the bundled Godot editor to develop the scene. Runtime meshes create the field, aircraft and interface; no asset downloads are needed while flying.

To rebuild dependencies on another Mac with Apple's command-line developer tools:

```sh
./tools/setup.sh
```

Versions and download checksums are pinned. `tools/patch_betaflight.py` applies narrow simulator transport fixes: a real-time clock rather than per-packet time scaling, sensor initialization guards, and serialized TCP stream operations. Betaflight's PID implementation is unchanged. The configure script operates exclusively on the private localhost simulator and its `runtime/eeprom.bin`; it never connects to a physical flight controller.

Logs and private Betaflight configuration live under `runtime/`. Godot user calibration lives outside the project in its standard application data location. Settings can be regenerated with `python3 tools/configure_betaflight.py` while the simulator is closed.

The bridge is pinned to the 4.5.2 packet ABI: 144-byte state packets to UDP 9003, 40-byte RC packets to UDP 9004, 68-byte raw motor packets from UDP 9001, and MSP status on TCP 5761. `scripts/betaflight.gd` documents coordinate transformations and motor order. Don't upgrade the firmware without checking these boundaries.

See [THIRD_PARTY.md](THIRD_PARTY.md) for dependencies and source/licensing information.

Godot's internal application name remains `Deadcat Flight Lab` so existing Pocket calibration is preserved. The flight window and aircraft selector use the new names. Git excludes `.tools/`, `vendor/`, `.godot/` and `runtime/`; pinned dependency downloads, source patches and build instructions are tracked instead.
