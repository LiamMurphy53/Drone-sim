# GoPro Geometry Lab for Liftoff

Experimental Mac mod for comparing the GoPro Drone's deadcat motor layout inside Liftoff. It changes the **actual propeller force application points**, rigid-body mass, center of gravity, and full inertia tensor. Apply also installs a geometry-aware motor allocator and adjusts Liftoff's native controller gains to the new body-axis inertia. The selected propulsion and stick rates remain Liftoff's.

This is a working development prototype, not a calibrated simulation of the finished aircraft. Read the limitations below before using its handling to make design decisions.

## Start on this Mac

1. Keep Steam open and signed in. Quit any already-running Liftoff instance.
2. Double-click **Launch Liftoff Geometry Lab.command**.
3. Acknowledge Liftoff's external modifications notice. The game disables certain competitive features while mods are loaded; this mod does not alter that protection.
4. Open **Single Player → Free Flight**, choose a conventional four-motor drone with suitable motors/props, and stop/reset it on the ground.
5. Open the overlay with **F8** (or its top-left button), then click **Apply GoPro Drone / edited geometry**.
6. Fly using Liftoff's own controller configuration. **Restore stock drone** returns to the original geometry, mass properties, and native PID objects (including their prior state). A reset clears the override; apply it again after resetting.

Use a four-motor drone with the supported ZetaFlight controller. A DJI FPV configuration has also been used for the integration check, but its selected propulsion and drag are still inherited. The prototype checks for the inspected ZetaFlight controller and refuses unsupported builds. The overlay's **LIVE** status means the game's propeller force method has been called on the modified motors; it is not a certificate of real-world accuracy.

Press F8 to hide the editor while flying. With the editor open, F9 applies and F10 restores stock (only while stopped). Liftoff also uses F9 for its PID graph display; use the on-screen Apply button to avoid toggling that display. To run without the mod, quit the game and launch Liftoff normally from Steam. No installed game binaries or Steam launch settings are changed.

## Fresh checkout

Install Liftoff through Steam and run **Setup Geometry Lab.command** once. Setup downloads pinned, checksum-verified dependencies into the ignored `.tools` folder, compiles the mod against your own game installation, runs the geometry checks, and stages the plugin. Requires Python 3 and Apple's Rosetta on Apple Silicon. The game is deliberately launched in Intel mode for the tested BepInEx/MonoMod combination; the build tools run natively.

For a non-default Steam library:

```sh
python3 tools/setup_mac.py --game "/path/to/Liftoff.app"
python3 tools/launch_mac.py --game "/path/to/Liftoff.app"
```

Rebuild after editing C# code with `python3 tools/setup_mac.py --build-only`. Quit and relaunch Liftoff to load the new plugin.

## Editing the drone

The supplied `profiles/gopro-drone.json` imports the original model's 808.819 g mass, CAD motor coordinates, CG, and inertia from `flight_sim/config/aircraft.json`. It includes the off-diagonal inertia terms.

The live editable copy is `.tools/bepinex/BepInEx/config/GoProGeometry/active.json`. The overlay opens its containing folder. **Save edits** saves there; **Reload profile JSON** reads it; **Apply** changes the current drone. Building does not overwrite an existing active profile. Copy a tuned profile into `profiles/` if you want to commit it.

The editor shows millimeters and grams. JSON uses meters, kilograms, and kg·m². Its motor array is flattened in **FL, FR, RL, RR** order, each with **X left, Y backward, Z up**. The inertia array is row-major, about the CG, in those same CAD axes.

The adapter converts to Unity right/up/forward, aligns the average motor center to the stock drone's motor center, and shifts CG by the same offset. This preserves the physically relevant motor-to-CG vectors. Principal-axis decomposition converts the full inertia tensor into Unity's tensor plus rotation.

**Moving motors does not automatically recompute mass, CG, or inertia.** Update those from CAD when evaluating a changed frame. For a controlled comparison you can hold them fixed, but that isolates lever-arm effects rather than predicting the entire redesigned aircraft.

## What it models—and what remains approximate

| Item | Current behavior |
| --- | --- |
| Motor lever arms | Actual Liftoff propeller transforms and force locations change |
| Mass, CG, inertia | Original model values applied to the flight Rigidbody; full tensor retained |
| Radio input and rates | Liftoff's selected ZetaFlight configuration; not real Betaflight firmware |
| Pitch/roll mixer | Allocates bounded motor forces from actual motor-to-CG arms, with automatic hover balance |
| Yaw | Liftoff's simplified direct yaw torque remains; allocator balances opposite spin pairs |
| Thrust, motor response, prop model | Inherited from the stock Liftoff build you selected |
| Body mesh, drag, collision geometry | Stock frame remains; motor/prop assemblies move, arms are not remodeled |
| Controller gains | Native PID objects rebuilt with body-axis inertia scaling; auto mode adds bounded pitch/roll integral correction |
| Windows | Source is portable C#, but loader setup and assembly compatibility remain untested |

The selected stock propulsion and the simplified controller can dominate handling. This version lets us establish whether custom geometry reaches the game's physics, and explore pitch/roll lever arms. It does **not** yet establish that real Betaflight on the GoPro Drone will behave the same way, especially near saturation or in yaw. Your motor/prop thrust data and real flight/blackbox comparisons are still needed for calibration. Real reaction-torque behavior and custom drag/collision geometry remain follow-up work.

## Apply now includes controller compensation

The imported CG is forward of the average motor center. Equal motor thrust creates a nose-down moment; this is not proof that a controlled deadcat must continually pitch forward. Apply solves for the thrust needed at each real force location. The supplied profile needs approximately **63.77% front / 36.23% rear** in level hover. The physical CG and lever arms are not moved to hide this imbalance.

Pitch and roll commands use the same force allocation. Each motor remains between the selected idle force and the game's available maximum. Collective is adjusted to preserve attitude control where possible; impossible pitch/roll requests are scaled together. A forward CG therefore still uses more front-motor headroom and can reduce maximum balanced collective thrust. Profiles that cannot balance positive motor thrust are rejected.

Native ZetaFlight PID gains are rescaled using the full tensor's **body-axis** diagonal, rather than mistaking principal moments for body-axis moments. Existing manual gain proportions are retained. In the game's P-only auto mode, pitch/roll gain I is set to twice P (0.5 s integral time) to reject remaining steady disturbances. Stored integral correction is limited to a moment equivalent to a 20 mm load offset and stops accumulating further into mixer saturation. Input rates/expo and flight-mode selection are unchanged. This is an experimental controller adapter, not a Betaflight firmware implementation or a measured real-aircraft tune.

The adapter replaces the final motor force/RPM outputs before Liftoff applies its own propeller forces and airflow effects. Liftoff's earlier throttle/headroom policy remains in place, so it can be more conservative than the allocator alone. Normal Acro flight is the intended test mode; special launch-assist branches bypass ordinary motor mixing and are not modeled by this correction. Liftoff still supplies the simplified yaw torque independently of motor reaction torque.

**Restore** puts back the original native PID dictionary along with the physical geometry. Changing controller settings while the profile is active disables the override; stop and Apply again. Reset still returns to stock. Neither Apply nor Restore writes over the saved Liftoff drone configuration.

## Compatibility and validation

Target inspected: **Liftoff 1.7.5 Mac**, Steam build **25118475**, Unity **2022.3.62f3**, game assembly MVID `a2543f11-0d93-4fe0-8ffd-ff5ded8253a3`. A version guard disables changes on other game assemblies instead of guessing at obfuscated internals. Do not remove that guard without inspecting and testing the new build.

Validated during development:

- Release compilation with zero warnings/errors.
- 22 math checks: coordinate orientation, full tensor, invalid geometry, CG-balanced hover, moment signs, full-throttle headroom, saturation, reverse thrust, symmetric geometry, and a sweep of 1,000 bounded throttle/attitude requests.
- New isolated controller check using the game's native PID and actual propeller force method with the GoPro full inertia tensor: neutral throttle steps produced only 0.000045 rad/s peak numerical rotation; an initial 2 rad/s pitch plus 1 rad/s roll disturbance settled below 0.01 rad/s; a persistent 0.02 Nm pitch disturbance was rejected. These isolate controller/force correctness and do not include the selected drone's drag, wind, or RC input.
- Loaded in the Mac game and displayed the editor in Free Flight.
- Applied the GoPro geometry to a loaded drone: mass changed from 0.795000 to 0.808819 kg; the CG, full inertia, and four motor positions changed. A subsequent reset returned stock values. World/local motor readback was within 0.1 mm in this test; the live guard allows 1 mm for large-map floating-point roundoff.
- Called **Liftoff's actual `Propeller.ApplyControllerForceAtPropeller`** in an isolated Unity physics scene. At 1 N for 2 ms, a 100 mm lateral arm produced 0.050000 rad/s; a 200 mm arm produced 0.100000 rad/s. A forward arm gave the expected pitch sign and magnitude. No game drone or global simulation settings were changed by this test.
- **Controller adapter integration, September 18, 2026:** Apply installed the new auto PI controller and geometry allocator on the loaded 0.795 kg DJI FPV configuration. The log confirmed native powered motor calls after 55 geometry-mixer steps, with FL/FR/RL/RR hover shares 31.89/31.88/18.12/18.11%. Restore returned the stock controller, 0.795 kg mass and original mass properties; applying again restored the corrected GoPro configuration. The initial Apply attempt was correctly refused while the body had not settled.
- **Powered Free Flight smoke check, September 18, 2026 (before controller adapter):** launched the GitHub repository build with the RadioMaster Pocket connected. Liftoff detected `Radiomaster Pocket Joystick`; the user confirmed arming and flight, and the view showed the drone airborne. After the GoPro profile was applied, the plugin logged `Verified live Liftoff propeller-force calls using the custom motor positions.` A subsequent reset rebound the stock 0.795000 kg drone and its original mass properties. No geometry-guard errors were logged during this check.

This confirms integration during powered flight; **it does not validate real-world handling accuracy**. A powered user comparison of the new controller compensation and calibration against the real aircraft remain outstanding. Motor-allocation saturation is tested; real yaw authority is not. Resets clear the override: reapply the profile before each GoPro test flight. Automated mouse input was unreliable on this Mac; keyboard shortcuts are provided as a fallback.

Logs:

- `runtime/player.log`: Unity/game log.
- `.tools/bepinex/BepInEx/LogOutput.log`: loader/plugin log and force check.
- `.tools/bepinex/BepInEx/config/GoProGeometry/last-applied.txt`: actual applied mass properties and motor positions, when applied.

If startup stalls, check Steam is signed in and focus the Liftoff window. If the game is already open, quit before relaunching. The isolated loader uses `HideManagerGameObject=true` to retain the plugin across scene cleanup; Liftoff's modification detection remains active.

## Dependencies and source inspection

- [BepInEx 5.4.23.5](https://github.com/BepInEx/BepInEx/releases/tag/v5.4.23.5), universal Mac loader.
- [Microsoft .NET 8 SDK](https://dotnet.microsoft.com/download/dotnet/8.0), pinned to 8.0.425.
- Unity/Liftoff assemblies referenced from the owner's installed game; **not redistributed**.

`tools/Inspector` is an optional read-only Mono.Cecil developer utility for finding force-related call sites. Proprietary inspection output, logs, downloads, compiled binaries, and active profiles are ignored by Git. Only our adapter, math, tests, launcher, profile, and documentation belong in the repository.
