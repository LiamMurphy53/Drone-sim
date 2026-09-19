# GoPro Geometry Lab for Liftoff

Experimental Mac mod for comparing the GoPro Drone's deadcat motor layout inside Liftoff. It changes the **actual propeller force application points**, rigid-body mass, center of gravity, and full inertia tensor. It keeps Liftoff's selected propulsion and controller.

This is a working development prototype, not a calibrated simulation of the finished aircraft. Read the limitations below before using its handling to make design decisions.

## Start on this Mac

1. Keep Steam open and signed in. Quit any already-running Liftoff instance.
2. Double-click **Launch Liftoff Geometry Lab.command**.
3. Acknowledge Liftoff's external modifications notice. The game disables certain competitive features while mods are loaded; this mod does not alter that protection.
4. Open **Single Player → Free Flight**, choose a conventional four-motor drone with suitable motors/props, and stop/reset it on the ground.
5. Open the overlay with **F8** (or its top-left button), then click **Apply GoPro Drone / edited geometry**.
6. Fly using Liftoff's own controller configuration. **Restore stock drone** returns to the original geometry and mass properties. A reset clears the override; apply it again after resetting.

Start with a regular quad, not the DJI FPV or a non-quad. The prototype checks for the inspected ZetaFlight controller and refuses unsupported builds. The overlay's **LIVE** status means the game's propeller force method has been called on the modified motors; it is not a certificate of real-world accuracy.

Press F8 to hide the editor while flying. With the editor open, F9 applies and F10 restores stock (only while stopped). To run without the mod, quit the game and launch Liftoff normally from Steam. No installed game binaries or Steam launch settings are changed.

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
| Radio input, rates, stabilization | Liftoff's selected ZetaFlight configuration; not real Betaflight firmware |
| Mixer and yaw | Stock symmetric mixer and simplified direct yaw torque remain |
| Thrust, motor response, prop model | Inherited from the stock Liftoff build you selected |
| Body mesh, drag, collision geometry | Stock frame remains; motor/prop assemblies move, arms are not remodeled |
| Controller gains and cached effects | Stock initialization retained; gains are not retuned for the GoPro mass/inertia |
| Windows | Source is portable C#, but loader setup and assembly compatibility remain untested |

The selected stock propulsion and the simplified controller can dominate handling. This version lets us establish whether custom geometry reaches the game's physics, and explore pitch/roll lever arms. It does **not** yet establish that real Betaflight on the GoPro Drone will behave the same way, especially near saturation or in yaw. Your motor/prop thrust data and real flight/blackbox comparisons are still needed for calibration. Geometry-aware mixing, real reaction-torque behavior, and custom drag/collision geometry would be separate follow-up work.

## Compatibility and validation

Target inspected: **Liftoff 1.7.5 Mac**, Steam build **25118475**, Unity **2022.3.62f3**, game assembly MVID `a2543f11-0d93-4fe0-8ffd-ff5ded8253a3`. A version guard disables changes on other game assemblies instead of guessing at obfuscated internals. Do not remove that guard without inspecting and testing the new build.

Validated during development:

- Release compilation with zero warnings/errors.
- Geometry math tests: coordinate orientation, CG lever arms, full tensor reconstruction, proper principal-axis rotation, and invalid profile rejection.
- Loaded in the Mac game and displayed the editor in Free Flight.
- Applied the GoPro geometry to a loaded drone: mass changed from 0.795000 to 0.808819 kg; the CG, full inertia, and four motor positions changed. A subsequent reset returned stock values. World/local motor readback was within 0.1 mm in this test; the live guard allows 1 mm for large-map floating-point roundoff.
- Called **Liftoff's actual `Propeller.ApplyControllerForceAtPropeller`** in an isolated Unity physics scene. At 1 N for 2 ms, a 100 mm lateral arm produced 0.050000 rad/s; a 200 mm arm produced 0.100000 rad/s. A forward arm gave the expected pitch sign and magnitude. No game drone or global simulation settings were changed by this test.

**Powered flight and controller handling are not yet verified.** The loaded drone remained disarmed during the application/reset check. Do not treat this as flight-validated until an armed session logs `Verified live Liftoff propeller-force calls` and its handling is tested. Automated mouse input was unreliable on this Mac; keyboard shortcuts are provided as a fallback.

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
