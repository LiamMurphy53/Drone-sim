# Aircraft profiles and provenance

## GoPro Drone

The original deadcat aircraft has been renamed **GoPro Drone** in the simulator. Its mass, CG, full inertia tensor, motor locations, spin directions, 13.5 N per-motor thrust estimate, 0.013 m reaction-torque/thrust estimate and 50 ms response-time estimate come from the existing MATLAB inputs. Propeller radius is exported from the same inputs (5.1-inch diameter). None of those inherited performance estimates has been replaced by invented bench measurements.

`tools/import_config.py` exports this aircraft to `config/aircraft.json`. Its aerodynamic/contact settings are in `config/physics.json`; its optional measured thrust curve is `config/propulsion_curve.json`.

## DJI FPV

This is an approximate rigid-body aircraft profile, not a digital twin or reproduction of DJI firmware. The simulation uses Betaflight Acro with the same starter rates/tune as the GoPro Drone. The two aircraft have different geometry, inertia, drag, thrust and clearance; the controller is shared. Procedural artwork gives each aircraft a distinct silhouette but is not dimensionally exact CAD.

Official sources retrieved September 18, 2026:

- [DJI FPV specifications/support](https://www.dji.com/dji-fpv): approximately 795 g takeoff weight; 178 × 232 × 127 mm propeller-free envelope; 245 mm motor diagonal. Published Manual-mode speed and acceleration figures are recorded as reference metadata only.
- [DJI FPV propellers](https://store.dji.com/product/dji-fpv-propellers): 134.6 mm diameter, 71.1 mm pitch, 5.2 g per propeller. Radius is used in ground effect, ground-strike checks and rendering. Pitch and propeller mass are metadata, not a blade-element model.

The referenced specifications do not supply a thrust/torque sweep, inertia tensor, motor response curve or DJI controller implementation. These remain estimates in the profile:

| Quantity | Assumption and derivation |
|---|---|
| Motor locations / CG | Centered CG, coplanar rotors, symmetric rectangle. Let L = 0.178 m, W = 0.232 m and d = 0.245 m. Half-width x = d / (2 sqrt(1 + (L/W)²)), half-length y = x L/W. This preserves the published diagonal; the envelope aspect ratio is an approximation to the motor rectangle. |
| Inertia | Uniform solid box of mass 0.795 kg with the full propeller-free envelope. Ixx = m(L²+H²)/12, Iyy = m(W²+H²)/12, Izz = m(W²+L²)/12. Products of inertia are zero. This is a rough prior, not a component mass model. |
| Maximum thrust | Assumed total thrust/weight = 3.8, giving about 7.41 N per motor. This is unverified and is not derived from a DJI thrust test. |
| Static thrust curve | Linear normalized motor command until DJI-specific measurements are imported. |
| Motor lag / reaction torque | Inherited provisional 0.05 s response and 0.013 m Q/T; not DJI measurements. |
| Body drag | CdA = 0.75 × projected envelope area for each body axis. The factor combines assumed effective area and drag coefficient; not fitted to DJI speed claims. |
| Other aero | Shared provisional rotor-in-plane drag and rotational damping; see PHYSICS.md. |
| Landing geometry | Four support points at 80% of the motor arms, clearance H/2 = 0.0635 m. No frame flex. |
| Motor spin signs | Match the simulator's Betaflight mixer; DJI firmware motor conventions are not claimed. |

The public maximum speed and acceleration numbers do not uniquely determine thrust, drag, inertia or control behavior. The simulator does not clamp speed to those numbers or claim to reproduce them. DJI Normal/Sport modes, emergency braking, GPS/vision positioning, gimbal stabilization, battery management and proprietary control laws are not implemented.

`tools/build_dji_profile.py` reproduces the approximation and records its sources in the generated JSON. DJI-specific bench data belongs in `config/aircraft/dji_fpv_propulsion_curve.json`, never in GoPro Drone's curve.

## Switching and extending profiles

`config/aircraft_catalog.json` lists each ID, display name, aircraft file, physics file and optional measured curve. The selector instantiates a fresh aircraft, resets the motors and flight state, clears the arm request, and allows a disarmed interval for Betaflight before rearming. Radio calibration and the camera-tilt control are shared across aircraft. Last selection is stored separately from radio calibration in Godot's user-data folder.
