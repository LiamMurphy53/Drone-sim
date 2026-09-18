# Drone motor-layout model

This is a first-pass MATLAB model for comparing multirotor geometries, including
deadcat layouts. It accepts:

- any number of motors;
- each motor's position in millimeters;
- a short name for each motor;
- each propeller's CW/CCW direction viewed from above;
- one shared maximum thrust or a separate maximum for every motor;
- a propeller reaction-torque-to-thrust estimate for yaw;
- total flight mass in grams and center of mass in millimeters; and
- the full 3-by-3 inertia tensor in `g*mm^2`.

## Project layout

The files are organized by how they are used:

```text
sim/
├── run_all_simulations.m              choose and run workflows from a checklist
├── inputs/                             the only normal user-editable files
│   ├── drone_config_inputs.m          physical and propulsion inputs
│   ├── simulation_inputs.m            programmed maneuver, dynamics, and display
│   ├── geometry_analysis_inputs.m      geometry sweeps and comparison file list
│   └── configurations/                 additional frame input scripts
├── run_drone_config.m                 analyze layout and hover
├── run_cg_sensitivity.m               map battery/payload CG placement effects
├── run_moment_envelope_3d.m           exact fixed-lift Mx-My-Mz envelope
├── run_mixer_quality.m                normalized mixer geometry diagnostics
├── run_configuration_comparison.m     compare multiple physical input files
├── run_roll_pid_simulation.m          roll view of the programmed 6DOF flight
├── run_pitch_pid_simulation.m         pitch view of the programmed 6DOF flight
├── run_yaw_pid_simulation.m           yaw view of the programmed 6DOF flight
├── run_6dof_simulation.m              run programmable roll/pitch/yaw/altitude
├── run_roll_3d_simulation.m           legacy shortcut to the 6DOF animation
├── run_directional_authority.m        sweep control authority through 360 deg
├── setup_project.m                     add source folders to the MATLAB path
├── src/
│   ├── core/                           force, moment, and trajectory dynamics
│   ├── control/                        hover analysis, allocation, and 6DOF control
│   └── visualization/                  plots and 3D animation
└── tests/                              MATLAB unit tests
```

## Running the project

The normal entry point is:

```matlab
run_all_simulations
```

MATLAB opens a checklist with all stages selected by default. Uncheck anything
you do not want on that run:

- drone layout and hover analysis;
- 360-degree directional authority;
- center-of-mass placement sensitivity;
- full `Mx-My-Mz` moment envelope;
- normalized mixer geometry quality;
- multi-configuration frame comparison;
- roll view of the programmed 6DOF maneuver;
- pitch view of the programmed 6DOF maneuver;
- yaw view of the programmed 6DOF maneuver;
- programmed 6DOF roll/pitch/yaw comparison;
- full 6DOF response plots; and
- full 6DOF 3D animation.

The physical configuration is always calculated because every other stage
depends on it. If the layout item is unchecked, its layout figure is hidden.
All roll, pitch, yaw, comparison, full-response, and animation choices share one
coupled 6DOF simulation. The master script runs the maneuver once, then derives
the selected axis views from that exact result. The loop, banked turns, barrel
roll, motor limiting, and cross-axis coupling are therefore identical in every
format. If MATLAB is running without the desktop UI, the master script runs
every stage.

The individual `run_*.m` scripts remain available when only one workflow is
needed. They call `setup_project.m` and load the current input files
automatically. The implementation under `src/` normally does not need editing.

## Input files

All normal inputs now live under `inputs/`:

- `inputs/drone_config_inputs.m` contains motor locations/names, spin directions,
  maximum thrust, reaction-torque estimate, mass, CG, inertia tensor, propulsion
  metadata, and the optional example motor-thrust operating point. This is the
  file to update from Onshape or hardware data.
- `inputs/simulation_inputs.m` contains the update rate, motor time constant,
  directional sweep resolution, programmable full-6DOF attitude waypoints and
  rate-mode segments, 6DOF controller targets, and 3D playback speed.
- `inputs/geometry_analysis_inputs.m` contains the CG grid, optional movable-
  component placement data, fixed-collective envelope setting, and the list of
  frame input files to compare.
- `inputs/configurations/` contains additional scripts that each create a
  complete `droneInputs` structure. `current_deadcat_inputs.m` is a self-contained
  snapshot of the current active setup. The supplied symmetric-X file inherits
  that snapshot and is a controlled motor-placement reference, not a second
  CAD-accurate vehicle.

Workflow scripts compare the configuration currently in memory with
`drone_config_inputs.m`. If the input file changed, they automatically rebuild
the configuration rather than reusing stale data.

Running the layout stage plots the motor geometry and reports force, moments,
thrust-to-weight ratio, level-hover trim, and available constant-altitude control
authority. The number of rows in `droneInputs.motorLocations_mm` sets the number
of motors. Each plotted motor is labeled with its spin direction and maximum
thrust; all motor markers use the same color.

## Coordinates and units

- Locations are `[x, y, z]` in millimeters.
- `motorLocations_mm` and `centerOfMass_mm` must use the same body coordinate
  frame.
- `+x` points left, `+y` points backward, and `+z` points up, forming a
  right-handed coordinate system.
- Motor thrust is in newtons and acts along `+z`.
- Total mass is entered in grams.
- Moments are in newton-meters and follow the right-hand rule.
- Enter the inertia tensor in `g*mm^2`, about the center of mass, and expressed
  in the same body axes. The model converts it to `kg*m^2` internally using
  `1 g*mm^2 = 1e-9 kg*m^2`.

The physical input fields deliberately include their units in their names:

```matlab
droneInputs.motorLocations_mm
droneInputs.motorNames
droneInputs.motorSpinDirection
droneInputs.yawTorquePerThrust_m
droneInputs.centerOfMass_mm
droneInputs.totalMass_g
droneInputs.inertiaTensor_g_mm2
droneInputs.maxThrust_N
```

Motor names follow their physical location in the current quad configuration:

```text
FL = front-left
FR = front-right
RL = rear-left
RR = rear-right
```

The rows of `droneInputs.motorLocations_mm`, entries in
`droneInputs.motorNames`, motor thrust values, and per-motor maximum thrust
values must all use the same order. These labels are used in the layout plot,
hover report, and simulation motor-thrust plots.

Spin directions are entered as `CW` or `CCW` viewed from above. With `+z` up,
the equal-and-opposite frame reaction from a CW propeller is `+Mz`; a CCW
propeller contributes `-Mz`. The current standard layout is:

```text
FL CW    FR CCW
RL CCW   RR CW
```

All location, mass, and inertia inputs can therefore be copied from Onshape in
these units. The model converts them to meters, kilograms, and `kg*m^2`
internally before performing dynamics calculations.

## 360-degree directional authority

Run:

```matlab
run_directional_authority
```

The analysis sweeps directions around the horizontal control plane. For each
direction it maximizes the available control while:

- holding total thrust equal to vehicle weight; and
- forcing the perpendicular moment or angular acceleration to zero.

The left polar plot is the geometry-driven moment envelope. The right polar plot
includes the full inertia tensor and shows angular-acceleration authority. On
both plots, `0 deg` means `+Mx` and `90 deg` means `+My`. The dashed circle is
the worst-direction authority, the red marker is the weakest sampled direction,
and the green marker is the strongest.

The console reports minimum and maximum authority, best/worst directions, and
anisotropy:

```text
anisotropy = best-direction authority / worst-direction authority
```

An ideal directionally uniform configuration approaches `1.0`. A larger value
means the frame or inertia makes some maneuver directions substantially stronger
than others. Full results are stored in `directionalAuthority`, including the
motor thrust vector that reaches every directional limit.

For a vertical motor with center-of-mass-relative lever arm `[rx, ry, rz]`
producing thrust `T`, the model uses:

```text
Fz = T
Mx = ry*T
My = -rx*T
```

The `z` location has no effect until tilted motors or nonvertical forces are
included.

With these axes, `Mx` is the moment about the left-pointing axis and `My` is the
moment about the backward-pointing axis. The output uses axis names instead of
calling these roll and pitch, which avoids hiding a sign or axis conversion.

## Center-of-mass placement sensitivity

Run:

```matlab
run_cg_sensitivity
```

This sweeps the **final assembled vehicle CG** over the x/y grid in
`geometry_analysis_inputs.m`. At every point it solves a bounded level-hover
trim with `[Fz, Mx, My, Mz] = [weight, 0, 0, 0]`, then records:

- whether exact zero-moment hover is possible;
- the hover command required from every motor;
- the smallest two-sided motor-command margin;
- which motor and upper/lower bound limits that margin; and
- decoupled two-sided roll (`My`), pitch (`Mx`), and yaw (`Mz`) authority while
  holding lift at weight and forcing the other two moments to zero.

This helps choose battery or payload placement because a component move shifts
the total CG and therefore changes every motor lever arm and baseline hover
command. A motor already working harder just to hold level has less downward
command range in one control direction, while a lightly loaded motor has less
room to reduce thrust. The useful target is therefore a region with adequate
two-sided margin and authority, not simply the geometric center of the frame.

For a battery or other component already included in the entered total mass and
CG, candidate component positions convert to total-CG positions through:

```text
CG_new = CG_current + (componentMass / totalMass) *
                         (componentPosition_new - componentPosition_current)
```

For a new payload not yet included in the vehicle data:

```text
CG_new = (totalMass*CG_current + payloadMass*payloadPosition) /
         (totalMass + payloadMass)
```

Enable `geometryInputs.cgSensitivity.componentPlacement` and enter the component
mass and candidate coordinates to overlay those resulting total-CG points on
the maps. The script handles the increased weight for `add-payload` mode. It
holds the entered inertia tensor fixed, so update the CAD inertia when a heavy
component move significantly changes mass distribution.

For the current frame, the exact current-CG hover commands are approximately
`18.74%, 18.73%, 10.65%, 10.64%` for `FL, FR, RL, RR`. The best-balanced sampled
CG is near `[0.0, 8.65, 44.844] mm`, where the two-sided command margin reaches
about `14.69%`; the current CG margin is `10.64%`. That point is a motor-loading
reference, not an instruction to move the CG there regardless of packaging or
aerodynamics. Results are stored in `cgSensitivity`.

## Full Mx-My-Mz moment envelope

Run:

```matlab
run_moment_envelope_3d
```

Separate maximum-roll, maximum-pitch, and maximum-yaw values do not show what is
available when the controller requests several moments simultaneously. The 3D
envelope contains every achievable `[Mx, My, Mz]` combination satisfying:

```text
sum(motorThrust) = fixedCollectiveThrust
0 <= motorThrust_i <= maximumThrust_i
```

The computation exactly enumerates the bounded thrust polytope rather than
sampling a few directions. A desired combined moment inside the hull is
instantaneously achievable at that collective thrust; one outside is not. This
exposes tradeoffs such as yaw authority being consumed by a strong simultaneous
roll demand during a banked turn.

`geometryInputs.momentEnvelope.collectiveWeightScale = 1` uses vehicle weight,
representing level-hover lift. The left plot preserves equal physical `N*m`
scales, making weak axes visibly flat. The right plot scales each axis by its own
maximum magnitude so the envelope shape can be inspected; that normalized view
must not be read as absolute authority.

At the current hover weight, the exact envelope has four vertices and ranges of
about `Mx = -0.384 to 0.675 N*m`, `My = -0.843 to 0.843 N*m`, and
`Mz = -0.103 to 0.103 N*m`. Its smallest Euclidean combined-moment margin around
zero trim is about `0.0685 N*m`. Raw per-axis envelope extrema may include
off-axis moment; use the CG map or directional-authority analysis when the other
moments must be cancelled. Results are stored in `momentEnvelope3D`.

## Normalized mixer geometry quality

Run:

```matlab
run_mixer_quality
```

Rank only answers whether collective, roll, pitch, and yaw are mathematically
controllable. Mixer quality measures how poorly conditioned that control is. The
analysis uses normalized motor commands, divides force by maximum total thrust,
and divides all three moments by one common physical scale:

```text
momentScale = maximumTotalThrust * thrustWeightedRMSMotorRadius
```

Using one common moment scale preserves a genuinely weak yaw axis; independently
normalizing every row would incorrectly make all axes look equally strong. The
fixed-collective score first removes motor-command changes that alter total
thrust, then calculates:

```text
mixerQualityPercent = 100 * smallestSingularValue / largestSingularValue
```

`100%` would mean equally effective orthogonal moment directions; `0%` means at
least one moment direction is uncontrollable. This is an isotropy/conditioning
score, not a measure of absolute maximum moment. The current frame scores about
`12.98%`, with condition number `7.706`; its weakest normalized direction is
almost entirely yaw. Because yaw uses the provisional `Q/T` estimate, that part
of the score will change when measured reaction-torque data replaces the
estimate. Results are stored in `mixerQuality`.

## Comparing frame configurations

Run:

```matlab
run_configuration_comparison
```

The file list is in `geometryInputs.configurationComparison.inputFiles`. Each
listed MATLAB script must create one complete `droneInputs` structure, so a new
Onshape design can be compared by copying the physical-input format into
`inputs/configurations/` and adding its file path to that list.

The supplied comparison uses:

- `Current deadcat`, a self-contained snapshot stored in
  `inputs/configurations/current_deadcat_inputs.m`; and
- `Symmetric X reference (motor placement only)`, which inherits the current
  deadcat snapshot's mass, CG, inertia, thrust, and propulsion data and changes
  only the motor x/y locations while retaining the same mean motor radius.

That reference isolates motor-placement geometry. It is not a prediction for a
real X-frame build unless its mass properties are replaced with CAD data. The
snapshot deliberately does not follow later edits to `drone_config_inputs.m`;
update or replace it when you want a new active design captured for comparison.
The comparison plots hover reserve, horizontal moment and angular-acceleration
authority, scale-normalized geometry scores, directional unevenness, and the
fixed-collective mixer-quality score. With the current inputs, the deadcat versus
reference-X results include `10.64%` versus `13.01%` hover margin, `0.384` versus
`0.599 N*m` worst-direction horizontal moment, and `12.98%` versus `15.26%`
mixer quality.

Static geometry comparison is the default. Setting
`geometryInputs.configurationComparison.runShared6DOF = true` additionally flies
the same programmed maneuver with every complete vehicle, but those results also
contain inertia, controller, and trajectory effects and should not be described
as motor-layout-only. Results are stored in `configurationComparison`.

## Current scope

The static geometry suite now covers current-CG hover trim, CG placement maps,
360-degree horizontal authority, the exact fixed-collective 3D moment polytope,
normalized mixer conditioning, and comparison across multiple physical input
files. These analyses are instantaneous rigid-body/allocation calculations and
do not include motor response time or aerodynamics.

The project uses one full 6DOF simulation as the source for the 3D animation,
full response plots, and focused roll, pitch, and yaw plots. It simultaneously
integrates 3D position, velocity, quaternion attitude, and body angular
velocity. It includes the full inertia tensor, gyroscopic coupling, gravity,
motor lag, individual thrust limits, propeller reaction torque, four-axis motor
allocation, attitude PID, body-rate control, and altitude PID.

The 6DOF controller holds attitude and world altitude, but it does not hold
world x/y position. A tilted drone therefore drifts laterally, which is expected.
The current plant also omits aerodynamic drag, propwash interactions, motor/RPM
electrical dynamics, battery sag, sensor noise/delay, and ground contact. It is
a strong configuration and controller-development model, but not yet a
high-fidelity substitute for flight testing.

The configured [T-Motor V2207 V3 1750 KV](https://www.t-hobby.com/products/tmotor-velox-v2207-v3-motor),
[Gemfan Hurricane 51433](https://www.racedayquads.com/products/gemfan-hurricane-51433-durable-tri-blade-5-prop-4-pack-choose-your-color),
and [Tattu 1400 mAh 6S battery](https://genstattu.com/tattu-r-line-version-5-0-1400mah-6s-150c-22-2v-lipo-battery-pack-with-xt60-plug/)
are recorded as propulsion metadata. They are not yet used to calculate thrust
from throttle. `maxThrust_N` remains the thrust limit used by the plant.

## What this says about dynamics and stability

The current geometry, mass, and inertia data determine several important flight
characteristics:

- **Hover trim:** the individual motor thrusts needed to support the weight while
  producing zero x- and y-axis moment. A center-of-mass offset generally makes
  these thrusts unequal. `analyzeHoverDynamics` reports each motor's thrust,
  command percentage, and remaining upward thrust margin.
- **Control authority:** the positive and negative moments still available from
  the hover condition before one or more motors reaches zero or maximum thrust.
  The reported x-axis limits hold total thrust equal to weight and cancel the
  y-axis moment; the y-axis limits do the converse.
- **Angular agility:** moment becomes angular acceleration through
  `angularAcceleration = inertiaTensor \ moment`. Lower inertia or a longer
  motor lever arm produces more angular acceleration for the same thrust change.
- **Axis coupling:** off-diagonal inertia terms can cause a moment about one body
  axis to produce acceleration about multiple axes.

These quantities describe how controllable and responsive the configuration is,
but geometry alone does not establish closed-loop stability. An uncontrolled
multirotor does not naturally return to level after an attitude disturbance.
The 6DOF simulation adds controllers, update rate, thrust limits, and a simple
motor-response model so tracking error, angular rate, moment demand, and motor
limiting can be inspected. Its accuracy still depends on replacing the
estimated motor time constant with measured data and later adding effects such
as delay, aerodynamics, and electrical propulsion dynamics.

The hover report is stored in the `hover` structure. Useful fields include:

```matlab
hover.hoverFeasible
hover.motorThrust
hover.motorCommandPercent
hover.upperThrustMargin
hover.minimumNormalizedMargin
hover.xAxis.momentRange
hover.xAxis.axisAngularAccelerationRange
hover.yAxis.momentRange
hover.yAxis.axisAngularAccelerationRange
```

The original `hover.xAxis`, `hover.yAxis`, and 360-degree polar limits are
instantaneous horizontal-axis rigid-body limits at level hover. They do not
include motor response time, propeller aerodynamics, battery voltage sag, or
controller delay. The newer CG and 3D-envelope workflows include the configured
reaction-torque yaw row. All of these measure allocation authority and
saturation margin rather than closed-loop stability.

## Programmed 6DOF axis views and comparison

The project has focused views for all three attitude axes:

```matlab
run_roll_pid_simulation
run_pitch_pid_simulation
run_yaw_pid_simulation
```

Their physical mappings are:

```text
Pitch = body x / Mx / Ixx
Roll  = body y / My / Iyy
Yaw   = body z / Mz / Izz
```

The filenames retain `pid` so existing shortcuts continue to work, but these
are no longer separate scalar test flights. Each script runs the maneuver in
`simulationInputs.full6DOF` and then shows only its own flight-axis data. The
result is stored as `rollSimulation`, `pitchSimulation`, or `yawSimulation`.

The axis figure shows attitude tracking, rate tracking, cumulative rate-mode
rotation, demanded versus achieved moment, and all motor thrusts. Attitude is
hidden during rate-mode segments because a continuous flip passes through Euler
angle folding; rate and cumulative rotation are the meaningful measurements
there.

Select **Programmed 6DOF roll, pitch, and yaw comparison** in
`run_all_simulations` to compare the three views. The master workflow runs the
6DOF plant only once and extracts all three from the exact same state history.
The comparison shows commanded/actual angles, commanded/actual rates, observed
peak rate, and RMS attitude-hold error. MATLAB also prints each axis's rate-mode
rotation and moment-tracking error plus the shared allocation-limited percentage.

This is a maneuver-specific comparison: the axes do not receive identical
commands. For example, the supplied loop mainly exercises pitch, the barrel
roll mainly exercises roll, and the banked turns exercise roll and yaw together.
That makes the plots faithful to the programmed flight and its coupling, but a
larger peak rate does not by itself mean that axis has more maximum authority.
Use the 360-degree authority analysis for configuration-only limits.

Pitch and roll use motor lever-arm moments. Yaw uses the configured CW/CCW
reaction-torque map and requires rank-four allocation; its accuracy therefore
depends directly on `yawTorquePerThrust_m`. The current yaw coefficient remains
an engineering estimate until it is replaced with measured motor/prop torque.

## Programmable full 6DOF simulation

Edit the `simulationInputs.full6DOF` attitude waypoints and optional rate
segments, then run any axis view, the comparison, the full plots, or:

```matlab
run_6dof_simulation
```

Each row of the waypoint table is:

```text
[time_s, roll_deg, pitch_deg, yaw_deg, altitude_m]
```

Commands are linearly interpolated between rows. Positive roll lowers the left
side (`+My`), positive pitch lowers the nose (`+Mx`), positive yaw turns the nose
left (`+Mz`), and positive altitude is world `+z`. The final waypoint time must
be an exact multiple of `simulationInputs.dynamics.timeStep_s`.

Ordinary attitude waypoints represent orientations, so angles separated by
360 degrees are physically identical. They cannot specify which way to pass
through 180 degrees. Flips therefore use
`simulationInputs.full6DOF.rateCommandSegments`. Each row is:

```text
[start_s, end_s, rollRate_deg_s, pitchRate_deg_s, yawRate_deg_s, collectiveWeightScale]
```

During a rate segment, attitude and altitude hold pause. The controller tracks
the requested body rate continuously through 180 degrees and commands a fixed
collective thrust. `collectiveWeightScale = 1` means total thrust equal to the
vehicle's weight; a smaller value reduces the downward force while inverted.
Sequential segments may touch but cannot overlap.
If `rateCommandSegments` is omitted from a settings structure, the simulator
runs ordinary attitude and altitude control with no rate-mode segments. In
that case, an attitude waypoint at or beyond `+/-180 deg` now stops with a
`FlipNeedsRateMode` error instead of silently attempting the ambiguous
orientation command. Add a rate segment to request continuous rotation.

The supplied maneuver starts at 8 m, pitches nose-down to build forward speed
along world `-y`, levels, and begins a vertical loop at 4 seconds. Eight rate
segments divide the loop into 45-degree path sectors. Their pitch rates and
collective-thrust scales vary around the circle because a quadrotor needs more
inward thrust at the bottom than at the top. Three short zero-rate segments
then brake the loop rotation and taper thrust toward hover.

After the loop, ordinary attitude waypoints command a `25 deg` left bank while
yawing `45 deg` left, followed by a `-25 deg` right bank that returns yaw to the
original heading. With the current drone data, the velocity heading reaches
about `42-45 deg` left and returns to within roughly `2 deg` of its original
direction before the barrel roll.

At 11.25 seconds the preset begins a right barrel roll while continuing
forward. Its roll-rate command ramps from `-240` through `-720 deg/s`, then
ramps down again instead of applying one abrupt rate step. A final zero-rate
segment uses weight-level collective thrust to brake the remaining rotation.
The roll reaches about `-359.3 deg`; the complete maneuver is
allocation-limited for about `1.19%` of its samples and returns to roughly 8 m
altitude by the end of the simulation.

With the current drone data, the preset enters the loop at about `8.22 m/s`,
reaches about `13.14 m` altitude at the top, completes `-360.0 deg` of pitch,
and exits forward at about `8.43 m/s`. No loop samples are allocation-limited.
The loop schedule and barrel-roll timing are calibrated to the current inertia,
maximum thrust, motor time constant, and controller bandwidth; rerun and retune
them after changing the physical configuration.

The initial position, velocity, attitude, and angular velocity are also in the
full-6DOF input section. Initial attitude is entered as
`[roll; pitch; yaw]` in degrees. The attitude controller uses a quaternion plant,
so it avoids Euler-angle integration singularities; the reported flight angles
are only a human-readable view of that quaternion.

At every time step the model:

1. calculates attitude/altitude demands or rate-mode demands;
2. forms the desired wrench `[Fz; Mx; My; Mz]`;
3. solves a bounded motor allocation with every thrust constrained from zero to
   its configured maximum;
4. applies motor lag and computes actual force and reaction torque; and
5. integrates the full rigid-body rotational and translational equations.

The response figure compares commanded and actual roll, pitch, yaw, and
altitude, then shows angular rates and motor thrust. A second figure appears when
rate mode is used, showing rate tracking and cumulative flip rotation. Euler
pitch folds after passing vertical, so this cumulative-rotation plot is the
correct place to judge a full flip. Red points on the motor plot mark samples
where the requested four-axis wrench could not be achieved within the motor
limits. The animation shows the real deadcat geometry, spin labels, individual
thrust arrows, body axes, the 3D path, requested altitude, rate command, and flip
progress. Its main view stays close enough to distinguish the motors, while a
small full-path inset supplies trajectory context. The trail is lengthened for
rate-mode maneuvers. Results are stored in `full6DOFSimulation`.

Yaw uses:

```text
Mz = sum(spinSign_i * yawTorquePerThrust_i * thrust_i)
```

The current `yawTorquePerThrust_m = 0.013` is provisional because the available
motor/propeller data does not directly publish shaft torque for this exact
combination. It uses the official V2207 V3 1750 KV test point with a similar
T5143S-3 prop at 40% throttle—451 g thrust, 18,437 RPM, and 160.8 W—and assumes
70% electrical-to-shaft efficiency:

```text
Q/T ~= 0.70 * electricalPower / (angularSpeed * thrust) ~= 0.013 m
```

The [published motor test table](https://www.ligpower.com/product/v2207-v3-kv1750-fpv-motor.html)
supports the power, RPM, and thrust values, but the 70% efficiency is an
engineering assumption. Replace the estimate with `motor torque / motor thrust`
from a torque stand if that measurement becomes available. Changing KV alone
does not determine this coefficient. The older `run_roll_3d_simulation`
filename remains as a legacy shortcut, but it now animates the same programmed
6DOF maneuver rather than a separate one-axis flight.

The inertia tensor should have the standard form:

```text
[ Ixx  Ixy  Ixz ]
[ Iyx  Iyy  Iyz ]
[ Izx  Izy  Izz ]
```

It must be symmetric and positive definite. If your source uses negative
products of inertia in its matrix convention, enter the matrix exactly as that
source defines the physical inertia tensor rather than changing signs by hand.

Run the checks with:

```matlab
setup_project
results = runtests(fullfile('tests', 'test_drone_config.m'))
```
