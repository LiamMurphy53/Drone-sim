%% Controller, maneuver, and visualization inputs
% Edit this file when changing a test without changing the physical drone.

simulationInputs = struct();

%% Shared simulation dynamics
% These settings apply to the programmed 6DOF maneuver and every roll,
% pitch, and yaw view derived from it.
simulationInputs.dynamics.timeStep_s = 0.002;          % 500 Hz
simulationInputs.dynamics.motorTimeConstant_s = 0.05; % Replace when measured

%% Static directional-authority sweep
% 180 directions gives a 2-degree angular spacing.
simulationInputs.directionalAuthority.numberOfDirections = 180;

%% 3D animation
% 1 = real time, 2 = twice as fast, 0.5 = half speed.
simulationInputs.animation3D.playbackSpeed = 1.0;

%% Full six-degree-of-freedom programmed maneuver
simulationInputs.full6DOF.maneuverName = ...
    ['Vertical loop, left/right banked turns, and right ' ...
     'barrel roll'];

% Waypoint columns are:
% [time_s, roll_deg, pitch_deg, yaw_deg, altitude_m]
%
% Positive roll lowers the left side (+My), positive pitch lowers the nose
% (+Mx), positive yaw turns the nose left (+Mz), and altitude is world +z.
% Commands are linearly interpolated between rows. Use closely spaced rows
% when you want an almost instantaneous step. Attitude waypoints describe
% orientation only; use the rate-command table below for a continuous flip.
%
% This preset pitches nose-down to accelerate along world -y (forward),
% performs a vertical loop, makes coordinated left and right banked turns,
% then performs a right barrel roll while continuing forward.
simulationInputs.full6DOF.commandWaypoints = [
    0.00,   0,  0, 0, 8.0;
    1.00,   0,  0, 0, 8.0;
    1.25,   0, 20, 0, 8.0;
    3.25,   0, 20, 0, 8.0;
    3.50,   0,  0, 0, 8.0;
    4.00,   0,  0, 0, 8.0;
    6.00,   0,  0, 0, 8.0;
    6.25,   0,  0, 0, 8.0;
    6.75,   0,  0,  0, 8.0;
    7.00,   0,  0,  0, 8.0;
    7.25,  25,  0,  0, 8.0;
    8.45,  25,  0, 45, 8.0;
    8.70,   0,  0, 45, 8.0;
    8.95,   0,  0, 45, 8.0;
    9.20, -25,  0, 45, 8.0;
   10.40, -25,  0,  0, 8.0;
   10.65,   0,  0,  0, 8.0;
   11.25,   0,  0,  0, 8.0;
   12.11,   0,  0,  0, 8.0;
   14.75,   0,  0,  0, 8.0;
];

% Rate-mode columns are:
% [start_s, end_s, rollRate_deg_s, pitchRate_deg_s, yawRate_deg_s, ...
%  collectiveWeightScale]
% Rows 1-11 form and brake the vertical loop. The banked turns above use
% ordinary attitude waypoints. Rows 12-16 ramp into and out of the right
% barrel roll; row 17 brakes it with added collective for motor authority.
simulationInputs.full6DOF.rateCommandSegments = [
    4.00, 4.25, 0, -132.0, 0, 3.56;
    4.25, 4.50, 0, -144.4, 0, 3.14;
    4.50, 4.75, 0, -182.4, 0, 2.42;
    4.75, 5.00, 0, -261.2, 0, 1.74;
    5.00, 5.25, 0, -261.2, 0, 1.74;
    5.25, 5.50, 0, -182.4, 0, 2.42;
    5.50, 5.75, 0, -144.4, 0, 3.14;
    5.75, 6.00, 0, -132.0, 0, 3.56;
    6.00, 6.08, 0,    0.0, 0, 3.00;
    6.08, 6.16, 0,    0.0, 0, 2.00;
    6.16, 6.25, 0,    0.0, 0, 1.00;
   11.25, 11.33, -240, 0, 0, 0.45;
   11.33, 11.41, -480, 0, 0, 0.45;
   11.41, 11.75, -720, 0, 0, 0.45;
   11.75, 11.83, -480, 0, 0, 0.45;
   11.83, 11.91, -240, 0, 0, 0.45;
   11.91, 12.11,    0, 0, 0, 1.00
];
simulationInputs.full6DOF.rateControlBandwidth_rad_s = [20; 20; 10];

% Desired closed-loop attitude response about body [x, y, z]. Body x is
% pitch, body y is roll, and body z is yaw. Yaw is slower because reaction-
% torque authority is much smaller than roll/pitch lever-arm authority.
simulationInputs.full6DOF.attitudeNaturalFrequency_rad_s = [8; 8; 4];
simulationInputs.full6DOF.attitudeDampingRatio = [0.9; 0.9; 0.9];
simulationInputs.full6DOF.attitudeIntegralPole_rad_s = [0.15; 0.15; 0.10];
simulationInputs.full6DOF.maxAttitudeIntegral_rad_s = [0.4; 0.4; 0.5];

% Altitude controller response and limits.
simulationInputs.full6DOF.altitudeNaturalFrequency_rad_s = 3.0;
simulationInputs.full6DOF.altitudeDampingRatio = 1.0;
simulationInputs.full6DOF.altitudeIntegralPole_rad_s = 0.25;
simulationInputs.full6DOF.maxAltitudeIntegral_m_s = 1.0;
simulationInputs.full6DOF.maxVerticalAcceleration_m_s2 = 8.0;

% Initial world state, flight attitude [roll, pitch, yaw], and body rate.
simulationInputs.full6DOF.initialPosition_m = [0; 0; 8];
simulationInputs.full6DOF.initialVelocity_m_s = [0; 0; 0];
simulationInputs.full6DOF.initialAttitude_deg = [0; 0; 0];
simulationInputs.full6DOF.initialAngularVelocity_rad_s = [0; 0; 0];

% Allocation priorities after normalizing each wrench axis by its authority.
% Values above one favor that axis when the requested wrench is infeasible.
simulationInputs.full6DOF.allocationPriority = [1.0; 2.0; 2.0; 1.5];
