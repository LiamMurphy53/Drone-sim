%% Current deadcat configuration snapshot
% Self-contained copy of the active drone setup when this file was created.
% Unlike inputs/drone_config_inputs.m, this comparison configuration does
% not change automatically when the active design is edited.

droneInputs = struct();
droneInputs.configurationName = 'Current deadcat';

% One row per motor: [x, y, z] in mm. Axes are +x left, +y backward, +z up.
droneInputs.motorLocations_mm = [
     106.252, -58.089, 7;  % FL: front-left
    -106.252, -58.089, 7;  % FR: front-right
     93.252,   75.381, 7;  % RL: rear-left
    -93.252,   75.381, 7;  % RR: rear-right
];
droneInputs.motorNames = {'FL', 'FR', 'RL', 'RR'};
droneInputs.motorSpinDirection = {'CW', 'CCW', 'CCW', 'CW'};

% Provisional propeller reaction-torque ratio Q/T in meters.
droneInputs.yawTorquePerThrust_m = 0.013;

% Propulsion metadata.
droneInputs.motorKV_rpm_per_V = 1750;
droneInputs.propellerDiameter_in = 5.1;
droneInputs.propellerBladeCount = 3;
droneInputs.batteryCellCount = 6;
droneInputs.batteryNominalVoltage_V = 22.2;
droneInputs.batteryCapacity_mAh = 1400;

droneInputs.maxThrust_N = 13.5;
droneInputs.totalMass_g = 808.819;
droneInputs.centerOfMass_mm = [0.021, -9.73, 44.844];

% Inertia tensor about the CG in g*mm^2.
droneInputs.inertiaTensor_g_mm2 = [
    2.639*10^6, -1760.996, -3127.566;
    -1760.996, 3.044*10^6, 363462.914;
    -3127.566, 363462.914, 4.027*10^6
];

droneInputs.exampleMotorThrust_N = 6;
