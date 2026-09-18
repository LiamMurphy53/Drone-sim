%% Physical drone inputs
% This is the main file to edit when the Onshape design changes.
% Coordinates use +x left, +y backward, and +z up.

droneInputs = struct();
droneInputs.configurationName = 'Current deadcat';

% One row per motor: [x, y, z] in mm.
droneInputs.motorLocations_mm = [
     106.252, -58.089, 7;  % FL: front-left
    -106.252, -58.089, 7;  % FR: front-right
     93.252,   75.381, 7;  % RL: rear-left
    -93.252,   75.381, 7;  % RR: rear-right
];
droneInputs.motorNames = {'FL', 'FR', 'RL', 'RR'};

% Propeller direction viewed from above. The drone reaction torque acts in
% the opposite direction. Order must match the motor-location rows.
droneInputs.motorSpinDirection = {'CW', 'CCW', 'CCW', 'CW'};

% Provisional reaction-torque ratio Q/T in meters. The 0.013 m estimate uses
% the published V2207 V3 1750KV / T5143S-3 40% test point and an assumed 70%
% electrical-to-shaft efficiency: Q/T ~= 0.70*P/(omega*T). Replace it if
% direct torque-stand data for the Gemfan 51433 becomes available.
droneInputs.yawTorquePerThrust_m = 0.013;

% Propulsion metadata. These values document the hardware and leave room
% for a later RPM/current/battery-sag model; the first 6DOF model still uses
% commanded thrust and the motor time constant.
droneInputs.motorKV_rpm_per_V = 1750;
droneInputs.propellerDiameter_in = 5.1;
droneInputs.propellerBladeCount = 3;
droneInputs.batteryCellCount = 6;
droneInputs.batteryNominalVoltage_V = 22.2;
droneInputs.batteryCapacity_mAh = 1400;

% Use one value if all motors match, or one value per motor.
droneInputs.maxThrust_N = 13.5;

% Complete flight mass in grams and center of mass in mm.
droneInputs.totalMass_g = 808.819;
droneInputs.centerOfMass_mm = [0.021, -9.73, 44.844];

% 3-by-3 inertia tensor about the center of mass, in g*mm^2.
droneInputs.inertiaTensor_g_mm2 = [
    2.639*10^6, -1760.996, -3127.566;
    -1760.996, 3.044*10^6, 363462.914;
    -3127.566, 363462.914, 4.027*10^6
];

% Optional force/moment example printed by run_drone_config.m. A scalar
% applies the same thrust to every motor; a vector uses one value per motor.
droneInputs.exampleMotorThrust_N = 6;
