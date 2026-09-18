%% Analyze the physical drone configuration
% Edit inputs/drone_config_inputs.m, then run this workflow or the master
% run_all_simulations.m launcher.

projectRoot = fileparts(mfilename('fullpath'));
run(fullfile(projectRoot, 'setup_project.m'));
run(fullfile(projectRoot, 'inputs', 'drone_config_inputs.m'));

motorLocations_mm = droneInputs.motorLocations_mm;
motorNames = droneInputs.motorNames;
maxThrust = droneInputs.maxThrust_N;
totalMass_g = droneInputs.totalMass_g;
centerOfMass_mm = droneInputs.centerOfMass_mm;
inertiaTensor_g_mm2 = droneInputs.inertiaTensor_g_mm2;

%% Analyze the layout
config = analyzeDroneConfig(motorLocations_mm, maxThrust, totalMass_g, ...
    centerOfMass_mm, inertiaTensor_g_mm2, motorNames, droneInputs);
if ~exist('showDroneConfigPlot', 'var') || showDroneConfigPlot
    plotDroneConfig(config);
end
hover = analyzeHoverDynamics(config);

%% Example operating point
% Actual thrust produced by each motor, in the same order as motorLocations_mm.
motorThrust = droneInputs.exampleMotorThrust_N;
if isscalar(motorThrust)
    motorThrust = repmat(motorThrust, config.numberOfMotors, 1);
else
    motorThrust = motorThrust(:);
end
wrench = motorWrench(config, motorThrust);
acceleration = droneAcceleration(config, motorThrust);

fprintf('\nRequested operating point\n');
fprintf('  Vertical force Fz: %.3f N\n', wrench(1));
fprintf('  X-axis moment Mx:  %.3f N*m\n', wrench(2));
fprintf('  Y-axis moment My:  %.3f N*m\n', wrench(3));
fprintf('  Level vertical acceleration: %.3f m/s^2\n', ...
    acceleration.levelVerticalAcceleration);
fprintf('  Angular acceleration [x y z]: [%.3f %.3f %.3f] rad/s^2\n', ...
    acceleration.angularAcceleration);
