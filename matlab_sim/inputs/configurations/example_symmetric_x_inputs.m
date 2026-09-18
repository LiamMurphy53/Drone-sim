%% Example symmetric-X motor layout for controlled geometry comparison
% This inherits the current-deadcat comparison snapshot's mass, CG, inertia,
% thrust, and propulsion data, then changes only the horizontal motor
% locations. It is not a second CAD model; it isolates motor placement at
% equal mean motor radius.

configurationFile = mfilename('fullpath');
projectRoot = fileparts(fileparts(fileparts(configurationFile)));
run(fullfile(projectRoot, 'inputs', 'configurations', ...
    'current_deadcat_inputs.m'));

meanMotorRadius_mm = mean(vecnorm( ...
    droneInputs.motorLocations_mm(:, 1:2), 2, 2));
diagonalProjection_mm = meanMotorRadius_mm / sqrt(2);
motorZ_mm = mean(droneInputs.motorLocations_mm(:, 3));
droneInputs.motorLocations_mm = [
     diagonalProjection_mm, -diagonalProjection_mm, motorZ_mm;  % FL
    -diagonalProjection_mm, -diagonalProjection_mm, motorZ_mm;  % FR
     diagonalProjection_mm,  diagonalProjection_mm, motorZ_mm;  % RL
    -diagonalProjection_mm,  diagonalProjection_mm, motorZ_mm;  % RR
];
droneInputs.configurationName = ...
    'Symmetric X reference (motor placement only)';
