%% Frame-geometry analysis inputs
% This script is loaded after drone_config_inputs.m, so ranges can follow
% the current CAD-derived geometry automatically.

geometryInputs = struct();

%% Center-of-mass sensitivity map
% Sweep the complete vehicle CG over the motor footprint while holding z,
% total mass, and inertia fixed. This isolates placement/lever-arm effects.
geometryInputs.cgSensitivity.xCoordinates_mm = linspace( ...
    min(droneInputs.motorLocations_mm(:, 1)), ...
    max(droneInputs.motorLocations_mm(:, 1)), 51);
geometryInputs.cgSensitivity.yCoordinates_mm = linspace( ...
    min(droneInputs.motorLocations_mm(:, 2)), ...
    max(droneInputs.motorLocations_mm(:, 2)), 51);
geometryInputs.cgSensitivity.fixedZ_mm = ...
    droneInputs.centerOfMass_mm(3);

% The current total CG is always marked. To overlay candidate battery or
% payload placements, enable this section and enter measured component data.
% 'move-included': component is already included in totalMass_g/current CG.
% 'add-payload': component is not yet included in totalMass_g/current CG.
geometryInputs.cgSensitivity.componentPlacement.enabled = false;
geometryInputs.cgSensitivity.componentPlacement.mode = 'move-included';
geometryInputs.cgSensitivity.componentPlacement.mass_g = [];
geometryInputs.cgSensitivity.componentPlacement.currentPosition_mm = [];
geometryInputs.cgSensitivity.componentPlacement.candidatePositions_mm = ...
    zeros(0, 3);

%% Full fixed-collective Mx-My-Mz envelope
% 1.0 means total thrust equals vehicle weight, so the envelope represents
% level-altitude hover authority. Increase it to inspect a climbing/high-
% collective operating point without changing the motor geometry.
geometryInputs.momentEnvelope.collectiveWeightScale = 1.0;

%% Configuration comparison
% Each listed script must create a droneInputs structure. Add or remove files
% here to compare real CAD exports. The supplied X reference inherits the
% deadcat snapshot and changes only motor placement.
geometryInputs.configurationComparison.inputFiles = {
    fullfile(projectRoot, 'inputs', 'configurations', ...
        'current_deadcat_inputs.m');
    fullfile(projectRoot, 'inputs', 'configurations', ...
        'example_symmetric_x_inputs.m')
};
geometryInputs.configurationComparison.numberOfDirections = 180;

% Static geometry comparison is the default. Set this true only when you
% also want every configuration to fly the exact programmed 6DOF maneuver;
% that adds controller/trajectory effects and takes longer.
geometryInputs.configurationComparison.runShared6DOF = false;
