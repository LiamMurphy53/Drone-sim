%% Full programmable six-degree-of-freedom simulation
% Edit inputs/simulation_inputs.m commandWaypoints and rateCommandSegments,
% then run this script or select the full 6DOF stages in
% run_all_simulations.m.

projectRoot = fileparts(mfilename('fullpath'));
run(fullfile(projectRoot, 'setup_project.m'));
run(fullfile(projectRoot, 'inputs', 'drone_config_inputs.m'));
run(fullfile(projectRoot, 'inputs', 'simulation_inputs.m'));

configurationIsMissing = ~exist('config', 'var');
configurationIsOutdated = ~configurationIsMissing && ...
    ~droneConfigMatchesInputs(config, droneInputs);
if configurationIsMissing || configurationIsOutdated
    showDroneConfigPlot = false; %#ok<NASGU>
    run(fullfile(projectRoot, 'run_drone_config.m'));
    clear showDroneConfigPlot;
end

full6DOFSettings = simulationInputs.full6DOF;
full6DOFSettings.timeStep_s = simulationInputs.dynamics.timeStep_s;
full6DOFSettings.motorTimeConstant_s = ...
    simulationInputs.dynamics.motorTimeConstant_s;
fprintf('\n6DOF maneuver input: %s\n', ...
    fullfile(projectRoot, 'inputs', 'simulation_inputs.m'));
if isfield(full6DOFSettings, 'maneuverName')
    fprintf('Maneuver: %s\n', full6DOFSettings.maneuverName);
end
if isfield(full6DOFSettings, 'rateCommandSegments')
    fprintf('Rate-control segments loaded: %d\n', ...
        size(full6DOFSettings.rateCommandSegments, 1));
else
    fprintf('Rate-control segments loaded: 0\n');
end
full6DOFSimulation = simulateDrone6DOF(config, full6DOFSettings);

if ~exist('showFull6DOFPlots', 'var') || showFull6DOFPlots
    plotDrone6DOF(full6DOFSimulation);
end
if ~exist('animateFull6DOF', 'var') || animateFull6DOF
    animateDrone6DOF(config, full6DOFSimulation, ...
        simulationInputs.animation3D.playbackSpeed);
end
