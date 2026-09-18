%% Analyze control authority in every horizontal direction

projectRoot = fileparts(mfilename('fullpath'));
run(fullfile(projectRoot, 'setup_project.m'));
run(fullfile(projectRoot, 'inputs', 'drone_config_inputs.m'));
run(fullfile(projectRoot, 'inputs', 'simulation_inputs.m'));

configurationIsMissing = ~exist('config', 'var') || ~exist('hover', 'var');
configurationIsOutdated = ~configurationIsMissing && ...
    (~isfield(config, 'motorNames') || ~isfield(hover, 'hoverFeasible') || ...
     ~droneConfigMatchesInputs(config, droneInputs));
if configurationIsMissing || configurationIsOutdated
    run(fullfile(projectRoot, 'run_drone_config.m'));
end

numberOfDirections = ...
    simulationInputs.directionalAuthority.numberOfDirections;
directionalAuthority = analyzeDirectionalAuthority(config, hover, ...
    numberOfDirections);
plotDirectionalAuthority(directionalAuthority);
