%% Analyze the exact achievable Mx-My-Mz set at fixed collective thrust

projectRoot = fileparts(mfilename('fullpath'));
run(fullfile(projectRoot, 'setup_project.m'));
run(fullfile(projectRoot, 'inputs', 'drone_config_inputs.m'));
run(fullfile(projectRoot, 'inputs', 'geometry_analysis_inputs.m'));

configurationIsMissing = ~exist('config', 'var');
configurationIsOutdated = ~configurationIsMissing && ...
    ~droneConfigMatchesInputs(config, droneInputs);
if configurationIsMissing || configurationIsOutdated
    showDroneConfigPlot = false; %#ok<NASGU>
    run(fullfile(projectRoot, 'run_drone_config.m'));
    clear showDroneConfigPlot;
end

collectiveThrust_N = ...
    geometryInputs.momentEnvelope.collectiveWeightScale * config.weight;
momentEnvelope3D = analyzeMomentEnvelope3D(config, collectiveThrust_N);
plotMomentEnvelope3D(momentEnvelope3D);
