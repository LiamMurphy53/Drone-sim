%% Analyze normalized motor-mixer geometry and its weakest direction

projectRoot = fileparts(mfilename('fullpath'));
run(fullfile(projectRoot, 'setup_project.m'));
run(fullfile(projectRoot, 'inputs', 'drone_config_inputs.m'));

configurationIsMissing = ~exist('config', 'var');
configurationIsOutdated = ~configurationIsMissing && ...
    ~droneConfigMatchesInputs(config, droneInputs);
if configurationIsMissing || configurationIsOutdated
    showDroneConfigPlot = false; %#ok<NASGU>
    run(fullfile(projectRoot, 'run_drone_config.m'));
    clear showDroneConfigPlot;
end

mixerQuality = analyzeMixerQuality(config);
plotMixerQuality(mixerQuality);
