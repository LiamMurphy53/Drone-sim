%% Compare multiple drone configurations
% Edit inputs/geometry_analysis_inputs.m to choose input files.

projectRoot = fileparts(mfilename('fullpath'));
run(fullfile(projectRoot, 'setup_project.m'));
run(fullfile(projectRoot, 'inputs', 'drone_config_inputs.m'));
run(fullfile(projectRoot, 'inputs', 'geometry_analysis_inputs.m'));

comparisonSettings = geometryInputs.configurationComparison;
comparisonOptions.numberOfDirections = ...
    comparisonSettings.numberOfDirections;
comparisonOptions.printUnderlyingAnalysis = false;
comparisonOptions.printSummary = true;
comparisonOptions.runShared6DOF = comparisonSettings.runShared6DOF;
comparisonOptions.continueOnSimulationFailure = true;
comparisonOptions.storeSimulationResults = true;

if comparisonOptions.runShared6DOF
    run(fullfile(projectRoot, 'inputs', 'simulation_inputs.m'));
    sharedSettings = simulationInputs.full6DOF;
    sharedSettings.timeStep_s = simulationInputs.dynamics.timeStep_s;
    sharedSettings.motorTimeConstant_s = ...
        simulationInputs.dynamics.motorTimeConstant_s;
    comparisonOptions.shared6DOFSettings = sharedSettings;
end

configurationComparison = compareDroneConfigurations( ...
    comparisonSettings.inputFiles, comparisonOptions);
plotDroneConfigurationComparison(configurationComparison);
