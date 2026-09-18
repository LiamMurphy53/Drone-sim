%% Roll view of the programmed 6DOF maneuver (body y / My)
% The historical filename is retained so existing shortcuts still work.

projectRoot = fileparts(mfilename('fullpath'));
run(fullfile(projectRoot, 'setup_project.m'));
showFull6DOFPlots = false; %#ok<NASGU>
animateFull6DOF = false; %#ok<NASGU>
run(fullfile(projectRoot, 'run_6dof_simulation.m'));
clear showFull6DOFPlots animateFull6DOF;

rollSimulation = extractDrone6DOFAxisResponse(full6DOFSimulation, 'Roll');
if ~exist('showRollAxisPlot', 'var') || showRollAxisPlot
    plotDrone6DOFAxisResponse(rollSimulation);
end
