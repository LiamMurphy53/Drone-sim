%% Legacy shortcut: animate the current programmed 6DOF maneuver in 3D
% The historical filename is retained so existing shortcuts still work.

projectRoot = fileparts(mfilename('fullpath'));
run(fullfile(projectRoot, 'setup_project.m'));
showFull6DOFPlots = false; %#ok<NASGU>
animateFull6DOF = true; %#ok<NASGU>
run(fullfile(projectRoot, 'run_6dof_simulation.m'));
clear showFull6DOFPlots animateFull6DOF;
