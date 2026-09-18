clc
clear
close all;
%% Run selected drone analyses and simulations
% Press Run, then choose the stages you want from the checklist.

projectRoot = fileparts(mfilename('fullpath'));
run(fullfile(projectRoot, 'setup_project.m'));
run(fullfile(projectRoot, 'inputs', 'simulation_inputs.m'));

stageLabels = { ...
    'Drone layout and hover analysis', ...
    '360-degree directional authority', ...
    'Center-of-mass placement sensitivity', ...
    'Full Mx-My-Mz moment envelope', ...
    'Normalized mixer geometry quality', ...
    'Compare multiple frame configurations', ...
    'Roll view of programmed 6DOF maneuver', ...
    'Pitch view of programmed 6DOF maneuver', ...
    'Yaw view of programmed 6DOF maneuver', ...
    'Programmed 6DOF roll, pitch, and yaw comparison', ...
    'Full 6DOF response plots', ...
    'Full 6DOF 3D animation'};

if usejava('desktop')
    [selectedStages, selectionConfirmed] = listdlg( ...
        'Name', 'Run drone simulations', ...
        'PromptString', 'Choose what to run:', ...
        'ListString', stageLabels, ...
        'SelectionMode', 'multiple', ...
        'InitialValue', 1:numel(stageLabels), ...
        'ListSize', [440, 330]);
else
    selectedStages = 1:numel(stageLabels);
    selectionConfirmed = true;
    fprintf(['MATLAB desktop selection is unavailable; running all ' ...
        'simulation stages.\n']);
end

if ~selectionConfirmed || isempty(selectedStages)
    fprintf('No simulation stages selected.\n');
    return;
end

runLayout = ismember(1, selectedStages);
runAuthority = ismember(2, selectedStages);
runCGSensitivity = ismember(3, selectedStages);
runMomentEnvelope = ismember(4, selectedStages);
runMixerQuality = ismember(5, selectedStages);
runConfigurationComparison = ismember(6, selectedStages);
runRoll = ismember(7, selectedStages);
runPitch = ismember(8, selectedStages);
runYaw = ismember(9, selectedStages);
runAxisComparison = ismember(10, selectedStages);
runFull6DOFPlots = ismember(11, selectedStages);
runFull6DOFAnimation = ismember(12, selectedStages);

fprintf('\nRunning selected drone workflows\n');
for selectedIndex = selectedStages
    fprintf('  - %s\n', stageLabels{selectedIndex});
end

% Every downstream calculation needs the physical configuration. Suppress
% only its layout figure when that stage was not selected.
showDroneConfigPlot = runLayout; %#ok<NASGU> Used by run_drone_config.m.
run(fullfile(projectRoot, 'run_drone_config.m'));
clear showDroneConfigPlot;

if runAuthority
    run(fullfile(projectRoot, 'run_directional_authority.m'));
end
if runCGSensitivity
    run(fullfile(projectRoot, 'run_cg_sensitivity.m'));
end
if runMomentEnvelope
    run(fullfile(projectRoot, 'run_moment_envelope_3d.m'));
end
if runMixerQuality
    run(fullfile(projectRoot, 'run_mixer_quality.m'));
end
if runConfigurationComparison
    run(fullfile(projectRoot, 'run_configuration_comparison.m'));
end

% All axis views, the comparison, the full plots, and the animation consume
% one exact coupled simulation of the maneuver in simulation_inputs.m.
needsFull6DOF = runRoll || runPitch || runYaw || runAxisComparison || ...
    runFull6DOFPlots || runFull6DOFAnimation;
if needsFull6DOF
    showFull6DOFPlots = runFull6DOFPlots; %#ok<NASGU>
    animateFull6DOF = runFull6DOFAnimation; %#ok<NASGU>
    run(fullfile(projectRoot, 'run_6dof_simulation.m'));
    clear showFull6DOFPlots animateFull6DOF;

    if runRoll || runAxisComparison
        rollSimulation = extractDrone6DOFAxisResponse( ...
            full6DOFSimulation, 'Roll');
        if runRoll
            plotDrone6DOFAxisResponse(rollSimulation);
        end
    end
    if runPitch || runAxisComparison
        pitchSimulation = extractDrone6DOFAxisResponse( ...
            full6DOFSimulation, 'Pitch');
        if runPitch
            plotDrone6DOFAxisResponse(pitchSimulation);
        end
    end
    if runYaw || runAxisComparison
        yawSimulation = extractDrone6DOFAxisResponse( ...
            full6DOFSimulation, 'Yaw');
        if runYaw
            plotDrone6DOFAxisResponse(yawSimulation);
        end
    end
    if runAxisComparison
        axisComparison = {rollSimulation, pitchSimulation, yawSimulation};
        plotDrone6DOFAxisComparison(axisComparison);
    end
end

fprintf('\nSelected drone workflows complete.\n');
