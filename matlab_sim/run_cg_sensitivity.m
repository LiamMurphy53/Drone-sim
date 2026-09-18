%% Map hover loading and control authority versus total center of mass
% Edit inputs/geometry_analysis_inputs.m to change the sweep or overlay
% candidate battery/payload positions.

projectRoot = fileparts(mfilename('fullpath'));
run(fullfile(projectRoot, 'setup_project.m'));
run(fullfile(projectRoot, 'inputs', 'drone_config_inputs.m'));
run(fullfile(projectRoot, 'inputs', 'geometry_analysis_inputs.m'));

cgSettings = geometryInputs.cgSensitivity;
analysisDroneInputs = droneInputs;
plotOptions = struct();
componentCGCandidates = struct();

placement = cgSettings.componentPlacement;
if placement.enabled
    componentCGCandidates = calculateComponentCGCandidates( ...
        droneInputs.totalMass_g, droneInputs.centerOfMass_mm, ...
        placement.mass_g, placement.candidatePositions_mm, ...
        placement.mode, placement.currentPosition_mm);
    plotOptions.candidateCG_mm = componentCGCandidates.totalCG_mm;
    numberOfCandidates = size(componentCGCandidates.totalCG_mm, 1);
    plotOptions.candidateLabels = arrayfun(@(index) ...
        sprintf('Candidate %d', index), 1:numberOfCandidates, ...
        'UniformOutput', false);

    if strcmp(componentCGCandidates.mode, 'add-payload')
        analysisDroneInputs.totalMass_g = ...
            componentCGCandidates.totalMass_g(1);
        plotOptions.showBaseCG = false;
    end

    fprintf('\nComponent placement converted to final vehicle CG\n');
    fprintf(['  Candidate     Component position [x y z] mm' ...
        '       Final CG [x y z] mm\n']);
    for candidateIndex = 1:numberOfCandidates
        fprintf(['  %5d       [%8.2f %8.2f %8.2f]' ...
            '       [%8.2f %8.2f %8.2f]\n'], candidateIndex, ...
            componentCGCandidates.componentCandidatePositions_mm( ...
            candidateIndex, :), ...
            componentCGCandidates.totalCG_mm(candidateIndex, :));
    end
    fprintf(['  Note: inertia is held at the entered CAD value; update it ' ...
        'when a component move materially changes mass distribution.\n']);
end

analysisOptions.fixedZ_mm = cgSettings.fixedZ_mm;
analysisOptions.printSummary = true;
cgSensitivity = analyzeCGSensitivity(analysisDroneInputs, ...
    cgSettings.xCoordinates_mm, cgSettings.yCoordinates_mm, ...
    analysisOptions);
cgSensitivityGraphics = plotCGSensitivity(cgSensitivity, plotOptions);
