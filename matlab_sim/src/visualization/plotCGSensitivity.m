function graphics = plotCGSensitivity(sensitivity, options)
%PLOTCGSENSITIVITY Plot hover and authority maps versus final vehicle CG.
%
% graphics = plotCGSensitivity(sensitivity)
% graphics = plotCGSensitivity(sensitivity, options)
%
% The input is the output of analyzeCGSensitivity. Infeasible CG points are
% gray. Axes are displayed as a top view: +x is visually left and +y
% (backward) is visually down.
%
% Optional fields
%   showHoverMaps      - Plot hover margin and motor commands (default true)
%   showAuthorityMaps  - Plot roll/pitch/yaw authority (default true)
%   showBaseCG         - Mark the configured base CG (default true)
%   showBestMarginCG   - Mark the best sampled hover-margin CG (default true)
%   candidateCG_mm     - N-by-2 or N-by-3 final assembled CG candidates
%   candidateLabels    - N labels for candidateCG_mm

    if nargin < 2 || isempty(options)
        options = struct();
    end
    validateSensitivity(sensitivity);
    options = validatePlotOptions(options, sensitivity);

    graphics = struct('hoverFigure', [], 'hoverAxes', [], ...
        'authorityFigure', [], 'authorityAxes', []);
    if options.showHoverMaps
        [graphics.hoverFigure, graphics.hoverAxes] = ...
            plotHoverMaps(sensitivity, options);
    end
    if options.showAuthorityMaps
        [graphics.authorityFigure, graphics.authorityAxes] = ...
            plotAuthorityMaps(sensitivity, options);
    end
end

function [figureHandle, axesHandles] = plotHoverMaps(sensitivity, options)
    numberOfMotors = numel(sensitivity.motorNames);
    numberOfPanels = numberOfMotors + 2;
    numberOfColumns = min(3, max(2, ceil(sqrt(numberOfPanels))));
    numberOfRows = ceil(numberOfPanels / numberOfColumns);
    figureHandle = figure('Name', 'CG sensitivity: hover and motor loading', ...
        'Color', 'w', 'Position', [80, 80, 1450, 820]);
    layout = tiledlayout(figureHandle, numberOfRows, numberOfColumns, ...
        'TileSpacing', 'compact', 'Padding', 'compact');
    axesHandles = gobjects(numberOfPanels, 1);
    marginUpperLimit = roundedUpperLimit( ...
        sensitivity.minimumThrustMarginPercent, 5, 50);
    commandUpperLimit = roundedUpperLimit( ...
        sensitivity.hoverCommandPercent, 10, 100);

    axesHandles(1) = nexttile(layout);
    drawMap(axesHandles(1), sensitivity, ...
        sensitivity.minimumThrustMarginPercent, ...
        'Minimum two-sided motor margin', '%', ...
        [0, marginUpperLimit], options, true);

    axesHandles(2) = nexttile(layout);
    drawMap(axesHandles(2), sensitivity, ...
        sensitivity.maximumHoverCommandPercent, ...
        'Highest hover command', '%', [0, commandUpperLimit], ...
        options, false);

    for motorIndex = 1:numberOfMotors
        axesHandles(motorIndex + 2) = nexttile(layout);
        motorCommand = sensitivity.hoverCommandPercent(:, :, motorIndex);
        drawMap(axesHandles(motorIndex + 2), sensitivity, motorCommand, ...
            sprintf('%s hover command', ...
            sensitivity.motorNames{motorIndex}), '%', ...
            [0, commandUpperLimit], ...
            options, false);
    end
    colormap(figureHandle, parula(256));
    title(layout, sprintf( ...
        'Final-CG hover loading at z = %.3f mm  |  gray = infeasible', ...
        sensitivity.fixedCGZ_mm), 'FontWeight', 'bold');
end

function [figureHandle, axesHandles] = plotAuthorityMaps(sensitivity, options)
    figureHandle = figure('Name', 'CG sensitivity: control authority', ...
        'Color', 'w', 'Position', [100, 120, 1500, 520]);
    layout = tiledlayout(figureHandle, 1, 3, ...
        'TileSpacing', 'compact', 'Padding', 'compact');
    axesHandles = gobjects(3, 1);
    for axisIndex = 1:3
        axesHandles(axisIndex) = nexttile(layout);
        authorityMap = sensitivity.twoSidedMomentAuthority_Nm(:, :, axisIndex);
        titleText = sprintf('%s: both directions (%s)', ...
            sensitivity.axisNames{axisIndex}, ...
            sensitivity.momentNames{axisIndex});
        drawMap(axesHandles(axisIndex), sensitivity, authorityMap, ...
            titleText, 'N m', [], options, axisIndex == 1);
    end
    colormap(figureHandle, turbo(256));
    title(layout, sprintf([ ...
        'Constant-lift, decoupled moment authority versus final CG  |  ' ...
        'z = %.3f mm'], sensitivity.fixedCGZ_mm), 'FontWeight', 'bold');
end

function drawMap(axesHandle, sensitivity, mapData, titleText, ...
        unitText, fixedLimits, options, showLegend)
    finiteMask = isfinite(mapData) & sensitivity.hoverFeasible;
    imageHandle = imagesc(axesHandle, sensitivity.cgX_mm, ...
        sensitivity.cgY_mm, mapData);
    imageHandle.AlphaData = double(finiteMask);
    axesHandle.Color = [0.84, 0.84, 0.84];
    axesHandle.XDir = 'reverse';
    axesHandle.YDir = 'reverse';
    axesHandle.DataAspectRatio = [1, 1, 1];
    xlim(axesHandle, gridLimits(sensitivity.cgX_mm));
    ylim(axesHandle, gridLimits(sensitivity.cgY_mm));
    grid(axesHandle, 'on');
    box(axesHandle, 'on');
    hold(axesHandle, 'on');

    if isempty(fixedLimits)
        applyDataLimits(axesHandle, mapData(finiteMask));
    else
        clim(axesHandle, fixedLimits);
    end
    colorBar = colorbar(axesHandle);
    colorBar.Label.String = unitText;
    title(axesHandle, titleText);
    xlabel(axesHandle, 'Final CG x (mm; + left)');
    ylabel(axesHandle, 'Final CG y (mm; + backward)');

    [markerHandles, markerLabels] = overlayCGMarkers(axesHandle, ...
        sensitivity, options, showLegend);
    if showLegend && ~isempty(markerHandles)
        legend(axesHandle, markerHandles, markerLabels, ...
            'Location', 'best', 'Color', 'w');
    end
    hold(axesHandle, 'off');
end

function [markerHandles, markerLabels] = overlayCGMarkers(axesHandle, ...
        sensitivity, options, showCandidateLabels)
    markerHandles = gobjects(0, 1);
    markerLabels = cell(0, 1);
    if options.showBaseCG
        baseCG = sensitivity.baseCenterOfMass_mm;
        markerHandles(end + 1, 1) = scatter(axesHandle, baseCG(1), ...
            baseCG(2), 65, 'wo', 'filled', 'MarkerEdgeColor', 'k', ...
            'LineWidth', 1.1);
        markerLabels{end + 1, 1} = 'Current CG';
    end
    if options.showBestMarginCG && ...
            all(isfinite(sensitivity.bestMarginCG_mm(1:2)))
        bestCG = sensitivity.bestMarginCG_mm;
        markerHandles(end + 1, 1) = scatter(axesHandle, bestCG(1), ...
            bestCG(2), 100, 'p', 'filled', 'MarkerFaceColor', ...
            [1.0, 0.85, 0.1], 'MarkerEdgeColor', 'k', ...
            'LineWidth', 1.0);
        markerLabels{end + 1, 1} = 'Best sampled margin';
    end
    if ~isempty(options.candidateCG_mm)
        candidates = options.candidateCG_mm;
        markerHandles(end + 1, 1) = scatter(axesHandle, candidates(:, 1), ...
            candidates(:, 2), 48, 'd', 'filled', 'MarkerFaceColor', ...
            [0.95, 0.25, 0.7], 'MarkerEdgeColor', 'k', ...
            'LineWidth', 0.8);
        markerLabels{end + 1, 1} = 'Candidate placement CG';
        if showCandidateLabels
            for candidateIndex = 1:size(candidates, 1)
                text(axesHandle, candidates(candidateIndex, 1), ...
                    candidates(candidateIndex, 2), ...
                    ['  ', options.candidateLabels{candidateIndex}], ...
                    'FontSize', 8, 'Color', [0.15, 0.05, 0.12], ...
                    'Clipping', 'on');
            end
        end
    end
end

function limits = gridLimits(values)
    if numel(values) > 1
        limits = [values(1), values(end)];
    else
        halfWidth = max(1, abs(values(1)) * 0.05);
        limits = values(1) + [-halfWidth, halfWidth];
    end
end

function applyDataLimits(axesHandle, finiteData)
    if isempty(finiteData)
        clim(axesHandle, [0, 1]);
        return;
    end
    lowerLimit = min(finiteData);
    upperLimit = max(finiteData);
    if upperLimit <= lowerLimit
        padding = max(1e-9, abs(lowerLimit) * 0.05);
        clim(axesHandle, [lowerLimit - padding, upperLimit + padding]);
    else
        clim(axesHandle, [lowerLimit, upperLimit]);
    end
end

function upperLimit = roundedUpperLimit(data, increment, hardMaximum)
    finiteData = data(isfinite(data));
    if isempty(finiteData)
        upperLimit = increment;
        return;
    end
    upperLimit = increment * ceil(max(finiteData) / increment);
    upperLimit = min(hardMaximum, max(increment, upperLimit));
end

function options = validatePlotOptions(options, sensitivity)
    validateattributes(options, {'struct'}, {'scalar'}, mfilename, 'options');
    validFields = {'showHoverMaps', 'showAuthorityMaps', 'showBaseCG', ...
        'showBestMarginCG', 'candidateCG_mm', 'candidateLabels'};
    unknownFields = setdiff(fieldnames(options), validFields);
    if ~isempty(unknownFields)
        error('plotCGSensitivity:UnknownOption', ...
            'Unknown option field: %s.', unknownFields{1});
    end
    logicalDefaults = {
        'showHoverMaps', true;
        'showAuthorityMaps', true;
        'showBaseCG', true;
        'showBestMarginCG', true};
    for defaultIndex = 1:size(logicalDefaults, 1)
        fieldName = logicalDefaults{defaultIndex, 1};
        if ~isfield(options, fieldName)
            options.(fieldName) = logicalDefaults{defaultIndex, 2};
        end
        validateattributes(options.(fieldName), {'logical', 'numeric'}, ...
            {'scalar', 'real', 'finite'}, mfilename, ['options.', fieldName]);
        if isnumeric(options.(fieldName)) && ...
                options.(fieldName) ~= 0 && options.(fieldName) ~= 1
            error('plotCGSensitivity:InvalidLogicalOption', ...
                'options.%s must be true or false.', fieldName);
        end
        options.(fieldName) = logical(options.(fieldName));
    end

    if ~isfield(options, 'candidateCG_mm')
        if isfield(sensitivity, 'candidateCG_mm')
            options.candidateCG_mm = sensitivity.candidateCG_mm;
        else
            options.candidateCG_mm = zeros(0, 2);
        end
    end
    validateattributes(options.candidateCG_mm, {'numeric'}, ...
        {'2d', 'real', 'finite'}, mfilename, 'options.candidateCG_mm');
    if ~isempty(options.candidateCG_mm) && ...
            size(options.candidateCG_mm, 2) ~= 2 && ...
            size(options.candidateCG_mm, 2) ~= 3
        error('plotCGSensitivity:InvalidCandidateCGShape', ...
            'options.candidateCG_mm must have two or three columns.');
    end
    numberOfCandidates = size(options.candidateCG_mm, 1);
    if ~isfield(options, 'candidateLabels')
        if isfield(sensitivity, 'candidateLabels')
            options.candidateLabels = sensitivity.candidateLabels;
        else
            options.candidateLabels = arrayfun(@(index) ...
                sprintf('Candidate %d', index), 1:numberOfCandidates, ...
                'UniformOutput', false);
        end
    end
    if isstring(options.candidateLabels)
        options.candidateLabels = cellstr(options.candidateLabels);
    end
    if ~iscell(options.candidateLabels) || ...
            numel(options.candidateLabels) ~= numberOfCandidates || ...
            ~all(cellfun(@(value) ischar(value) && ~isempty(value), ...
            options.candidateLabels))
        error('plotCGSensitivity:InvalidCandidateLabels', ...
            'Provide one nonempty candidate label per candidate CG point.');
    end
    options.candidateLabels = options.candidateLabels(:);
end

function validateSensitivity(sensitivity)
    validateattributes(sensitivity, {'struct'}, {'scalar'}, ...
        mfilename, 'sensitivity');
    requiredFields = {'cgX_mm', 'cgY_mm', 'fixedCGZ_mm', ...
        'hoverFeasible', 'minimumThrustMarginPercent', ...
        'maximumHoverCommandPercent', 'hoverCommandPercent', ...
        'twoSidedMomentAuthority_Nm', 'motorNames', 'axisNames', ...
        'momentNames', 'baseCenterOfMass_mm', 'bestMarginCG_mm'};
    missingFields = setdiff(requiredFields, fieldnames(sensitivity));
    if ~isempty(missingFields)
        error('plotCGSensitivity:IncompleteAnalysis', ...
            'CG sensitivity structure is missing field: %s.', ...
            missingFields{1});
    end
    expectedGridSize = [numel(sensitivity.cgY_mm), ...
        numel(sensitivity.cgX_mm)];
    if ~isequal(size(sensitivity.hoverFeasible), expectedGridSize)
        error('plotCGSensitivity:InvalidGridShape', ...
            'hoverFeasible does not match cgX_mm and cgY_mm.');
    end
    numberOfMotors = numel(sensitivity.motorNames);
    expectedCommandSize = [expectedGridSize, numberOfMotors];
    if ~isequal(size(sensitivity.hoverCommandPercent), expectedCommandSize)
        error('plotCGSensitivity:InvalidMotorMapShape', ...
            'hoverCommandPercent must be y-by-x-by-motor.');
    end
    if ~isequal(size(sensitivity.twoSidedMomentAuthority_Nm), ...
            [expectedGridSize, 3])
        error('plotCGSensitivity:InvalidAuthorityMapShape', ...
            'twoSidedMomentAuthority_Nm must be y-by-x-by-3.');
    end
end
