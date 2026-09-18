function comparison = compareDroneConfigurations(configurationSources, options)
%COMPAREDRONECONFIGURATIONS Compare physical performance and geometry.
%
% comparison = compareDroneConfigurations(configurationSources)
% comparison = compareDroneConfigurations(configurationSources, options)
%
% configurationSources may be:
%   * a string array or cell array of drone-input .m file names;
%   * droneInputs structures (one structure or a cell array);
%   * descriptor structures with fields label and either inputFile,
%     droneInputs, or config;
%   * an N-by-2 cell array whose columns are label and source.
%
% The static comparison is the default. It includes hover trim, the full
% horizontal directional-authority sweep, yaw authority when modeled, and
% row-normalized allocation-matrix quality. Raw metrics describe the actual
% vehicle. Dimensionless metrics divide out force, size, and inertia scales
% so layouts of different sizes can be compared more fairly.
%
% Options (all optional structure fields)
%   numberOfDirections          - Authority sweep samples (default 180).
%   printUnderlyingAnalysis     - Show each analyzer's output (false).
%   printSummary                - Print the comparison table (true).
%   runShared6DOF               - Run the same command program (false).
%   shared6DOFSettings          - Settings passed to simulateDrone6DOF.
%   continueOnSimulationFailure - Record a failed program and continue
%                                 (true).
%   storeSimulationResults      - Retain complete 6DOF results (true).
%
% A shared 6DOF run is optional because it compares complete vehicles and
% controller/allocation behavior, not frame geometry alone. The same
% command settings are reused, while simulateDrone6DOF redesigns gains from
% each vehicle's inertia using the same response specifications.

    if nargin < 2 || isempty(options)
        options = struct();
    end
    options = completeAndValidateOptions(options);
    sources = normalizeConfigurationSources(configurationSources);
    numberOfConfigurations = numel(sources);
    if numberOfConfigurations < 1
        error('compareDroneConfigurations:NoConfigurations', ...
            'Provide at least one configuration source.');
    end

    emptyEntry = struct( ...
        'label', '', ...
        'sourceType', '', ...
        'sourcePath', '', ...
        'droneInputs', struct(), ...
        'config', struct(), ...
        'hover', struct(), ...
        'directionalAuthority', struct(), ...
        'yawAuthority', struct(), ...
        'mixerQuality', struct(), ...
        'metrics', struct(), ...
        'sharedManeuver', emptyManeuverResult(false));
    entries = repmat(emptyEntry, numberOfConfigurations, 1);

    for configurationIndex = 1:numberOfConfigurations
        source = sources(configurationIndex);
        if isempty(fieldnames(source.config))
            config = buildConfiguration(source.droneInputs, ...
                source.label, options.printUnderlyingAnalysis);
        else
            config = validateAnalyzedConfiguration(source.config, ...
                source.label);
        end

        hover = runHoverAnalysis(config, ...
            options.printUnderlyingAnalysis);
        if hover.hoverFeasible
            directionalAuthority = runDirectionalAnalysis(config, hover, ...
                options.numberOfDirections, ...
                options.printUnderlyingAnalysis);
        else
            directionalAuthority = unavailableDirectionalAuthority();
        end
        yawAuthority = calculateYawAuthority(config, hover.hoverFeasible);
        mixerQuality = calculateMixerQuality(config, ...
            options.printUnderlyingAnalysis);
        metrics = collectMetrics(config, hover, directionalAuthority, ...
            yawAuthority, mixerQuality);

        maneuver = emptyManeuverResult(options.runShared6DOF);
        if options.runShared6DOF
            maneuver = runSharedManeuver(config, ...
                options.shared6DOFSettings, options);
        end

        entries(configurationIndex).label = source.label;
        entries(configurationIndex).sourceType = source.sourceType;
        entries(configurationIndex).sourcePath = source.sourcePath;
        entries(configurationIndex).droneInputs = source.droneInputs;
        entries(configurationIndex).config = config;
        entries(configurationIndex).hover = hover;
        entries(configurationIndex).directionalAuthority = ...
            directionalAuthority;
        entries(configurationIndex).yawAuthority = yawAuthority;
        entries(configurationIndex).mixerQuality = mixerQuality;
        entries(configurationIndex).metrics = metrics;
        entries(configurationIndex).sharedManeuver = maneuver;
    end

    comparison.labels = {entries.label}.';
    comparison.entries = entries;
    comparison.metrics = vertcat(entries.metrics);
    comparison.metricTable = createMetricTable(entries);
    comparison.options = options;
    comparison.normalization.characteristicArm = ...
        ['Thrust-capacity-weighted RMS horizontal distance from the ' ...
         'center of mass.'];
    comparison.normalization.hoverBalanceScore = ...
        ['Actual two-sided hover margin divided by the best possible ' ...
         'margin at the same total thrust loading.'];
    comparison.normalization.momentScore = ...
        'Worst constant-altitude moment divided by weight times RMS arm.';
    comparison.normalization.angularAccelerationScore = ...
        ['Worst angular acceleration times effective horizontal inertia, ' ...
         'divided by weight times RMS arm.'];
    comparison.normalization.mixerIsotropy = ...
        ['Smallest/largest fixed-collective moment singular-value ratio ' ...
         'using one common moment scale, so weak yaw is not hidden; one ' ...
         'is ideal.'];
    comparison.sharedManeuver.requested = options.runShared6DOF;
    comparison.sharedManeuver.settings = options.shared6DOFSettings;
    comparison.sharedManeuver.interpretation = ...
        ['Optional maneuver metrics compare each complete vehicle under ' ...
         'one command program; static normalized metrics are the fairer ' ...
         'frame-geometry comparison.'];

    if options.printSummary
        printComparisonSummary(entries, options.runShared6DOF);
    end
end

function options = completeAndValidateOptions(options)
    if ~isstruct(options) || ~isscalar(options)
        error('compareDroneConfigurations:InvalidOptions', ...
            'options must be one scalar structure.');
    end
    allowedFields = {'numberOfDirections', 'printUnderlyingAnalysis', ...
        'printSummary', 'runShared6DOF', 'shared6DOFSettings', ...
        'continueOnSimulationFailure', 'storeSimulationResults'};
    unknownFields = setdiff(fieldnames(options), allowedFields);
    if ~isempty(unknownFields)
        error('compareDroneConfigurations:UnknownOption', ...
            'Unknown comparison option: %s', unknownFields{1});
    end

    sharedSettingsWereProvided = isfield(options, 'shared6DOFSettings') && ...
        ~isempty(options.shared6DOFSettings);
    runWasSpecified = isfield(options, 'runShared6DOF');
    defaults.numberOfDirections = 180;
    defaults.printUnderlyingAnalysis = false;
    defaults.printSummary = true;
    defaults.runShared6DOF = false;
    defaults.shared6DOFSettings = struct();
    defaults.continueOnSimulationFailure = true;
    defaults.storeSimulationResults = true;
    defaultNames = fieldnames(defaults);
    for fieldIndex = 1:numel(defaultNames)
        name = defaultNames{fieldIndex};
        if ~isfield(options, name)
            options.(name) = defaults.(name);
        end
    end
    if sharedSettingsWereProvided && ~runWasSpecified
        options.runShared6DOF = true;
    end

    validateattributes(options.numberOfDirections, {'numeric'}, ...
        {'scalar', 'integer', '>=', 12, 'finite'}, mfilename, ...
        'options.numberOfDirections');
    logicalFields = {'printUnderlyingAnalysis', 'printSummary', ...
        'runShared6DOF', 'continueOnSimulationFailure', ...
        'storeSimulationResults'};
    for fieldIndex = 1:numel(logicalFields)
        name = logicalFields{fieldIndex};
        if ~(islogical(options.(name)) && isscalar(options.(name)))
            error('compareDroneConfigurations:InvalidOption', ...
                'options.%s must be one logical value.', name);
        end
    end
    if options.runShared6DOF && ...
            (~isstruct(options.shared6DOFSettings) || ...
             ~isscalar(options.shared6DOFSettings) || ...
             isempty(fieldnames(options.shared6DOFSettings)))
        error('compareDroneConfigurations:MissingShared6DOFSettings', ...
            ['Set options.shared6DOFSettings to the programmed maneuver ' ...
             'settings before enabling runShared6DOF.']);
    end
end

function sources = normalizeConfigurationSources(rawSources)
    if isstring(rawSources)
        sourceItems = cellstr(rawSources(:));
        explicitLabels = repmat({''}, numel(sourceItems), 1);
    elseif ischar(rawSources)
        sourceItems = {rawSources};
        explicitLabels = {''};
    elseif iscell(rawSources)
        isLabeledMatrix = size(rawSources, 2) == 2 && ...
            all(cellfun(@isTextScalar, rawSources(:, 1)));
        if isLabeledMatrix && isvector(rawSources) && ...
                all(cellfun(@isTextScalar, rawSources)) && ...
                sourceFileExists(rawSources{1})
            % A two-file row vector is more likely two sources than one
            % label/source pair. Use a descriptor for an ambiguous pair.
            isLabeledMatrix = false;
        end
        if isLabeledMatrix
            explicitLabels = rawSources(:, 1);
            sourceItems = rawSources(:, 2);
        else
            sourceItems = rawSources(:);
            explicitLabels = repmat({''}, numel(sourceItems), 1);
        end
    elseif isstruct(rawSources)
        sourceItems = num2cell(rawSources(:));
        explicitLabels = repmat({''}, numel(sourceItems), 1);
    else
        error('compareDroneConfigurations:InvalidSources', ...
            ['configurationSources must contain file names, input ' ...
             'structures, analyzed configurations, or descriptors.']);
    end

    if isempty(sourceItems)
        sources = repmat(emptySource(), 0, 1);
        return;
    end
    sources = repmat(emptySource(), numel(sourceItems), 1);
    for itemIndex = 1:numel(sourceItems)
        sources(itemIndex) = normalizeOneSource(sourceItems{itemIndex}, ...
            explicitLabels{itemIndex}, itemIndex);
    end
    labels = cellfun(@lower, {sources.label}, 'UniformOutput', false);
    if numel(unique(labels)) ~= numel(labels)
        error('compareDroneConfigurations:DuplicateLabels', ...
            'Every compared configuration must have a unique label.');
    end
end

function source = normalizeOneSource(value, explicitLabel, itemIndex)
    source = emptySource();
    if isTextScalar(value)
        fileName = char(value);
        source.droneInputs = loadDroneInputFile(fileName);
        source.sourceType = 'input file';
        source.sourcePath = fileName;
        [~, defaultLabel] = fileparts(fileName);
        defaultLabel = defaultStructureLabel(source.droneInputs, ...
            itemIndex, defaultLabel);
    elseif isstruct(value) && isscalar(value)
        if isfield(value, 'inputFile') || isfield(value, 'droneInputs') || ...
                isfield(value, 'config')
            descriptorFields = {'inputFile', 'droneInputs', 'config'};
            populated = descriptorFields(cellfun(@(name) ...
                isfield(value, name) && ~isempty(value.(name)), ...
                descriptorFields));
            if numel(populated) ~= 1
                error('compareDroneConfigurations:InvalidDescriptor', ...
                    ['A source descriptor must populate exactly one of ' ...
                     'inputFile, droneInputs, or config.']);
            end
            selectedField = populated{1};
            selectedValue = value.(selectedField);
            switch selectedField
                case 'inputFile'
                    if ~isTextScalar(selectedValue)
                        error('compareDroneConfigurations:InvalidInputFile', ...
                            'Descriptor inputFile must be one text scalar.');
                    end
                    source.droneInputs = loadDroneInputFile(selectedValue);
                    source.sourceType = 'input file';
                    source.sourcePath = char(selectedValue);
                    [~, defaultLabel] = fileparts(source.sourcePath);
                    defaultLabel = defaultStructureLabel( ...
                        source.droneInputs, itemIndex, defaultLabel);
                case 'droneInputs'
                    if ~isstruct(selectedValue) || ~isscalar(selectedValue)
                        error('compareDroneConfigurations:InvalidDroneInputs', ...
                            'Descriptor droneInputs must be a scalar structure.');
                    end
                    source.droneInputs = selectedValue;
                    source.sourceType = 'input structure';
                    defaultLabel = sprintf('Configuration %d', itemIndex);
                case 'config'
                    source.config = selectedValue;
                    source.sourceType = 'analyzed configuration';
                    defaultLabel = sprintf('Configuration %d', itemIndex);
            end
            if isempty(explicitLabel) && isfield(value, 'label')
                explicitLabel = value.label;
            end
        elseif looksLikeDroneInputs(value)
            source.droneInputs = value;
            source.sourceType = 'input structure';
            defaultLabel = defaultStructureLabel(value, itemIndex);
        elseif looksLikeAnalyzedConfig(value)
            source.config = value;
            source.sourceType = 'analyzed configuration';
            defaultLabel = defaultStructureLabel(value, itemIndex);
        else
            error('compareDroneConfigurations:UnrecognizedStructure', ...
                ['A source structure must be droneInputs, an analyzed ' ...
                 'configuration, or a source descriptor.']);
        end
    else
        error('compareDroneConfigurations:InvalidSourceItem', ...
            'Configuration source %d has an unsupported type.', itemIndex);
    end

    if isempty(explicitLabel)
        source.label = defaultLabel;
    else
        if ~isTextScalar(explicitLabel) || ...
                isempty(strtrim(char(explicitLabel)))
            error('compareDroneConfigurations:InvalidLabel', ...
                'Every explicit label must be one nonempty text scalar.');
        end
        source.label = strtrim(char(explicitLabel));
    end
end

function source = emptySource()
    source = struct('label', '', 'sourceType', '', 'sourcePath', '', ...
        'droneInputs', struct(), 'config', struct());
end

function label = defaultStructureLabel(value, itemIndex, fallbackLabel)
    if nargin < 3
        fallbackLabel = sprintf('Configuration %d', itemIndex);
    end
    if isfield(value, 'configurationName') && ...
            isTextScalar(value.configurationName)
        label = char(value.configurationName);
    elseif isfield(value, 'comparisonLabel') && ...
            isTextScalar(value.comparisonLabel)
        label = char(value.comparisonLabel);
    elseif isfield(value, 'label') && isTextScalar(value.label)
        label = char(value.label);
    else
        label = fallbackLabel;
    end
end

function tf = isTextScalar(value)
    tf = (ischar(value) && isrow(value)) || ...
        (isstring(value) && isscalar(value));
end

function tf = sourceFileExists(value)
    if ~isTextScalar(value)
        tf = false;
        return;
    end
    value = char(value);
    tf = isfile(value) || ~isempty(which(value));
end

function tf = looksLikeDroneInputs(value)
    requiredFields = {'motorLocations_mm', 'maxThrust_N', ...
        'totalMass_g', 'centerOfMass_mm', 'inertiaTensor_g_mm2'};
    tf = isstruct(value) && isscalar(value) && ...
        all(isfield(value, requiredFields));
end

function tf = looksLikeAnalyzedConfig(value)
    requiredFields = {'numberOfMotors', 'maxThrust', 'weight', ...
        'motorLeverArms', 'inertiaTensor', 'thrustToWrench'};
    tf = isstruct(value) && isscalar(value) && ...
        all(isfield(value, requiredFields));
end

function config = buildConfiguration(droneInputs, label, showOutput)
    if ~looksLikeDroneInputs(droneInputs)
        error('compareDroneConfigurations:MissingInputField', ...
            ['Configuration "%s" must define motorLocations_mm, ' ...
             'maxThrust_N, totalMass_g, centerOfMass_mm, and ' ...
             'inertiaTensor_g_mm2.'], label);
    end
    if isfield(droneInputs, 'motorNames')
        motorNames = droneInputs.motorNames;
    else
        motorNames = [];
    end
    hasSpinDirections = isfield(droneInputs, 'motorSpinDirection');
    hasYawCoefficient = isfield(droneInputs, 'yawTorquePerThrust_m');
    if xor(hasSpinDirections, hasYawCoefficient)
        error('compareDroneConfigurations:IncompletePropulsionInputs', ...
            ['Configuration "%s" must provide both motorSpinDirection ' ...
             'and yawTorquePerThrust_m, or neither.'], label);
    end

    analysisCall = @() analyzeDroneConfig( ...
        droneInputs.motorLocations_mm, droneInputs.maxThrust_N, ...
        droneInputs.totalMass_g, droneInputs.centerOfMass_mm, ...
        droneInputs.inertiaTensor_g_mm2, motorNames, ...
        propulsionOrEmpty(droneInputs, hasSpinDirections));
    if showOutput
        config = analysisCall();
    else
        evalc('config = analysisCall();');
    end
end

function propulsion = propulsionOrEmpty(droneInputs, hasPropulsion)
    if hasPropulsion
        propulsion = droneInputs;
    else
        propulsion = [];
    end
end

function config = validateAnalyzedConfiguration(config, label)
    if ~looksLikeAnalyzedConfig(config)
        error('compareDroneConfigurations:InvalidAnalyzedConfiguration', ...
            'The analyzed configuration "%s" is missing core fields.', label);
    end
    if ~isfield(config, 'maxTotalThrust')
        config.maxTotalThrust = sum(config.maxThrust);
    end
    if ~isfield(config, 'totalMass_g') && isfield(config, 'totalMass')
        config.totalMass_g = 1000 * config.totalMass;
    end
    if ~isfield(config, 'maxThrustToWeight')
        config.maxThrustToWeight = config.maxTotalThrust / config.weight;
    end
    if ~isfield(config, 'commandToWrench')
        config.commandToWrench = config.thrustToWrench * ...
            diag(config.maxThrust);
    end
    if ~isfield(config, 'thrustToWrench4')
        config.thrustToWrench4 = [config.thrustToWrench; ...
            zeros(1, config.numberOfMotors)];
    end
    if ~isfield(config, 'commandToWrench4')
        config.commandToWrench4 = config.thrustToWrench4 * ...
            diag(config.maxThrust);
    end
    if ~isfield(config, 'controlRank4')
        config.controlRank4 = rank(config.thrustToWrench4);
    end
end

function hover = runHoverAnalysis(config, showOutput)
    analysisCall = @() analyzeHoverDynamics(config);
    if showOutput
        hover = analysisCall();
    else
        evalc('hover = analysisCall();');
    end
end

function authority = runDirectionalAnalysis(config, hover, ...
        numberOfDirections, showOutput)
    analysisCall = @() analyzeDirectionalAuthority(config, hover, ...
        numberOfDirections);
    if showOutput
        authority = analysisCall();
    else
        evalc('authority = analysisCall();');
    end
end

function authority = unavailableDirectionalAuthority()
    authority.available = false;
    authority.minimumMoment_Nm = NaN;
    authority.maximumMoment_Nm = NaN;
    authority.momentAnisotropy = NaN;
    authority.minimumAngularAcceleration_rad_s2 = NaN;
    authority.maximumAngularAcceleration_rad_s2 = NaN;
    authority.angularAccelerationAnisotropy = NaN;
end

function authority = calculateYawAuthority(config, hoverFeasible)
    authority.modeled = isfield(config, 'thrustToWrench4') && ...
        any(abs(config.thrustToWrench4(4, :)) > 1e-14);
    authority.feasible = false;
    authority.momentRange_Nm = [NaN, NaN];
    authority.twoSidedMoment_Nm = NaN;
    authority.normalizedTwoSidedMoment = NaN;
    authority.minimumMotorThrust_N = nan(config.numberOfMotors, 1);
    authority.maximumMotorThrust_N = nan(config.numberOfMotors, 1);
    if ~authority.modeled || ~hoverFeasible
        return;
    end

    map = config.thrustToWrench4;
    [minimumYaw, maximumYaw, minimumThrust, maximumThrust] = ...
        boundedLinearExtrema(map(4, :).', map(1:3, :), ...
        [config.weight; 0; 0], zeros(config.numberOfMotors, 1), ...
        config.maxThrust);
    authority.feasible = isfinite(minimumYaw) && isfinite(maximumYaw);
    if ~authority.feasible
        return;
    end
    authority.momentRange_Nm = [minimumYaw, maximumYaw];
    authority.twoSidedMoment_Nm = ...
        min(max(0, -minimumYaw), max(0, maximumYaw));
    authority.minimumMotorThrust_N = minimumThrust;
    authority.maximumMotorThrust_N = maximumThrust;
    yawScale = config.weight * weightedYawArm(config);
    if yawScale > 0
        authority.normalizedTwoSidedMoment = ...
            authority.twoSidedMoment_Nm / yawScale;
    end
end

function yawArm = weightedYawArm(config)
    yawPerThrust = abs(config.thrustToWrench4(4, :)).';
    yawArm = sum(config.maxThrust .* yawPerThrust) / ...
        sum(config.maxThrust);
end

function quality = calculateMixerQuality(config, showOutput)
    % These independently row-normalized values are retained as diagnostic
    % rank/correlation information only. They are not the geometry score,
    % because independently scaling Mz would make weak reaction-torque yaw
    % authority look as strong as lever-arm roll and pitch authority.
    quality.threeAxis = singularValueQuality(config.commandToWrench);
    yawModeled = any(abs(config.commandToWrench4(4, :)) > 1e-14);
    quality.fourAxisAvailable = yawModeled;
    if yawModeled
        quality.fourAxis = singularValueQuality(config.commandToWrench4);
    else
        quality.fourAxis = unavailableSVDQuality(4, ...
            config.numberOfMotors);
    end
    quality.fixedCollective = fixedCollectiveMixerFallback(config);
    quality.qualityScore_percent = ...
        quality.fixedCollective.qualityScore_percent;
    quality.authoritativeSource = 'internal common-scale fixed-collective SVD';

    quality.externalAvailable = false;
    quality.external = struct();
    quality.externalError = '';
    if exist('analyzeMixerQuality', 'file') == 2
        try
            analysisCall = @() analyzeMixerQuality(config);
            if showOutput
                externalQuality = analysisCall();
            else
                evalc('externalQuality = analysisCall();');
            end
            quality.externalAvailable = true;
            quality.external = externalQuality;
            if isstruct(externalQuality) && ...
                    isfield(externalQuality, 'fixedCollective') && ...
                    isfield(externalQuality.fixedCollective, ...
                    'qualityScore_percent')
                quality.fixedCollective = externalQuality.fixedCollective;
                quality.qualityScore_percent = ...
                    externalQuality.fixedCollective.qualityScore_percent;
                quality.authoritativeSource = ...
                    'analyzeMixerQuality fixedCollective';
            end
        catch exception
            % The canonical SVD metrics above remain available even if a
            % newer optional analyzer has a different requirement.
            quality.externalError = exception.message;
        end
    end
end

function fixedCollective = fixedCollectiveMixerFallback(config)
    maximumTotalThrust = sum(config.maxThrust);
    horizontalRadiusSquared = ...
        sum(config.motorLeverArms(:, 1:2).^2, 2);
    referenceArm = sqrt(sum(config.maxThrust .* horizontalRadiusSquared) / ...
        maximumTotalThrust);
    fixedCollective.referenceArm_m = referenceArm;
    fixedCollective.qualityScore_percent = 0;
    fixedCollective.isotropyIndex = 0;
    fixedCollective.conditionNumber = Inf;
    fixedCollective.rank = 0;
    fixedCollective.normalizedSingularValues = zeros(3, 1);
    if referenceArm <= 1e-14
        return;
    end

    outputScales = [maximumTotalThrust; ...
        repmat(maximumTotalThrust * referenceArm, 3, 1)];
    normalizedMap = diag(1 ./ outputScales) * ...
        config.commandToWrench4;
    constantCollectiveBasis = null(normalizedMap(1, :));
    momentMap = normalizedMap(2:4, :) * constantCollectiveBasis;
    singularValues = svd(momentMap);
    singularValues(end + 1:3, 1) = 0;
    singularValues = singularValues(1:3);
    tolerance = max(size(momentMap)) * eps(max([1; singularValues]));
    fixedCollective.normalizedMomentMap = momentMap;
    fixedCollective.normalizedSingularValues = singularValues;
    fixedCollective.rank = sum(singularValues > tolerance);
    if fixedCollective.rank == 3
        fixedCollective.conditionNumber = ...
            singularValues(1) / singularValues(3);
        fixedCollective.isotropyIndex = ...
            singularValues(3) / singularValues(1);
        fixedCollective.qualityScore_percent = ...
            100 * fixedCollective.isotropyIndex;
    end
end

function quality = singularValueQuality(commandMap)
    rowNorms = vecnorm(commandMap, 2, 2);
    quality.rowNorms = rowNorms;
    quality.normalizedMap = zeros(size(commandMap));
    nonzeroRows = rowNorms > 1e-14 * max(1, max(rowNorms));
    quality.normalizedMap(nonzeroRows, :) = ...
        commandMap(nonzeroRows, :) ./ rowNorms(nonzeroRows);
    singularValues = svd(quality.normalizedMap);
    tolerance = max(size(quality.normalizedMap)) * eps( ...
        max([1; singularValues]));
    quality.singularValues = singularValues;
    quality.rank = sum(singularValues > tolerance);
    quality.fullRowRank = quality.rank == size(commandMap, 1);
    if quality.fullRowRank
        quality.conditionNumber = singularValues(1) / ...
            singularValues(end);
        quality.isotropyScore = singularValues(end) / singularValues(1);
    else
        quality.conditionNumber = Inf;
        quality.isotropyScore = 0;
    end
    rowCorrelation = quality.normalizedMap * quality.normalizedMap.';
    rowCorrelation(1:size(rowCorrelation, 1) + 1:end) = 0;
    quality.maximumAbsoluteRowCorrelation = ...
        max(abs(rowCorrelation), [], 'all');
end

function quality = unavailableSVDQuality(numberOfRows, numberOfColumns)
    quality.rowNorms = nan(numberOfRows, 1);
    quality.normalizedMap = nan(numberOfRows, numberOfColumns);
    quality.singularValues = nan(min(numberOfRows, numberOfColumns), 1);
    quality.rank = NaN;
    quality.fullRowRank = false;
    quality.conditionNumber = NaN;
    quality.isotropyScore = NaN;
    quality.maximumAbsoluteRowCorrelation = NaN;
end

function metrics = collectMetrics(config, hover, authority, yaw, mixer)
    horizontalRadius_m = vecnorm(config.motorLeverArms(:, 1:2), 2, 2);
    characteristicArm_m = sqrt(sum(config.maxThrust .* ...
        horizontalRadius_m.^2) / sum(config.maxThrust));
    inverseInertia = inv(config.inertiaTensor);
    effectiveHorizontalInertia = 2 / trace(inverseInertia(1:2, 1:2));
    weightArmScale = config.weight * characteristicArm_m;
    fullThrustArmScale = config.maxTotalThrust * characteristicArm_m;

    metrics.numberOfMotors = config.numberOfMotors;
    metrics.totalMass_g = config.totalMass_g;
    metrics.maxThrustToWeight = config.maxThrustToWeight;
    metrics.hoverFeasible = hover.hoverFeasible;
    metrics.minimumHoverMargin_percent = ...
        100 * hover.minimumNormalizedMargin;
    idealHoverCommand = config.weight / config.maxTotalThrust;
    bestPossibleMargin = min(idealHoverCommand, 1 - idealHoverCommand);
    if hover.hoverFeasible && bestPossibleMargin > 0
        metrics.hoverBalanceScore = min(1, max(0, ...
            hover.minimumNormalizedMargin / bestPossibleMargin));
    else
        metrics.hoverBalanceScore = NaN;
    end
    metrics.characteristicArm_mm = 1000 * characteristicArm_m;
    metrics.effectiveHorizontalInertia_kg_m2 = ...
        effectiveHorizontalInertia;
    metrics.worstMoment_Nm = authority.minimumMoment_Nm;
    metrics.bestMoment_Nm = authority.maximumMoment_Nm;
    metrics.momentAnisotropy = authority.momentAnisotropy;
    metrics.worstAngularAcceleration_rad_s2 = ...
        authority.minimumAngularAcceleration_rad_s2;
    metrics.bestAngularAcceleration_rad_s2 = ...
        authority.maximumAngularAcceleration_rad_s2;
    metrics.angularAccelerationAnisotropy = ...
        authority.angularAccelerationAnisotropy;

    metrics.normalizedWorstMoment_weightArm = safeRatio( ...
        metrics.worstMoment_Nm, weightArmScale);
    metrics.normalizedWorstMoment_fullThrustArm = safeRatio( ...
        metrics.worstMoment_Nm, fullThrustArmScale);
    metrics.normalizedWorstAngularAcceleration = safeRatio( ...
        metrics.worstAngularAcceleration_rad_s2 * ...
        effectiveHorizontalInertia, weightArmScale);
    metrics.yawModeled = yaw.modeled;
    metrics.twoSidedYawMoment_Nm = yaw.twoSidedMoment_Nm;
    metrics.normalizedTwoSidedYawMoment = yaw.normalizedTwoSidedMoment;
    metrics.threeAxisMixerRank = mixer.threeAxis.rank;
    metrics.threeAxisMixerCondition = mixer.threeAxis.conditionNumber;
    metrics.threeAxisMixerIsotropy = mixer.threeAxis.isotropyScore;
    metrics.fourAxisMixerAvailable = mixer.fourAxisAvailable;
    metrics.fourAxisMixerRank = mixer.fourAxis.rank;
    metrics.fourAxisMixerCondition = mixer.fourAxis.conditionNumber;
    metrics.fourAxisMixerIsotropy = mixer.fourAxis.isotropyScore;
    metrics.mixerQualityScore_percent = mixer.qualityScore_percent;
    metrics.preferredMixerIsotropy = mixer.qualityScore_percent / 100;
end

function value = safeRatio(numerator, denominator)
    if isfinite(numerator) && isfinite(denominator) && denominator > 0
        value = numerator / denominator;
    else
        value = NaN;
    end
end

function result = emptyManeuverResult(requested)
    result.requested = requested;
    result.status = 'not requested';
    result.message = '';
    result.metrics = struct( ...
        'rmsAttitudeError_deg', nan(1, 3), ...
        'rmsAltitudeError_m', NaN, ...
        'allocationLimitedPercent', NaN, ...
        'maximumSpeed_m_s', NaN);
    result.simulation = struct();
end

function result = runSharedManeuver(config, settings, options)
    result = emptyManeuverResult(true);
    if ~isfield(config, 'controlRank4') || config.controlRank4 < 4
        result.status = 'skipped';
        result.message = ...
            'The modeled collective/roll/pitch/yaw map is not rank four.';
        return;
    end
    try
        simulationCall = @() simulateDrone6DOF(config, settings);
        if options.printUnderlyingAnalysis
            simulation = simulationCall();
        else
            evalc('simulation = simulationCall();');
        end
        result.status = 'completed';
        result.metrics.rmsAttitudeError_deg = ...
            simulation.metrics.rmsAttitudeError_deg;
        result.metrics.rmsAltitudeError_m = ...
            simulation.metrics.rmsAltitudeError_m;
        result.metrics.allocationLimitedPercent = ...
            simulation.metrics.allocationLimitedPercent;
        result.metrics.maximumSpeed_m_s = ...
            simulation.metrics.maximumSpeed_m_s;
        if options.storeSimulationResults
            result.simulation = simulation;
        end
    catch exception
        result.status = 'failed';
        result.message = exception.message;
        if ~options.continueOnSimulationFailure
            rethrow(exception);
        end
    end
end

function metricTable = createMetricTable(entries)
    metrics = vertcat(entries.metrics);
    metricTable = table(string({entries.label}).', ...
        vertcat(metrics.numberOfMotors), ...
        vertcat(metrics.totalMass_g), ...
        vertcat(metrics.maxThrustToWeight), ...
        vertcat(metrics.hoverFeasible), ...
        vertcat(metrics.minimumHoverMargin_percent), ...
        vertcat(metrics.hoverBalanceScore), ...
        vertcat(metrics.characteristicArm_mm), ...
        vertcat(metrics.worstMoment_Nm), ...
        vertcat(metrics.bestMoment_Nm), ...
        vertcat(metrics.momentAnisotropy), ...
        vertcat(metrics.normalizedWorstMoment_weightArm), ...
        vertcat(metrics.worstAngularAcceleration_rad_s2), ...
        vertcat(metrics.bestAngularAcceleration_rad_s2), ...
        vertcat(metrics.angularAccelerationAnisotropy), ...
        vertcat(metrics.normalizedWorstAngularAcceleration), ...
        vertcat(metrics.twoSidedYawMoment_Nm), ...
        vertcat(metrics.mixerQualityScore_percent), ...
        vertcat(metrics.threeAxisMixerIsotropy), ...
        vertcat(metrics.fourAxisMixerIsotropy), ...
        'VariableNames', {'Configuration', 'Motors', 'Mass_g', ...
        'ThrustToWeight', 'HoverFeasible', 'MinimumHoverMargin_pct', ...
        'HoverBalanceScore', 'CharacteristicArm_mm', 'WorstMoment_Nm', ...
        'BestMoment_Nm', 'MomentAnisotropy', ...
        'NormalizedWorstMoment', ...
        'WorstAngularAcceleration_rad_s2', ...
        'BestAngularAcceleration_rad_s2', 'AccelerationAnisotropy', ...
        'NormalizedWorstAngularAcceleration', 'TwoSidedYawMoment_Nm', ...
        'MixerQualityScore_pct', ...
        'ThreeAxisMixerIsotropy', 'FourAxisMixerIsotropy'});
end

function printComparisonSummary(entries, includeManeuver)
    fprintf('\nDrone configuration comparison\n');
    fprintf(['  Configuration          N   T/W   Hover margin   Worst M   ' ...
        'Worst alpha   Mixer\n']);
    fprintf(['                                    (%%)         (N*m)     ' ...
        '(rad/s^2)   isotropy\n']);
    for index = 1:numel(entries)
        item = entries(index);
        fprintf('  %-20s %2d  %5.2f     %7.2f      %7.3f      %8.1f    %6.3f\n', ...
            truncateLabel(item.label, 20), ...
            item.metrics.numberOfMotors, ...
            item.metrics.maxThrustToWeight, ...
            item.metrics.minimumHoverMargin_percent, ...
            item.metrics.worstMoment_Nm, ...
            item.metrics.worstAngularAcceleration_rad_s2, ...
            item.metrics.preferredMixerIsotropy);
    end
    fprintf(['  Normalized scores remove the main force, arm-length, and ' ...
        'inertia scales; larger is better.\n']);
    if includeManeuver
        fprintf('\n  Shared programmed maneuver\n');
        for index = 1:numel(entries)
            item = entries(index);
            fprintf('    %-20s %s', truncateLabel(item.label, 20), ...
                item.sharedManeuver.status);
            if strcmp(item.sharedManeuver.status, 'completed')
                fprintf(', allocation limited %.2f %%', ...
                    item.sharedManeuver.metrics.allocationLimitedPercent);
            elseif ~isempty(item.sharedManeuver.message)
                fprintf(': %s', item.sharedManeuver.message);
            end
            fprintf('\n');
        end
    end
end

function shortLabel = truncateLabel(label, maximumLength)
    if numel(label) <= maximumLength
        shortLabel = label;
    else
        shortLabel = [label(1:maximumLength - 3), '...'];
    end
end
