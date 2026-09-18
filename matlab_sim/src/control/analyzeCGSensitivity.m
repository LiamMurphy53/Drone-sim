function sensitivity = analyzeCGSensitivity(baseConfigOrInputs, cgX_mm, ...
    cgY_mm, options)
%ANALYZECGSENSITIVITY Map hover trim and control authority versus CG.
%
% sensitivity = analyzeCGSensitivity(baseConfigOrInputs, cgX_mm, cgY_mm)
% sensitivity = analyzeCGSensitivity(..., options)
%
% Inputs
%   baseConfigOrInputs - Either the output of analyzeDroneConfig or the
%                        droneInputs structure from drone_config_inputs.m.
%   cgX_mm             - Strictly increasing vector of final-vehicle CG x
%                        coordinates in mm.
%   cgY_mm             - Strictly increasing vector of final-vehicle CG y
%                        coordinates in mm.
%   options.fixedZ_mm  - Fixed final-vehicle CG z coordinate in mm.
%                        Defaults to the base CG z coordinate.
%   options.printSummary - Logical scalar. Defaults to true.
%
% Coordinate and unit convention
%   +x = left, +y = backward, +z = up. Locations are in mm, mass is in g,
%   thrust is in N, and moment authority is returned in N*m.
%
% Each grid point represents the FINAL assembled vehicle CG. The motor
% locations, total mass, inertia tensor about the CG, thrust limits, and yaw
% reaction-torque coefficients are held fixed during the sweep. If a
% battery or payload moves enough to materially change the inertia tensor,
% update that tensor separately before relying on angular acceleration.
%
% Hover trim enforces [Fz Mx My Mz] = [weight 0 0 0]. Axis authority holds
% Fz equal to weight, cancels the other two moments, and maximizes/minimizes
% the requested moment. Therefore twoSidedMomentAuthority_Nm is the moment
% available in BOTH directions without losing level lift or introducing a
% commanded moment about another axis.

    if nargin < 4 || isempty(options)
        options = struct();
    end
    validateattributes(baseConfigOrInputs, {'struct'}, {'scalar'}, ...
        mfilename, 'baseConfigOrInputs');
    cgX_mm = validateGridVector(cgX_mm, 'cgX_mm');
    cgY_mm = validateGridVector(cgY_mm, 'cgY_mm');
    options = validateOptions(options);

    baseConfig = normalizeBaseConfig(baseConfigOrInputs);
    validateBaseConfig(baseConfig);
    if isempty(options.fixedZ_mm)
        fixedZ_mm = baseConfig.centerOfMass_mm(3);
    else
        validateattributes(options.fixedZ_mm, {'numeric'}, ...
            {'scalar', 'real', 'finite'}, mfilename, 'options.fixedZ_mm');
        fixedZ_mm = double(options.fixedZ_mm);
    end

    [cgXGrid_mm, cgYGrid_mm] = meshgrid(cgX_mm, cgY_mm);
    gridSize = size(cgXGrid_mm);
    numberOfGridPoints = numel(cgXGrid_mm);
    if numberOfGridPoints > 50000
        error('analyzeCGSensitivity:GridTooLarge', ...
            ['The requested grid has %d points. Use at most 50000 points ' ...
             'per sweep.'], numberOfGridPoints);
    end

    numberOfMotors = baseConfig.numberOfMotors;
    numberOfAxes = 3;
    hoverFeasible = false(gridSize);
    hoverCommand = nan([gridSize, numberOfMotors]);
    hoverThrust_N = nan([gridSize, numberOfMotors]);
    minimumNormalizedThrustMargin = nan(gridSize);
    maximumHoverCommand = nan(gridSize);
    commandSpread = nan(gridSize);
    limitingMotorIndex = nan(gridSize);
    limitingBoundCode = nan(gridSize);
    hoverWrenchResidual = nan([gridSize, 4]);

    authorityFeasible = false([gridSize, numberOfAxes]);
    negativeMomentLimit_Nm = nan([gridSize, numberOfAxes]);
    positiveMomentLimit_Nm = nan([gridSize, numberOfAxes]);
    twoSidedMomentAuthority_Nm = nan([gridSize, numberOfAxes]);
    negativeAngularAccelerationLimit_rad_s2 = ...
        nan([gridSize, numberOfAxes]);
    positiveAngularAccelerationLimit_rad_s2 = ...
        nan([gridSize, numberOfAxes]);
    twoSidedAngularAccelerationAuthority_rad_s2 = ...
        nan([gridSize, numberOfAxes]);
    negativeAuthorityMotorThrust_N = ...
        nan([gridSize, numberOfAxes, numberOfMotors]);
    positiveAuthorityMotorThrust_N = ...
        nan([gridSize, numberOfAxes, numberOfMotors]);

    lowerThrust = zeros(numberOfMotors, 1);
    upperThrust = baseConfig.maxThrust(:);
    inverseInertia = baseConfig.inertiaTensor \ eye(3);
    % User-facing flight axes are roll, pitch, yaw. In this project's
    % +x-left/+y-back convention, roll is body y/My and pitch is body x/Mx.
    requestedWrenchRows = [3, 2, 4];
    requestedBodyAxisIndices = [2, 1, 3];

    for gridIndex = 1:numberOfGridPoints
        centerOfMass_mm = [cgXGrid_mm(gridIndex), ...
            cgYGrid_mm(gridIndex), fixedZ_mm];
        config = configAtCenterOfMass(baseConfig, centerOfMass_mm);
        targetWrench = [config.weight; 0; 0; 0];

        [isFeasible, command, residual] = balancedFeasibleCommand( ...
            config.commandToWrench4, targetWrench);
        hoverFeasible(gridIndex) = isFeasible;
        hoverWrenchResidual(gridIndex) = residual(1);
        hoverWrenchResidual(gridIndex + numberOfGridPoints) = residual(2);
        hoverWrenchResidual(gridIndex + 2 * numberOfGridPoints) = residual(3);
        hoverWrenchResidual(gridIndex + 3 * numberOfGridPoints) = residual(4);
        if ~isFeasible
            continue;
        end

        thrust = command .* upperThrust;
        for motorIndex = 1:numberOfMotors
            hoverCommand(gridIndex + ...
                (motorIndex - 1) * numberOfGridPoints) = command(motorIndex);
            hoverThrust_N(gridIndex + ...
                (motorIndex - 1) * numberOfGridPoints) = thrust(motorIndex);
        end
        marginByBound = [command; 1 - command];
        [minimumMargin, marginIndex] = min(marginByBound);
        minimumNormalizedThrustMargin(gridIndex) = minimumMargin;
        maximumHoverCommand(gridIndex) = max(command);
        commandSpread(gridIndex) = max(command) - min(command);
        if marginIndex <= numberOfMotors
            limitingMotorIndex(gridIndex) = marginIndex;
            limitingBoundCode(gridIndex) = -1;
        else
            limitingMotorIndex(gridIndex) = marginIndex - numberOfMotors;
            limitingBoundCode(gridIndex) = 1;
        end

        wrenchMap = config.thrustToWrench4;
        for axisIndex = 1:numberOfAxes
            requestedWrenchRow = requestedWrenchRows(axisIndex);
            bodyAxisIndex = requestedBodyAxisIndices(axisIndex);
            otherMomentRows = setdiff(2:4, requestedWrenchRow);
            equalityRows = [1, otherMomentRows];
            equalityMatrix = wrenchMap(equalityRows, :);
            equalityTarget = [config.weight; 0; 0];
            objective = wrenchMap(requestedWrenchRow, :).';

            [minimumMoment, maximumMoment, minimumThrust, maximumThrust] = ...
                boundedLinearExtrema(objective, equalityMatrix, ...
                equalityTarget, lowerThrust, upperThrust);
            isAuthorityFeasible = isfinite(minimumMoment) && ...
                isfinite(maximumMoment);
            authorityFeasible(gridIndex + ...
                (axisIndex - 1) * numberOfGridPoints) = isAuthorityFeasible;
            if ~isAuthorityFeasible
                continue;
            end

            negativeMagnitude = max(0, -minimumMoment);
            positiveMagnitude = max(0, maximumMoment);
            twoSidedMagnitude = min(negativeMagnitude, positiveMagnitude);
            negativeMomentLimit_Nm(gridIndex + ...
                (axisIndex - 1) * numberOfGridPoints) = negativeMagnitude;
            positiveMomentLimit_Nm(gridIndex + ...
                (axisIndex - 1) * numberOfGridPoints) = positiveMagnitude;
            twoSidedMomentAuthority_Nm(gridIndex + ...
                (axisIndex - 1) * numberOfGridPoints) = twoSidedMagnitude;

            axisAccelerationGain = ...
                inverseInertia(bodyAxisIndex, bodyAxisIndex);
            negativeAngularAccelerationLimit_rad_s2(gridIndex + ...
                (axisIndex - 1) * numberOfGridPoints) = ...
                negativeMagnitude * axisAccelerationGain;
            positiveAngularAccelerationLimit_rad_s2(gridIndex + ...
                (axisIndex - 1) * numberOfGridPoints) = ...
                positiveMagnitude * axisAccelerationGain;
            twoSidedAngularAccelerationAuthority_rad_s2(gridIndex + ...
                (axisIndex - 1) * numberOfGridPoints) = ...
                twoSidedMagnitude * axisAccelerationGain;

            for motorIndex = 1:numberOfMotors
                negativeAuthorityMotorThrust_N(gridIndex + ...
                    (axisIndex - 1) * numberOfGridPoints + ...
                    (motorIndex - 1) * numberOfGridPoints * numberOfAxes) = ...
                    minimumThrust(motorIndex);
                positiveAuthorityMotorThrust_N(gridIndex + ...
                    (axisIndex - 1) * numberOfGridPoints + ...
                    (motorIndex - 1) * numberOfGridPoints * numberOfAxes) = ...
                    maximumThrust(motorIndex);
            end
        end
    end

    sensitivity.cgX_mm = cgX_mm;
    sensitivity.cgY_mm = cgY_mm;
    sensitivity.cgXGrid_mm = cgXGrid_mm;
    sensitivity.cgYGrid_mm = cgYGrid_mm;
    sensitivity.fixedCGZ_mm = fixedZ_mm;
    sensitivity.centerOfMassConvention = ...
        'Final assembled vehicle CG; +x left, +y backward, +z up';
    sensitivity.baseCenterOfMass_mm = baseConfig.centerOfMass_mm;
    sensitivity.motorNames = baseConfig.motorNames;
    sensitivity.motorLocations_mm = baseConfig.motorLocations_mm;
    sensitivity.maxThrust_N = upperThrust;
    sensitivity.totalMass_g = baseConfig.totalMass_g;
    sensitivity.inertiaTensor_g_mm2 = baseConfig.inertiaTensor_g_mm2;
    sensitivity.axisNames = {'Roll', 'Pitch', 'Yaw'};
    sensitivity.bodyAxisNames = {'y', 'x', 'z'};
    sensitivity.bodyAxisIndices = requestedBodyAxisIndices;
    sensitivity.momentNames = {'My', 'Mx', 'Mz'};
    sensitivity.hoverFeasible = hoverFeasible;
    sensitivity.hoverCommand = hoverCommand;
    sensitivity.hoverCommandPercent = 100 * hoverCommand;
    sensitivity.hoverThrust_N = hoverThrust_N;
    sensitivity.minimumNormalizedThrustMargin = ...
        minimumNormalizedThrustMargin;
    sensitivity.minimumThrustMarginPercent = ...
        100 * minimumNormalizedThrustMargin;
    sensitivity.maximumHoverCommand = maximumHoverCommand;
    sensitivity.maximumHoverCommandPercent = 100 * maximumHoverCommand;
    sensitivity.hoverCommandSpread = commandSpread;
    sensitivity.hoverCommandSpreadPercent = 100 * commandSpread;
    sensitivity.limitingMotorIndex = limitingMotorIndex;
    sensitivity.limitingBoundCode = limitingBoundCode;
    sensitivity.limitingBoundCodeMeaning = ...
        '-1 = lower/zero-thrust side, +1 = upper/max-thrust side';
    sensitivity.hoverWrenchResidual = hoverWrenchResidual;
    sensitivity.hoverWrenchResidualNames = {'Fz', 'Mx', 'My', 'Mz'};
    sensitivity.hoverWrenchResidualUnits = {'N', 'N m', 'N m', 'N m'};
    sensitivity.authorityFeasible = authorityFeasible;
    sensitivity.negativeMomentLimit_Nm = negativeMomentLimit_Nm;
    sensitivity.positiveMomentLimit_Nm = positiveMomentLimit_Nm;
    sensitivity.twoSidedMomentAuthority_Nm = ...
        twoSidedMomentAuthority_Nm;
    sensitivity.rollTwoSidedMomentAuthority_Nm = ...
        twoSidedMomentAuthority_Nm(:, :, 1);
    sensitivity.pitchTwoSidedMomentAuthority_Nm = ...
        twoSidedMomentAuthority_Nm(:, :, 2);
    sensitivity.yawTwoSidedMomentAuthority_Nm = ...
        twoSidedMomentAuthority_Nm(:, :, 3);
    sensitivity.negativeAngularAccelerationLimit_rad_s2 = ...
        negativeAngularAccelerationLimit_rad_s2;
    sensitivity.positiveAngularAccelerationLimit_rad_s2 = ...
        positiveAngularAccelerationLimit_rad_s2;
    sensitivity.twoSidedAngularAccelerationAuthority_rad_s2 = ...
        twoSidedAngularAccelerationAuthority_rad_s2;
    sensitivity.negativeAuthorityMotorThrust_N = ...
        negativeAuthorityMotorThrust_N;
    sensitivity.positiveAuthorityMotorThrust_N = ...
        positiveAuthorityMotorThrust_N;
    sensitivity.authorityDefinition = ...
        ['Fz is held at weight and the other two moments are zero while ' ...
         'the requested roll, pitch, or yaw moment is extremized.'];
    sensitivity.assumptions = struct( ...
        'finalAssembledCGIsSwept', true, ...
        'motorLocationsHeldFixed', true, ...
        'totalMassHeldFixed', true, ...
        'inertiaTensorHeldFixed', true, ...
        'maximumThrustHeldFixed', true, ...
        'fixedCGZ_mm', fixedZ_mm, ...
        'aerodynamicInterferenceIgnored', true);

    feasibleCount = nnz(hoverFeasible);
    sensitivity.numberOfGridPoints = numberOfGridPoints;
    sensitivity.numberOfFeasiblePoints = feasibleCount;
    sensitivity.feasibleFraction = feasibleCount / numberOfGridPoints;
    if feasibleCount > 0
        [bestMargin, bestLinearIndex] = ...
            max(minimumNormalizedThrustMargin(:), [], 'omitnan');
        sensitivity.maximumMinimumNormalizedMargin = bestMargin;
        sensitivity.bestMarginCG_mm = [cgXGrid_mm(bestLinearIndex), ...
            cgYGrid_mm(bestLinearIndex), fixedZ_mm];
        commandMatrix = reshape(hoverCommand, numberOfGridPoints, ...
            numberOfMotors);
        sensitivity.bestMarginMotorCommand = ...
            commandMatrix(bestLinearIndex, :).';
    else
        sensitivity.maximumMinimumNormalizedMargin = NaN;
        sensitivity.bestMarginCG_mm = [NaN, NaN, fixedZ_mm];
        sensitivity.bestMarginMotorCommand = nan(numberOfMotors, 1);
    end

    if options.printSummary
        fprintf('\nCG sensitivity sweep\n');
        fprintf('  Grid:                      %d x %d (%d points)\n', ...
            numel(cgY_mm), numel(cgX_mm), numberOfGridPoints);
        fprintf('  Final CG x range:          %.3f to %.3f mm\n', ...
            cgX_mm(1), cgX_mm(end));
        fprintf('  Final CG y range:          %.3f to %.3f mm\n', ...
            cgY_mm(1), cgY_mm(end));
        fprintf('  Fixed final CG z:          %.3f mm\n', fixedZ_mm);
        fprintf('  Hover-feasible points:     %d (%.1f %%)\n', ...
            feasibleCount, 100 * sensitivity.feasibleFraction);
        if feasibleCount > 0
            fprintf('  Best two-sided margin:     %.2f %%\n', ...
                100 * sensitivity.maximumMinimumNormalizedMargin);
            fprintf('  Best sampled final CG:     [%.3f %.3f %.3f] mm\n', ...
                sensitivity.bestMarginCG_mm);
        end
    end
end

function gridVector = validateGridVector(gridVector, argumentName)
    validateattributes(gridVector, {'numeric'}, ...
        {'vector', 'real', 'finite', 'nonempty'}, mfilename, argumentName);
    gridVector = double(gridVector(:).');
    if any(diff(gridVector) <= 0)
        error('analyzeCGSensitivity:GridNotIncreasing', ...
            '%s must be strictly increasing with no repeated values.', ...
            argumentName);
    end
end

function options = validateOptions(options)
    validateattributes(options, {'struct'}, {'scalar'}, mfilename, 'options');
    validFields = {'fixedZ_mm', 'printSummary'};
    unknownFields = setdiff(fieldnames(options), validFields);
    if ~isempty(unknownFields)
        error('analyzeCGSensitivity:UnknownOption', ...
            'Unknown option field: %s.', unknownFields{1});
    end
    if ~isfield(options, 'fixedZ_mm')
        options.fixedZ_mm = [];
    end
    if ~isfield(options, 'printSummary')
        options.printSummary = true;
    end
    validateattributes(options.printSummary, {'logical', 'numeric'}, ...
        {'scalar', 'real', 'finite'}, mfilename, 'options.printSummary');
    if isnumeric(options.printSummary) && ...
            options.printSummary ~= 0 && options.printSummary ~= 1
        error('analyzeCGSensitivity:InvalidPrintSummary', ...
            'options.printSummary must be true or false.');
    end
    options.printSummary = logical(options.printSummary);
end

function baseConfig = normalizeBaseConfig(baseConfigOrInputs)
    if isfield(baseConfigOrInputs, 'thrustToWrench4') && ...
            isfield(baseConfigOrInputs, 'maxThrust')
        baseConfig = baseConfigOrInputs;
        return;
    end

    requiredInputFields = {'motorLocations_mm', 'maxThrust_N', ...
        'totalMass_g', 'centerOfMass_mm', 'inertiaTensor_g_mm2'};
    missingFields = setdiff(requiredInputFields, fieldnames(baseConfigOrInputs));
    if ~isempty(missingFields)
        error('analyzeCGSensitivity:InvalidBaseInput', ...
            ['The structure is neither an analyzed configuration nor a ' ...
             'droneInputs structure. Missing field: %s.'], missingFields{1});
    end
    if isfield(baseConfigOrInputs, 'motorNames')
        motorNames = baseConfigOrInputs.motorNames;
    else
        motorNames = [];
    end
    hasSpin = isfield(baseConfigOrInputs, 'motorSpinDirection');
    hasYawCoefficient = isfield(baseConfigOrInputs, 'yawTorquePerThrust_m');
    if xor(hasSpin, hasYawCoefficient)
        error('analyzeCGSensitivity:IncompletePropulsionInput', ...
            ['droneInputs must provide both motorSpinDirection and ' ...
             'yawTorquePerThrust_m, or neither.']);
    end
    if hasSpin
        propulsionInputs = struct( ...
            'motorSpinDirection', {baseConfigOrInputs.motorSpinDirection}, ...
            'yawTorquePerThrust_m', ...
            baseConfigOrInputs.yawTorquePerThrust_m);
    else
        propulsionInputs = [];
    end

    % Reuse the project's central validator and unit conversion. This call
    % prints one base-configuration summary; the grid loop itself is quiet.
    baseConfig = analyzeDroneConfig( ...
        baseConfigOrInputs.motorLocations_mm, ...
        baseConfigOrInputs.maxThrust_N, baseConfigOrInputs.totalMass_g, ...
        baseConfigOrInputs.centerOfMass_mm, ...
        baseConfigOrInputs.inertiaTensor_g_mm2, motorNames, ...
        propulsionInputs);
end

function validateBaseConfig(config)
    requiredFields = {'numberOfMotors', 'motorNames', ...
        'motorLocations_mm', 'maxThrust', 'totalMass_g', ...
        'centerOfMass_mm', 'inertiaTensor_g_mm2', 'inertiaTensor', ...
        'weight', 'thrustToWrench4'};
    missingFields = setdiff(requiredFields, fieldnames(config));
    if ~isempty(missingFields)
        error('analyzeCGSensitivity:IncompleteConfig', ...
            'Analyzed configuration is missing field: %s.', missingFields{1});
    end
    validateattributes(config.numberOfMotors, {'numeric'}, ...
        {'scalar', 'integer', 'positive'}, mfilename, ...
        'baseConfig.numberOfMotors');
    numberOfMotors = config.numberOfMotors;
    validateattributes(config.motorLocations_mm, {'numeric'}, ...
        {'size', [numberOfMotors, 3], 'real', 'finite'}, mfilename, ...
        'baseConfig.motorLocations_mm');
    validateattributes(config.maxThrust, {'numeric'}, ...
        {'vector', 'numel', numberOfMotors, 'real', 'finite', 'positive'}, ...
        mfilename, 'baseConfig.maxThrust');
    validateattributes(config.totalMass_g, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'}, mfilename, ...
        'baseConfig.totalMass_g');
    validateattributes(config.centerOfMass_mm, {'numeric'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, mfilename, ...
        'baseConfig.centerOfMass_mm');
    validateattributes(config.inertiaTensor, {'numeric'}, ...
        {'size', [3, 3], 'real', 'finite'}, mfilename, ...
        'baseConfig.inertiaTensor');
    validateattributes(config.inertiaTensor_g_mm2, {'numeric'}, ...
        {'size', [3, 3], 'real', 'finite'}, mfilename, ...
        'baseConfig.inertiaTensor_g_mm2');
    symmetryTolerance = 1e-10 * max(1, norm(config.inertiaTensor, 'fro'));
    if norm(config.inertiaTensor - config.inertiaTensor.', 'fro') > ...
            symmetryTolerance
        error('analyzeCGSensitivity:NonSymmetricInertia', ...
            'baseConfig.inertiaTensor must be symmetric.');
    end
    [~, positiveDefiniteFlag] = chol(config.inertiaTensor);
    if positiveDefiniteFlag ~= 0
        error('analyzeCGSensitivity:InvalidInertia', ...
            'baseConfig.inertiaTensor must be positive definite.');
    end
    validateattributes(config.weight, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'}, mfilename, ...
        'baseConfig.weight');
    validateattributes(config.thrustToWrench4, {'numeric'}, ...
        {'size', [4, numberOfMotors], 'real', 'finite'}, mfilename, ...
        'baseConfig.thrustToWrench4');
    if ~iscell(config.motorNames) || numel(config.motorNames) ~= numberOfMotors
        error('analyzeCGSensitivity:InvalidMotorNames', ...
            'baseConfig.motorNames must contain one label per motor.');
    end
end

function config = configAtCenterOfMass(baseConfig, centerOfMass_mm)
    config = baseConfig;
    config.centerOfMass_mm = centerOfMass_mm;
    config.centerOfMass = centerOfMass_mm * 1e-3;
    config.motorLocations = baseConfig.motorLocations_mm * 1e-3;
    config.motorLeverArms = config.motorLocations - config.centerOfMass;
    x = config.motorLeverArms(:, 1);
    y = config.motorLeverArms(:, 2);
    yawMomentPerThrust = baseConfig.thrustToWrench4(4, :);
    config.thrustToWrench = [ones(1, config.numberOfMotors); y.'; -x.'];
    config.thrustToWrench4 = [config.thrustToWrench; yawMomentPerThrust];
    config.commandToWrench = config.thrustToWrench * diag(config.maxThrust);
    config.commandToWrench4 = ...
        config.thrustToWrench4 * diag(config.maxThrust);
    centerOfThrustAtFull = sum(config.motorLocations .* ...
        config.maxThrust(:), 1) / sum(config.maxThrust);
    config.centerOfThrustAtFull = centerOfThrustAtFull;
    config.centerOfThrustAtFull_mm = 1000 * centerOfThrustAtFull;
    config.centerOfThrustOffset = centerOfThrustAtFull - config.centerOfMass;
    config.centerOfThrustOffset_mm = 1000 * config.centerOfThrustOffset;
end

function [feasible, command, residual] = balancedFeasibleCommand( ...
        commandMap, targetWrench)
    numberOfMotors = size(commandMap, 2);
    lowerCommand = zeros(numberOfMotors, 1);
    upperCommand = ones(numberOfMotors, 1);
    equalCommand = targetWrench(1) / sum(commandMap(1, :));
    referenceCommand = equalCommand * ones(numberOfMotors, 1);
    command = referenceCommand + pinv(commandMap) * ...
        (targetWrench - commandMap * referenceCommand);
    scale = max([1; abs(targetWrench); abs(commandMap(:))]);
    equalityTolerance = 1e-8 * scale;
    boundTolerance = 1e-9;
    feasible = norm(commandMap * command - targetWrench, inf) <= ...
        equalityTolerance && all(command >= -boundTolerance) && ...
        all(command <= 1 + boundTolerance);

    if ~feasible
        % Average extrema for every motor. A convex average of feasible
        % equality-constrained points is feasible and avoids selecting an
        % arbitrary box vertex as the nominal hover trim.
        feasiblePoints = zeros(numberOfMotors, 0);
        for motorIndex = 1:numberOfMotors
            objective = zeros(numberOfMotors, 1);
            objective(motorIndex) = 1;
            [minimumValue, maximumValue, minimumPoint, maximumPoint] = ...
                boundedLinearExtrema(objective, commandMap, targetWrench, ...
                lowerCommand, upperCommand);
            if isfinite(minimumValue) && isfinite(maximumValue)
                feasiblePoints(:, end + 1:end + 2) = ...
                    [minimumPoint, maximumPoint];
            end
        end
        feasible = ~isempty(feasiblePoints);
        if feasible
            command = mean(feasiblePoints, 2);
        else
            command = nan(numberOfMotors, 1);
        end
    end

    if feasible
        command = min(max(command, 0), 1);
        residual = commandMap * command - targetWrench;
        feasible = norm(residual, inf) <= equalityTolerance;
    else
        residual = nan(size(targetWrench));
    end
end
