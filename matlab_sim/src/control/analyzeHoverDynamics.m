function hover = analyzeHoverDynamics(config)
%ANALYZEHOVERDYNAMICS Calculate level-hover trim and control authority.
%
% The trim supports the vehicle weight while producing zero x- and y-axis
% moment. Authority limits preserve hover lift and cancel moment about the
% other horizontal axis. Motor thrust is constrained between zero and its
% configured maximum.

    numberOfMotors = config.numberOfMotors;
    mixer = config.thrustToWrench;
    targetWrench = [config.weight; 0; 0];
    lowerThrust = zeros(numberOfMotors, 1);
    upperThrust = config.maxThrust;
    tolerance = 1e-9 * max(1, norm(targetWrench));

    % Prefer a trim closest to equal normalized motor commands.
    equalCommand = config.weight / config.maxTotalThrust;
    referenceCommand = equalCommand * ones(numberOfMotors, 1);
    commandMixer = mixer * diag(config.maxThrust);
    hoverCommand = referenceCommand + pinv(commandMixer) * ...
        (targetWrench - commandMixer * referenceCommand);

    hoverFeasible = norm(commandMixer * hoverCommand - targetWrench) <= ...
        tolerance && all(hoverCommand >= -tolerance) && ...
        all(hoverCommand <= 1 + tolerance);

    if ~hoverFeasible
        % If the closest-to-equal solution violates a bound, average the
        % feasible vertices to obtain a valid interior trim when possible.
        vertices = boundedEqualityVertices(commandMixer, targetWrench, ...
            zeros(numberOfMotors, 1), ones(numberOfMotors, 1));
        hoverFeasible = ~isempty(vertices);
        if hoverFeasible
            hoverCommand = mean(vertices, 2);
        else
            hoverCommand = nan(numberOfMotors, 1);
        end
    end

    hover.hoverFeasible = hoverFeasible;
    hover.targetWrench = targetWrench;

    if ~hoverFeasible
        hover.motorThrust = nan(numberOfMotors, 1);
        hover.motorCommand = hoverCommand;
        hover.motorCommandPercent = 100 * hoverCommand;
        hover.lowerThrustMargin = nan(numberOfMotors, 1);
        hover.upperThrustMargin = nan(numberOfMotors, 1);
        hover.minimumNormalizedMargin = NaN;
        hover.xAxis = unavailableAuthority(numberOfMotors);
        hover.yAxis = unavailableAuthority(numberOfMotors);
        warning('analyzeHoverDynamics:HoverInfeasible', ...
            ['Level hover is not feasible within the motor thrust limits. ' ...
             'Check thrust-to-weight ratio, center of mass, and motor layout.']);
        return;
    end

    hoverCommand = min(max(hoverCommand, 0), 1);
    hoverThrust = hoverCommand .* config.maxThrust;
    hover.motorThrust = hoverThrust;
    hover.motorCommand = hoverCommand;
    hover.motorCommandPercent = 100 * hoverCommand;
    hover.lowerThrustMargin = hoverThrust - lowerThrust;
    hover.upperThrustMargin = upperThrust - hoverThrust;
    hover.minimumNormalizedMargin = min([hoverCommand; 1 - hoverCommand]);
    hover.trimWrench = mixer * hoverThrust;

    hover.xAxis = axisAuthority(config, 2, 3);
    hover.yAxis = axisAuthority(config, 3, 2);

    fprintf('\nLevel-hover trim\n');
    fprintf('  Motor       Thrust (N)       Command       Up margin (N)\n');
    for motorIndex = 1:numberOfMotors
        fprintf('  %-5s       %8.3f        %6.2f %%          %8.3f\n', ...
            config.motorNames{motorIndex}, hoverThrust(motorIndex), ...
            hover.motorCommandPercent(motorIndex), ...
            hover.upperThrustMargin(motorIndex));
    end
    fprintf('  Smallest two-sided normalized margin: %.2f %%\n', ...
        100 * hover.minimumNormalizedMargin);

    fprintf('\nConstant-altitude control authority\n');
    printAxisAuthority('X', hover.xAxis, 1);
    printAxisAuthority('Y', hover.yAxis, 2);
end

function authority = axisAuthority(config, objectiveRow, cancelledMomentRow)
    mixer = config.thrustToWrench;
    equalityMatrix = mixer([1, cancelledMomentRow], :);
    equalityTarget = [config.weight; 0];
    objective = mixer(objectiveRow, :).';

    [minimumMoment, maximumMoment, minimumThrust, maximumThrust] = ...
        boundedLinearExtrema(objective, equalityMatrix, equalityTarget, ...
        zeros(config.numberOfMotors, 1), config.maxThrust);

    authority.feasible = isfinite(minimumMoment) && isfinite(maximumMoment);
    authority.momentRange = [minimumMoment, maximumMoment];
    authority.minimumMomentThrust = minimumThrust;
    authority.maximumMomentThrust = maximumThrust;

    if ~authority.feasible
        authority.angularAccelerationAtMinimum = nan(3, 1);
        authority.angularAccelerationAtMaximum = nan(3, 1);
        authority.axisAngularAccelerationRange = [NaN, NaN];
        return;
    end

    minimumWrench = mixer * minimumThrust;
    maximumWrench = mixer * maximumThrust;
    minimumAngularAcceleration = config.inertiaTensor \ ...
        [minimumWrench(2); minimumWrench(3); 0];
    maximumAngularAcceleration = config.inertiaTensor \ ...
        [maximumWrench(2); maximumWrench(3); 0];

    axisIndex = objectiveRow - 1;
    authority.angularAccelerationAtMinimum = minimumAngularAcceleration;
    authority.angularAccelerationAtMaximum = maximumAngularAcceleration;
    authority.axisAngularAccelerationRange = ...
        [minimumAngularAcceleration(axisIndex), ...
         maximumAngularAcceleration(axisIndex)];
end

function authority = unavailableAuthority(numberOfMotors)
    authority.feasible = false;
    authority.momentRange = [NaN, NaN];
    authority.minimumMomentThrust = nan(numberOfMotors, 1);
    authority.maximumMomentThrust = nan(numberOfMotors, 1);
    authority.angularAccelerationAtMinimum = nan(3, 1);
    authority.angularAccelerationAtMaximum = nan(3, 1);
    authority.axisAngularAccelerationRange = [NaN, NaN];
end

function printAxisAuthority(axisName, authority, axisIndex)
    if ~authority.feasible
        fprintf('  %s axis: no feasible constant-altitude solution\n', axisName);
        return;
    end

    fprintf('  %s moment: %.3f to %.3f N*m\n', axisName, ...
        authority.momentRange(1), authority.momentRange(2));
    fprintf('  %s angular acceleration: %.3f to %.3f rad/s^2\n', ...
        axisName, authority.axisAngularAccelerationRange(1), ...
        authority.axisAngularAccelerationRange(2));

    otherAxes = setdiff(1:3, axisIndex);
    maximumCoupling = authority.angularAccelerationAtMaximum(otherAxes);
    fprintf('    Coupled acceleration at positive limit: ');
    fprintf('[%.3f %.3f %.3f] rad/s^2\n', ...
        authority.angularAccelerationAtMaximum);
    if any(abs(maximumCoupling) > 1e-9)
        fprintf('    Off-axis coupling is present because of the inertia tensor.\n');
    end
end

function [minimumValue, maximumValue, minimumPoint, maximumPoint] = ...
        boundedLinearExtrema(objective, equalityMatrix, equalityTarget, ...
        lowerBound, upperBound)
    vertices = boundedEqualityVertices(equalityMatrix, equalityTarget, ...
        lowerBound, upperBound);

    if isempty(vertices)
        minimumValue = NaN;
        maximumValue = NaN;
        minimumPoint = nan(numel(lowerBound), 1);
        maximumPoint = nan(numel(lowerBound), 1);
        return;
    end

    values = objective.' * vertices;
    [minimumValue, minimumIndex] = min(values);
    [maximumValue, maximumIndex] = max(values);
    minimumPoint = vertices(:, minimumIndex);
    maximumPoint = vertices(:, maximumIndex);
end

function vertices = boundedEqualityVertices(equalityMatrix, equalityTarget, ...
        lowerBound, upperBound)
%BOUNDEDEQUALITYVERTICES Enumerate vertices of E*x=d with box constraints.
% This avoids requiring MATLAB's Optimization Toolbox for typical multirotors.

    numberOfVariables = numel(lowerBound);
    equalityTarget = equalityTarget(:);
    lowerBound = lowerBound(:);
    upperBound = upperBound(:);
    scale = max([1; abs(equalityTarget); abs(equalityMatrix(:))]);
    equalityTolerance = 1e-8 * scale;
    boundTolerance = 1e-9 * max(1, max(abs(upperBound)));
    matrixRank = rank(equalityMatrix);

    if matrixRank == 0
        if norm(equalityTarget) <= equalityTolerance
            vertices = [lowerBound, upperBound];
        else
            vertices = zeros(numberOfVariables, 0);
        end
        return;
    end

    [~, independentRows] = rref(equalityMatrix.');
    independentRows = independentRows(1:matrixRank);
    reducedMatrix = equalityMatrix(independentRows, :);
    reducedTarget = equalityTarget(independentRows);

    freeVariableSets = nchoosek(1:numberOfVariables, matrixRank);
    numberOfFixedVariables = numberOfVariables - matrixRank;
    patternsPerSet = 2^numberOfFixedVariables;
    estimatedCandidates = size(freeVariableSets, 1) * patternsPerSet;
    if estimatedCandidates > 2e6
        error('analyzeHoverDynamics:TooManyMotorCombinations', ...
            ['The toolbox-free authority search would require too many ' ...
             'combinations for this motor count.']);
    end

    vertices = zeros(numberOfVariables, 0);
    allVariables = 1:numberOfVariables;
    for setIndex = 1:size(freeVariableSets, 1)
        freeVariables = freeVariableSets(setIndex, :);
        fixedVariables = setdiff(allVariables, freeVariables);
        freeMatrix = reducedMatrix(:, freeVariables);
        if rank(freeMatrix) < matrixRank
            continue;
        end

        for pattern = 0:(patternsPerSet - 1)
            candidate = zeros(numberOfVariables, 1);
            if numberOfFixedVariables > 0
                atUpperBound = bitget(pattern, 1:numberOfFixedVariables).';
                candidate(fixedVariables) = lowerBound(fixedVariables) + ...
                    atUpperBound .* ...
                    (upperBound(fixedVariables) - lowerBound(fixedVariables));
            end
            candidate(freeVariables) = freeMatrix \ ...
                (reducedTarget - reducedMatrix(:, fixedVariables) * ...
                candidate(fixedVariables));

            satisfiesBounds = all(candidate >= lowerBound - boundTolerance) && ...
                all(candidate <= upperBound + boundTolerance);
            satisfiesEqualities = norm(equalityMatrix * candidate - ...
                equalityTarget, inf) <= equalityTolerance;
            if satisfiesBounds && satisfiesEqualities
                candidate = min(max(candidate, lowerBound), upperBound);
                vertices(:, end + 1) = candidate; %#ok<AGROW>
            end
        end
    end

    if ~isempty(vertices)
        vertices = uniquetol(vertices.', 1e-9, 'ByRows', true).';
    end
end
