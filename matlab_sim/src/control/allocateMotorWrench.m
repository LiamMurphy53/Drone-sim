function allocation = allocateMotorWrench(config, desiredWrench, priority)
%ALLOCATEMOTORWRENCH Bounded least-squares [Fz;Mx;My;Mz] allocation.
%
% The toolbox-free search enumerates which motors are free, at zero, or at
% maximum thrust. It is intended for typical multirotors, not large arrays.

    validateattributes(desiredWrench, {'numeric'}, ...
        {'vector', 'numel', 4, 'real', 'finite'}, mfilename, ...
        'desiredWrench');
    if nargin < 3 || isempty(priority)
        priority = ones(4, 1);
    end
    validateattributes(priority, {'numeric'}, ...
        {'vector', 'numel', 4, 'real', 'finite', 'positive'}, ...
        mfilename, 'priority');
    desiredWrench = desiredWrench(:);
    priority = priority(:);

    wrenchMap = config.thrustToWrench4;
    numberOfMotors = config.numberOfMotors;
    lowerBound = zeros(numberOfMotors, 1);
    upperBound = config.maxThrust;
    wrenchScale = sum(abs(wrenchMap) .* upperBound.', 2);
    wrenchScale = max(wrenchScale, [config.weight; 1e-3; 1e-3; 1e-3]);
    weighting = diag(priority ./ wrenchScale);
    weightedMap = weighting * wrenchMap;
    weightedTarget = weighting * desiredWrench;

    numberOfPatterns = 3^numberOfMotors;
    if numberOfPatterns > 2e6
        error('allocateMotorWrench:TooManyMotors', ...
            'The bounded allocation search is too large for this motor count.');
    end

    boundTolerance = 1e-10 * max(1, max(upperBound));
    unconstrainedThrust = pinv(weightedMap) * weightedTarget;
    unconstrainedIsFeasible = ...
        all(unconstrainedThrust >= lowerBound - boundTolerance) && ...
        all(unconstrainedThrust <= upperBound + boundTolerance);
    if unconstrainedIsFeasible
        bestThrust = min(max(unconstrainedThrust, lowerBound), upperBound);
        patternIndices = zeros(1, 0);
    else
        bestThrust = nan(numberOfMotors, 1);
        patternIndices = 0:(numberOfPatterns - 1);
    end
    bestScore = inf;
    for patternIndex = patternIndices
        state = zeros(numberOfMotors, 1);
        remainingPattern = patternIndex;
        for motorIndex = 1:numberOfMotors
            state(motorIndex) = mod(remainingPattern, 3);
            remainingPattern = floor(remainingPattern / 3);
        end

        freeMotors = find(state == 0);
        lowerMotors = find(state == 1);
        upperMotors = find(state == 2);
        candidate = zeros(numberOfMotors, 1);
        candidate(lowerMotors) = lowerBound(lowerMotors);
        candidate(upperMotors) = upperBound(upperMotors);

        fixedMotors = [lowerMotors; upperMotors];
        remainingTarget = weightedTarget;
        if ~isempty(fixedMotors)
            remainingTarget = remainingTarget - ...
                weightedMap(:, fixedMotors) * candidate(fixedMotors);
        end
        if ~isempty(freeMotors)
            candidate(freeMotors) = ...
                pinv(weightedMap(:, freeMotors)) * remainingTarget;
        end

        if any(candidate < lowerBound - boundTolerance) || ...
                any(candidate > upperBound + boundTolerance)
            continue;
        end
        candidate = min(max(candidate, lowerBound), upperBound);
        weightedResidual = weightedMap * candidate - weightedTarget;
        score = sum(weightedResidual.^2) + ...
            1e-12 * sum((candidate ./ max(upperBound, 1e-9)).^2);
        if score < bestScore
            bestScore = score;
            bestThrust = candidate;
        end
    end

    if any(~isfinite(bestThrust))
        error('allocateMotorWrench:NoAllocation', ...
            'No bounded motor allocation could be calculated.');
    end

    achievedWrench = wrenchMap * bestThrust;
    weightedResidual = weighting * (achievedWrench - desiredWrench);
    allocation.motorThrust_N = bestThrust;
    allocation.achievedWrench = achievedWrench;
    allocation.wrenchError = achievedWrench - desiredWrench;
    allocation.normalizedWeightedError = norm(weightedResidual);
    allocation.limited = allocation.normalizedWeightedError > 1e-6;
    allocation.motorAtLowerLimit = bestThrust <= lowerBound + boundTolerance;
    allocation.motorAtUpperLimit = bestThrust >= upperBound - boundTolerance;
end
