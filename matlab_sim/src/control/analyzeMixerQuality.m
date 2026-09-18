function quality = analyzeMixerQuality(config)
%ANALYZEMIXERQUALITY Dimensionless geometry and allocation quality metrics.
%
% quality = analyzeMixerQuality(config)
%
% Motor inputs are normalized commands (0 to 1), so unequal motor thrust
% limits remain represented through diag(config.maxThrust). Wrench outputs
% are normalized by
%
%   F_scale = sum(maxThrust)
%   M_scale = F_scale * r_ref,
%
% where r_ref is the thrust-weighted RMS horizontal motor radius about the
% center of mass. The resulting normalized wrench [Fz Mx My Mz] is
% dimensionless and preserves weak yaw leverage instead of independently
% rescaling every row to appear equally strong.
%
% The full-mixer SVD measures collective-plus-attitude allocation. A second
% SVD projects motor command changes into the nullspace of Fz first, then
% measures Mx-My-Mz quality while collective thrust is held constant.

    validateConfig(config);
    maximumThrust = config.maxThrust(:);
    maximumTotalThrust = sum(maximumThrust);
    horizontalRadiusSquared = sum(config.motorLeverArms(:, 1:2).^2, 2);
    referenceArm_m = sqrt(sum(maximumThrust .* horizontalRadiusSquared) / ...
        maximumTotalThrust);
    if referenceArm_m <= 1e-12
        error('analyzeMixerQuality:ZeroReferenceArm', ...
            ['A mixer quality score requires at least one motor with a ' ...
             'nonzero horizontal lever arm about the center of mass.']);
    end

    forceScale_N = maximumTotalThrust;
    momentScale_Nm = maximumTotalThrust * referenceArm_m;
    outputScales = [forceScale_N; repmat(momentScale_Nm, 3, 1)];
    commandToWrench = config.thrustToWrench4 * diag(maximumThrust);
    normalizedMap = diag(1 ./ outputScales) * commandToWrench;

    [leftVectors, singularMatrix, rightVectors] = svd(normalizedMap);
    singularValues = paddedSingularValues(singularMatrix, 4);
    numericalRank = numericalMatrixRank(singularValues, normalizedMap);
    [conditionNumber, isotropyIndex] = conditioningMetrics( ...
        singularValues, numericalRank, 4);
    weakestDirection = canonicalDirection(leftVectors(:, 4));

    collectiveRow = normalizedMap(1, :);
    constantCollectiveBasis = null(collectiveRow);
    constantCollectiveMomentMap = normalizedMap(2:4, :) * ...
        constantCollectiveBasis;
    [momentLeftVectors, momentSingularMatrix, momentRightVectors] = ...
        svd(constantCollectiveMomentMap);
    momentSingularValues = paddedSingularValues(momentSingularMatrix, 3);
    momentRank = numericalMatrixRank(momentSingularValues, ...
        constantCollectiveMomentMap);
    [momentConditionNumber, momentIsotropyIndex] = conditioningMetrics( ...
        momentSingularValues, momentRank, 3);

    weakestMomentDirection = momentLeftVectors(:, 3);
    weakestMomentCommand = zeros(config.numberOfMotors, 1);
    if ~isempty(momentRightVectors)
        rightIndex = min(3, size(momentRightVectors, 2));
        weakestMomentCommand = constantCollectiveBasis * ...
            momentRightVectors(:, rightIndex);
    end
    canonicalSign = directionSign(weakestMomentDirection);
    weakestMomentDirection = canonicalSign * weakestMomentDirection;
    weakestMomentCommand = canonicalSign * weakestMomentCommand;

    perAxisEffectiveness = vecnorm(normalizedMap, 2, 2);
    perAxisFullCommandSpan = sum(abs(normalizedMap), 2);
    physicalFullCommandSpan = sum(abs(commandToWrench), 2);
    momentEffectivenessAtConstantCollective = ...
        vecnorm(constantCollectiveMomentMap, 2, 2);
    physicalConstantCollectiveMomentMap = ...
        commandToWrench(2:4, :) * constantCollectiveBasis;
    physicalMomentEffectiveness = ...
        vecnorm(physicalConstantCollectiveMomentMap, 2, 2);

    quality.axisLabels = {'Fz', 'Mx', 'My', 'Mz'};
    quality.momentAxisLabels = {'Mx', 'My', 'Mz'};
    quality.motorNames = config.motorNames;
    quality.referenceArm_m = referenceArm_m;
    quality.forceScale_N = forceScale_N;
    quality.momentScale_Nm = momentScale_Nm;
    quality.outputNormalizationScales = outputScales;
    quality.normalizationDescription = [ ...
        'Motor columns use normalized commands through their individual ' ...
        'maximum thrusts. Fz is divided by maximum total thrust; Mx, My, ' ...
        'and Mz are divided by maximum total thrust times the ' ...
        'thrust-weighted RMS horizontal motor radius.'];
    quality.commandToWrench = commandToWrench;
    quality.normalizedWrenchMap = normalizedMap;
    quality.normalizedSingularValues = singularValues;
    quality.normalizedLeftSingularVectors = leftVectors;
    quality.normalizedRightSingularVectors = rightVectors;
    quality.rank = numericalRank;
    quality.conditionNumber = conditionNumber;
    quality.isotropyIndex = isotropyIndex;
    quality.qualityScore_percent = 100 * isotropyIndex;
    quality.weakestDirectionNormalizedWrench = weakestDirection;
    quality.perAxisEffectivenessNormalized = perAxisEffectiveness;
    quality.perAxisFullCommandSpanNormalized = perAxisFullCommandSpan;
    quality.perAxisFullCommandSpanPhysical = physicalFullCommandSpan;

    quality.fixedCollective.normalizedMomentMap = ...
        constantCollectiveMomentMap;
    quality.fixedCollective.motorCommandBasis = constantCollectiveBasis;
    quality.fixedCollective.normalizedSingularValues = ...
        momentSingularValues;
    quality.fixedCollective.rank = momentRank;
    quality.fixedCollective.conditionNumber = momentConditionNumber;
    quality.fixedCollective.isotropyIndex = momentIsotropyIndex;
    quality.fixedCollective.qualityScore_percent = ...
        100 * momentIsotropyIndex;
    quality.fixedCollective.weakestMomentDirection = ...
        weakestMomentDirection;
    quality.fixedCollective.weakestMotorCommandPattern = ...
        weakestMomentCommand;
    quality.fixedCollective.perAxisEffectivenessNormalized = ...
        momentEffectivenessAtConstantCollective;
    quality.fixedCollective.perAxisEffectiveness_Nm = ...
        physicalMomentEffectiveness;

    fprintf('\nNormalized mixer quality\n');
    fprintf('  Reference arm:              %.3f mm\n', ...
        1000 * referenceArm_m);
    fprintf('  Force/moment scales:        %.3f N / %.3f N*m\n', ...
        forceScale_N, momentScale_Nm);
    fprintf('  Four-axis rank:             %d of 4\n', numericalRank);
    fprintf('  Full-mixer singular values: [%s]\n', ...
        numberList(singularValues));
    fprintf('  Full-mixer condition:       %s\n', ...
        conditionText(conditionNumber));
    fprintf('  Fixed-collective moment singular values: [%s]\n', ...
        numberList(momentSingularValues));
    fprintf('  Fixed-collective condition: %s\n', ...
        conditionText(momentConditionNumber));
    fprintf('  Fixed-collective quality:   %.2f %% (weakest/strongest)\n', ...
        quality.fixedCollective.qualityScore_percent);
    fprintf('  Weakest moment direction [Mx My Mz]: [%s]\n', ...
        numberList(weakestMomentDirection));
end

function validateConfig(config)
    requiredFields = {'numberOfMotors', 'maxThrust', 'motorLeverArms', ...
        'thrustToWrench4', 'motorNames'};
    for fieldIndex = 1:numel(requiredFields)
        if ~isfield(config, requiredFields{fieldIndex})
            error('analyzeMixerQuality:InvalidConfig', ...
                'config is missing the %s field.', requiredFields{fieldIndex});
        end
    end
    if ~isequal(size(config.thrustToWrench4), ...
            [4, config.numberOfMotors]) || ...
            size(config.motorLeverArms, 1) ~= config.numberOfMotors
        error('analyzeMixerQuality:InvalidConfig', ...
            'The mixer and lever-arm dimensions do not match the motor count.');
    end
end

function values = paddedSingularValues(singularMatrix, count)
    values = zeros(count, 1);
    diagonal = diag(singularMatrix);
    values(1:min(count, numel(diagonal))) = ...
        diagonal(1:min(count, numel(diagonal)));
end

function matrixRank = numericalMatrixRank(singularValues, matrix)
    tolerance = max(size(matrix)) * eps(max(1, singularValues(1)));
    matrixRank = sum(singularValues > tolerance);
end

function [conditionNumber, isotropyIndex] = conditioningMetrics( ...
        singularValues, matrixRank, requiredRank)
    if singularValues(1) <= 0 || matrixRank < requiredRank
        conditionNumber = Inf;
        isotropyIndex = 0;
    else
        conditionNumber = singularValues(1) / singularValues(requiredRank);
        isotropyIndex = singularValues(requiredRank) / singularValues(1);
    end
end

function direction = canonicalDirection(direction)
    direction = directionSign(direction) * direction;
end

function signValue = directionSign(direction)
    [~, largestIndex] = max(abs(direction));
    signValue = sign(direction(largestIndex));
    if signValue == 0
        signValue = 1;
    end
end

function text = numberList(values)
    text = strtrim(sprintf('%.4f ', values));
end

function text = conditionText(value)
    if isfinite(value)
        text = sprintf('%.3f', value);
    else
        text = 'Inf (rank deficient)';
    end
end
