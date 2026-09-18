function [minimumValue, maximumValue, minimumPoint, maximumPoint] = ...
        boundedLinearExtrema(objective, equalityMatrix, equalityTarget, ...
        lowerBound, upperBound)
%BOUNDEDLINEAREXTREMA Optimize a linear value with equalities and bounds.
%
% This toolbox-free solver enumerates vertices of the bounded feasible
% region. It is intended for typical multirotors rather than large problems.

    vertices = boundedEqualityVertices(equalityMatrix, equalityTarget, ...
        lowerBound, upperBound);

    if isempty(vertices)
        minimumValue = NaN;
        maximumValue = NaN;
        minimumPoint = nan(numel(lowerBound), 1);
        maximumPoint = nan(numel(lowerBound), 1);
        return;
    end

    values = objective(:).' * vertices;
    [minimumValue, minimumIndex] = min(values);
    [maximumValue, maximumIndex] = max(values);
    minimumPoint = vertices(:, minimumIndex);
    maximumPoint = vertices(:, maximumIndex);
end

function vertices = boundedEqualityVertices(equalityMatrix, equalityTarget, ...
        lowerBound, upperBound)
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
        error('boundedLinearExtrema:TooManyCombinations', ...
            ['The toolbox-free search would require too many combinations ' ...
             'for this motor count.']);
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
