function envelope = analyzeMomentEnvelope3D(config, collectiveThrust_N)
%ANALYZEMOMENTENVELOPE3D Exact fixed-collective Mx-My-Mz envelope.
%
% envelope = analyzeMomentEnvelope3D(config)
% envelope = analyzeMomentEnvelope3D(config, collectiveThrust_N)
%
% The default collective thrust is the vehicle weight. The returned
% polytope contains every moment vector that can be produced while
% satisfying
%
%   sum(T_i) = collectiveThrust_N,   0 <= T_i <= maxThrust_i.
%
% Moments use the project convention: +x is left, +y is backward, +z is
% up, and positive Mx, My, and Mz follow the right-hand rule. The motor
% reaction-torque row in config.thrustToWrench4 supplies Mz.
%
% This is an exact vertex enumeration, not a directional sample. A vertex
% of a box cut by one collective-thrust equality has at most one motor away
% from a bound. Mapping all such thrust vertices through the linear moment
% mixer therefore gives every vertex needed for the convex moment hull.

    if nargin < 2 || isempty(collectiveThrust_N)
        collectiveThrust_N = config.weight;
    end
    validateConfig(config);
    validateattributes(collectiveThrust_N, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'nonnegative'}, mfilename, ...
        'collectiveThrust_N');

    maximumCollective = sum(config.maxThrust);
    thrustTolerance = 1e-10 * max(1, maximumCollective);
    if collectiveThrust_N > maximumCollective + thrustTolerance
        error('analyzeMomentEnvelope3D:CollectiveInfeasible', ...
            ['collectiveThrust_N must be between zero and the maximum ' ...
             'total thrust (%.6g N).'], maximumCollective);
    end
    collectiveThrust_N = min(max(collectiveThrust_N, 0), ...
        maximumCollective);

    thrustVertices = fixedCollectiveThrustVertices( ...
        config.maxThrust, collectiveThrust_N);
    if isempty(thrustVertices)
        error('analyzeMomentEnvelope3D:CollectiveInfeasible', ...
            'No motor thrusts satisfy the requested collective thrust.');
    end

    momentMap = config.thrustToWrench4(2:4, :);
    mappedMoments = momentMap * thrustVertices;
    [momentVertices, retainedIndices] = uniqueColumns(mappedMoments);
    thrustVertices = thrustVertices(:, retainedIndices);

    momentCentroid = mean(momentVertices, 2);
    centeredMoments = momentVertices - momentCentroid;
    singularValues = svd(centeredMoments, 'econ');
    dimensionTolerance = 1e-10 * max(1, norm(momentVertices, 'fro'));
    effectiveDimension = sum(singularValues > dimensionTolerance);

    % A linear projection of a higher-motor-count thrust polytope can map
    % some input vertices into the interior of the 3D output hull. Remove
    % those mapped interior points so the public vertex list remains the
    % exact set of moment-polytope vertices for general N as well as quads.
    hullVertexIndices = convexHullVertexIndices(momentVertices, ...
        centeredMoments, effectiveDimension, dimensionTolerance);
    momentVertices = momentVertices(:, hullVertexIndices);
    thrustVertices = thrustVertices(:, hullVertexIndices);
    momentCentroid = mean(momentVertices, 2);
    centeredMoments = momentVertices - momentCentroid;

    boundaryFacets = zeros(0, 3);
    boundaryVertexOrder = zeros(0, 1);
    hullVolume = 0;
    hullArea = 0;
    hullLength = 0;
    facetNormals = zeros(0, 3);
    facetOffsets = zeros(0, 1);

    if effectiveDimension == 3
        [boundaryFacets, hullVolume] = convhulln(momentVertices.');
        [facetNormals, facetOffsets] = facetHalfSpaces( ...
            momentVertices, boundaryFacets, momentCentroid);
    elseif effectiveDimension == 2
        [leftVectors, ~, ~] = svd(centeredMoments, 'econ');
        planeBasis = leftVectors(:, 1:2);
        planeCoordinates = planeBasis.' * centeredMoments;
        [boundaryVertexOrder, hullArea] = convhull( ...
            planeCoordinates(1, :).', planeCoordinates(2, :).');
    elseif effectiveDimension == 1
        direction = centeredMoments(:, 1);
        if norm(direction) <= dimensionTolerance
            [~, farthestIndex] = max(vecnorm(centeredMoments, 2, 1));
            direction = centeredMoments(:, farthestIndex);
        end
        direction = direction / norm(direction);
        coordinates = direction.' * momentVertices;
        [minimumCoordinate, minimumIndex] = min(coordinates);
        [maximumCoordinate, maximumIndex] = max(coordinates);
        boundaryVertexOrder = [minimumIndex; maximumIndex];
        hullLength = maximumCoordinate - minimumCoordinate;
    end

    momentMinimum = min(momentVertices, [], 2);
    momentMaximum = max(momentVertices, [], 2);
    normalizedShapeScale = max(abs([momentMinimum, momentMaximum]), [], 2);
    normalizedShapeScale(normalizedShapeScale <= dimensionTolerance) = 1;
    normalizedMomentVertices = momentVertices ./ normalizedShapeScale;
    momentMagnitude = vecnorm(momentVertices, 2, 1);
    [maximumMagnitude, maximumMagnitudeIndex] = max(momentMagnitude);

    pureCollective = allocateMotorWrench(config, ...
        [collectiveThrust_N; 0; 0; 0]);
    residualTolerance = 1e-8 * max(1, collectiveThrust_N);
    pureMomentTrimFeasible = norm(pureCollective.wrenchError, inf) <= ...
        residualTolerance;

    originCenteredRadius = NaN;
    limitingFacetNormal = nan(3, 1);
    originInsideEnvelope = pureMomentTrimFeasible;
    if effectiveDimension == 3
        hullTolerance = 1e-9 * max(1, max(abs(facetOffsets)));
        originInsideEnvelope = all(facetOffsets >= -hullTolerance);
    end
    if effectiveDimension == 3 && originInsideEnvelope
        planeDistanceFromOrigin = facetOffsets;
        [originCenteredRadius, limitingFacetIndex] = min( ...
            planeDistanceFromOrigin);
        originCenteredRadius = max(0, originCenteredRadius);
        limitingFacetNormal = facetNormals(limitingFacetIndex, :).';
    end

    envelope.collectiveThrust_N = collectiveThrust_N;
    envelope.collectiveFractionOfMaximum = ...
        collectiveThrust_N / maximumCollective;
    envelope.maximumCollectiveThrust_N = maximumCollective;
    envelope.motorNames = config.motorNames;
    envelope.motorThrustVertices_N = thrustVertices;
    envelope.momentVertices_Nm = momentVertices;
    envelope.numberOfVertices = size(momentVertices, 2);
    envelope.effectiveDimension = effectiveDimension;
    envelope.boundaryFacets = boundaryFacets;
    envelope.boundaryVertexOrder = boundaryVertexOrder;
    envelope.facetNormals = facetNormals;
    envelope.facetOffsets_Nm = facetOffsets;
    envelope.momentCentroid_Nm = momentCentroid;
    envelope.momentRange_Nm = [momentMinimum, momentMaximum];
    envelope.momentSpan_Nm = momentMaximum - momentMinimum;
    envelope.normalizedShapeScale_Nm = normalizedShapeScale;
    envelope.normalizedMomentVertices = normalizedMomentVertices;
    envelope.maximumMomentMagnitude_Nm = maximumMagnitude;
    envelope.maximumMagnitudeMoment_Nm = ...
        momentVertices(:, maximumMagnitudeIndex);
    envelope.volume_Nm3 = hullVolume;
    envelope.area_Nm2 = hullArea;
    envelope.length_Nm = hullLength;
    envelope.pureMomentTrimFeasible = pureMomentTrimFeasible;
    envelope.pureMomentTrimThrust_N = pureCollective.motorThrust_N;
    envelope.pureMomentTrimCommand = ...
        pureCollective.motorThrust_N ./ config.maxThrust;
    envelope.pureMomentTrimResidual = pureCollective.wrenchError;
    envelope.originInsideEnvelope = originInsideEnvelope;
    envelope.originCenteredRadius_Nm = originCenteredRadius;
    envelope.originCenteredLimitingDirection = limitingFacetNormal;
    envelope.axisLabels = {'Mx', 'My', 'Mz'};

    fprintf('\nFixed-collective 3D moment envelope\n');
    fprintf('  Collective thrust:          %.3f N (%.1f %% of maximum)\n', ...
        collectiveThrust_N, 100 * envelope.collectiveFractionOfMaximum);
    fprintf('  Exact moment vertices:      %d\n', envelope.numberOfVertices);
    fprintf('  Effective moment dimension: %d of 3\n', effectiveDimension);
    fprintf('  Mx range:                   %.3f to %.3f N*m\n', ...
        momentMinimum(1), momentMaximum(1));
    fprintf('  My range:                   %.3f to %.3f N*m\n', ...
        momentMinimum(2), momentMaximum(2));
    fprintf('  Mz range:                   %.3f to %.3f N*m\n', ...
        momentMinimum(3), momentMaximum(3));
    if effectiveDimension == 3
        fprintf('  Envelope volume:            %.6g (N*m)^3\n', hullVolume);
        if pureMomentTrimFeasible
            fprintf('  Smallest combined-moment margin about trim: %.4f N*m\n', ...
                originCenteredRadius);
        end
    end
    fprintf('  Pure collective trim:       %s\n', ...
        feasibleText(pureMomentTrimFeasible));
end

function validateConfig(config)
    requiredFields = {'numberOfMotors', 'maxThrust', 'weight', ...
        'thrustToWrench4', 'motorNames'};
    for fieldIndex = 1:numel(requiredFields)
        if ~isfield(config, requiredFields{fieldIndex})
            error('analyzeMomentEnvelope3D:InvalidConfig', ...
                'config is missing the %s field.', requiredFields{fieldIndex});
        end
    end
    if ~isequal(size(config.thrustToWrench4), ...
            [4, config.numberOfMotors])
        error('analyzeMomentEnvelope3D:InvalidConfig', ...
            'config.thrustToWrench4 must be 4-by-numberOfMotors.');
    end
end

function vertices = fixedCollectiveThrustVertices(maximumThrust, collective)
    maximumThrust = maximumThrust(:);
    numberOfMotors = numel(maximumThrust);
    patternsPerFreeMotor = 2^(numberOfMotors - 1);
    candidateCount = numberOfMotors * patternsPerFreeMotor;
    if candidateCount > 2e6
        error('analyzeMomentEnvelope3D:TooManyMotorCombinations', ...
            ['Exact fixed-collective vertex enumeration would require ' ...
             'more than two million candidates for this motor count.']);
    end

    tolerance = 1e-9 * max(1, max(maximumThrust));
    vertices = zeros(numberOfMotors, 0);
    allMotors = 1:numberOfMotors;
    for freeMotor = allMotors
        fixedMotors = setdiff(allMotors, freeMotor);
        for pattern = 0:(patternsPerFreeMotor - 1)
            candidate = zeros(numberOfMotors, 1);
            atUpperBound = bitget(pattern, 1:numel(fixedMotors)).';
            candidate(fixedMotors) = atUpperBound .* ...
                maximumThrust(fixedMotors);
            candidate(freeMotor) = collective - sum(candidate(fixedMotors));
            if candidate(freeMotor) >= -tolerance && ...
                    candidate(freeMotor) <= maximumThrust(freeMotor) + tolerance
                candidate = min(max(candidate, 0), maximumThrust);
                vertices(:, end + 1) = candidate; %#ok<AGROW>
            end
        end
    end

    if ~isempty(vertices)
        vertices = uniquetol(vertices.', 1e-10, 'ByRows', true).';
    end
end

function [uniqueValues, retainedIndices] = uniqueColumns(values)
    [~, retainedIndices] = uniquetol(values.', 1e-10, ...
        'ByRows', true, 'OutputAllIndices', false);
    retainedIndices = sort(retainedIndices);
    uniqueValues = values(:, retainedIndices);
end

function indices = convexHullVertexIndices(vertices, centeredVertices, ...
        effectiveDimension, tolerance)
    if effectiveDimension == 3
        facets = convhulln(vertices.');
        indices = unique(facets(:), 'stable');
    elseif effectiveDimension == 2
        [leftVectors, ~, ~] = svd(centeredVertices, 'econ');
        coordinates = leftVectors(:, 1:2).' * centeredVertices;
        boundaryOrder = convhull(coordinates(1, :).', coordinates(2, :).');
        indices = unique(boundaryOrder(1:end - 1), 'stable');
    elseif effectiveDimension == 1
        direction = centeredVertices(:, 1);
        if norm(direction) <= tolerance
            [~, farthestIndex] = max(vecnorm(centeredVertices, 2, 1));
            direction = centeredVertices(:, farthestIndex);
        end
        coordinates = direction.' * vertices;
        [~, minimumIndex] = min(coordinates);
        [~, maximumIndex] = max(coordinates);
        indices = unique([minimumIndex; maximumIndex], 'stable');
    else
        indices = 1;
    end
end

function [normals, offsets] = facetHalfSpaces(vertices, facets, interiorPoint)
    numberOfFacets = size(facets, 1);
    normals = zeros(numberOfFacets, 3);
    offsets = zeros(numberOfFacets, 1);
    for facetIndex = 1:numberOfFacets
        points = vertices(:, facets(facetIndex, :));
        normal = cross(points(:, 2) - points(:, 1), ...
            points(:, 3) - points(:, 1));
        normal = normal / norm(normal);
        offset = normal.' * points(:, 1);
        if normal.' * interiorPoint > offset
            normal = -normal;
            offset = -offset;
        end
        normals(facetIndex, :) = normal.';
        offsets(facetIndex) = offset;
    end
end

function text = feasibleText(isFeasible)
    if isFeasible
        text = 'feasible';
    else
        text = 'not feasible';
    end
end
