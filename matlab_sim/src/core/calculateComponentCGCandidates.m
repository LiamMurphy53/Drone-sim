function result = calculateComponentCGCandidates(totalMass_g, totalCG_mm, ...
    componentMass_g, candidatePositions_mm, mode, currentPosition_mm)
%CALCULATECOMPONENTCGCANDIDATES Convert component placement to total CG.
%
% mode = 'move-included' means the component is already included in the
% supplied total mass and CG. Moving it leaves total mass unchanged.
% mode = 'add-payload' means the component is not yet included. Each row of
% candidatePositions_mm is treated as a possible added payload location.

    validateattributes(totalMass_g, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'}, mfilename, 'totalMass_g');
    validateattributes(totalCG_mm, {'numeric'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, mfilename, 'totalCG_mm');
    validateattributes(componentMass_g, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'}, mfilename, ...
        'componentMass_g');
    validateattributes(candidatePositions_mm, {'numeric'}, ...
        {'2d', 'ncols', 3, 'real', 'finite', 'nonempty'}, mfilename, ...
        'candidatePositions_mm');

    mode = validatestring(mode, {'move-included', 'add-payload'}, ...
        mfilename, 'mode');
    totalCG_mm = totalCG_mm(:).';

    switch mode
        case 'move-included'
            if nargin < 6 || isempty(currentPosition_mm)
                error('calculateComponentCGCandidates:MissingCurrentPosition', ...
                    ['currentPosition_mm is required when moving a component ' ...
                     'already included in the vehicle.']);
            end
            validateattributes(currentPosition_mm, {'numeric'}, ...
                {'vector', 'numel', 3, 'real', 'finite'}, mfilename, ...
                'currentPosition_mm');
            if componentMass_g > totalMass_g
                error('calculateComponentCGCandidates:ComponentTooHeavy', ...
                    'An included component cannot exceed the total mass.');
            end
            currentPosition_mm = currentPosition_mm(:).';
            candidateCG_mm = totalCG_mm + componentMass_g / totalMass_g * ...
                (candidatePositions_mm - currentPosition_mm);
            candidateTotalMass_g = repmat(totalMass_g, ...
                size(candidatePositions_mm, 1), 1);

        case 'add-payload'
            candidateTotalMass_g = repmat(totalMass_g + componentMass_g, ...
                size(candidatePositions_mm, 1), 1);
            candidateCG_mm = (totalMass_g * totalCG_mm + ...
                componentMass_g * candidatePositions_mm) ./ ...
                candidateTotalMass_g;
    end

    result.mode = mode;
    result.componentMass_g = componentMass_g;
    result.componentCandidatePositions_mm = candidatePositions_mm;
    result.totalCG_mm = candidateCG_mm;
    result.totalMass_g = candidateTotalMass_g;
    if strcmp(mode, 'move-included')
        result.componentCurrentPosition_mm = currentPosition_mm;
    end
end
