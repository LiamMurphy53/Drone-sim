function results = analyzeDroneConfig(motorLocations_mm, maxThrust, ...
    totalMass_g, centerOfMass_mm, inertiaTensor_g_mm2, motorNames, ...
    propulsionInputs)
%ANALYZEDRONECONFIG Calculate force/moment authority from a motor layout.
%
% results = analyzeDroneConfig(motorLocations_mm, maxThrust, totalMass_g, ...
%     centerOfMass_mm, inertiaTensor_g_mm2, motorNames, propulsionInputs)
%
% Inputs
%   motorLocations_mm - N-by-2 or N-by-3 matrix [x y (z)] in mm.
%   maxThrust      - Scalar shared maximum thrust, or N-by-1 values, in N.
%   totalMass_g    - Complete flight mass in grams.
%   centerOfMass_mm - [x y z] in mm, in the same frame as motorLocations_mm.
%   inertiaTensor_g_mm2 - 3-by-3 inertia tensor about the center of mass,
%                         in g*mm^2. It is converted to kg*m^2 internally.
%   motorNames      - Optional cell array or string array with one label per
%                     motor. Defaults to M1, M2, and so on.
%   propulsionInputs - Optional structure containing motorSpinDirection and
%                      yawTorquePerThrust_m for yaw reaction torque.
%
% Coordinate convention
%   +x = left, +y = backward, +z = up. Each motor produces force along +z.
%   Moments follow the right-hand rule.
%
% With propulsionInputs present, the model also maps propeller reaction
% torque into yaw moment. Legacy calls without it retain a zero-yaw map.

    validateattributes(motorLocations_mm, {'numeric'}, ...
        {'2d', 'real', 'finite', 'nonempty'}, mfilename, 'motorLocations_mm');

    numberOfMotors = size(motorLocations_mm, 1);
    numberOfCoordinates = size(motorLocations_mm, 2);
    if numberOfCoordinates ~= 2 && numberOfCoordinates ~= 3
        error('analyzeDroneConfig:InvalidLocationShape', ...
            'motorLocations_mm must have two or three columns: [x y] or [x y z].');
    end

    if numberOfCoordinates == 2
        motorLocations_mm = [motorLocations_mm, zeros(numberOfMotors, 1)];
    end

    validateattributes(maxThrust, {'numeric'}, ...
        {'vector', 'real', 'finite', 'positive'}, mfilename, 'maxThrust');
    if isscalar(maxThrust)
        maxThrust = repmat(maxThrust, numberOfMotors, 1);
    else
        maxThrust = maxThrust(:);
    end

    if numel(maxThrust) ~= numberOfMotors
        error('analyzeDroneConfig:InvalidThrustCount', ...
            'maxThrust must be a scalar or contain one value per motor.');
    end

    if nargin < 6 || isempty(motorNames)
        motorNames = arrayfun(@(index) sprintf('M%d', index), ...
            1:numberOfMotors, 'UniformOutput', false);
    elseif isstring(motorNames)
        motorNames = cellstr(motorNames);
    end
    if ~iscell(motorNames) || numel(motorNames) ~= numberOfMotors || ...
            ~all(cellfun(@(name) ischar(name) && ~isempty(name), motorNames))
        error('analyzeDroneConfig:InvalidMotorNames', ...
            'motorNames must contain one nonempty text label per motor.');
    end
    motorNames = motorNames(:);

    if nargin < 7 || isempty(propulsionInputs)
        motorSpinDirection = repmat({'unknown'}, numberOfMotors, 1);
        yawReactionSign = zeros(numberOfMotors, 1);
        yawTorquePerThrust_m = zeros(numberOfMotors, 1);
    else
        if ~isstruct(propulsionInputs) || ...
                ~isfield(propulsionInputs, 'motorSpinDirection') || ...
                ~isfield(propulsionInputs, 'yawTorquePerThrust_m')
            error('analyzeDroneConfig:InvalidPropulsionInputs', ...
                ['propulsionInputs must contain motorSpinDirection and ' ...
                 'yawTorquePerThrust_m.']);
        end
        motorSpinDirection = propulsionInputs.motorSpinDirection;
        if isstring(motorSpinDirection)
            motorSpinDirection = cellstr(motorSpinDirection);
        end
        if ~iscell(motorSpinDirection) || ...
                numel(motorSpinDirection) ~= numberOfMotors
            error('analyzeDroneConfig:InvalidSpinDirections', ...
                'Provide one CW or CCW spin direction per motor.');
        end
        motorSpinDirection = cellfun(@upper, motorSpinDirection(:), ...
            'UniformOutput', false);
        validSpinDirection = cellfun(@(value) ...
            strcmp(value, 'CW') || strcmp(value, 'CCW'), ...
            motorSpinDirection);
        if ~all(validSpinDirection)
            error('analyzeDroneConfig:InvalidSpinDirections', ...
                'Motor spin directions must be CW or CCW viewed from above.');
        end

        % A CW propeller applies an equal/opposite CCW (+Mz) reaction to
        % the frame. A CCW propeller therefore contributes -Mz.
        yawReactionSign = cellfun(@(value) ...
            double(strcmp(value, 'CW')) - double(strcmp(value, 'CCW')), ...
            motorSpinDirection);
        yawTorquePerThrust_m = propulsionInputs.yawTorquePerThrust_m;
        validateattributes(yawTorquePerThrust_m, {'numeric'}, ...
            {'vector', 'real', 'finite', 'nonnegative'}, mfilename, ...
            'yawTorquePerThrust_m');
        if isscalar(yawTorquePerThrust_m)
            yawTorquePerThrust_m = repmat(yawTorquePerThrust_m, ...
                numberOfMotors, 1);
        else
            yawTorquePerThrust_m = yawTorquePerThrust_m(:);
        end
        if numel(yawTorquePerThrust_m) ~= numberOfMotors
            error('analyzeDroneConfig:InvalidYawCoefficientCount', ...
                'Provide one yaw coefficient or one value per motor.');
        end
    end

    validateattributes(totalMass_g, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'}, mfilename, 'totalMass_g');
    validateattributes(centerOfMass_mm, {'numeric'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, mfilename, 'centerOfMass_mm');
    centerOfMass_mm = centerOfMass_mm(:).';

    validateattributes(inertiaTensor_g_mm2, {'numeric'}, ...
        {'size', [3, 3], 'real', 'finite'}, mfilename, 'inertiaTensor_g_mm2');
    symmetryTolerance = 1e-10 * max(1, norm(inertiaTensor_g_mm2, 'fro'));
    if norm(inertiaTensor_g_mm2 - inertiaTensor_g_mm2.', 'fro') > ...
            symmetryTolerance
        error('analyzeDroneConfig:NonSymmetricInertia', ...
            'inertiaTensor_g_mm2 must be symmetric.');
    end
    [~, positiveDefiniteFlag] = chol(inertiaTensor_g_mm2);
    if positiveDefiniteFlag ~= 0
        error('analyzeDroneConfig:InvalidInertia', ...
            'inertiaTensor_g_mm2 must be positive definite.');
    end

    % Convert the Onshape-friendly input units to SI for dynamics.
    motorLocations = motorLocations_mm * 1e-3;
    centerOfMass = centerOfMass_mm * 1e-3;
    totalMass = totalMass_g * 1e-3;
    inertiaTensor = inertiaTensor_g_mm2 * 1e-9;

    motorLeverArms = motorLocations - centerOfMass;
    x = motorLeverArms(:, 1);
    y = motorLeverArms(:, 2);

    % Maps individual thrusts [N] to [Fz; Mx; My].
    thrustToWrench = [ones(1, numberOfMotors); y.'; -x.'];
    yawMomentPerThrust = yawReactionSign .* yawTorquePerThrust_m;
    thrustToWrench4 = [thrustToWrench; yawMomentPerThrust.'];

    % Maps normalized commands (0 to 1) to [Fz; Mx; My].
    commandToWrench = thrustToWrench * diag(maxThrust);
    commandToWrench4 = thrustToWrench4 * diag(maxThrust);
    fullThrottleWrench = commandToWrench * ones(numberOfMotors, 1);

    xMomentContributions = commandToWrench(2, :);
    yMomentContributions = commandToWrench(3, :);

    results.numberOfMotors = numberOfMotors;
    results.motorNames = motorNames;
    results.motorLocations_mm = motorLocations_mm;
    results.motorLocations = motorLocations;
    results.motorLeverArms = motorLeverArms;
    results.maxThrust = maxThrust;
    results.totalMass_g = totalMass_g;
    results.totalMass = totalMass;
    results.centerOfMass_mm = centerOfMass_mm;
    results.centerOfMass = centerOfMass;
    results.inertiaTensor_g_mm2 = inertiaTensor_g_mm2;
    results.inertiaTensor = inertiaTensor;
    results.gravity = 9.80665;
    results.weight = totalMass * results.gravity;
    results.thrustToWrench = thrustToWrench;
    results.thrustToWrench4 = thrustToWrench4;
    results.motorSpinDirection = motorSpinDirection;
    results.yawReactionSign = yawReactionSign;
    results.yawTorquePerThrust_m = yawTorquePerThrust_m;
    results.commandToWrench = commandToWrench;
    results.commandToWrench4 = commandToWrench4;
    if nargin >= 7 && isstruct(propulsionInputs)
        results.propulsionInputs = propulsionInputs;
    end
    results.maxTotalThrust = sum(maxThrust);
    results.fullThrottleWrench = fullThrottleWrench;
    results.centerOfThrustAtFull = ...
        sum(motorLocations .* maxThrust, 1) / sum(maxThrust);
    results.centerOfThrustOffset = ...
        results.centerOfThrustAtFull - centerOfMass;
    results.centerOfThrustAtFull_mm = results.centerOfThrustAtFull * 1000;
    results.centerOfThrustOffset_mm = results.centerOfThrustOffset * 1000;
    results.maxThrustToWeight = results.maxTotalThrust / results.weight;
    results.hoverCollectiveFraction = results.weight / results.maxTotalThrust;
    results.xAxisMomentRange = [sum(min(xMomentContributions, 0)), ...
                                sum(max(xMomentContributions, 0))];
    results.yAxisMomentRange = [sum(min(yMomentContributions, 0)), ...
                                sum(max(yMomentContributions, 0))];
    results.controlRank = rank(thrustToWrench);
    results.controlRank4 = rank(thrustToWrench4);
    results.fullThrottleAngularAcceleration = inertiaTensor \ ...
        [fullThrottleWrench(2); fullThrottleWrench(3); 0];

    fprintf('\nDrone configuration summary\n');
    fprintf('  Motors:                    %d\n', numberOfMotors);
    fprintf('  Total mass:                %.3f g\n', totalMass_g);
    fprintf('  Maximum total thrust:      %.3f N\n', results.maxTotalThrust);
    fprintf('  Maximum thrust-to-weight:  %.3f\n', results.maxThrustToWeight);
    fprintf('  Ideal hover command:       %.1f %%\n', ...
        100 * results.hoverCollectiveFraction);
    fprintf('  Full-throttle x moment:    %.3f N*m\n', fullThrottleWrench(2));
    fprintf('  Full-throttle y moment:    %.3f N*m\n', fullThrottleWrench(3));
    fprintf('  X-axis moment range:       %.3f to %.3f N*m\n', ...
        results.xAxisMomentRange(1), results.xAxisMomentRange(2));
    fprintf('  Y-axis moment range:       %.3f to %.3f N*m\n', ...
        results.yAxisMomentRange(1), results.yAxisMomentRange(2));
    fprintf('  Center-of-thrust offset:   x=%.3f, y=%.3f mm\n', ...
        results.centerOfThrustOffset_mm(1), ...
        results.centerOfThrustOffset_mm(2));
    if any(yawTorquePerThrust_m > 0)
        fprintf('  Yaw Q/T estimate:          %.4f m\n', ...
            mean(yawTorquePerThrust_m));
        fprintf('  Four-axis allocation rank: %d\n', results.controlRank4);
    end

    if results.controlRank < 3
        warning(['This layout cannot independently produce vertical thrust, ' ...
                 'x-axis moment, and y-axis moment.']);
    end
    if any(yawTorquePerThrust_m > 0) && results.controlRank4 < 4
        warning(['This spin layout cannot independently produce collective ' ...
                 'thrust plus x, y, and yaw moments.']);
    end
    if results.maxThrustToWeight <= 1
        warning('Maximum total thrust is not greater than vehicle weight.');
    end
end
