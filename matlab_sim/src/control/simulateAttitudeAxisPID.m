function simulation = simulateAttitudeAxisPID(config, hover, controller, ...
    settings, axisIndex)
%SIMULATEATTITUDEAXISPID Simulate scalar PID control about one body axis.
%
% axisIndex = 1 controls Mx with My cancelled. axisIndex = 2 controls My
% with Mx cancelled. axisIndex = 3 controls reaction-torque Mz while Mx and
% My are cancelled. This intentionally remains a scalar attitude model: it
% uses the selected diagonal inertia and omits cross-axis rotation.

    validateattributes(axisIndex, {'numeric'}, ...
        {'scalar', 'integer', '>=', 1, '<=', 3}, mfilename, 'axisIndex');
    validateattributes(controller.Kp, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'nonnegative'});
    validateattributes(controller.Ki, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'nonnegative'});
    validateattributes(controller.Kd, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'nonnegative'});
    validateattributes(settings.timeStep_s, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'});
    validateattributes(settings.motorTimeConstant_s, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'});
    validateattributes(settings.time_s, {'numeric'}, ...
        {'vector', 'real', 'finite', 'nonnegative'});
    validateattributes(settings.angleCommand_rad, {'numeric'}, ...
        {'vector', 'real', 'finite'});

    time = settings.time_s(:);
    angleCommand = settings.angleCommand_rad(:);
    if numel(time) ~= numel(angleCommand)
        error('simulateAttitudeAxisPID:CommandSize', ...
            'time_s and angleCommand_rad must have the same length.');
    end
    if numel(time) < 2 || any(abs(diff(time) - settings.timeStep_s) > ...
            1e-9 * max(1, settings.timeStep_s))
        error('simulateAttitudeAxisPID:InvalidTimeVector', ...
            'time_s must be uniformly spaced by timeStep_s.');
    end
    if ~hover.hoverFeasible
        error('simulateAttitudeAxisPID:HoverInfeasible', ...
            'The supplied configuration cannot maintain level hover.');
    end

    numberOfSamples = numel(time);
    numberOfMotors = config.numberOfMotors;
    timeStep = settings.timeStep_s;
    axisInertia = config.inertiaTensor(axisIndex, axisIndex);
    momentRow = axisIndex + 1;

    % When yaw data is available, every axis test uses the same four-axis
    % mixer and cancels all uncommanded moments. Legacy pitch/roll callers
    % without yaw data retain the original three-axis mixer.
    fourAxisAvailable = isfield(config, 'thrustToWrench4') && ...
        isfield(config, 'controlRank4') && config.controlRank4 >= 4 && ...
        isfield(config, 'yawTorquePerThrust_m') && ...
        any(config.yawTorquePerThrust_m > 0);
    if fourAxisAvailable
        wrenchMap = config.thrustToWrench4;
        yawHover = allocateMotorWrench(config, ...
            [config.weight; 0; 0; 0], ones(4, 1));
        if yawHover.normalizedWeightedError > 1e-5
            error('simulateAttitudeAxisPID:YawNeutralHover', ...
                'A zero-yaw level-hover allocation could not be found.');
        end
        referenceMotorThrust = yawHover.motorThrust_N;
    else
        if axisIndex == 3
            error('simulateAttitudeAxisPID:YawNotControllable', ...
                ['Yaw needs a rank-four collective/roll/pitch/yaw map. ' ...
                 'Check motor spin directions and yaw Q/T data.']);
        end
        wrenchMap = config.thrustToWrench;
        referenceMotorThrust = hover.motorThrust;
    end

    % Change only the selected moment while preserving total thrust and
    % cancelling every other moment represented by the selected mixer.
    desiredUnitWrench = zeros(size(wrenchMap, 1), 1);
    desiredUnitWrench(momentRow) = 1;
    momentAllocation = pinv(wrenchMap) * desiredUnitWrench;
    allocationError = norm(wrenchMap * momentAllocation - ...
        desiredUnitWrench);
    if allocationError > 1e-8
        error('simulateAttitudeAxisPID:AxisNotControllable', ...
            ['This motor layout cannot independently command the selected ' ...
             'moment.']);
    end
    otherMomentRows = setdiff(2:size(wrenchMap, 1), momentRow);
    if any(abs(wrenchMap(otherMomentRows, :) * momentAllocation) > 1e-8)
        error('simulateAttitudeAxisPID:AllocationCoupling', ...
            'The calculated allocation does not cancel the other moments.');
    end

    [minimumMoment, maximumMoment] = allocationScaleLimits( ...
        referenceMotorThrust, momentAllocation, ...
        zeros(numberOfMotors, 1), config.maxThrust);

    angle = zeros(numberOfSamples, 1);
    angularRate = zeros(numberOfSamples, 1);
    angleError = zeros(numberOfSamples, 1);
    integralError = zeros(numberOfSamples, 1);
    requestedMoment = zeros(numberOfSamples, 1);
    commandedMoment = zeros(numberOfSamples, 1);
    achievedMoment = zeros(numberOfSamples, 1);
    motorThrustCommand = zeros(numberOfSamples, numberOfMotors);
    motorThrust = zeros(numberOfSamples, numberOfMotors);
    momentSaturated = false(numberOfSamples, 1);

    motorThrust(1, :) = referenceMotorThrust.';
    motorThrustCommand(1, :) = referenceMotorThrust.';
    achievedMoment(1) = wrenchMap(momentRow, :) * referenceMotorThrust;

    for sampleIndex = 1:(numberOfSamples - 1)
        errorNow = angleCommand(sampleIndex) - angle(sampleIndex);
        angleError(sampleIndex) = errorNow;

        % Derivative on measurement avoids derivative kick at a command step.
        rawMoment = controller.Kp * errorNow + ...
            controller.Ki * integralError(sampleIndex) - ...
            controller.Kd * angularRate(sampleIndex);
        limitedMoment = min(max(rawMoment, minimumMoment), maximumMoment);
        requestedMoment(sampleIndex) = rawMoment;
        commandedMoment(sampleIndex) = limitedMoment;
        momentSaturated(sampleIndex) = abs(limitedMoment - rawMoment) > 1e-12;

        % Pause the integrator only when error would drive farther into a
        % saturated moment limit.
        integrateError = ~momentSaturated(sampleIndex) || ...
            (limitedMoment >= maximumMoment && errorNow < 0) || ...
            (limitedMoment <= minimumMoment && errorNow > 0);
        integralError(sampleIndex + 1) = integralError(sampleIndex);
        if integrateError
            integralError(sampleIndex + 1) = integralError(sampleIndex) + ...
                errorNow * timeStep;
        end

        thrustCommand = referenceMotorThrust + ...
            momentAllocation * limitedMoment;
        thrustCommand = min(max(thrustCommand, 0), config.maxThrust);
        motorThrustCommand(sampleIndex, :) = thrustCommand.';

        % First-order motor/propeller thrust response.
        actualThrust = motorThrust(sampleIndex, :).';
        thrustRate = (thrustCommand - actualThrust) / ...
            settings.motorTimeConstant_s;
        nextThrust = actualThrust + thrustRate * timeStep;
        nextThrust = min(max(nextThrust, 0), config.maxThrust);
        motorThrust(sampleIndex + 1, :) = nextThrust.';

        actualWrench = wrenchMap * actualThrust;
        achievedMoment(sampleIndex) = actualWrench(momentRow);
        angularAcceleration = actualWrench(momentRow) / axisInertia;

        angularRate(sampleIndex + 1) = angularRate(sampleIndex) + ...
            angularAcceleration * timeStep;
        angle(sampleIndex + 1) = angle(sampleIndex) + ...
            angularRate(sampleIndex + 1) * timeStep;
    end

    angleError(end) = angleCommand(end) - angle(end);
    requestedMoment(end) = requestedMoment(end - 1);
    commandedMoment(end) = commandedMoment(end - 1);
    momentSaturated(end) = momentSaturated(end - 1);
    motorThrustCommand(end, :) = motorThrustCommand(end - 1, :);
    finalWrench = wrenchMap * motorThrust(end, :).';
    achievedMoment(end) = finalWrench(momentRow);

    axisNames = {'x', 'y', 'z'};
    momentNames = {'Mx', 'My', 'Mz'};
    motionNames = {'Pitch', 'Roll', 'Yaw'};
    simulation.time_s = time;
    simulation.angleCommand_rad = angleCommand;
    simulation.angle_rad = angle;
    simulation.angularRate_rad_s = angularRate;
    simulation.angleError_rad = angleError;
    simulation.integralError_rad_s = integralError;
    simulation.requestedMoment_Nm = requestedMoment;
    simulation.commandedMoment_Nm = commandedMoment;
    simulation.achievedMoment_Nm = achievedMoment;
    simulation.motorThrustCommand_N = motorThrustCommand;
    simulation.motorThrust_N = motorThrust;
    simulation.momentSaturated = momentSaturated;
    simulation.momentLimits_Nm = [minimumMoment, maximumMoment];
    simulation.momentAllocation_N_per_Nm = momentAllocation;
    simulation.axisIndex = axisIndex;
    simulation.axisName = axisNames{axisIndex};
    simulation.momentName = momentNames{axisIndex};
    simulation.motionName = motionNames{axisIndex};
    simulation.axisInertia_kg_m2 = axisInertia;
    simulation.referenceMotorThrust_N = referenceMotorThrust;
    if isfield(config, 'motorNames') && ...
            numel(config.motorNames) == numberOfMotors
        simulation.motorNames = config.motorNames;
    else
        simulation.motorNames = arrayfun(@(index) sprintf('M%d', index), ...
            1:numberOfMotors, 'UniformOutput', false).';
    end
    simulation.controller = controller;
    simulation.settings = settings;
    simulation.metrics = calculateMetrics(simulation, config);

    printSummary = ~isfield(settings, 'printSummary') || settings.printSummary;
    if printSummary
        fprintf('\n%s PID simulation (body %s / %s)\n', ...
            simulation.motionName, simulation.axisName, ...
            simulation.momentName);
        fprintf('  Mixer moment limits: %.3f to %.3f N*m\n', ...
            minimumMoment, maximumMoment);
        fprintf('  Peak absolute angle: %.2f deg\n', ...
            max(abs(angle)) * 180 / pi);
        fprintf('  Peak angular rate: %.2f deg/s\n', ...
            simulation.metrics.peakRate_deg_s);
        fprintf('  Active-command saturation: %.2f %%\n', ...
            simulation.metrics.activeSaturationPercent);
    end
end

function metrics = calculateMetrics(simulation, config)
    radiansToDegrees = 180 / pi;
    time = simulation.time_s;
    command = simulation.angleCommand_rad;
    activeIndices = find(abs(command) > 1e-12);

    metrics.peakRate_deg_s = ...
        max(abs(simulation.angularRate_rad_s)) * radiansToDegrees;
    normalizedMotorThrust = bsxfun(@rdivide, ...
        simulation.motorThrust_N, config.maxThrust.');
    metrics.peakMotorUtilizationPercent = ...
        100 * max(normalizedMotorThrust(:));
    metrics.riseTime_s = NaN;
    metrics.overshoot_deg = NaN;
    metrics.returnSettlingTime_s = NaN;
    metrics.activeTrackingRMSError_deg = NaN;
    metrics.activeSaturationPercent = 0;
    metrics.directionalMomentLimit_Nm = NaN;

    if isempty(activeIndices)
        return;
    end

    directionSign = sign(mean(command(activeIndices)));
    if directionSign == 0
        directionSign = 1;
    end
    commandMagnitude = max(abs(command(activeIndices)));
    signedAngle = directionSign * simulation.angle_rad;
    activeSignedAngle = signedAngle(activeIndices);
    activeTime = time(activeIndices);
    tenPercentIndex = find(activeSignedAngle >= 0.1 * commandMagnitude, ...
        1, 'first');
    ninetyPercentIndex = find(activeSignedAngle >= 0.9 * commandMagnitude, ...
        1, 'first');
    if ~isempty(tenPercentIndex) && ~isempty(ninetyPercentIndex) && ...
            ninetyPercentIndex >= tenPercentIndex
        metrics.riseTime_s = activeTime(ninetyPercentIndex) - ...
            activeTime(tenPercentIndex);
    end

    metrics.overshoot_deg = max(0, ...
        max(activeSignedAngle) - commandMagnitude) * radiansToDegrees;
    activeError = command(activeIndices) - ...
        simulation.angle_rad(activeIndices);
    metrics.activeTrackingRMSError_deg = ...
        sqrt(mean(activeError.^2)) * radiansToDegrees;
    metrics.activeSaturationPercent = 100 * ...
        mean(simulation.momentSaturated(activeIndices));
    if directionSign > 0
        metrics.directionalMomentLimit_Nm = simulation.momentLimits_Nm(2);
    else
        metrics.directionalMomentLimit_Nm = ...
            abs(simulation.momentLimits_Nm(1));
    end

    firstReturnIndex = activeIndices(end) + 1;
    if firstReturnIndex <= numel(time)
        if isfield(simulation.settings, 'settlingBand_deg')
            settlingBand_deg = simulation.settings.settlingBand_deg;
        else
            settlingBand_deg = 2;
        end
        returnIndices = firstReturnIndex:numel(time);
        insideBand = abs(simulation.angle_rad(returnIndices)) <= ...
            settlingBand_deg / radiansToDegrees;
        for localIndex = 1:numel(returnIndices)
            if all(insideBand(localIndex:end))
                metrics.returnSettlingTime_s = ...
                    time(returnIndices(localIndex)) - ...
                    time(firstReturnIndex);
                break;
            end
        end
    end
end

function [minimumScale, maximumScale] = allocationScaleLimits( ...
        referencePoint, direction, lowerBound, upperBound)
    minimumScale = -inf;
    maximumScale = inf;

    for variableIndex = 1:numel(referencePoint)
        if abs(direction(variableIndex)) < 1e-14
            continue;
        end

        firstLimit = (lowerBound(variableIndex) - ...
            referencePoint(variableIndex)) / direction(variableIndex);
        secondLimit = (upperBound(variableIndex) - ...
            referencePoint(variableIndex)) / direction(variableIndex);
        minimumScale = max(minimumScale, min(firstLimit, secondLimit));
        maximumScale = min(maximumScale, max(firstLimit, secondLimit));
    end

    if ~isfinite(minimumScale) || ~isfinite(maximumScale) || ...
            minimumScale > maximumScale
        error('simulateAttitudeAxisPID:InvalidAllocationLimits', ...
            'Could not determine valid motor-allocation limits.');
    end
end
