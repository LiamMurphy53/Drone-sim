function simulation = simulateDrone6DOF(config, settings)
%SIMULATEDRONE6DOF Simulate attitude, rate-mode, and altitude commands.
%
% The rigid-body plant includes quaternion attitude, the full inertia tensor,
% gyroscopic coupling, 3D translation, gravity, individual motor lag and
% limits, propeller yaw reaction torque, and bounded four-axis allocation.

    if ~isfield(settings, 'rateCommandSegments')
        settings.rateCommandSegments = zeros(0, 6);
    end
    if ~isfield(settings, 'rateControlBandwidth_rad_s')
        settings.rateControlBandwidth_rad_s = [20; 20; 10];
    end

    requiredFields = {'commandWaypoints', 'timeStep_s', ...
        'motorTimeConstant_s', 'attitudeNaturalFrequency_rad_s', ...
        'attitudeDampingRatio', 'attitudeIntegralPole_rad_s', ...
        'maxAttitudeIntegral_rad_s', ...
        'altitudeNaturalFrequency_rad_s', 'altitudeDampingRatio', ...
        'altitudeIntegralPole_rad_s', 'maxAltitudeIntegral_m_s', ...
        'maxVerticalAcceleration_m_s2', 'initialPosition_m', ...
        'initialVelocity_m_s', 'initialAttitude_deg', ...
        'initialAngularVelocity_rad_s', ...
        'allocationPriority'};
    missingFields = requiredFields(~isfield(settings, requiredFields));
    if ~isempty(missingFields)
        error('simulateDrone6DOF:MissingSetting', ...
            'The settings structure is missing %s.', missingFields{1});
    end
    if ~isfield(config, 'thrustToWrench4') || config.controlRank4 < 4
        error('simulateDrone6DOF:YawNotControllable', ...
            ['The configuration needs a rank-four collective/roll/pitch/yaw ' ...
             'motor map. Check spin directions and the yaw coefficient.']);
    end

    validateattributes(settings.commandWaypoints, {'numeric'}, ...
        {'2d', 'real', 'finite', 'nonempty'}, mfilename, ...
        'commandWaypoints');
    if size(settings.commandWaypoints, 2) ~= 5
        error('simulateDrone6DOF:InvalidWaypointShape', ...
            ['commandWaypoints must have five columns: time, roll, pitch, ' ...
             'yaw, and altitude.']);
    end
    validateattributes(settings.timeStep_s, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'});
    validateattributes(settings.motorTimeConstant_s, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'});
    validateattributes(settings.maxAltitudeIntegral_m_s, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'nonnegative'});
    validateattributes(settings.maxVerticalAcceleration_m_s2, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'});
    validateThreeState(settings.initialPosition_m, 'initialPosition_m');
    validateThreeState(settings.initialVelocity_m_s, 'initialVelocity_m_s');
    validateThreeState(settings.initialAttitude_deg, 'initialAttitude_deg');
    validateThreeState(settings.initialAngularVelocity_rad_s, ...
        'initialAngularVelocity_rad_s');
    waypoints = settings.commandWaypoints;
    if waypoints(1, 1) ~= 0 || any(diff(waypoints(:, 1)) <= 0)
        error('simulateDrone6DOF:InvalidWaypointTime', ...
            'Waypoint time must start at zero and increase strictly.');
    end

    duration = waypoints(end, 1);
    time = (0:settings.timeStep_s:duration).';
    if abs(time(end) - duration) > ...
            1e-9 * max(1, duration)
        error('simulateDrone6DOF:NonIntegralDuration', ...
            'The final waypoint time must be an integer number of time steps.');
    end
    interpolatedCommand = interp1(waypoints(:, 1), waypoints(:, 2:5), ...
        time, 'linear');
    commandAngles_rad = interpolatedCommand(:, 1:3) * pi / 180;
    altitudeCommand_m = interpolatedCommand(:, 4);
    altitudeCommandRate_m_s = gradient(altitudeCommand_m, ...
        settings.timeStep_s);

    rateSegments = settings.rateCommandSegments;
    validateattributes(rateSegments, {'numeric'}, ...
        {'2d', 'real', 'finite'}, mfilename, 'rateCommandSegments');
    if ~isempty(rateSegments) && size(rateSegments, 2) ~= 6
        error('simulateDrone6DOF:InvalidRateSegmentShape', ...
            ['rateCommandSegments must have six columns: start, end, ' ...
             'roll rate, pitch rate, yaw rate, and collective scale.']);
    end
    if isempty(rateSegments) && any(abs(waypoints(:, 2:4)) >= 180, 'all')
        error('simulateDrone6DOF:FlipNeedsRateMode', ...
            ['An attitude waypoint at or beyond +/-180 degrees is ' ...
             'ambiguous and cannot command a continuous flip. Define ' ...
             'rateCommandSegments for the rotation, then keep the ' ...
             'attitude waypoints at the desired post-flip orientation.']);
    end
    rateCommandFlight_deg_s = zeros(numel(time), 3);
    rateCollectiveWeightScale = nan(numel(time), 1);
    rateSegmentIndex = zeros(numel(time), 1);
    for segmentIndex = 1:size(rateSegments, 1)
        segmentStart = rateSegments(segmentIndex, 1);
        segmentEnd = rateSegments(segmentIndex, 2);
        collectiveScale = rateSegments(segmentIndex, 6);
        if segmentStart < 0 || segmentEnd <= segmentStart || ...
                segmentEnd > duration || collectiveScale < 0
            error('simulateDrone6DOF:InvalidRateSegment', ...
                ['Every rate segment must lie inside the simulation, have ' ...
                 'end > start, and use a nonnegative collective scale.']);
        end
        activeMask = time >= segmentStart & time < segmentEnd;
        if any(rateSegmentIndex(activeMask) ~= 0)
            error('simulateDrone6DOF:OverlappingRateSegments', ...
                'Rate-control segments cannot overlap.');
        end
        rateCommandFlight_deg_s(activeMask, :) = repmat( ...
            rateSegments(segmentIndex, 3:5), sum(activeMask), 1);
        rateCollectiveWeightScale(activeMask) = collectiveScale;
        rateSegmentIndex(activeMask) = segmentIndex;
    end
    rateModeActive = rateSegmentIndex > 0;
    rateCommandBody_rad_s = [rateCommandFlight_deg_s(:, 2), ...
        rateCommandFlight_deg_s(:, 1), rateCommandFlight_deg_s(:, 3)] * ...
        pi / 180;

    attitudeNaturalFrequency = expandThreeVector( ...
        settings.attitudeNaturalFrequency_rad_s, ...
        'attitudeNaturalFrequency_rad_s');
    attitudeDamping = expandThreeVector( ...
        settings.attitudeDampingRatio, 'attitudeDampingRatio');
    attitudeIntegralPole = expandThreeVector( ...
        settings.attitudeIntegralPole_rad_s, ...
        'attitudeIntegralPole_rad_s');
    maxAttitudeIntegral = expandThreeVector( ...
        settings.maxAttitudeIntegral_rad_s, ...
        'maxAttitudeIntegral_rad_s');
    rateControlBandwidth = expandThreeVector( ...
        settings.rateControlBandwidth_rad_s, ...
        'rateControlBandwidth_rad_s');
    if any(rateControlBandwidth <= 0)
        error('simulateDrone6DOF:InvalidRateBandwidth', ...
            'rateControlBandwidth_rad_s values must be positive.');
    end

    attitudeKp = zeros(3, 1);
    attitudeKi = zeros(3, 1);
    attitudeKd = zeros(3, 1);
    for axisIndex = 1:3
        controller = designAttitudePID( ...
            config.inertiaTensor(axisIndex, axisIndex), ...
            attitudeNaturalFrequency(axisIndex), ...
            attitudeDamping(axisIndex), ...
            attitudeIntegralPole(axisIndex));
        attitudeKp(axisIndex) = controller.Kp;
        attitudeKi(axisIndex) = controller.Ki;
        attitudeKd(axisIndex) = controller.Kd;
    end
    rateKp = diag(config.inertiaTensor) .* rateControlBandwidth;
    altitudeController = designAttitudePID(1, ...
        settings.altitudeNaturalFrequency_rad_s, ...
        settings.altitudeDampingRatio, ...
        settings.altitudeIntegralPole_rad_s);

    numberOfSamples = numel(time);
    numberOfMotors = config.numberOfMotors;
    timeStep = settings.timeStep_s;
    position = zeros(numberOfSamples, 3);
    velocity = zeros(numberOfSamples, 3);
    acceleration = zeros(numberOfSamples, 3);
    quaternion = zeros(numberOfSamples, 4);
    rotationBodyToWorld = zeros(3, 3, numberOfSamples);
    actualAngles_rad = zeros(numberOfSamples, 3);
    angularVelocity = zeros(numberOfSamples, 3);
    angularAcceleration = zeros(numberOfSamples, 3);
    attitudeError = zeros(numberOfSamples, 3);
    attitudeIntegral = zeros(numberOfSamples, 3);
    altitudeIntegral = zeros(numberOfSamples, 1);
    desiredWrench = zeros(numberOfSamples, 4);
    achievedWrench = zeros(numberOfSamples, 4);
    motorThrustCommand = zeros(numberOfSamples, numberOfMotors);
    motorThrust = zeros(numberOfSamples, numberOfMotors);
    allocationLimited = false(numberOfSamples, 1);
    allocationError = zeros(numberOfSamples, 1);
    verticalThrustProjection = zeros(numberOfSamples, 1);

    position(1, :) = settings.initialPosition_m(:).';
    velocity(1, :) = settings.initialVelocity_m_s(:).';
    angularVelocity(1, :) = settings.initialAngularVelocity_rad_s(:).';
    initialAttitude_rad = settings.initialAttitude_deg(:) * pi / 180;
    initialRotation = flightAnglesToRotation(initialAttitude_rad(1), ...
        initialAttitude_rad(2), initialAttitude_rad(3));
    quaternion(1, :) = rotationToQuaternion(initialRotation).';

    hoverAllocation = allocateMotorWrench(config, ...
        [config.weight; 0; 0; 0], settings.allocationPriority);
    if hoverAllocation.normalizedWeightedError > 1e-5
        error('simulateDrone6DOF:HoverAllocation', ...
            'A zero-moment four-axis hover allocation is infeasible.');
    end
    motorThrust(1, :) = hoverAllocation.motorThrust_N.';

    gravityWorld = [0; 0; -config.gravity];
    for sampleIndex = 1:numberOfSamples
        rotationNow = quaternionToRotation(quaternion(sampleIndex, :).');
        rotationBodyToWorld(:, :, sampleIndex) = rotationNow;
        actualAngles_rad(sampleIndex, :) = ...
            rotationToFlightAngles(rotationNow).';

        desiredRotation = flightAnglesToRotation( ...
            commandAngles_rad(sampleIndex, 1), ...
            commandAngles_rad(sampleIndex, 2), ...
            commandAngles_rad(sampleIndex, 3));
        relativeDesiredRotation = rotationNow.' * desiredRotation;
        attitudeErrorNow = 0.5 * [ ...
            relativeDesiredRotation(3, 2) - relativeDesiredRotation(2, 3); ...
            relativeDesiredRotation(1, 3) - relativeDesiredRotation(3, 1); ...
            relativeDesiredRotation(2, 1) - relativeDesiredRotation(1, 2)];
        attitudeError(sampleIndex, :) = attitudeErrorNow.';

        angularVelocityNow = angularVelocity(sampleIndex, :).';
        gyroscopicMoment = cross(angularVelocityNow, ...
            config.inertiaTensor * angularVelocityNow);
        if rateModeActive(sampleIndex)
            rateErrorNow = rateCommandBody_rad_s(sampleIndex, :).' - ...
                angularVelocityNow;
            requestedMoment = rateKp .* rateErrorNow + gyroscopicMoment;
        else
            requestedMoment = attitudeKp .* attitudeErrorNow + ...
                attitudeKi .* attitudeIntegral(sampleIndex, :).' - ...
                attitudeKd .* angularVelocityNow + gyroscopicMoment;
        end

        altitudeError = altitudeCommand_m(sampleIndex) - ...
            position(sampleIndex, 3);
        altitudeRateError = altitudeCommandRate_m_s(sampleIndex) - ...
            velocity(sampleIndex, 3);
        requestedVerticalAcceleration = ...
            altitudeController.Kp * altitudeError + ...
            altitudeController.Ki * altitudeIntegral(sampleIndex) + ...
            altitudeController.Kd * altitudeRateError;
        requestedVerticalAcceleration = min(max( ...
            requestedVerticalAcceleration, ...
            -settings.maxVerticalAcceleration_m_s2), ...
            settings.maxVerticalAcceleration_m_s2);

        verticalProjection = rotationNow(3, 3);
        verticalThrustProjection(sampleIndex) = verticalProjection;
        if rateModeActive(sampleIndex)
            requestedCollective = config.weight * ...
                rateCollectiveWeightScale(sampleIndex);
        else
            tiltCompensationProjection = max(verticalProjection, 0.25);
            requestedCollective = config.totalMass * ...
                (config.gravity + requestedVerticalAcceleration) / ...
                tiltCompensationProjection;
            requestedCollective = max(0, requestedCollective);
        end

        desiredWrenchNow = [requestedCollective; requestedMoment];
        allocation = allocateMotorWrench(config, desiredWrenchNow, ...
            settings.allocationPriority);
        desiredWrench(sampleIndex, :) = desiredWrenchNow.';
        motorThrustCommand(sampleIndex, :) = ...
            allocation.motorThrust_N.';
        allocationLimited(sampleIndex) = allocation.limited;
        allocationError(sampleIndex) = allocation.normalizedWeightedError;

        actualThrust = motorThrust(sampleIndex, :).';
        actualWrench = motorWrench4(config, actualThrust);
        achievedWrench(sampleIndex, :) = actualWrench.';
        forceWorld = rotationNow * [0; 0; actualWrench(1)];
        accelerationNow = forceWorld / config.totalMass + gravityWorld;
        angularAccelerationNow = config.inertiaTensor \ ...
            (actualWrench(2:4) - gyroscopicMoment);
        acceleration(sampleIndex, :) = accelerationNow.';
        angularAcceleration(sampleIndex, :) = angularAccelerationNow.';

        if sampleIndex < numberOfSamples
            if rateModeActive(sampleIndex)
                nextAttitudeIntegral = attitudeIntegral(sampleIndex, :).';
                nextAltitudeIntegral = altitudeIntegral(sampleIndex);
            elseif ~allocation.limited
                nextAttitudeIntegral = ...
                    attitudeIntegral(sampleIndex, :).' + ...
                    attitudeErrorNow * timeStep;
                nextAltitudeIntegral = altitudeIntegral(sampleIndex) + ...
                    altitudeError * timeStep;
            else
                nextAttitudeIntegral = attitudeIntegral(sampleIndex, :).';
                nextAltitudeIntegral = altitudeIntegral(sampleIndex);
            end
            nextAttitudeIntegral = min(max(nextAttitudeIntegral, ...
                -maxAttitudeIntegral), maxAttitudeIntegral);
            nextAltitudeIntegral = min(max(nextAltitudeIntegral, ...
                -settings.maxAltitudeIntegral_m_s), ...
                settings.maxAltitudeIntegral_m_s);
            attitudeIntegral(sampleIndex + 1, :) = ...
                nextAttitudeIntegral.';
            altitudeIntegral(sampleIndex + 1) = nextAltitudeIntegral;

            thrustRate = (allocation.motorThrust_N - actualThrust) / ...
                settings.motorTimeConstant_s;
            nextThrust = actualThrust + thrustRate * timeStep;
            nextThrust = min(max(nextThrust, 0), config.maxThrust);
            motorThrust(sampleIndex + 1, :) = nextThrust.';

            nextAngularVelocity = angularVelocityNow + ...
                angularAccelerationNow * timeStep;
            angularVelocity(sampleIndex + 1, :) = ...
                nextAngularVelocity.';
            quaternionDerivative = 0.5 * quaternionMultiply( ...
                quaternion(sampleIndex, :).', [0; angularVelocityNow]);
            nextQuaternion = quaternion(sampleIndex, :).' + ...
                quaternionDerivative * timeStep;
            nextQuaternion = nextQuaternion / norm(nextQuaternion);
            quaternion(sampleIndex + 1, :) = nextQuaternion.';

            position(sampleIndex + 1, :) = position(sampleIndex, :) + ...
                velocity(sampleIndex, :) * timeStep + ...
                0.5 * accelerationNow.' * timeStep^2;
            velocity(sampleIndex + 1, :) = velocity(sampleIndex, :) + ...
                accelerationNow.' * timeStep;
        end
    end

    actualAngles_rad(:, 3) = unwrap(actualAngles_rad(:, 3));
    simulation.time_s = time;
    simulation.commandAngles_rad = commandAngles_rad;
    simulation.commandAngles_deg = commandAngles_rad * 180 / pi;
    simulation.altitudeCommand_m = altitudeCommand_m;
    simulation.position_m = position;
    simulation.velocity_m_s = velocity;
    simulation.acceleration_m_s2 = acceleration;
    simulation.quaternionBodyToWorld = quaternion;
    simulation.rotationBodyToWorld = rotationBodyToWorld;
    simulation.actualAngles_rad = actualAngles_rad;
    simulation.actualAngles_deg = actualAngles_rad * 180 / pi;
    simulation.angularVelocity_rad_s = angularVelocity;
    simulation.angularAcceleration_rad_s2 = angularAcceleration;
    simulation.attitudeError_rad = attitudeError;
    simulation.attitudeIntegral_rad_s = attitudeIntegral;
    simulation.altitudeIntegral_m_s = altitudeIntegral;
    simulation.desiredWrench = desiredWrench;
    simulation.achievedWrench = achievedWrench;
    simulation.motorThrustCommand_N = motorThrustCommand;
    simulation.motorThrust_N = motorThrust;
    simulation.allocationLimited = allocationLimited;
    simulation.allocationError = allocationError;
    simulation.verticalThrustProjection = verticalThrustProjection;
    simulation.rateCommandFlight_deg_s = rateCommandFlight_deg_s;
    simulation.rateCommandBody_rad_s = rateCommandBody_rad_s;
    simulation.rateModeActive = rateModeActive;
    simulation.rateSegmentIndex = rateSegmentIndex;
    simulation.rateCollectiveWeightScale = rateCollectiveWeightScale;
    simulation.settings = settings;
    simulation.controller.attitudeKp = attitudeKp;
    simulation.controller.attitudeKi = attitudeKi;
    simulation.controller.attitudeKd = attitudeKd;
    simulation.controller.rateKp = rateKp;
    simulation.controller.altitude = altitudeController;
    simulation.motorNames = config.motorNames;

    rawAngleError = commandAngles_rad - actualAngles_rad;
    attitudeError_deg = atan2(sin(rawAngleError), cos(rawAngleError)) * ...
        180 / pi;
    attitudeHoldMask = ~rateModeActive;
    simulation.metrics.rmsAttitudeError_deg = ...
        sqrt(mean(attitudeError_deg(attitudeHoldMask, :).^2, 1));
    simulation.metrics.peakAttitudeError_deg = ...
        max(abs(attitudeError_deg(attitudeHoldMask, :)), [], 1);
    simulation.metrics.rmsAltitudeError_m = sqrt(mean( ...
        (altitudeCommand_m(attitudeHoldMask) - ...
        position(attitudeHoldMask, 3)).^2));
    simulation.metrics.allocationLimitedPercent = ...
        100 * mean(allocationLimited);
    simulation.metrics.rateModePercent = 100 * mean(rateModeActive);
    simulation.metrics.maximumSpeed_m_s = ...
        max(vecnorm(velocity, 2, 2));
    actualFlightRate_deg_s = [angularVelocity(:, 2), ...
        angularVelocity(:, 1), angularVelocity(:, 3)] * 180 / pi;
    simulation.actualFlightRate_deg_s = actualFlightRate_deg_s;
    simulation.cumulativeFlightRotation_deg = cumtrapz(time, ...
        actualFlightRate_deg_s);
    simulation.rateModeCumulativeActualRotation_deg = cumtrapz(time, ...
        actualFlightRate_deg_s .* rateModeActive);
    simulation.rateModeCumulativeCommandRotation_deg = cumtrapz(time, ...
        rateCommandFlight_deg_s);
    segmentMetrics = repmat(struct('start_s', 0, 'end_s', 0, ...
        'desiredRotation_deg', zeros(1, 3), ...
        'actualRotation_deg', zeros(1, 3)), size(rateSegments, 1), 1);
    for segmentIndex = 1:size(rateSegments, 1)
        segmentMask = time >= rateSegments(segmentIndex, 1) & ...
            time <= rateSegments(segmentIndex, 2);
        segmentMetrics(segmentIndex).start_s = ...
            rateSegments(segmentIndex, 1);
        segmentMetrics(segmentIndex).end_s = ...
            rateSegments(segmentIndex, 2);
        segmentMetrics(segmentIndex).desiredRotation_deg = ...
            rateSegments(segmentIndex, 3:5) * ...
            (rateSegments(segmentIndex, 2) - rateSegments(segmentIndex, 1));
        segmentMetrics(segmentIndex).actualRotation_deg = trapz( ...
            time(segmentMask), actualFlightRate_deg_s(segmentMask, :), 1);
    end
    simulation.metrics.rateSegments = segmentMetrics;
    if isempty(segmentMetrics)
        simulation.metrics.totalDesiredRateModeRotation_deg = zeros(1, 3);
    else
        simulation.metrics.totalDesiredRateModeRotation_deg = sum( ...
            vertcat(segmentMetrics.desiredRotation_deg), 1);
    end
    simulation.metrics.totalActualRateModeRotation_deg = ...
        simulation.rateModeCumulativeActualRotation_deg(end, :);

    fprintf('\nFull 6DOF programmed maneuver\n');
    fprintf('  RMS [roll pitch yaw] error: [%.2f %.2f %.2f] deg\n', ...
        simulation.metrics.rmsAttitudeError_deg);
    fprintf('  RMS altitude error: %.3f m\n', ...
        simulation.metrics.rmsAltitudeError_m);
    fprintf('  Allocation limited: %.2f %% of samples\n', ...
        simulation.metrics.allocationLimitedPercent);
    fprintf('  Final position [x y z]: [%.3f %.3f %.3f] m\n', ...
        position(end, :));
    fprintf('  Maximum speed: %.3f m/s\n', ...
        simulation.metrics.maximumSpeed_m_s);
    for segmentIndex = 1:numel(segmentMetrics)
        fprintf(['  Rate segment %d desired/actual [roll pitch yaw]: ' ...
            '[%.1f %.1f %.1f] / [%.1f %.1f %.1f] deg\n'], ...
            segmentIndex, segmentMetrics(segmentIndex).desiredRotation_deg, ...
            segmentMetrics(segmentIndex).actualRotation_deg);
    end
    if ~isempty(segmentMetrics)
        fprintf(['  Total rate-mode actual [roll pitch yaw]: ' ...
            '[%.1f %.1f %.1f] deg\n'], ...
            simulation.metrics.totalActualRateModeRotation_deg);
    end
end

function validateThreeState(value, argumentName)
    validateattributes(value, {'numeric'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, mfilename, argumentName);
end

function vector = expandThreeVector(value, argumentName)
    validateattributes(value, {'numeric'}, ...
        {'vector', 'real', 'finite', 'nonnegative'}, mfilename, argumentName);
    if isscalar(value)
        vector = repmat(value, 3, 1);
    else
        vector = value(:);
    end
    if numel(vector) ~= 3
        error('simulateDrone6DOF:VectorSize', ...
            '%s must be scalar or contain three values.', argumentName);
    end
end

function product = quaternionMultiply(first, second)
    firstScalar = first(1);
    firstVector = first(2:4);
    secondScalar = second(1);
    secondVector = second(2:4);
    product = [firstScalar * secondScalar - dot(firstVector, secondVector); ...
        firstScalar * secondVector + secondScalar * firstVector + ...
        cross(firstVector, secondVector)];
end
