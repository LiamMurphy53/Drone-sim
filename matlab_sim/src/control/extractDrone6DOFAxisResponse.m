function response = extractDrone6DOFAxisResponse(simulation, motion)
%EXTRACTDRONE6DOFAXISRESPONSE Focus one axis of a coupled 6DOF result.
%
% Roll, pitch, and yaw use the flight-angle convention exposed to the user.
% With +x left and +y backward, flight roll maps to body y / My, flight
% pitch maps to body x / Mx, and yaw maps to body z / Mz.

    requiredFields = {'time_s', 'commandAngles_deg', 'actualAngles_deg', ...
        'actualFlightRate_deg_s', 'rateCommandFlight_deg_s', ...
        'rateModeActive', 'rateModeCumulativeActualRotation_deg', ...
        'rateModeCumulativeCommandRotation_deg', 'desiredWrench', ...
        'achievedWrench', 'motorThrust_N', 'motorNames', ...
        'allocationLimited'};
    missingFields = requiredFields(~isfield(simulation, requiredFields));
    if ~isempty(missingFields)
        error('extractDrone6DOFAxisResponse:MissingField', ...
            'The 6DOF result is missing %s.', missingFields{1});
    end

    motionNames = {'Roll', 'Pitch', 'Yaw'};
    axisNames = {'y', 'x', 'z'};
    momentNames = {'My', 'Mx', 'Mz'};
    bodyAxisIndices = [2, 1, 3];
    wrenchIndices = [3, 2, 4];

    motionText = validatestring(motion, motionNames, mfilename, 'motion');
    flightAxisIndex = find(strcmpi(motionText, motionNames), 1);
    bodyAxisIndex = bodyAxisIndices(flightAxisIndex);
    wrenchIndex = wrenchIndices(flightAxisIndex);

    response.motionName = motionNames{flightAxisIndex};
    response.axisName = axisNames{flightAxisIndex};
    response.momentName = momentNames{flightAxisIndex};
    response.flightAxisIndex = flightAxisIndex;
    response.bodyAxisIndex = bodyAxisIndex;
    response.wrenchIndex = wrenchIndex;
    response.time_s = simulation.time_s;
    response.angleCommand_deg = ...
        simulation.commandAngles_deg(:, flightAxisIndex);
    response.angleActual_deg = ...
        simulation.actualAngles_deg(:, flightAxisIndex);
    response.actualFlightRate_deg_s = ...
        simulation.actualFlightRate_deg_s(:, flightAxisIndex);
    response.rateCommand_deg_s = ...
        simulation.rateCommandFlight_deg_s(:, flightAxisIndex);
    response.rateModeActive = simulation.rateModeActive;
    response.cumulativeActualRateModeRotation_deg = ...
        simulation.rateModeCumulativeActualRotation_deg(:, flightAxisIndex);
    response.cumulativeCommandRateModeRotation_deg = ...
        simulation.rateModeCumulativeCommandRotation_deg(:, flightAxisIndex);
    response.desiredMoment_Nm = simulation.desiredWrench(:, wrenchIndex);
    response.achievedMoment_Nm = simulation.achievedWrench(:, wrenchIndex);
    response.motorThrust_N = simulation.motorThrust_N;
    response.motorNames = simulation.motorNames;
    response.allocationLimited = simulation.allocationLimited;
    if isfield(simulation, 'settings') && ...
            isfield(simulation.settings, 'maneuverName')
        response.maneuverName = simulation.settings.maneuverName;
    else
        response.maneuverName = 'Programmed 6DOF maneuver';
    end

    angleError_deg = wrapDegrees( ...
        response.angleCommand_deg - response.angleActual_deg);
    attitudeHoldMask = ~response.rateModeActive;
    if any(attitudeHoldMask)
        response.metrics.rmsAttitudeHoldError_deg = sqrt(mean( ...
            angleError_deg(attitudeHoldMask).^2));
        response.metrics.peakAttitudeHoldError_deg = max(abs( ...
            angleError_deg(attitudeHoldMask)));
    else
        response.metrics.rmsAttitudeHoldError_deg = NaN;
        response.metrics.peakAttitudeHoldError_deg = NaN;
    end
    if any(response.rateModeActive)
        rateError_deg_s = response.rateCommand_deg_s - ...
            response.actualFlightRate_deg_s;
        response.metrics.rmsRateModeError_deg_s = sqrt(mean( ...
            rateError_deg_s(response.rateModeActive).^2));
    else
        response.metrics.rmsRateModeError_deg_s = NaN;
    end
    response.metrics.peakRate_deg_s = ...
        max(abs(response.actualFlightRate_deg_s));
    response.metrics.peakRequestedMoment_Nm = ...
        max(abs(response.desiredMoment_Nm));
    response.metrics.peakAchievedMoment_Nm = ...
        max(abs(response.achievedMoment_Nm));
    response.metrics.momentTrackingRMSError_Nm = sqrt(mean( ...
        (response.desiredMoment_Nm - response.achievedMoment_Nm).^2));
    response.metrics.actualRateModeRotation_deg = ...
        response.cumulativeActualRateModeRotation_deg(end);
    response.metrics.commandRateModeRotation_deg = ...
        response.cumulativeCommandRateModeRotation_deg(end);
    response.metrics.allocationLimitedPercent = ...
        100 * mean(response.allocationLimited);
end

function wrapped = wrapDegrees(angle_deg)
    wrapped = atan2(sin(angle_deg * pi / 180), ...
        cos(angle_deg * pi / 180)) * 180 / pi;
end
