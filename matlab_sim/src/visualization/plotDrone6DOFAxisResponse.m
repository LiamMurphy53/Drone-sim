function plotDrone6DOFAxisResponse(response)
%PLOTDRONE6DOFAXISRESPONSE Plot one view of a programmed 6DOF maneuver.

    time = response.time_s;
    commandAngle = wrapDegrees(response.angleCommand_deg);
    actualAngle = wrapDegrees(response.angleActual_deg);
    actualAngle(response.rateModeActive) = NaN;

    figure('Color', 'w', ...
        'Name', sprintf('%s view of programmed 6DOF maneuver', ...
        response.motionName), 'Position', [100, 40, 1050, 940]);

    subplot(5, 1, 1);
    plot(time, commandAngle, '--', 'LineWidth', 1.3);
    hold on;
    plot(time, actualAngle, 'LineWidth', 1.5);
    grid on;
    ylabel('Angle (deg)');
    title(sprintf('%s: body %s / %s (attitude mode)', ...
        response.motionName, response.axisName, response.momentName));
    legend('Command', 'Actual', 'Location', 'best');

    subplot(5, 1, 2);
    plot(time, response.rateCommand_deg_s, '--', 'LineWidth', 1.3);
    hold on;
    plot(time, response.actualFlightRate_deg_s, 'LineWidth', 1.5);
    grid on;
    ylabel('Rate (deg/s)');
    title('Rate response; dashed command is active during rate mode');
    legend('Command', 'Actual', 'Location', 'best');

    subplot(5, 1, 3);
    plot(time, response.cumulativeCommandRateModeRotation_deg, '--', ...
        'LineWidth', 1.3);
    hold on;
    plot(time, response.cumulativeActualRateModeRotation_deg, ...
        'LineWidth', 1.5);
    grid on;
    ylabel('Rotation (deg)');
    title('Cumulative rotation during rate-mode segments');
    legend('Command', 'Actual', 'Location', 'best');

    subplot(5, 1, 4);
    plot(time, response.desiredMoment_Nm, '--', 'LineWidth', 1.3);
    hold on;
    plot(time, response.achievedMoment_Nm, 'LineWidth', 1.4);
    grid on;
    ylabel(sprintf('%s (N m)', response.momentName));
    title('Controller moment demand and achieved motor moment');
    legend('Demand', 'Achieved', 'Location', 'best');

    subplot(5, 1, 5);
    plot(time, response.motorThrust_N, 'LineWidth', 1.1);
    hold on;
    limitedIndices = find(response.allocationLimited);
    if ~isempty(limitedIndices)
        markerHeight = max(response.motorThrust_N, [], 2);
        plot(time(limitedIndices), markerHeight(limitedIndices), '.', ...
            'Color', [0.85, 0.15, 0.15], 'MarkerSize', 6);
    end
    grid on;
    ylabel('Thrust (N)');
    xlabel('Time (s)');
    title(sprintf('Shared 6DOF motor thrust; allocation limited %.2f%%', ...
        response.metrics.allocationLimitedPercent));
    legendEntries = response.motorNames(:).';
    if ~isempty(limitedIndices)
        legendEntries{end + 1} = 'Allocation limited';
    end
    legend(legendEntries, 'Location', 'best');

    sgtitle(response.maneuverName);
end

function wrapped = wrapDegrees(angle_deg)
    wrapped = atan2(sin(angle_deg * pi / 180), ...
        cos(angle_deg * pi / 180)) * 180 / pi;
end
