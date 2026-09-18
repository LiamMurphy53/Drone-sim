function plotDrone6DOFAxisComparison(responses)
%PLOTDRONE6DOFAXISCOMPARISON Compare three views of the same 6DOF flight.

    if ~iscell(responses) || numel(responses) ~= 3
        error('plotDrone6DOFAxisComparison:InvalidInput', ...
            'Provide {roll, pitch, yaw} views from one 6DOF result.');
    end
    motionNames = cellfun(@(item) item.motionName, responses, ...
        'UniformOutput', false);
    colors = lines(3);

    figure('Color', 'w', ...
        'Name', 'Programmed 6DOF roll, pitch, and yaw comparison', ...
        'Position', [100, 80, 1200, 760]);
    subplot(2, 2, 1);
    hold on;
    legendEntries = cell(1, 6);
    for index = 1:3
        item = responses{index};
        commandAngle = wrapDegrees(item.angleCommand_deg);
        actualAngle = wrapDegrees(item.angleActual_deg);
        actualAngle(item.rateModeActive) = NaN;
        plot(item.time_s, commandAngle, '--', ...
            'Color', colors(index, :), 'LineWidth', 1.0);
        plot(item.time_s, actualAngle, ...
            'Color', colors(index, :), 'LineWidth', 1.4);
        legendEntries{2 * index - 1} = [motionNames{index}, ' command'];
        legendEntries{2 * index} = [motionNames{index}, ' actual'];
    end
    grid on;
    ylabel('Angle (deg)');
    title('Attitude-mode portions of the same maneuver');
    legend(legendEntries, 'Location', 'best');

    subplot(2, 2, 2);
    hold on;
    legendEntries = cell(1, 6);
    for index = 1:3
        item = responses{index};
        plot(item.time_s, item.rateCommand_deg_s, '--', ...
            'Color', colors(index, :), 'LineWidth', 1.0);
        plot(item.time_s, item.actualFlightRate_deg_s, ...
            'Color', colors(index, :), 'LineWidth', 1.4);
        legendEntries{2 * index - 1} = [motionNames{index}, ' command'];
        legendEntries{2 * index} = [motionNames{index}, ' actual'];
    end
    grid on;
    ylabel('Rate (deg/s)');
    title('Axis rates throughout the same maneuver');
    legend(legendEntries, 'Location', 'best');

    peakRates = cellfun(@(item) item.metrics.peakRate_deg_s, responses);
    subplot(2, 2, 3);
    bar(peakRates, 'FaceColor', [0.20, 0.55, 0.75]);
    grid on;
    ylabel('Peak absolute rate (deg/s)');
    title('Observed rate demand and response');
    applyLabels(motionNames);

    rmsErrors = cellfun(@(item) ...
        item.metrics.rmsAttitudeHoldError_deg, responses);
    subplot(2, 2, 4);
    bar(rmsErrors, 'FaceColor', [0.85, 0.40, 0.18]);
    grid on;
    ylabel('RMS angle error (deg)');
    title('Attitude-hold tracking outside rate mode');
    applyLabels(motionNames);

    sgtitle(responses{1}.maneuverName);

    fprintf('\nProgrammed 6DOF axis comparison (one shared flight)\n');
    fprintf(['  Motion  Axis  Peak rate  RMS hold error  Rate-mode rotation  ' ...
        'Moment RMS error\n']);
    fprintf(['                 (deg/s)       (deg)             (deg)         ' ...
        '(N*m)\n']);
    for index = 1:3
        item = responses{index};
        fprintf('  %-6s   %s    %8.1f       %8.2f        %10.1f       %9.4f\n', ...
            item.motionName, item.axisName, ...
            item.metrics.peakRate_deg_s, ...
            item.metrics.rmsAttitudeHoldError_deg, ...
            item.metrics.actualRateModeRotation_deg, ...
            item.metrics.momentTrackingRMSError_Nm);
    end
    fprintf('  Shared allocation-limited samples: %.2f %%\n', ...
        responses{1}.metrics.allocationLimitedPercent);
end

function applyLabels(labels)
    axesHandle = gca;
    axesHandle.XTick = 1:numel(labels);
    axesHandle.XTickLabel = labels;
end

function wrapped = wrapDegrees(angle_deg)
    wrapped = atan2(sin(angle_deg * pi / 180), ...
        cos(angle_deg * pi / 180)) * 180 / pi;
end
