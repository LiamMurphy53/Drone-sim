function plotDrone6DOF(simulation)
%PLOTDRONE6DOF Plot commands, full-state response, and motor activity.

    time = simulation.time_s;
    command = wrapDegrees(simulation.commandAngles_deg);
    actual = wrapDegrees(simulation.actualAngles_deg);
    if any(simulation.rateModeActive)
        actual(simulation.rateModeActive, :) = NaN;
    end
    angleNames = {'Roll: left side down (+My)', ...
        'Pitch: nose down (+Mx)', 'Yaw: nose left (+Mz)'};

    figure('Color', 'w', 'Name', 'Full 6DOF programmed maneuver');
    for angleIndex = 1:3
        subplot(3, 2, angleIndex);
        plot(time, command(:, angleIndex), '--', 'LineWidth', 1.3);
        hold on;
        plot(time, actual(:, angleIndex), 'LineWidth', 1.5);
        grid on;
        ylabel('Angle (deg)');
        if any(simulation.rateModeActive)
            title([angleNames{angleIndex}, ' (attitude mode only)']);
        else
            title(angleNames{angleIndex});
        end
        legend('Command', 'Actual', 'Location', 'best');
    end

    subplot(3, 2, 4);
    plot(time, simulation.altitudeCommand_m, '--', 'LineWidth', 1.3);
    hold on;
    plot(time, simulation.position_m(:, 3), 'LineWidth', 1.5);
    grid on;
    ylabel('World z (m)');
    title('Altitude hold');
    legend('Command', 'Actual', 'Location', 'best');

    subplot(3, 2, 5);
    colorOrder = get(gca, 'ColorOrder');
    hold on;
    for axisIndex = 1:3
        plot(time, simulation.actualFlightRate_deg_s(:, axisIndex), ...
            'Color', colorOrder(axisIndex, :), 'LineWidth', 1.2);
    end
    for axisIndex = 1:3
        plot(time, simulation.rateCommandFlight_deg_s(:, axisIndex), '--', ...
            'Color', colorOrder(axisIndex, :), 'LineWidth', 1.0);
    end
    grid on;
    ylabel('Rate (deg/s)');
    xlabel('Time (s)');
    title('Flight rates; dashed lines are rate-mode commands');
    legend('Actual roll', 'Actual pitch', 'Actual yaw', ...
        'Command roll', 'Command pitch', 'Command yaw', ...
        'Location', 'best');

    subplot(3, 2, 6);
    plot(time, simulation.motorThrust_N, 'LineWidth', 1.15);
    hold on;
    limitedIndices = find(simulation.allocationLimited);
    if ~isempty(limitedIndices)
        yLimitMarker = max(simulation.motorThrust_N, [], 2);
        plot(time(limitedIndices), yLimitMarker(limitedIndices), '.', ...
            'Color', [0.85, 0.15, 0.15], 'MarkerSize', 6);
    end
    grid on;
    ylabel('Thrust (N)');
    xlabel('Time (s)');
    title(sprintf('Motor thrust; allocation limited %.1f%%', ...
        simulation.metrics.allocationLimitedPercent));
    legendEntries = simulation.motorNames(:).';
    if ~isempty(limitedIndices)
        legendEntries{end + 1} = 'Allocation limited';
    end
    legend(legendEntries, 'Location', 'best');

    if any(simulation.rateModeActive)
        figure('Color', 'w', 'Name', 'Rate-mode maneuver detail');
        subplot(2, 1, 1);
        hold on;
        for axisIndex = 1:3
            plot(time, simulation.actualFlightRate_deg_s(:, axisIndex), ...
                'Color', colorOrder(axisIndex, :), 'LineWidth', 1.3);
            plot(time, simulation.rateCommandFlight_deg_s(:, axisIndex), ...
                '--', 'Color', colorOrder(axisIndex, :), 'LineWidth', 1.1);
        end
        grid on;
        ylabel('Rate (deg/s)');
        title('Rate-mode tracking');
        legend('Actual roll', 'Command roll', 'Actual pitch', ...
            'Command pitch', 'Actual yaw', 'Command yaw', ...
            'Location', 'best');

        subplot(2, 1, 2);
        hold on;
        for axisIndex = 1:3
            plot(time, ...
                simulation.rateModeCumulativeActualRotation_deg(:, axisIndex), ...
                'Color', colorOrder(axisIndex, :), 'LineWidth', 1.3);
            plot(time, ...
                simulation.rateModeCumulativeCommandRotation_deg(:, axisIndex), ...
                '--', 'Color', colorOrder(axisIndex, :), 'LineWidth', 1.1);
        end
        grid on;
        ylabel('Rotation during rate mode (deg)');
        xlabel('Time (s)');
        title('Cumulative flip progress');
        legend('Actual roll', 'Command roll', 'Actual pitch', ...
            'Command pitch', 'Actual yaw', 'Command yaw', ...
            'Location', 'best');
    end
end

function wrapped = wrapDegrees(angle_deg)
    wrapped = atan2(sin(angle_deg * pi / 180), ...
        cos(angle_deg * pi / 180)) * 180 / pi;
end
