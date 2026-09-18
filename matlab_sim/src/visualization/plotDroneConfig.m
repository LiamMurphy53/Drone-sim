function plotDroneConfig(config)
%PLOTDRONECONFIG Plot motor locations and label maximum thrust.

    locations = config.motorLocations_mm;
    centerOfMass = config.centerOfMass_mm;
    maxThrust = config.maxThrust;

    figure('Color', 'w', 'Name', 'Drone motor layout');
    hold on;
    grid on;
    axis equal;

    for motorIndex = 1:config.numberOfMotors
        plot([centerOfMass(1), locations(motorIndex, 1)], ...
             [centerOfMass(2), locations(motorIndex, 2)], '-', ...
             'Color', [0.65, 0.65, 0.65], 'LineWidth', 1.5);
    end

    scatter(locations(:, 1), locations(:, 2), 180, ...
        'filled', 'MarkerFaceColor', [0.12, 0.47, 0.71], ...
        'MarkerEdgeColor', [0.15, 0.15, 0.15]);
    scatter(centerOfMass(1), centerOfMass(2), 100, 'k', 'x', 'LineWidth', 2);
    text(centerOfMass(1), centerOfMass(2), '  Center of mass', ...
        'VerticalAlignment', 'bottom', 'FontWeight', 'bold');

    for motorIndex = 1:config.numberOfMotors
        if isfield(config, 'motorSpinDirection') && ...
                ~strcmp(config.motorSpinDirection{motorIndex}, 'unknown')
            label = sprintf('  %s %s (%.1f N)', ...
                config.motorNames{motorIndex}, ...
                config.motorSpinDirection{motorIndex}, ...
                maxThrust(motorIndex));
        else
            label = sprintf('  %s (%.1f N)', ...
                config.motorNames{motorIndex}, maxThrust(motorIndex));
        end
        text(locations(motorIndex, 1), locations(motorIndex, 2), label, ...
            'FontWeight', 'bold');
    end

    xlabel('x: left (mm)');
    ylabel('y: backward (mm)');
    title('Motor layout and center of mass');

    padding = 0.08 * max(1, max(abs(locations(:, 1:2)), [], 'all'));
    xlim([min([locations(:, 1); centerOfMass(1)]) - padding, ...
          max([locations(:, 1); centerOfMass(1)]) + padding]);
    ylim([min([locations(:, 2); centerOfMass(2)]) - padding, ...
          max([locations(:, 2); centerOfMass(2)]) + padding]);
end
