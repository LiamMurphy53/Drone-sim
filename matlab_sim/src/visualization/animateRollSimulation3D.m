function animateRollSimulation3D(config, rollSimulation, trajectory, ...
    playbackSpeed)
%ANIMATEROLLSIMULATION3D Animate the drone geometry and 3D trajectory.

% playbackSpeed = 1 plays in real time; 2 plays twice as fast.

    if nargin < 3 || isempty(trajectory)
        trajectory = simulateRollTrajectory3D(config, rollSimulation);
    end
    if nargin < 4
        playbackSpeed = 1;
    end
    validateattributes(playbackSpeed, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'}, mfilename, 'playbackSpeed');

    time = trajectory.time_s;
    position = trajectory.position_m;
    angle = rollSimulation.angle_rad;
    angleCommand = rollSimulation.angleCommand_rad;
    motorThrust = rollSimulation.motorThrust_N;
    motorLeverArms = config.motorLeverArms;
    numberOfMotors = config.numberOfMotors;
    if isfield(config, 'motorNames') && ...
            numel(config.motorNames) == numberOfMotors
        motorNames = config.motorNames;
    else
        motorNames = arrayfun(@(index) sprintf('M%d', index), ...
            1:numberOfMotors, 'UniformOutput', false).';
    end

    typicalTimeStep = median(diff(time));
    targetFrameRate = 30;
    samplesPerFrame = max(1, round(1 / ...
        (targetFrameRate * typicalTimeStep)));
    frameIndices = unique([1:samplesPerFrame:numel(time), numel(time)]);

    armLength = max(vecnorm(motorLeverArms, 2, 2));
    cameraHalfWidth = max(0.45, 3.5 * armLength);
    bodyAxisLength = max(0.12, 0.9 * armLength);
    thrustArrowScale_m_per_N = max(0.004, armLength / ...
        (2 * max(config.maxThrust)));

    figureHandle = figure('Color', 'w', 'Name', '3D drone simulation');
    axesHandle = axes(figureHandle);
    hold(axesHandle, 'on');
    grid(axesHandle, 'on');
    axis(axesHandle, 'equal');
    view(axesHandle, 38, 24);
    xlabel(axesHandle, 'x: left (m)');
    ylabel(axesHandle, 'y: backward (m)');
    zlabel(axesHandle, 'z: up (m)');

    centerHandle = scatter3(axesHandle, 0, 0, 0, 90, 'k', 'filled');
    motorHandle = scatter3(axesHandle, zeros(numberOfMotors, 1), ...
        zeros(numberOfMotors, 1), zeros(numberOfMotors, 1), 150, ...
        'filled', 'MarkerFaceColor', [0.12, 0.47, 0.71], ...
        'MarkerEdgeColor', [0.1, 0.1, 0.1]);
    armHandles = gobjects(numberOfMotors, 1);
    labelHandles = gobjects(numberOfMotors, 1);
    for motorIndex = 1:numberOfMotors
        armHandles(motorIndex) = plot3(axesHandle, [0, 0], [0, 0], [0, 0], ...
            '-', 'Color', [0.25, 0.25, 0.25], 'LineWidth', 3);
        labelHandles(motorIndex) = text(axesHandle, 0, 0, 0, ...
            motorNames{motorIndex}, 'FontWeight', 'bold');
    end

    thrustHandle = quiver3(axesHandle, zeros(numberOfMotors, 1), ...
        zeros(numberOfMotors, 1), zeros(numberOfMotors, 1), ...
        zeros(numberOfMotors, 1), zeros(numberOfMotors, 1), ...
        zeros(numberOfMotors, 1), 0, 'Color', [0.85, 0.2, 0.15], ...
        'LineWidth', 1.6, 'MaxHeadSize', 0.7);
    xAxisHandle = quiver3(axesHandle, 0, 0, 0, 0, 0, 0, 0, ...
        'Color', [0.85, 0.15, 0.15], 'LineWidth', 2);
    yAxisHandle = quiver3(axesHandle, 0, 0, 0, 0, 0, 0, 0, ...
        'Color', [0.15, 0.65, 0.2], 'LineWidth', 2);
    zAxisHandle = quiver3(axesHandle, 0, 0, 0, 0, 0, 0, 0, ...
        'Color', [0.15, 0.3, 0.9], 'LineWidth', 2);
    trailHandle = plot3(axesHandle, 0, 0, 0, '-', ...
        'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.2);

    for frameNumber = 1:numel(frameIndices)
        if ~isvalid(figureHandle)
            return;
        end

        sampleIndex = frameIndices(frameNumber);
        center = position(sampleIndex, :).';
        rotationBodyToWorld = yAxisRotation(angle(sampleIndex));
        motorWorld = center + rotationBodyToWorld * motorLeverArms.';
        thrustDirection = rotationBodyToWorld * [0; 0; 1];
        thrustVectors = thrustDirection * ...
            (motorThrust(sampleIndex, :) * thrustArrowScale_m_per_N);

        set(centerHandle, 'XData', center(1), 'YData', center(2), ...
            'ZData', center(3));
        set(motorHandle, 'XData', motorWorld(1, :), ...
            'YData', motorWorld(2, :), 'ZData', motorWorld(3, :));

        for motorIndex = 1:numberOfMotors
            set(armHandles(motorIndex), ...
                'XData', [center(1), motorWorld(1, motorIndex)], ...
                'YData', [center(2), motorWorld(2, motorIndex)], ...
                'ZData', [center(3), motorWorld(3, motorIndex)]);
            set(labelHandles(motorIndex), ...
                'Position', motorWorld(:, motorIndex).' + [0.015, 0.015, 0.015]);
        end

        set(thrustHandle, 'XData', motorWorld(1, :), ...
            'YData', motorWorld(2, :), 'ZData', motorWorld(3, :), ...
            'UData', thrustVectors(1, :), ...
            'VData', thrustVectors(2, :), ...
            'WData', thrustVectors(3, :));

        bodyAxes = rotationBodyToWorld * (bodyAxisLength * eye(3));
        updateAxisArrow(xAxisHandle, center, bodyAxes(:, 1));
        updateAxisArrow(yAxisHandle, center, bodyAxes(:, 2));
        updateAxisArrow(zAxisHandle, center, bodyAxes(:, 3));

        trailStart = max(1, sampleIndex - round(2 / typicalTimeStep));
        set(trailHandle, ...
            'XData', position(trailStart:sampleIndex, 1), ...
            'YData', position(trailStart:sampleIndex, 2), ...
            'ZData', position(trailStart:sampleIndex, 3));

        xlim(axesHandle, center(1) + [-cameraHalfWidth, cameraHalfWidth]);
        ylim(axesHandle, center(2) + [-cameraHalfWidth, cameraHalfWidth]);
        zlim(axesHandle, center(3) + [-cameraHalfWidth, cameraHalfWidth]);
        title(axesHandle, sprintf( ...
            't = %.2f s   command = %.1f deg   actual = %.1f deg', ...
            time(sampleIndex), angleCommand(sampleIndex) * 180 / pi, ...
            angle(sampleIndex) * 180 / pi));

        drawnow;
        if frameNumber < numel(frameIndices)
            frameTime = time(frameIndices(frameNumber + 1)) - ...
                time(sampleIndex);
            pause(max(0, frameTime / playbackSpeed));
        end
    end
end

function rotation = yAxisRotation(angle)
    cosine = cos(angle);
    sine = sin(angle);
    rotation = [cosine, 0, sine; 0, 1, 0; -sine, 0, cosine];
end

function updateAxisArrow(arrowHandle, origin, vector)
    set(arrowHandle, 'XData', origin(1), 'YData', origin(2), ...
        'ZData', origin(3), 'UData', vector(1), ...
        'VData', vector(2), 'WData', vector(3));
end
