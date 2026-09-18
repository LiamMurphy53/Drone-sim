function trajectory = simulateRollTrajectory3D(config, rollSimulation)
%SIMULATEROLLTRAJECTORY3D Add 3D translation to the roll PID result.
%
% The roll angle still comes from the one-axis attitude simulation. This
% function rotates the actual total motor thrust into the world frame, adds
% gravity, and integrates the center-of-mass position and velocity.

    time = rollSimulation.time_s(:);
    angle = rollSimulation.angle_rad(:);
    motorThrust = rollSimulation.motorThrust_N;
    numberOfSamples = numel(time);

    if numel(angle) ~= numberOfSamples || ...
            size(motorThrust, 1) ~= numberOfSamples
        error('simulateRollTrajectory3D:SignalSize', ...
            'Time, angle, and motor-thrust signals must have equal lengths.');
    end
    if size(motorThrust, 2) ~= config.numberOfMotors
        error('simulateRollTrajectory3D:MotorCount', ...
            'The motor-thrust signal does not match the configured motor count.');
    end

    position = zeros(numberOfSamples, 3);
    velocity = zeros(numberOfSamples, 3);
    acceleration = zeros(numberOfSamples, 3);
    thrustWorld = zeros(numberOfSamples, 3);

    gravityWorld = [0; 0; -config.gravity];
    for sampleIndex = 1:numberOfSamples
        rotationBodyToWorld = yAxisRotation(angle(sampleIndex));
        totalThrust = sum(motorThrust(sampleIndex, :));
        thrustWorldNow = rotationBodyToWorld * [0; 0; totalThrust];
        accelerationNow = thrustWorldNow / config.totalMass + gravityWorld;

        thrustWorld(sampleIndex, :) = thrustWorldNow.';
        acceleration(sampleIndex, :) = accelerationNow.';

        if sampleIndex < numberOfSamples
            timeStep = time(sampleIndex + 1) - time(sampleIndex);
            if timeStep <= 0
                error('simulateRollTrajectory3D:InvalidTime', ...
                    'Simulation time must increase monotonically.');
            end
            position(sampleIndex + 1, :) = position(sampleIndex, :) + ...
                velocity(sampleIndex, :) * timeStep + ...
                0.5 * acceleration(sampleIndex, :) * timeStep^2;
            velocity(sampleIndex + 1, :) = velocity(sampleIndex, :) + ...
                acceleration(sampleIndex, :) * timeStep;
        end
    end

    trajectory.time_s = time;
    trajectory.position_m = position;
    trajectory.velocity_m_s = velocity;
    trajectory.acceleration_m_s2 = acceleration;
    trajectory.thrustWorld_N = thrustWorld;
    trajectory.angle_rad = angle;

    finalPosition = position(end, :);
    fprintf('\n3D roll trajectory\n');
    fprintf('  Final displacement [x y z]: [%.3f %.3f %.3f] m\n', ...
        finalPosition);
    fprintf('  Lowest relative altitude: %.3f m\n', min(position(:, 3)));
end

function rotation = yAxisRotation(angle)
    cosine = cos(angle);
    sine = sin(angle);
    rotation = [cosine, 0, sine; 0, 1, 0; -sine, 0, cosine];
end
