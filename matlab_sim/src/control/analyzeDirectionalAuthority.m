function authority = analyzeDirectionalAuthority(config, hover, ...
    numberOfDirections)
%ANALYZEDIRECTIONALAUTHORITY Sweep horizontal control directions at hover.
%
% Moment authority holds total thrust equal to weight and forces the moment
% perpendicular to the requested direction to zero. Angular-acceleration
% authority applies the equivalent constraints after the inertia tensor.

    if nargin < 3
        numberOfDirections = 180;
    end
    validateattributes(numberOfDirections, {'numeric'}, ...
        {'scalar', 'integer', '>=', 12}, mfilename, 'numberOfDirections');
    if ~hover.hoverFeasible
        error('analyzeDirectionalAuthority:HoverInfeasible', ...
            'Directional authority cannot be calculated without feasible hover.');
    end

    directionAngles = (0:numberOfDirections - 1).' * ...
        (2 * pi / numberOfDirections);
    numberOfMotors = config.numberOfMotors;
    thrustToWrench = config.thrustToWrench;
    momentMap = thrustToWrench(2:3, :);
    angularAccelerationMap = config.inertiaTensor \ ...
        [momentMap; zeros(1, numberOfMotors)];
    horizontalAccelerationMap = angularAccelerationMap(1:2, :);

    momentLimit = zeros(numberOfDirections, 1);
    accelerationLimit = zeros(numberOfDirections, 1);
    momentLimitMotorThrust = zeros(numberOfMotors, numberOfDirections);
    accelerationLimitMotorThrust = zeros(numberOfMotors, numberOfDirections);
    accelerationAtMomentLimit = zeros(3, numberOfDirections);
    accelerationAtAccelerationLimit = zeros(3, numberOfDirections);

    lowerThrust = zeros(numberOfMotors, 1);
    upperThrust = config.maxThrust;
    for directionIndex = 1:numberOfDirections
        direction = [cos(directionAngles(directionIndex)); ...
                     sin(directionAngles(directionIndex))];
        perpendicularDirection = [-direction(2); direction(1)];

        momentEquality = [thrustToWrench(1, :); ...
            perpendicularDirection.' * momentMap];
        momentObjective = momentMap.' * direction;
        [~, maximumMoment, ~, maximumMomentThrust] = ...
            boundedLinearExtrema(momentObjective, momentEquality, ...
            [config.weight; 0], lowerThrust, upperThrust);

        accelerationEquality = [thrustToWrench(1, :); ...
            perpendicularDirection.' * horizontalAccelerationMap];
        accelerationObjective = horizontalAccelerationMap.' * direction;
        [~, maximumAcceleration, ~, maximumAccelerationThrust] = ...
            boundedLinearExtrema(accelerationObjective, ...
            accelerationEquality, [config.weight; 0], ...
            lowerThrust, upperThrust);

        if ~isfinite(maximumMoment) || ~isfinite(maximumAcceleration)
            error('analyzeDirectionalAuthority:DirectionInfeasible', ...
                'No feasible authority solution at %.1f degrees.', ...
                directionAngles(directionIndex) * 180 / pi);
        end

        momentLimit(directionIndex) = max(0, maximumMoment);
        accelerationLimit(directionIndex) = max(0, maximumAcceleration);
        momentLimitMotorThrust(:, directionIndex) = maximumMomentThrust;
        accelerationLimitMotorThrust(:, directionIndex) = ...
            maximumAccelerationThrust;

        momentWrench = thrustToWrench * maximumMomentThrust;
        accelerationAtMomentLimit(:, directionIndex) = ...
            config.inertiaTensor \ [momentWrench(2); momentWrench(3); 0];
        accelerationWrench = thrustToWrench * maximumAccelerationThrust;
        accelerationAtAccelerationLimit(:, directionIndex) = ...
            config.inertiaTensor \ ...
            [accelerationWrench(2); accelerationWrench(3); 0];
    end

    [minimumMoment, worstMomentIndex] = min(momentLimit);
    [maximumMoment, bestMomentIndex] = max(momentLimit);
    [minimumAcceleration, worstAccelerationIndex] = min(accelerationLimit);
    [maximumAcceleration, bestAccelerationIndex] = max(accelerationLimit);

    authority.directionAngles_rad = directionAngles;
    authority.directionAngles_deg = directionAngles * 180 / pi;
    authority.momentLimit_Nm = momentLimit;
    authority.angularAccelerationLimit_rad_s2 = accelerationLimit;
    authority.momentLimitMotorThrust_N = momentLimitMotorThrust;
    authority.accelerationLimitMotorThrust_N = accelerationLimitMotorThrust;
    authority.angularAccelerationAtMomentLimit_rad_s2 = ...
        accelerationAtMomentLimit;
    authority.angularAccelerationAtAccelerationLimit_rad_s2 = ...
        accelerationAtAccelerationLimit;
    authority.minimumMoment_Nm = minimumMoment;
    authority.maximumMoment_Nm = maximumMoment;
    authority.momentAnisotropy = maximumMoment / minimumMoment;
    authority.worstMomentDirection_deg = ...
        authority.directionAngles_deg(worstMomentIndex);
    authority.bestMomentDirection_deg = ...
        authority.directionAngles_deg(bestMomentIndex);
    authority.minimumAngularAcceleration_rad_s2 = minimumAcceleration;
    authority.maximumAngularAcceleration_rad_s2 = maximumAcceleration;
    authority.angularAccelerationAnisotropy = ...
        maximumAcceleration / minimumAcceleration;
    authority.worstAngularAccelerationDirection_deg = ...
        authority.directionAngles_deg(worstAccelerationIndex);
    authority.bestAngularAccelerationDirection_deg = ...
        authority.directionAngles_deg(bestAccelerationIndex);

    fprintf('\n360-degree directional authority\n');
    fprintf('  Moment authority: %.3f to %.3f N*m\n', ...
        minimumMoment, maximumMoment);
    fprintf('  Moment anisotropy: %.3f (best/worst)\n', ...
        authority.momentAnisotropy);
    fprintf('  Worst/best moment directions: %.1f / %.1f deg\n', ...
        authority.worstMomentDirection_deg, ...
        authority.bestMomentDirection_deg);
    fprintf('  Angular acceleration: %.3f to %.3f rad/s^2\n', ...
        minimumAcceleration, maximumAcceleration);
    fprintf('  Angular-acceleration anisotropy: %.3f (best/worst)\n', ...
        authority.angularAccelerationAnisotropy);
    fprintf('  Worst/best acceleration directions: %.1f / %.1f deg\n', ...
        authority.worstAngularAccelerationDirection_deg, ...
        authority.bestAngularAccelerationDirection_deg);
end
