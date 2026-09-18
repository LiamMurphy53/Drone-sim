function wrench = motorWrench(config, motorThrust)
%MOTORWRENCH Return [Fz; Mx; My] for a set of motor thrusts.
%
% wrench = motorWrench(config, motorThrust)
%
% config is the result from analyzeDroneConfig. motorThrust contains one
% actual thrust value per motor in newtons.

    validateattributes(motorThrust, {'numeric'}, ...
        {'vector', 'real', 'finite', 'nonnegative'}, mfilename, 'motorThrust');
    motorThrust = motorThrust(:);

    if numel(motorThrust) ~= config.numberOfMotors
        error('motorWrench:InvalidThrustCount', ...
            'motorThrust must contain one value per motor.');
    end
    if any(motorThrust > config.maxThrust)
        error('motorWrench:ThrustExceedsMaximum', ...
            'A requested motor thrust exceeds that motor''s maximum thrust.');
    end

    wrench = config.thrustToWrench * motorThrust;
end
