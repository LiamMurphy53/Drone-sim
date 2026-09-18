function wrench = motorWrench4(config, motorThrust)
%MOTORWRENCH4 Return [Fz; Mx; My; Mz] for individual motor thrusts.

    validateattributes(motorThrust, {'numeric'}, ...
        {'vector', 'numel', config.numberOfMotors, 'real', 'finite', ...
         'nonnegative'}, mfilename, 'motorThrust');
    motorThrust = motorThrust(:);
    tolerance = 1e-12 * max(1, max(config.maxThrust));
    if any(motorThrust > config.maxThrust + tolerance)
        error('motorWrench4:ThrustExceedsMaximum', ...
            'A motor thrust exceeds its configured maximum.');
    end
    if ~isfield(config, 'thrustToWrench4')
        error('motorWrench4:YawModelMissing', ...
            'The configuration does not contain a four-axis wrench map.');
    end

    wrench = config.thrustToWrench4 * motorThrust;
end
