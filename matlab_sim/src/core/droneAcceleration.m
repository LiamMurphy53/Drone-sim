function acceleration = droneAcceleration(config, motorThrust, angularVelocity)
%DRONEACCELERATION Calculate instantaneous linear and angular acceleration.
%
% acceleration = droneAcceleration(config, motorThrust)
% acceleration = droneAcceleration(config, motorThrust, angularVelocity)
%
% angularVelocity is optional and defaults to [0; 0; 0] rad/s. The reported
% level vertical acceleration includes gravity and assumes the drone is level.

    if nargin < 3
        angularVelocity = zeros(3, 1);
    end
    validateattributes(angularVelocity, {'numeric'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, mfilename, 'angularVelocity');
    angularVelocity = angularVelocity(:);

    wrench = motorWrench(config, motorThrust);
    forceBody = [0; 0; wrench(1)];
    momentBody = [wrench(2); wrench(3); 0];

    linearAccelerationFromThrust = forceBody / config.totalMass;
    angularMomentum = config.inertiaTensor * angularVelocity;
    angularAcceleration = config.inertiaTensor \ ...
        (momentBody - cross(angularVelocity, angularMomentum));

    acceleration.forceBody = forceBody;
    acceleration.momentBody = momentBody;
    acceleration.linearAccelerationFromThrust = linearAccelerationFromThrust;
    acceleration.levelVerticalAcceleration = ...
        linearAccelerationFromThrust(3) - config.gravity;
    acceleration.angularAcceleration = angularAcceleration;
end
