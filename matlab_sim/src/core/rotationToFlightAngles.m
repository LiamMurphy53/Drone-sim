function angles_rad = rotationToFlightAngles(rotation)
%ROTATIONTOFLIGHTANGLES Return [roll, pitch, yaw] for the project sequence.

    validateattributes(rotation, {'numeric'}, ...
        {'size', [3, 3], 'real', 'finite'}, mfilename, 'rotation');
    pitch = asin(min(max(rotation(3, 2), -1), 1));
    roll = atan2(-rotation(3, 1), rotation(3, 3));
    yaw = atan2(-rotation(1, 2), rotation(2, 2));
    angles_rad = [roll; pitch; yaw];
end
