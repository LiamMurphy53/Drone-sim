function rotation = flightAnglesToRotation(roll_rad, pitch_rad, yaw_rad)
%FLIGHTANGLESTOROTATION Body-to-world rotation for project flight angles.
%
% Sequence: yaw about +z, pitch about the intermediate +x axis, then roll
% about the final +y axis: R = Rz(yaw)*Rx(pitch)*Ry(roll).

    validateattributes(roll_rad, {'numeric'}, ...
        {'scalar', 'real', 'finite'}, mfilename, 'roll_rad');
    validateattributes(pitch_rad, {'numeric'}, ...
        {'scalar', 'real', 'finite'}, mfilename, 'pitch_rad');
    validateattributes(yaw_rad, {'numeric'}, ...
        {'scalar', 'real', 'finite'}, mfilename, 'yaw_rad');

    cr = cos(roll_rad);
    sr = sin(roll_rad);
    cp = cos(pitch_rad);
    sp = sin(pitch_rad);
    cy = cos(yaw_rad);
    sy = sin(yaw_rad);

    rotationY = [cr, 0, sr; 0, 1, 0; -sr, 0, cr];
    rotationX = [1, 0, 0; 0, cp, -sp; 0, sp, cp];
    rotationZ = [cy, -sy, 0; sy, cy, 0; 0, 0, 1];
    rotation = rotationZ * rotationX * rotationY;
end
