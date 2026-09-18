function rotation = quaternionToRotation(quaternion)
%QUATERNIONTOROTATION Body-to-world rotation from [w; x; y; z].

    validateattributes(quaternion, {'numeric'}, ...
        {'vector', 'numel', 4, 'real', 'finite'}, mfilename, 'quaternion');
    quaternion = quaternion(:) / norm(quaternion);
    w = quaternion(1);
    x = quaternion(2);
    y = quaternion(3);
    z = quaternion(4);

    rotation = [ ...
        1 - 2 * (y^2 + z^2), 2 * (x*y - w*z), 2 * (x*z + w*y); ...
        2 * (x*y + w*z), 1 - 2 * (x^2 + z^2), 2 * (y*z - w*x); ...
        2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x^2 + y^2)];
end
