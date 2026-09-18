function simulation = simulateRollPID(config, hover, controller, settings)
%SIMULATEROLLPID Simulate PID attitude control about the body y axis.
%
% This compatibility wrapper keeps existing roll-only callers working while
% the shared scalar simulator also supports pitch and yaw.

    simulation = simulateAttitudeAxisPID(config, hover, controller, ...
        settings, 2);
end
