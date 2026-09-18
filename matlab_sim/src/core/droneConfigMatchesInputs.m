function matches = droneConfigMatchesInputs(config, droneInputs)
%DRONECONFIGMATCHESINPUTS True when CONFIG represents the current input file.

    requiredConfigFields = {'motorLocations_mm', 'motorNames', 'maxThrust', ...
        'totalMass_g', 'centerOfMass_mm', 'inertiaTensor_g_mm2'};
    requiredInputFields = {'motorLocations_mm', 'motorNames', 'maxThrust_N', ...
        'totalMass_g', 'centerOfMass_mm', 'inertiaTensor_g_mm2'};
    if ~isstruct(config) || ~isstruct(droneInputs) || ...
            ~all(isfield(config, requiredConfigFields)) || ...
            ~all(isfield(droneInputs, requiredInputFields))
        matches = false;
        return;
    end

    numberOfMotors = size(droneInputs.motorLocations_mm, 1);
    expectedMotorLocations = droneInputs.motorLocations_mm;
    if size(expectedMotorLocations, 2) == 2
        expectedMotorLocations = [expectedMotorLocations, ...
            zeros(numberOfMotors, 1)];
    end
    expectedMaxThrust = droneInputs.maxThrust_N;
    if isscalar(expectedMaxThrust)
        expectedMaxThrust = repmat(expectedMaxThrust, numberOfMotors, 1);
    else
        expectedMaxThrust = expectedMaxThrust(:);
    end
    expectedMotorNames = droneInputs.motorNames;
    if isstring(expectedMotorNames)
        expectedMotorNames = cellstr(expectedMotorNames);
    end
    expectedMotorNames = expectedMotorNames(:);

    matches = isequaln(config.motorLocations_mm, ...
            expectedMotorLocations) && ...
        isequaln(config.motorNames, expectedMotorNames) && ...
        isequaln(config.maxThrust, expectedMaxThrust) && ...
        isequaln(config.totalMass_g, droneInputs.totalMass_g) && ...
        isequaln(config.centerOfMass_mm, droneInputs.centerOfMass_mm(:).') && ...
        isequaln(config.inertiaTensor_g_mm2, ...
            droneInputs.inertiaTensor_g_mm2);

    if matches && isfield(droneInputs, 'motorSpinDirection')
        expectedSpinDirection = droneInputs.motorSpinDirection;
        if isstring(expectedSpinDirection)
            expectedSpinDirection = cellstr(expectedSpinDirection);
        end
        expectedSpinDirection = cellfun(@upper, expectedSpinDirection(:), ...
            'UniformOutput', false);
        matches = isfield(config, 'motorSpinDirection') && ...
            isequaln(config.motorSpinDirection, expectedSpinDirection);
    end
    if matches && isfield(droneInputs, 'yawTorquePerThrust_m')
        expectedYawCoefficient = droneInputs.yawTorquePerThrust_m;
        if isscalar(expectedYawCoefficient)
            expectedYawCoefficient = repmat(expectedYawCoefficient, ...
                numberOfMotors, 1);
        else
            expectedYawCoefficient = expectedYawCoefficient(:);
        end
        matches = isfield(config, 'yawTorquePerThrust_m') && ...
            isequaln(config.yawTorquePerThrust_m, expectedYawCoefficient);
    end
end
