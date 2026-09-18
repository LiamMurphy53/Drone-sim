function tests = test_drone_config
%TEST_DRONE_CONFIG Basic tests for the motor-layout model.
    tests = functiontests(localfunctions);
end

function setupOnce(~)
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(projectRoot, 'setup_project.m'));
end

function testSymmetricQuadHasNoMomentAtEqualThrust(testCase)
    locations_mm = [200, -200; 200, 200; -200, 200; -200, -200];
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [0, 0, 0], 1e9 * eye(3));
    wrench = motorWrench(config, 5 * ones(4, 1));

    verifyEqual(testCase, wrench, [20; 0; 0], 'AbsTol', 1e-12);
end

function testPreservesConfiguredMotorNames(testCase)
    locations_mm = [200, -200; -200, -200; 200, 200; -200, 200];
    names = {'FL', 'FR', 'RL', 'RR'};
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [0, 0, 0], 1e9 * eye(3), names);

    verifyEqual(testCase, config.motorNames, names.');
end

function testDetectsWhetherConfigurationMatchesInputStructure(testCase)
    droneInputs.motorLocations_mm = ...
        [200, -200; -200, -200; 200, 200; -200, 200];
    droneInputs.motorNames = {'FL', 'FR', 'RL', 'RR'};
    droneInputs.maxThrust_N = 10;
    droneInputs.totalMass_g = 2000;
    droneInputs.centerOfMass_mm = [0, 0, 0];
    droneInputs.inertiaTensor_g_mm2 = 1e9 * eye(3);
    config = analyzeDroneConfig(droneInputs.motorLocations_mm, ...
        droneInputs.maxThrust_N, droneInputs.totalMass_g, ...
        droneInputs.centerOfMass_mm, droneInputs.inertiaTensor_g_mm2, ...
        droneInputs.motorNames);

    verifyTrue(testCase, droneConfigMatchesInputs(config, droneInputs));
    droneInputs.centerOfMass_mm(2) = 1;
    verifyFalse(testCase, droneConfigMatchesInputs(config, droneInputs));
end

function testAsymmetricLayoutProducesExpectedMoment(testCase)
    locations_mm = [200, -100; 200, 100; -100, 200; -100, -200];
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [0, 0, 0], 1e9 * eye(3));
    wrench = motorWrench(config, 5 * ones(4, 1));

    verifyEqual(testCase, wrench, [20; 0; -1], 'AbsTol', 1e-12);
end

function testRejectsThrustAboveMotorLimit(testCase)
    config = analyzeDroneConfig([100, 0; -100, 0], [5; 6], ...
        1000, [0, 0, 0], 1e9 * eye(3));
    verifyError(testCase, @() motorWrench(config, [5; 7]), ...
        'motorWrench:ThrustExceedsMaximum');
end

function testCenterOfMassChangesMotorMomentArms(testCase)
    locations_mm = [1000, 0; -1000, 0];
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [100, 0, 0], 1e9 * eye(3));
    wrench = motorWrench(config, [5; 5]);

    verifyEqual(testCase, wrench, [10; 0; 1], 'AbsTol', 1e-12);
end

function testCalculatesLinearAndAngularAcceleration(testCase)
    locations_mm = [0, 1000; 0, -1000];
    inertia_g_mm2 = 1e9 * diag([2, 3, 4]);
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [0, 0, 0], inertia_g_mm2);
    acceleration = droneAcceleration(config, [10; 0]);

    verifyEqual(testCase, acceleration.levelVerticalAcceleration, ...
        5 - 9.80665, 'AbsTol', 1e-12);
    verifyEqual(testCase, acceleration.angularAcceleration, [5; 0; 0], ...
        'AbsTol', 1e-12);
end

function testCalculatesSymmetricHoverTrim(testCase)
    locations_mm = [200, -200; 200, 200; -200, 200; -200, -200];
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [0, 0, 0], 1e9 * eye(3));
    hover = analyzeHoverDynamics(config);

    expectedMotorThrust = (config.weight / 4) * ones(4, 1);
    verifyTrue(testCase, hover.hoverFeasible);
    verifyEqual(testCase, hover.motorThrust, expectedMotorThrust, ...
        'AbsTol', 1e-10);
    verifyEqual(testCase, hover.trimWrench, [config.weight; 0; 0], ...
        'AbsTol', 1e-10);
end

function testCalculatesConstantAltitudeAuthority(testCase)
    locations_mm = [200, -200; 200, 200; -200, 200; -200, -200];
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [0, 0, 0], 1e9 * eye(3));
    hover = analyzeHoverDynamics(config);
    expectedMoment = 0.2 * config.weight;

    verifyEqual(testCase, hover.xAxis.momentRange, ...
        [-expectedMoment, expectedMoment], 'AbsTol', 1e-10);
    verifyEqual(testCase, hover.yAxis.momentRange, ...
        [-expectedMoment, expectedMoment], 'AbsTol', 1e-10);
    verifyEqual(testCase, hover.xAxis.axisAngularAccelerationRange, ...
        [-expectedMoment, expectedMoment], 'AbsTol', 1e-10);
end

function testDetectsInfeasibleHover(testCase)
    locations_mm = [200, -200; 200, 200; -200, 200; -200, -200];
    config = analyzeDroneConfig(locations_mm, 2, 2000, ...
        [0, 0, 0], 1e9 * eye(3));

    warningState = warning('off', 'analyzeHoverDynamics:HoverInfeasible');
    cleanup = onCleanup(@() warning(warningState));
    hover = analyzeHoverDynamics(config);
    verifyFalse(testCase, hover.hoverFeasible);
end

function testZeroRollCommandRemainsAtHover(testCase)
    locations_mm = [200, -200; 200, 200; -200, 200; -200, -200];
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [0, 0, 0], 1e9 * eye(3));
    hover = analyzeHoverDynamics(config);

    controller.Kp = 1;
    controller.Ki = 0.1;
    controller.Kd = 0.5;
    settings.timeStep_s = 0.01;
    settings.duration_s = 0.1;
    settings.motorTimeConstant_s = 0.05;
    settings.time_s = (0:settings.timeStep_s:settings.duration_s).';
    settings.angleCommand_rad = zeros(size(settings.time_s));

    simulation = simulateRollPID(config, hover, controller, settings);
    verifyEqual(testCase, simulation.angle_rad, ...
        zeros(size(settings.time_s)), 'AbsTol', 1e-10);
    verifyEqual(testCase, simulation.motorThrust_N, ...
        repmat(hover.motorThrust.', numel(settings.time_s), 1), ...
        'AbsTol', 1e-10);
end

function testSingleAxisPIDSupportsPitchRollAndYaw(testCase)
    locations_mm = [200, -200; -200, -200; 200, 200; -200, 200];
    propulsion.motorSpinDirection = {'CW', 'CCW', 'CCW', 'CW'};
    propulsion.yawTorquePerThrust_m = 0.01;
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [0, 0, 0], 1e7 * eye(3), {'FL', 'FR', 'RL', 'RR'}, propulsion);
    hover = analyzeHoverDynamics(config);

    settings.timeStep_s = 0.01;
    settings.duration_s = 1.5;
    settings.motorTimeConstant_s = 0.05;
    settings.commandStart_s = 0.1;
    settings.commandEnd_s = 0.7;
    settings.settlingBand_deg = 1;
    settings.time_s = (0:settings.timeStep_s:settings.duration_s).';
    settings.angleCommand_rad = zeros(size(settings.time_s));
    settings.angleCommand_rad(settings.time_s >= settings.commandStart_s & ...
        settings.time_s < settings.commandEnd_s) = 5 * pi / 180;
    settings.printSummary = false;

    simulations = cell(3, 1);
    for axisIndex = 1:3
        controller = designAttitudePID( ...
            config.inertiaTensor(axisIndex, axisIndex), 10, 0.9, 0.5);
        simulations{axisIndex} = simulateAttitudeAxisPID( ...
            config, hover, controller, settings, axisIndex);
        verifyEqual(testCase, simulations{axisIndex}.axisIndex, axisIndex);
        verifyGreaterThan(testCase, ...
            simulations{axisIndex}.metrics.peakRate_deg_s, 0);
        verifyGreaterThan(testCase, ...
            simulations{axisIndex}.metrics.directionalMomentLimit_Nm, 0);
        verifyTrue(testCase, all(isfinite( ...
            simulations{axisIndex}.motorThrust_N), 'all'));
    end
    verifyEqual(testCase, ...
        {simulations{1}.motionName, simulations{2}.motionName, ...
         simulations{3}.motionName}, {'Pitch', 'Roll', 'Yaw'});
end

function testLevelHoverHasNoTranslationalAcceleration(testCase)
    locations_mm = [200, -200; -200, -200; 200, 200; -200, 200];
    names = {'FL', 'FR', 'RL', 'RR'};
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [0, 0, 0], 1e9 * eye(3), names);

    rollSimulation.time_s = [0; 0.01; 0.02];
    rollSimulation.angle_rad = zeros(3, 1);
    rollSimulation.motorThrust_N = repmat( ...
        (config.weight / 4) * ones(1, 4), 3, 1);
    trajectory = simulateRollTrajectory3D(config, rollSimulation);

    verifyEqual(testCase, trajectory.acceleration_m_s2, zeros(3, 3), ...
        'AbsTol', 1e-12);
    verifyEqual(testCase, trajectory.position_m, zeros(3, 3), ...
        'AbsTol', 1e-12);
end

function testSidewaysThrustProducesLateralAcceleration(testCase)
    locations_mm = [200, -200; -200, -200; 200, 200; -200, 200];
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [0, 0, 0], 1e9 * eye(3));

    rollSimulation.time_s = [0; 0.01];
    rollSimulation.angle_rad = (pi / 2) * ones(2, 1);
    rollSimulation.motorThrust_N = repmat( ...
        (config.weight / 4) * ones(1, 4), 2, 1);
    trajectory = simulateRollTrajectory3D(config, rollSimulation);

    verifyEqual(testCase, trajectory.acceleration_m_s2(1, :), ...
        [config.gravity, 0, -config.gravity], 'AbsTol', 1e-12);
end

function testCalculatesCardinalDirectionalAuthority(testCase)
    locations_mm = [200, -200; -200, -200; 200, 200; -200, 200];
    config = analyzeDroneConfig(locations_mm, 10, 2000, ...
        [0, 0, 0], 1e9 * eye(3));
    hover = analyzeHoverDynamics(config);
    authority = analyzeDirectionalAuthority(config, hover, 12);

    cardinalIndices = [1, 4, 7, 10];
    expectedMoment = 0.2 * config.weight;
    verifyEqual(testCase, authority.momentLimit_Nm(cardinalIndices), ...
        expectedMoment * ones(4, 1), 'AbsTol', 1e-10);
    verifyEqual(testCase, ...
        authority.angularAccelerationLimit_rad_s2(cardinalIndices), ...
        expectedMoment * ones(4, 1), 'AbsTol', 1e-10);
end

function testReactionTorqueUsesConfiguredSpinDirections(testCase)
    locations_mm = [100, -100; -100, -100; 100, 100; -100, 100];
    propulsion.motorSpinDirection = {'CW', 'CCW', 'CCW', 'CW'};
    propulsion.yawTorquePerThrust_m = 0.01;
    config = analyzeDroneConfig(locations_mm, 10, 1000, ...
        [0, 0, 0], 1e7 * eye(3), {'FL', 'FR', 'RL', 'RR'}, ...
        propulsion);

    wrench = motorWrench4(config, [2; 1; 1; 2]);
    verifyEqual(testCase, wrench, [6; 0; 0; 0.02], 'AbsTol', 1e-12);
    verifyEqual(testCase, config.yawReactionSign, [1; -1; -1; 1]);
    verifyEqual(testCase, config.controlRank4, 4);
end

function testPropulsionChangesInvalidateStoredConfiguration(testCase)
    droneInputs.motorLocations_mm = ...
        [100, -100; -100, -100; 100, 100; -100, 100];
    droneInputs.motorNames = {'FL', 'FR', 'RL', 'RR'};
    droneInputs.motorSpinDirection = {'CW', 'CCW', 'CCW', 'CW'};
    droneInputs.yawTorquePerThrust_m = 0.01;
    droneInputs.maxThrust_N = 10;
    droneInputs.totalMass_g = 1000;
    droneInputs.centerOfMass_mm = [0, 0, 0];
    droneInputs.inertiaTensor_g_mm2 = 1e7 * eye(3);
    config = analyzeDroneConfig(droneInputs.motorLocations_mm, ...
        droneInputs.maxThrust_N, droneInputs.totalMass_g, ...
        droneInputs.centerOfMass_mm, droneInputs.inertiaTensor_g_mm2, ...
        droneInputs.motorNames, droneInputs);

    verifyTrue(testCase, droneConfigMatchesInputs(config, droneInputs));
    droneInputs.motorSpinDirection{1} = 'CCW';
    verifyFalse(testCase, droneConfigMatchesInputs(config, droneInputs));
end

function testFlightAngleRotationRoundTripIsConsistent(testCase)
    expectedAngles = [20; -15; 40] * pi / 180;
    rotation = flightAnglesToRotation(expectedAngles(1), ...
        expectedAngles(2), expectedAngles(3));
    actualAngles = rotationToFlightAngles(rotation);
    quaternion = rotationToQuaternion(rotation);

    verifyEqual(testCase, actualAngles, expectedAngles, 'AbsTol', 1e-12);
    verifyEqual(testCase, quaternionToRotation(quaternion), rotation, ...
        'AbsTol', 1e-12);
end

function testBoundedAllocatorTracksFeasibleWrenchAndHonorsLimits(testCase)
    locations_mm = [100, -100; -100, -100; 100, 100; -100, 100];
    propulsion.motorSpinDirection = {'CW', 'CCW', 'CCW', 'CW'};
    propulsion.yawTorquePerThrust_m = 0.01;
    config = analyzeDroneConfig(locations_mm, 10, 1000, ...
        [0, 0, 0], 1e7 * eye(3), {'FL', 'FR', 'RL', 'RR'}, ...
        propulsion);

    feasibleWrench = [config.weight; 0.1; -0.1; 0.01];
    feasible = allocateMotorWrench(config, feasibleWrench, ones(4, 1));
    verifyEqual(testCase, feasible.achievedWrench, feasibleWrench, ...
        'AbsTol', 1e-10);
    verifyFalse(testCase, feasible.limited);

    infeasible = allocateMotorWrench(config, [100; 0; 0; 0], ones(4, 1));
    verifyTrue(testCase, infeasible.limited);
    verifyGreaterThanOrEqual(testCase, infeasible.motorThrust_N, zeros(4, 1));
    verifyLessThanOrEqual(testCase, infeasible.motorThrust_N, ...
        config.maxThrust);
end

function testZeroCommandSixDOFRemainsInLevelHover(testCase)
    locations_mm = [100, -100; -100, -100; 100, 100; -100, 100];
    propulsion.motorSpinDirection = {'CW', 'CCW', 'CCW', 'CW'};
    propulsion.yawTorquePerThrust_m = 0.01;
    config = analyzeDroneConfig(locations_mm, 10, 1000, ...
        [0, 0, 0], 1e7 * eye(3), {'FL', 'FR', 'RL', 'RR'}, ...
        propulsion);

    settings.commandWaypoints = [0, 0, 0, 0, 0; ...
        0.1, 0, 0, 0, 0];
    settings.timeStep_s = 0.01;
    settings.motorTimeConstant_s = 0.05;
    settings.attitudeNaturalFrequency_rad_s = [6; 6; 3];
    settings.attitudeDampingRatio = [0.9; 0.9; 0.9];
    settings.attitudeIntegralPole_rad_s = [0.1; 0.1; 0.1];
    settings.maxAttitudeIntegral_rad_s = [0.5; 0.5; 0.5];
    settings.altitudeNaturalFrequency_rad_s = 2;
    settings.altitudeDampingRatio = 1;
    settings.altitudeIntegralPole_rad_s = 0.1;
    settings.maxAltitudeIntegral_m_s = 0.5;
    settings.maxVerticalAcceleration_m_s2 = 5;
    settings.initialPosition_m = [0; 0; 0];
    settings.initialVelocity_m_s = [0; 0; 0];
    settings.initialAttitude_deg = [0; 0; 0];
    settings.initialAngularVelocity_rad_s = [0; 0; 0];
    settings.allocationPriority = ones(4, 1);

    simulation = simulateDrone6DOF(config, settings);
    verifyEqual(testCase, simulation.position_m, ...
        zeros(size(simulation.position_m)), 'AbsTol', 1e-9);
    verifyEqual(testCase, simulation.actualAngles_deg, ...
        zeros(size(simulation.actualAngles_deg)), 'AbsTol', 1e-9);
    verifyFalse(testCase, any(simulation.allocationLimited));

    rollResponse = extractDrone6DOFAxisResponse(simulation, 'Roll');
    pitchResponse = extractDrone6DOFAxisResponse(simulation, 'Pitch');
    yawResponse = extractDrone6DOFAxisResponse(simulation, 'Yaw');
    verifyEqual(testCase, rollResponse.bodyAxisIndex, 2);
    verifyEqual(testCase, rollResponse.wrenchIndex, 3);
    verifyEqual(testCase, pitchResponse.bodyAxisIndex, 1);
    verifyEqual(testCase, pitchResponse.wrenchIndex, 2);
    verifyEqual(testCase, yawResponse.bodyAxisIndex, 3);
    verifyEqual(testCase, yawResponse.wrenchIndex, 4);
    verifyEqual(testCase, rollResponse.actualFlightRate_deg_s, ...
        zeros(size(simulation.time_s)), 'AbsTol', 1e-9);
    verifyEqual(testCase, pitchResponse.angleCommand_deg, ...
        simulation.commandAngles_deg(:, 2), 'AbsTol', 1e-12);
end

function testSixDOFRateModeProducesContinuousRotation(testCase)
    locations_mm = [100, -100; -100, -100; 100, 100; -100, 100];
    propulsion.motorSpinDirection = {'CW', 'CCW', 'CCW', 'CW'};
    propulsion.yawTorquePerThrust_m = 0.01;
    config = analyzeDroneConfig(locations_mm, 10, 1000, ...
        [0, 0, 0], 1e7 * eye(3), {'FL', 'FR', 'RL', 'RR'}, ...
        propulsion);

    settings.commandWaypoints = [0, 0, 0, 0, 2; ...
        0.8, 0, 0, 0, 2];
    settings.rateCommandSegments = [0.2, 0.5, 0, -360, 0, 0.5; ...
        0.5, 0.65, 0, 0, 0, 0.5];
    settings.rateControlBandwidth_rad_s = [20; 20; 10];
    settings.timeStep_s = 0.002;
    settings.motorTimeConstant_s = 0.05;
    settings.attitudeNaturalFrequency_rad_s = [6; 6; 3];
    settings.attitudeDampingRatio = [0.9; 0.9; 0.9];
    settings.attitudeIntegralPole_rad_s = [0.1; 0.1; 0.1];
    settings.maxAttitudeIntegral_rad_s = [0.5; 0.5; 0.5];
    settings.altitudeNaturalFrequency_rad_s = 2;
    settings.altitudeDampingRatio = 1;
    settings.altitudeIntegralPole_rad_s = 0.1;
    settings.maxAltitudeIntegral_m_s = 0.5;
    settings.maxVerticalAcceleration_m_s2 = 5;
    settings.initialPosition_m = [0; 0; 2];
    settings.initialVelocity_m_s = [0; 0; 0];
    settings.initialAttitude_deg = [0; 0; 0];
    settings.initialAngularVelocity_rad_s = [0; 0; 0];
    settings.allocationPriority = ones(4, 1);

    simulation = simulateDrone6DOF(config, settings);
    verifyTrue(testCase, any(simulation.rateModeActive));
    verifyLessThan(testCase, ...
        simulation.metrics.totalActualRateModeRotation_deg(2), -20);
    verifyEqual(testCase, ...
        simulation.metrics.totalDesiredRateModeRotation_deg, ...
        [0, -108, 0], 'AbsTol', 1e-10);
    verifyTrue(testCase, all(isfinite(simulation.quaternionBodyToWorld), ...
        'all'));

    ambiguousSettings = rmfield(settings, 'rateCommandSegments');
    ambiguousSettings.commandWaypoints = [0, 0, 0, 0, 2; ...
        0.8, 0, -180, 0, 2];
    verifyError(testCase, ...
        @() simulateDrone6DOF(config, ambiguousSettings), ...
        'simulateDrone6DOF:FlipNeedsRateMode');
end

function testConvertsComponentPlacementToTotalCG(testCase)
    moved = calculateComponentCGCandidates(1000, [0, 0, 0], 100, ...
        [100, 0, 0; -100, 0, 0], 'move-included', [0, 0, 0]);
    verifyEqual(testCase, moved.totalCG_mm, ...
        [10, 0, 0; -10, 0, 0], 'AbsTol', 1e-12);
    verifyEqual(testCase, moved.totalMass_g, [1000; 1000]);

    added = calculateComponentCGCandidates(1000, [0, 0, 0], 100, ...
        [110, 0, 0], 'add-payload');
    verifyEqual(testCase, added.totalCG_mm, [10, 0, 0], ...
        'AbsTol', 1e-12);
    verifyEqual(testCase, added.totalMass_g, 1100);
end

function testCGSensitivityMatchesCenteredQuadHover(testCase)
    config = symmetricYawControllableQuad();
    options.fixedZ_mm = 0;
    options.printSummary = false;
    sensitivity = analyzeCGSensitivity(config, 0, 0, options);

    expectedCommand = config.weight / config.maxTotalThrust;
    verifyTrue(testCase, sensitivity.hoverFeasible);
    verifyEqual(testCase, reshape(sensitivity.hoverCommand, [], 1), ...
        expectedCommand * ones(4, 1), 'AbsTol', 1e-10);
    verifyGreaterThan(testCase, ...
        sensitivity.rollTwoSidedMomentAuthority_Nm, 0);
    verifyGreaterThan(testCase, ...
        sensitivity.pitchTwoSidedMomentAuthority_Nm, 0);
    verifyGreaterThan(testCase, ...
        sensitivity.yawTwoSidedMomentAuthority_Nm, 0);
end

function testMomentEnvelopeAndMixerQualityAreFullRank(testCase)
    config = symmetricYawControllableQuad();
    envelope = analyzeMomentEnvelope3D(config, config.weight);
    quality = analyzeMixerQuality(config);

    verifyEqual(testCase, envelope.effectiveDimension, 3);
    verifyEqual(testCase, envelope.numberOfVertices, 4);
    verifyEqual(testCase, sum(envelope.motorThrustVertices_N, 1), ...
        config.weight * ones(1, envelope.numberOfVertices), ...
        'AbsTol', 1e-10);
    verifyTrue(testCase, envelope.pureMomentTrimFeasible);
    verifyGreaterThan(testCase, envelope.originCenteredRadius_Nm, 0);
    verifyEqual(testCase, quality.rank, 4);
    verifyEqual(testCase, quality.fixedCollective.rank, 3);
    verifyGreaterThan(testCase, ...
        quality.fixedCollective.qualityScore_percent, 0);
    verifyLessThanOrEqual(testCase, ...
        quality.fixedCollective.qualityScore_percent, 100);
end

function testLoadsAndComparesConfigurationInputs(testCase)
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    inputFile = fullfile(projectRoot, 'inputs', 'drone_config_inputs.m');
    loaded = loadDroneInputFile(inputFile);
    verifyEqual(testCase, loaded.configurationName, 'Current deadcat');

    sources = {
        'First', loaded;
        'Second', loaded
    };
    options.numberOfDirections = 12;
    options.printUnderlyingAnalysis = false;
    options.printSummary = false;
    comparison = compareDroneConfigurations(sources, options);
    verifyEqual(testCase, comparison.labels, {'First'; 'Second'});
    verifyEqual(testCase, ...
        comparison.metrics(1).minimumHoverMargin_percent, ...
        comparison.metrics(2).minimumHoverMargin_percent, ...
        'AbsTol', 1e-12);
end

function config = symmetricYawControllableQuad()
    locations_mm = [100, -100; -100, -100; 100, 100; -100, 100];
    propulsion.motorSpinDirection = {'CW', 'CCW', 'CCW', 'CW'};
    propulsion.yawTorquePerThrust_m = 0.01;
    config = analyzeDroneConfig(locations_mm, 10, 1000, ...
        [0, 0, 0], 1e7 * eye(3), {'FL', 'FR', 'RL', 'RR'}, propulsion);
end
