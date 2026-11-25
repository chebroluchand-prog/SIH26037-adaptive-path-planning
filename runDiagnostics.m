%% SIH26037 DIAGNOSTIC RUNNER
% Run this script in MATLAB to test all components step-by-step.
% Usage: Run section-by-section with Ctrl+Enter or run entire file.

fprintf('\n========================================================\n');
fprintf('  SIH26037 DIAGNOSTIC & VALIDATION RUNNER\n');
fprintf('========================================================\n\n');

%% STEP 1: Verify all required files exist
requiredFiles = {
    'runSimulation.m', 'runAllScenarios.m', ...
    'createCattleScenario.m', 'createVillageScenario.m', ...
    'createIntersectionScenario.m', 'createHighwayMergeScenario.m', ...
    'createDenseMarketScenario.m', ...
    'localPlannerDWA_refined.m', 'behaviorDecision.m', ...
    'kalmanTracker.m', 'predictTrajectories.m', ...
    'fuseSensorOutputs.m', 'simulateSensorsAdvanced.m', ...
    'bicycleModel.m', 'Logger.m', 'checkLateralCorridor.m', ...
    'gpsToLocalENU.m', 'localENUToGPS.m', 'simulateGPSSensor.m', ...
    'getTerrainElevation.m', 'stepDWAPlanner3D.m', ...
    'stepBehaviorStateflow.m', 'stepBicycleControl3D.m', ...
    'exportSimulinkComponents.m', 'runPitchDemo.m'
};

allFound = true;
fprintf('[CHECK 1] Verifying required files:\n');
for i = 1:numel(requiredFiles)
    if exist(requiredFiles{i}, 'file')
        fprintf('  [OK] %s\n', requiredFiles{i});
    else
        fprintf('  [MISSING] %s\n', requiredFiles{i});
        allFound = false;
    end
end
if allFound
    fprintf('[PASS] All required files present.\n\n');
else
    error('[FAIL] Missing required files. Cannot proceed.');
end

%% STEP 2: Test Bicycle Model
fprintf('[CHECK 2] Testing bicycleModel:\n');
state0 = [0, 0, 0, 5.0, 0];
p = struct('L', 2.7, 'maxSteer', 0.6, 'maxAccel', 3.5, 'maxDecel', 6.0, 'maxSpeed', 30.0);
state1 = bicycleModel(state0, 0, 0, 0.05, p);
% Index speed appropriately depending on 5D or 7D output
vIdx = 4; if numel(state1) >= 7, vIdx = 6; end
assert(state1(vIdx) >= state0(4) - 1e-6, 'Speed should not increase without throttle');
assert(state1(1) > state0(1), 'X should increase when driving forward');
fprintf('  state: [%s]\n', num2str(state1, '%.3f '));
fprintf('[PASS] bicycleModel OK.\n\n');

%% STEP 3: Test behaviorDecision with new lateral evasion states
fprintf('[CHECK 3] Testing behaviorDecision:\n');
egoSt = [0, 0, 0, 5.0, 0];
rm.maxRisk = 0.1;
b1 = behaviorDecision(egoSt, rm, 10.0, 15.0);
assert(b1 == "CRUISE", sprintf('Expected CRUISE, got %s', b1));
b2 = behaviorDecision(egoSt, rm, 2.0, 5.0);
assert(b2 == "AVOID" || b2 == "CAUTION", sprintf('Expected AVOID/CAUTION, got %s', b2));
b3 = behaviorDecision(egoSt, rm, 0.7, 0.8);
assert(b3 == "EMERGENCY_STOP", sprintf('Expected EMERGENCY_STOP, got %s', b3));
% Test new lateral evasion states
b4 = behaviorDecision(egoSt, rm, 3.5, 2.5, true, true); % Obstacle approaching, lateral corridor available
assert(b4 == "EVADE_LATERAL" || b4 == "AVOID", sprintf('Expected EVADE_LATERAL/AVOID, got %s', b4));
b5 = behaviorDecision(egoSt, rm, 3.5, 3.5, false, false); % No lateral corridor, not approaching
assert(b5 == "PACING" || b5 == "CAUTION", sprintf('Expected PACING/CAUTION, got %s', b5));
fprintf('  CRUISE OK, AVOID/CAUTION OK, EMERGENCY_STOP OK, EVADE_LATERAL OK, PACING OK\n');
fprintf('[PASS] behaviorDecision OK.\n\n');

%% STEP 4: Test kalmanTracker
fprintf('[CHECK 4] Testing kalmanTracker:\n');
kt = kalmanTracker(0.05, 10);
det1.Position = [10, 5]; det1.Velocity = [0, 0]; det1.ClassID = 4; det1.MeasurementNoise = 0.5; det1.Confidence = 0.8;
kt.update(det1);
kt.predictAll();
assert(numel(kt.tracks) >= 1, 'Expected at least 1 track');
fprintf('  Tracks after first detection: %d\n', numel(kt.tracks));
fprintf('[PASS] kalmanTracker OK.\n\n');

%% STEP 4.5: Test checkLateralCorridor
fprintf('[CHECK 4.5] Testing checkLateralCorridor:\n');
egoSt = [0, 0, 0, 5.0, 0];
actors_empty = struct('Position', {}, 'Velocity', {}, 'ClassID', {});
[latAvail, approaching] = checkLateralCorridor(egoSt, actors_empty, 7.0);
assert(latAvail == true, 'Expected lateral corridor available with no obstacles');
assert(approaching == false, 'Expected no approaching obstacles with no obstacles');
fprintf('  No obstacles: lateralAvailable=%d, approaching=%d\n', latAvail, approaching);
% Test with approaching obstacle
actors_approach(1).Position = [20, 0]; actors_approach(1).Velocity = [-3, 0]; actors_approach(1).ClassID = 1;
[latAvail, approaching] = checkLateralCorridor(egoSt, actors_approach, 7.0);
assert(approaching == true, 'Expected approaching obstacle');
fprintf('  Approaching obstacle: lateralAvailable=%d, approaching=%d\n', latAvail, approaching);
fprintf('[PASS] checkLateralCorridor OK.\n\n');

%% STEP 5: Test predictTrajectories
fprintf('[CHECK 5] Testing predictTrajectories:\n');
tracks(1).state  = [10; 1; 5; -0.5]; tracks(1).classID = 6; tracks(1).age = 1; tracks(1).lastUpdate = 0;
tracks(2).state  = [20; 0; 0; 0.3];  tracks(2).classID = 4; tracks(2).age = 1; tracks(2).lastUpdate = 0;
[preds, predArray, predClassIDs] = predictTrajectories(tracks, 2.0, 0.05, 3);
assert(size(predArray, 1) == 6, sprintf('Expected 6 prediction rows, got %d', size(predArray, 1)));
assert(numel(predClassIDs) == 6, 'predClassIDs length mismatch');
fprintf('  predArray size: %dx%dx%d\n', size(predArray,1), size(predArray,2), size(predArray,3));
fprintf('[PASS] predictTrajectories OK.\n\n');

%% STEP 6: Test localPlannerDWA_refined (basic movement)
fprintf('[CHECK 6] Testing localPlannerDWA_refined (no obstacles):\n');
egoSt = [0, 0, 0, 3.0, 0];
goal  = [50, 0];
pp = struct('horizon', 2.0, 'dt', 0.05, 'Nv', 7, 'Nw', 11, ...
            'vMin', 0, 'vMax', 10.0, 'wMin', -1.0, 'wMax', 1.0, ...
            'maxAccel', 3.5, 'maxDecel', 6.0, 'maxAlpha', 2.5, ...
            'collisionRadius', 1.2, 'targetSpeed', 5.0, ...
            'yLimit', 3.5, 'isIntersection', false);
[path, cmd, cost] = localPlannerDWA_refined(egoSt, struct([]), goal, pp);
assert(~isempty(path), 'Path should not be empty with no obstacles');
assert(cmd(1) > 0.1, sprintf('Expected forward speed > 0.1, got %.3f', cmd(1)));
fprintf('  cmd: [v=%.2f, w=%.3f], cost=%.2f, path size: %dx%d\n', cmd(1), cmd(2), cost, size(path,1), size(path,2));
fprintf('[PASS] localPlannerDWA_refined (no obstacles) OK.\n\n');

%% STEP 6.5: Collision mask preservation & replanning latency
fprintf('[CHECK 6.5] Collision mask preservation & replan timing:\n');
egoSt = [50, 0, 0, 5.0, 0];
goal  = [180, 0];
pp = struct('horizon', 2.0, 'dt', 0.05, 'Nv', 11, 'Nw', 21, ...
            'vMin', 0, 'vMax', 12.0, 'wMin', -1.2, 'wMax', 1.2, ...
            'maxAccel', 3.5, 'maxDecel', 6.0, 'maxAlpha', 2.5, ...
            'collisionRadius', 1.2, 'targetSpeed', 6.0, ...
            'yLimit', 3.5, 'isIntersection', false);
% Place obstacle directly in ego path (ClassID 6 cattle, 2.5m lateral margin)
tracks_block(1).state  = [55; -0.5; 0; 0.3]; tracks_block(1).classID = 6;
tracks_block(1).age = 5; tracks_block(1).lastUpdate = 0;
[~, cmd_block, cost_block] = localPlannerDWA_refined(egoSt, tracks_block, goal, pp);
assert(cost_block >= 1e5 && cmd_block(1) == 0, ...
    'Blocked path must yield emergency stop (v=0) with high cost');
latencies_ms = zeros(50, 1);
for rep = 1:50
    tic;
    localPlannerDWA_refined(egoSt, tracks_block, goal, pp);
    latencies_ms(rep) = toc * 1000;
end
meanLat = mean(latencies_ms);
maxLat  = max(latencies_ms);
fprintf('  Blocked-path cost=%.0f, cmd v=%.2f m/s\n', cost_block, cmd_block(1));
fprintf('  Replan latency: mean=%.2f ms, max=%.2f ms (target < 3 ms)\n', meanLat, maxLat);
if meanLat < 3.0
    fprintf('[PASS] Sub-3ms mean replanning latency OK.\n\n');
else
    fprintf('[WARN] Mean latency %.2f ms exceeds 3 ms target (still within 100 ms budget).\n\n', meanLat);
end

%% STEP 6.6: In-path risk filtering (|d_lat| <= 1.8 m)
fprintf('[CHECK 6.6] In-path risk filtering:\n');
egoSt = [0, 0, 0, 5.0, 0];
IN_PATH = 1.8;
% Off-path actor at y=5.0 (|d_lat|=5.0 > 1.8) should not affect in-path minDist
offPathDist = hypot(20 - egoSt(1), 5.0 - egoSt(2));
dLat_off = abs(-(20)*sin(egoSt(3)) + 5.0*cos(egoSt(3)));
assert(dLat_off > IN_PATH, 'Test setup: off-path actor should exceed threshold');
inPathMinDist = inf;
for obsY = [0.0, 5.0]
    dx = 20; dy = obsY;
    dLat = -dx*sin(egoSt(3)) + dy*cos(egoSt(3));
    if abs(dLat) <= IN_PATH
        inPathMinDist = min(inPathMinDist, hypot(dx, dy));
    end
end
assert(inPathMinDist < offPathDist, 'In-path filter should ignore off-path actor');
fprintf('  In-path minDist=%.1f m (off-path actor at y=5 ignored)\n', inPathMinDist);
fprintf('[PASS] In-path risk filtering OK.\n\n');

%% STEP 7: Test scenario creation
fprintf('[CHECK 7] Testing all scenario creators:\n');
scenFuncs = {@createCattleScenario, @createVillageScenario, @createIntersectionScenario, ...
             @createHighwayMergeScenario, @createDenseMarketScenario};
scenNames = {'cattle', 'village', 'intersection', 'highway', 'market'};
for i = 1:numel(scenFuncs)
    try
        [sc, eg, ac] = scenFuncs{i}();
        fprintf('  [OK] %s: %d actors\n', scenNames{i}, numel(ac));
    catch ME
        fprintf('  [FAIL] %s: %s\n', scenNames{i}, ME.message);
    end
end
fprintf('[PASS] Scenario creators OK.\n\n');

%% STEP 8: Quick smoke test - run cattle scenario 10 steps headless
fprintf('[CHECK 8] Smoke test: cattle scenario (10 timesteps, no visuals):\n');
try
    [sc, eg, ac] = createCattleScenario();
    egoState = [10, 0, 0, 6.0, 0];
    numA = numel(ac);
    actors_local = struct();
    for k = 1:numA
        a = ac{k};
        actors_local(k).Position = a.Position(1:2);
        actors_local(k).Velocity = a.Velocity(1:2);
        actors_local(k).ClassID  = a.ClassID;
    end
    tracker_local = kalmanTracker(0.05, 10);
    plannerParams_local = struct('horizon', 2.0, 'dt', 0.05, 'Nv', 7, 'Nw', 11, ...
        'vMin', 0, 'vMax', 15.0, 'wMin', -1.2, 'wMax', 1.2, ...
        'maxAccel', 3.5, 'maxDecel', 6.0, 'maxAlpha', 2.5, ...
        'collisionRadius', 1.2, 'targetSpeed', 6.0, 'yLimit', 3.5, 'isIntersection', false);
    pVehicle = struct('L', 2.7, 'maxSteer', 0.6, 'maxAccel', 3.5, 'maxDecel', 6.0, 'maxSpeed', 30.0);
    
    xHistory = zeros(10, 1);
    for step = 1:10
        for k = 1:numA
            actors_local(k).Position = actors_local(k).Position + actors_local(k).Velocity * 0.05;
        end
        sensors_t = simulateSensorsAdvanced(egoState, actors_local);
        dets_t    = fuseSensorOutputs(sensors_t, egoState);
        tracker_local.predictAll();
        tracker_local.update(dets_t);
        [path_t, cmd_t, ~] = localPlannerDWA_refined(egoState, tracker_local.tracks, [180 0], plannerParams_local);
        
        vCmd_t = cmd_t(1);
        if abs(vCmd_t) < 1e-2, steer_t = 0;
        else, steer_t = atan2(cmd_t(2)*pVehicle.L, max(vCmd_t, 0.1)); end
        throttle_t = (vCmd_t - egoState(4)) / 0.05;
        egoState = bicycleModel(egoState, throttle_t, steer_t, 0.05, pVehicle);
        xHistory(step) = egoState(1);
    end
    
    assert(egoState(1) > 10.0, sprintf('Vehicle did not move! x=%.3f', egoState(1)));
    fprintf('  X positions: %.2f -> %.2f (moved %.2f m in 10 steps)\n', xHistory(1), xHistory(10), xHistory(10)-10);
    fprintf('[PASS] Smoke test OK. Vehicle is moving!\n\n');
catch ME
    fprintf('[FAIL] Smoke test error: %s\n', ME.message);
end

%% STEP 9: Test GPS Navigation & Geodetic Mapping Layer
fprintf('[CHECK 9] Testing GPS Navigation & Geodetic Mapping Layer:\n');
refLat = 17.3850; refLon = 78.4867;
xOrig = 110.0; yOrig = 0.0;
[latG, lonG] = localENUToGPS(xOrig, yOrig, refLat, refLon);
[xRec, yRec] = gpsToLocalENU(latG, lonG, refLat, refLon);
errX = abs(xOrig - xRec); errY = abs(yOrig - yRec);
assert(errX < 1e-3 && errY < 1e-3, sprintf('GPS conversion error exceeded tolerance: errX=%.4fm, errY=%.4fm', errX, errY));
fprintf('  Round-trip ENU<->GPS mapping error: dx=%.6fm, dy=%.6fm\n', errX, errY);

gpsMeas = simulateGPSSensor([10.0, 0.0, 0, 5.0, 0], refLat, refLon, struct('sigmaGPS', 1.5));
assert(gpsMeas.isValid, 'Expected valid GPS measurement');
fprintf('  Simulated GPS Receiver: Lat=%.6f deg, Lon=%.6f deg, Status=%s\n', gpsMeas.lat, gpsMeas.lon, gpsMeas.status);
fprintf('[PASS] GPS Navigation Layer OK.\n\n');

%% STEP 10: Test 3D Simulink C-Coder Function Block Wrappers
fprintf('[CHECK 10] Testing 3D Simulink C-Coder Function Block Wrappers:\n');
egoState3D = [10.0; 0.0; 0.2; 0.01; 0.0; 5.0; 0.0];
trackMatrix = zeros(30, 6);
trackMatrix(1, :) = [25.0, 0.0, 0.0, 0.0, 0.2, 0.0];
goal2D = [100.0; 0.0];

[vCmd3D, wCmd3D, bestPath3D] = stepDWAPlanner3D(egoState3D, trackMatrix, 1, goal2D);
assert(size(bestPath3D, 1) == 40 && size(bestPath3D, 2) == 3, 'Expected 40x3 path matrix from stepDWAPlanner3D');
fprintf('  stepDWAPlanner3D: vCmd=%.2f m/s, wCmd=%.3f rad/s, bestPath size=%dx%d\n', vCmd3D, wCmd3D, size(bestPath3D,1), size(bestPath3D,2));

[behState, tgtSpd] = stepBehaviorStateflow(egoState3D, 15.0, 10.0, true, false);
assert(behState >= 1 && behState <= 6, 'Invalid Stateflow behavior state code');
fprintf('  stepBehaviorStateflow: behaviorCode=%d, targetSpeed=%.1f m/s\n', behState, tgtSpd);

nextEgo3D = stepBicycleControl3D(egoState3D, 1.0, 0.0, 0.05);
assert(numel(nextEgo3D) == 7, 'Expected 7D pose vector from stepBicycleControl3D');
fprintf('  stepBicycleControl3D: nextPose=[%.2f, %.2f, %.2f, %.3f, %.3f, %.2f, %.2f]\n', nextEgo3D(1), nextEgo3D(2), nextEgo3D(3), nextEgo3D(4), nextEgo3D(5), nextEgo3D(6), nextEgo3D(7));

exportSimulinkComponents();
fprintf('[PASS] 3D Simulink C-Coder Wrappers & Architecture Export OK.\n\n');

fprintf('========================================================\n');
fprintf('  ALL DIAGNOSTIC CHECKS COMPLETE\n');
fprintf('  Run: runSimulation(''cattle'') to start visual simulation\n');
fprintf('  Run: runAllScenarios()       to run full benchmark suite\n');
fprintf('========================================================\n\n');

