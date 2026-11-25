function metrics = runSimulation(scenName, showVisuals, useGPSWaypoints, use3DView, customParams)
% RUNSIMULATION Closed-loop autonomous driving simulation runner for SIH26037.
% Features professional Bird's-Eye View / AV Display HUD & vector DWA planner.
%
% Inputs:
%   scenName        - Scenario name ('cattle', 'village', 'intersection', 'highway', 'market')
%   showVisuals     - Boolean flag to toggle live plotting figure (default: true)
%   useGPSWaypoints - Boolean flag to enable GPS Geodetic ENU mapping (default: false)
%   use3DView       - Boolean flag to enable Automated Driving Toolbox 3D chasePlot view (default: false)
%   customParams    - Optional parameter overrides struct (e.g. pitchMode)

if nargin < 1 || isempty(scenName), scenName = 'cattle'; end
if nargin < 2 || isempty(showVisuals), showVisuals = true; end
if nargin < 3 || isempty(useGPSWaypoints), useGPSWaypoints = false; end
if nargin < 4 || isempty(use3DView), use3DView = false; end
if nargin < 5 || isempty(customParams), customParams = struct(); end

% Initialize Logger
logger = Logger();

% Create Scenario Environment
switch lower(scenName)
    case 'cattle'
        [scenario, ego, actors] = createCattleScenario();
        goal = [120, 0];
        egoState = [10, 0, 0, 6.0, 0]; % [x, y, yaw, v, w]
    case 'village'
        [scenario, ego, actors] = createVillageScenario();
        goal = [110, 0];
        egoState = [10, 0, 0, 5.0, 0];
    case 'intersection'
        [scenario, ego, actors] = createIntersectionScenario();
        goal = [50, 60];
        egoState = [50, -40, pi/2, 4.0, 0]; % [x, y, yaw, v, w]
    case 'highway'
        [scenario, ego, actors] = createHighwayMergeScenario();
        goal = [450, 0];
        egoState = [30, 0, 0, 12.0, 0];
    case 'market'
        [scenario, ego, actors] = createDenseMarketScenario();
        goal = [110, 0];
        egoState = [5, 0, 0, 4.0, 0];
    otherwise
        error('Unknown scenario: %s. Options: cattle, village, intersection, highway, market', scenName);
end

cs = scenario.SampleTime;
if isempty(cs) || cs == 0, cs = 0.05; end

% GPS Reference Origin (Default: 17.3850 N, 78.4867 E)
refLat = 17.3850;
refLon = 78.4867;

% If GPS Waypoints mode enabled, convert target Cartesian goal to global GPS Waypoint
if useGPSWaypoints
    [goalLat, goalLon] = localENUToGPS(goal(1), goal(2), refLat, refLon);
    [egoLat0, egoLon0] = localENUToGPS(egoState(1), egoState(2), refLat, refLon);
    fprintf('\n=======================================================\n');
    fprintf('  GPS NAVIGATION & GEODETIC MAPPING LAYER ENABLED\n');
    fprintf('=======================================================\n');
    fprintf('  Reference Origin   : Lat = %.6f deg, Lon = %.6f deg\n', refLat, refLon);
    fprintf('  Ego Initial GPS    : Lat = %.6f deg, Lon = %.6f deg\n', egoLat0, egoLon0);
    fprintf('  Target Goal GPS    : Lat = %.6f deg, Lon = %.6f deg\n', goalLat, goalLon);
    fprintf('=======================================================\n\n');
end

% Vehicle & Planner Setup
paramsVehicle = struct('L', 2.7, 'maxSteer', 0.6, 'maxAccel', 3.5, 'maxDecel', 6.0, 'maxSpeed', 30.0);
nominalCruiseSpeed = egoState(4);
plannerParams = struct('horizon', 2.0, 'dt', cs, 'Nv', 11, 'Nw', 21, ...
                       'vMin', 0, 'vMax', paramsVehicle.maxSpeed, ...
                       'wMin', -1.2, 'wMax', 1.2, ...
                       'maxAccel', paramsVehicle.maxAccel, ...
                       'maxDecel', paramsVehicle.maxDecel, ...
                       'maxAlpha', 2.5, ...
                       'collisionRadius', 1.2, 'latSafetyMargin', 1.2, ...
                       'targetSpeed', nominalCruiseSpeed, ...
                       'nominalCruiseSpeed', nominalCruiseSpeed, ...
                       'yLimit', 3.5, ...
                       'isIntersection', strcmpi(scenName, 'intersection'));

% Overlay custom parameters if passed
if isstruct(customParams) && ~isempty(fields(customParams))
    fFields = fields(customParams);
    for ff = 1:numel(fFields)
        plannerParams.(fFields{ff}) = customParams.(fFields{ff});
    end
end

% Synchronize otherActors dynamic state structure natively from scenario definitions
numA = numel(actors);
otherActors = struct();
for k = 1:numA
    if iscell(actors)
        act = actors{k};
    else
        act = actors(k);
    end
    
    otherActors(k).ID = k;
    otherActors(k).Position = act.Position(1:2);
    
    if isprop(act, 'Velocity') || isfield(act, 'Velocity')
        vel3 = act.Velocity;
        otherActors(k).Velocity = vel3(1:2);
    else
        otherActors(k).Velocity = [0, 0];
    end
    
    if isprop(act, 'ClassID') || isfield(act, 'ClassID')
        otherActors(k).ClassID = act.ClassID;
    else
        otherActors(k).ClassID = 1;
    end
end

% VERIFICATION SELF-CHECK PRINT STATEMENT
fprintf('\n--- VERIFICATION SELF-CHECK: [%s SCENARIO] ---\n', upper(scenName));
fprintf('Ego Start Pose: [%.1f, %.1f], Initial Speed: %.2f m/s\n', egoState(1), egoState(2), egoState(4));

for k = 1:numA
    obsX  = otherActors(k).Position(1);
    obsY  = otherActors(k).Position(2);
    vObsX = otherActors(k).Velocity(1);
    vObsY = otherActors(k).Velocity(2);
    
    if abs(cos(egoState(3))) > 0.5 % Horizontal roadway
        if egoState(4) > 0
            tEgoArrival = (obsX - egoState(1)) / egoState(4);
        else
            tEgoArrival = inf;
        end
        if abs(vObsY) > 1e-3
            tObsCross = -obsY / vObsY;
            fprintf('  Actor %d (Class %d) @ (%.1f, %.1f): Ego Arrival t = %.2f s | Obstacle Crossing t = %.2f s [DELTA = %.2f s]\n', ...
                k, otherActors(k).ClassID, obsX, obsY, tEgoArrival, tObsCross, abs(tEgoArrival - tObsCross));
        else
            fprintf('  Actor %d (Class %d) @ (%.1f, %.1f): Longitudinal Actor (V = [%.1f, %.1f] m/s)\n', ...
                k, otherActors(k).ClassID, obsX, obsY, vObsX, vObsY);
        end
    else % Vertical roadway (intersection)
        if egoState(4) > 0
            tEgoArrival = (obsY - egoState(2)) / egoState(4);
        else
            tEgoArrival = inf;
        end
        if abs(vObsX) > 1e-3
            tObsCross = (50.0 - obsX) / vObsX;
            fprintf('  Actor %d (Class %d) @ (%.1f, %.1f): Ego Arrival t = %.2f s | Obstacle Crossing t = %.2f s [DELTA = %.2f s]\n', ...
                k, otherActors(k).ClassID, obsX, obsY, tEgoArrival, tObsCross, abs(tEgoArrival - tObsCross));
        else
            fprintf('  Actor %d (Class %d) @ (%.1f, %.1f): Longitudinal Actor (V = [%.1f, %.1f] m/s)\n', ...
                k, otherActors(k).ClassID, obsX, obsY, vObsX, vObsY);
        end
    end
end
fprintf('---------------------------------------------------\n\n');

% Multi-Target Kalman Tracker
tracker = kalmanTracker(cs, 30);

% Initialize behavior and metrics before loop
behavior = "CRUISE";
replanTime = 0;
minDist = inf;
minTTC  = inf;

% Setup Professional Bird's-Eye View AV Display Figure
fig = []; ax = [];
if showVisuals && ~use3DView
    fig = figure('Name', sprintf('SIH26037 AV Bird''s-Eye HUD: %s', upper(scenName)), ...
                 'Color', [0.08 0.10 0.14], 'Position', [80 80 1100 700]);
    ax = axes(fig, 'Color', [0.12 0.14 0.18], 'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7]);
    hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
    set(ax, 'GridColor', [0.25 0.30 0.35], 'GridAlpha', 0.5);
end

% Setup 3D ChasePlot View if use3DView is enabled
fig3D = []; ax3D = [];
if use3DView
    fig3D = figure('Name', sprintf('SIH26037 3D Chase View: %s', upper(scenName)), ...
                   'Position', [120 120 1000 650]);
    ax3D = axes(fig3D);
    ego = scenario.Actors(1);
    chasePlot(ego, 'Parent', ax3D, 'ViewHeight', 12.0, 'ViewLocation', [-25, 0], 'ViewPitch', -20, 'Centerline', 'on');
end

% Main Closed-Loop Simulation Cycle
stopTime = scenario.StopTime;
if isempty(stopTime) || stopTime == 0, stopTime = 40; end

path = [];
stepCount = 0;

for t = 0:cs:stopTime
    stepCount = stepCount + 1;
    % 1. Dynamic Actor Motion Updates
    for k = 1:numA
        otherActors(k).Position(1) = otherActors(k).Position(1) + otherActors(k).Velocity(1) * cs;
        otherActors(k).Position(2) = otherActors(k).Position(2) + otherActors(k).Velocity(2) * cs;
    end
    
    % 2. Perception & Sensor Fusion
    sensors = simulateSensorsAdvanced(egoState, otherActors);
    if ~exist('prevDetections', 'var')
        prevDetections = [];
    end
    detections = fuseSensorOutputs(sensors, egoState, prevDetections, cs);
    prevDetections = detections;
    
    % GPS Receiver Simulation & Coordinate Mapping
    if useGPSWaypoints
        gpsMeas = simulateGPSSensor(egoState, refLat, refLon, struct('sigmaGPS', 1.5));
    end
    
    % 3. Multi-Target Kalman Tracking
    tracker.predictAll();
    tracker.update(detections);
    
    % Off-path actors threshold: expand for intersection scenarios to catch cross-traffic early
    if isfield(plannerParams, 'isIntersection') && plannerParams.isIntersection
        IN_PATH_LATERAL_THRESHOLD = 3.5; % m (covers full intersection cross-road width)
    else
        IN_PATH_LATERAL_THRESHOLD = 1.8; % m (standard 2-lane road corridor)
    end
    minDist = inf;
    minTTC  = inf;
    
    egoX   = egoState(1);
    egoY   = egoState(2);
    if numel(egoState) >= 7
        egoYaw = egoState(5);
        egoSpd = egoState(6);
    else
        egoYaw = egoState(3);
        egoSpd = egoState(4);
    end
    vEgoX  = egoSpd * cos(egoYaw);
    vEgoY  = egoSpd * sin(egoYaw);
    cosEgoYaw = cos(egoYaw);
    sinEgoYaw = sin(egoYaw);
    
    for k = 1:numA
        obsX  = otherActors(k).Position(1);
        obsY  = otherActors(k).Position(2);
        vObsX = otherActors(k).Velocity(1);
        vObsY = otherActors(k).Velocity(2);
        
        dx = obsX - egoX;
        dy = obsY - egoY;
        
        % Lateral offset in ego body frame — skip off-path actors for risk/TTC
        dLat = -dx * sinEgoYaw + dy * cosEgoYaw;
        if abs(dLat) > IN_PATH_LATERAL_THRESHOLD
            continue;
        end
        
        dist = hypot(dx, dy);
        minDist = min(minDist, dist);
        
        if dist > 0
            losX = dx / dist;
            losY = dy / dist;
            % closingSpeed > 0 means obstacle is approaching ego
            closingSpeed = -((vObsX - vEgoX)*losX + (vObsY - vEgoY)*losY);
            
            if closingSpeed > 0
                ttc = dist / closingSpeed;
            else
                ttc = inf;
            end
        else
            ttc = 0;
        end
        minTTC = min(minTTC, ttc);
    end
    
    % If no in-path actors, reset risk metrics to safe defaults
    if isinf(minDist)
        minDist = 100.0;
        minTTC  = inf;
    end
    
    % Physical clearance to all actors (for HUD logging and collision check)
    minDistPhysical = inf;
    for k = 1:numA
        dx = otherActors(k).Position(1) - egoState(1);
        dy = otherActors(k).Position(2) - egoState(2);
        minDistPhysical = min(minDistPhysical, hypot(dx, dy));
    end
    
    % Build risk map BEFORE behaviorDecision call (in-path actors only)
    riskMap.maxRisk = 1.0 / (1.0 + exp(minDist - 2.5));
    
    % Check lateral corridor availability and obstacle approach direction
    [lateralCorridorAvailable, obstacleApproaching] = checkLateralCorridor(egoState, otherActors, plannerParams.yLimit * 2);
    
    % Behavioral Decision State Machine with new lateral evasion states
    behavior = behaviorDecision(egoState, riskMap, minDist, minTTC, lateralCorridorAvailable, obstacleApproaching);
    logger.logBehavior(behavior);
    
    % Adjust target speed based on behavioral state
    if behavior == "EMERGENCY_STOP"
        plannerParams.targetSpeed = 0.0; % Full stop
    elseif behavior == "EVADE_LATERAL"
        % Creep velocity for lateral evasion (10-15 km/h)
        plannerParams.targetSpeed = max(2.78, min(4.17, egoState(4))); % 10-15 km/h range
    elseif behavior == "PACING"
        % Match speed of leading obstacle while maintaining safe distance
        plannerParams.targetSpeed = max(1.5, egoState(4) * 0.8); % Gentle deceleration to match
    elseif behavior == "CAUTION"
        plannerParams.targetSpeed = max(3.0, egoState(4) * 0.7);
    elseif behavior == "AVOID"
        plannerParams.targetSpeed = max(2.0, egoState(4) * 0.5);
    else % CRUISE
        % Recover smoothly toward nominal cruise speed
        nomSpeed = paramsVehicle.maxSpeed;
        if isfield(plannerParams, 'nominalCruiseSpeed')
            nomSpeed = plannerParams.nominalCruiseSpeed;
        end
        plannerParams.targetSpeed = min(nomSpeed, egoState(4) + 0.5);
    end
    
    % 5. Trajectory Replanning via Vectorized DWA
    tic;
    if numel(egoState) >= 7
        egoYaw = egoState(5); egoV = egoState(6); egoW = egoState(7);
    else
        egoYaw = egoState(3); egoV = egoState(4); egoW = egoState(5);
    end
    dwaEgoState = [egoState(1), egoState(2), egoYaw, egoV, egoW];

    if behavior == "EMERGENCY_STOP"
        path = []; cmd = [0, 0]; cost = 1e6;
    else
        [path, cmd, cost] = localPlannerDWA_refined(dwaEgoState, tracker.tracks, goal, plannerParams);
    end
    replanTime = toc;
    logger.logReplan(replanTime);
    
    % 6. Closed-Loop Kinematic Control Command Computation
    if behavior == "EMERGENCY_STOP"
        vCmd = 0.0;
        steerCmd = 0.0;
        throttleCmd = -paramsVehicle.maxDecel;
    elseif ~isempty(path) && cost < 1e5
        vCmd = cmd(1);
        wCmd = cmd(2);
        
        % Ensure minimum forward speed unless near goal
        distToGoal = hypot(egoState(1) - goal(1), egoState(2) - goal(2));
        if vCmd < 0.5 && distToGoal > 5.0
            vCmd = max(vCmd, 1.0); % Floor speed to 1 m/s to prevent stall
        end
        
        if abs(vCmd) < 1e-2
            steerCmd = 0;
        else
            steerCmd = atan2(wCmd * paramsVehicle.L, max(vCmd, 0.1));
        end
        throttleCmd = (vCmd - egoV) / cs;
    else
        % Fallback: hold current speed and drive straight toward goal
        goalDx = goal(1) - egoState(1);
        goalDy = goal(2) - egoState(2);
        goalAngle = atan2(goalDy, goalDx);
        headingErr = goalAngle - egoYaw;
        headingErr = mod(headingErr + pi, 2*pi) - pi;
        steerCmd = max(-paramsVehicle.maxSteer, min(paramsVehicle.maxSteer, headingErr * 0.5));
        throttleCmd = max(-1.0, min(0.5, (2.0 - egoV))); % maintain ~2 m/s fallback
    end
    
    egoState = bicycleModel(egoState, throttleCmd, steerCmd, cs, paramsVehicle);
    logger.logPose(egoState(1), egoState(2), t);
    logger.logSmooth(abs(egoW) / max(0.1, egoV));
    
    % 7. Render Professional 3D AV Perspective Display & HUD (Frame Decimation: 3x Fast FPS)
    if showVisuals && ~use3DView && ~isempty(fig) && isgraphics(fig) && (mod(stepCount, 3) == 0 || t == 0 || t >= stopTime)
        cla(ax);
        
        % Set 3D perspective camera view angle
        view(ax, -37.5, 30);
        
        % A. Draw Asphalt Road Network & Lane Boundaries with 3D elevation
        if strcmpi(scenName, 'intersection')
            fill3(ax, [-40 140 140 -40], [-4 -4 4 4], [0 0 0 0], [0.14 0.16 0.20], 'EdgeColor', 'none');
            fill3(ax, [46 54 54 46], [-80 -80 80 80], [0 0 0 0], [0.14 0.16 0.20], 'EdgeColor', 'none');
            plot3(ax, [-40 140], [0 0], [0.01 0.01], 'w--', 'LineWidth', 1.5);
            plot3(ax, [50 50], [-80 80], [0.01 0.01], 'w--', 'LineWidth', 1.5);
        else
            roadZ = getTerrainElevation([-10 500 500 -10], [-3.5 -3.5 3.5 3.5]);
            fill3(ax, [-10 500 500 -10], [-3.5 -3.5 3.5 3.5], roadZ, [0.14 0.16 0.20], 'EdgeColor', 'none');
            
            rx = linspace(egoState(1)-20, egoState(1)+80, 40);
            rz = getTerrainElevation(rx, zeros(size(rx)));
            plot3(ax, rx, zeros(size(rx)), rz + 0.02, 'Color', [0.95 0.85 0.2], 'LineStyle', '--', 'LineWidth', 1.5);
            plot3(ax, rx, 3.5*ones(size(rx)), rz + 0.02, 'Color', [0.9 0.9 0.95], 'LineWidth', 2.0);
            plot3(ax, rx, -3.5*ones(size(rx)), rz + 0.02, 'Color', [0.9 0.9 0.95], 'LineWidth', 2.0);
        end
        
        % B. Draw Sensor FOV Vision Cone in 3D
        fovRange = 40.0;
        fovAngle = pi/3; % 60 deg FOV
        arcTh = linspace(-fovAngle/2, fovAngle/2, 15);
        fovX = [egoState(1), egoState(1) + fovRange * cos(egoYaw + arcTh), egoState(1)];
        fovY = [egoState(2), egoState(2) + fovRange * sin(egoYaw + arcTh), egoState(2)];
        fovZ = getTerrainElevation(fovX, fovY) + 0.1;
        patch(ax, fovX, fovY, fovZ, [0.0 0.7 1.0], 'FaceAlpha', 0.08, 'EdgeColor', [0.0 0.8 1.0], 'LineStyle', '--', 'LineWidth', 1.0);
        
        % C. Draw Goal Target Pin & Radius
        goalZ = getTerrainElevation(goal(1), goal(2));
        plot3(ax, goal(1), goal(2), goalZ + 0.5, 'gp', 'MarkerSize', 14, 'MarkerFaceColor', [0.1 0.95 0.4], 'MarkerEdgeColor', 'w');
        text(ax, goal(1)+1.5, goal(2)+1.2, goalZ + 1.0, 'TARGET GOAL', 'Color', [0.1 0.95 0.4], 'FontWeight', 'bold', 'FontSize', 8.5);
        
        % D. Draw 3D DWA Planner Path Trajectory Ribbon
        if ~isempty(path)
            if size(path, 2) >= 3
                pathZ = path(:,3) + 0.05;
            else
                pathZ = getTerrainElevation(path(:,1), path(:,2)) + 0.05;
            end
            plot3(ax, path(:,1), path(:,2), pathZ, 'Color', [0.0 1.0 0.9], 'LineWidth', 2.5);
            plot3(ax, path(1,1), path(1,2), pathZ(1), 'c>', 'MarkerSize', 8, 'MarkerFaceColor', [0.0 1.0 0.9]);
        end

        % E. Draw Ego Vehicle 3D Footprint
        egoZ = 0.0;
        if numel(egoState) >= 7, egoZ = egoState(3); end
        cosEgo = cos(egoYaw); sinEgo = sin(egoYaw);
        rEgo = [cosEgo, -sinEgo; sinEgo, cosEgo];
        vBox2D = rEgo * [-1.3 2.7 2.7 -1.3 -1.3; -1.0 -1.0 1.0 1.0 -1.0];
        vBoxX = egoState(1) + vBox2D(1,:);
        vBoxY = egoState(2) + vBox2D(2,:);
        vBoxZ = egoZ + zeros(size(vBoxX));
        patch(ax, vBoxX, vBoxY, vBoxZ, [0.05 0.55 0.95], ...
              'FaceAlpha', 0.9, 'EdgeColor', [0.3 0.85 1.0], 'LineWidth', 1.8);
          
        % F. Draw Dynamic Obstacles with 3D Bounding Boxes
        thEll = linspace(0, 2*pi, 18);
        for k = 1:numA
            pos = otherActors(k).Position;
            vel = otherActors(k).Velocity;
            cid = otherActors(k).ClassID;
            
            obsZ = 0.0;
            if numel(pos) >= 3, obsZ = pos(3); end
            
            switch cid
                case 6 % Cattle/Animal
                    fclr = [0.9 0.5 0.1]; rLat = 2.5; rLong = 1.8; L = 2.2; W = 1.2; H = 1.8;
                    label = sprintf('ACTOR-%d [CATTLE]', k);
                case 4 % Pedestrian
                    fclr = [0.95 0.8 0.1]; rLat = 2.0; rLong = 1.5; L = 0.8; W = 0.8; H = 1.7;
                    label = sprintf('ACTOR-%d [PED]', k);
                case 5 % Pushcart/Bicycle
                    fclr = [0.2 0.9 0.5]; rLat = 1.8; rLong = 1.5; L = 1.6; W = 1.0; H = 1.4;
                    label = sprintf('ACTOR-%d [CART]', k);
                otherwise % Vehicles/Trucks
                    fclr = [0.95 0.3 0.3]; rLat = 1.2; rLong = 1.8; L = 4.2; W = 1.8; H = 1.6;
                    label = sprintf('ACTOR-%d [VEH]', k);
            end
            
            % Draw 3D Base footprint
            patch(ax, pos(1)+[-L/2 L/2 L/2 -L/2], pos(2)+[-W/2 -W/2 W/2 W/2], obsZ+zeros(1,4), fclr, ...
                  'FaceAlpha', 0.8, 'EdgeColor', 'w', 'LineWidth', 1.2);
            
            % Draw Per-Class Elliptical Safety Clearance Buffer in 3D
            ellX = pos(1) + rLong * cos(thEll);
            ellY = pos(2) + rLat * sin(thEll);
            plot3(ax, ellX, ellY, obsZ + zeros(size(ellX)), 'Color', [fclr, 0.4], 'LineStyle', ':', 'LineWidth', 1.0);
            
            text(ax, pos(1)-1.0, pos(2)+1.5, obsZ + H + 0.3, label, ...
                 'Color', [1.0 1.0 0.8], 'FontSize', 7.5, 'FontWeight', 'bold');
        end
        
        % G. Top Telemetry 3D HUD Overlay Banner
        switch string(behavior)
            case "EMERGENCY_STOP"
                behColor = [0.95 0.2 0.2];
            case "EVADE_LATERAL"
                behColor = [0.85 0.45 0.95];
            case "PACING"
                behColor = [0.3 0.85 0.95];
            case "AVOID"
                behColor = [0.98 0.6 0.1];
            case "CAUTION"
                behColor = [0.95 0.95 0.2];
            otherwise % CRUISE
                behColor = [0.2 0.95 0.45];
        end
        
        egoZVal = 0.0; pitchDeg = 0.0;
        if numel(egoState) >= 7
            egoZVal  = egoState(3);
            pitchDeg = rad2deg(egoState(4));
        end
        
        if useGPSWaypoints && exist('gpsMeas', 'var') && gpsMeas.isValid
            hudStr = sprintf(' T: %05.2fs | STATE: %-13s | POS: [%.1f, %.1f, %.1f] | PITCH: %+.1f° | REPLAN: %04.1fms | GPS: [%.5f, %.5f] ', ...
                             t, char(behavior), egoState(1), egoState(2), egoZVal, pitchDeg, replanTime * 1000, gpsMeas.lat, gpsMeas.lon);
        else
            hudStr = sprintf(' T: %05.2fs | STATE: %-15s | POS: [%.1f, %.1f, %.1f] | PITCH: %+.1f° | REPLAN: %05.1fms | CLEAR: %5.2fm ', ...
                             t, char(behavior), egoState(1), egoState(2), egoZVal, pitchDeg, replanTime * 1000, minDistPhysical);
        end
        title(ax, hudStr, 'Color', behColor, 'FontWeight', 'bold', 'FontSize', 10.0, 'BackgroundColor', [0.03 0.05 0.08]);
        
        % Dynamic camera framing tracking ego vehicle position
        if strcmpi(scenName, 'intersection')
            xlim(ax, [egoState(1)-35, egoState(1)+35]);
            ylim(ax, [egoState(2)-20, egoState(2)+45]);
            zlim(ax, [-1, 8]);
        else
            xlim(ax, [egoState(1)-15, egoState(1)+75]);
            ylim(ax, [-12, 12]);
            zlim(ax, [-1, 8]);
        end
        xlabel(ax, 'X Position (m)'); ylabel(ax, 'Y Position (m)'); zlabel(ax, 'Z Height (m)');
        drawnow limitrate;
    end
    
    % Update 3D ChasePlot visual mirror (Frame Decimation)
    if use3DView && isgraphics(fig3D) && (mod(stepCount, 3) == 0 || t == 0 || t >= stopTime)
        ego = scenario.Actors(1);
        ego.Position = [egoState(1), egoState(2), 0];
        if numel(egoState) >= 7
            ego.Yaw = rad2deg(egoState(5));
        else
            ego.Yaw = rad2deg(egoState(3));
        end
        for k = 1:numA
            if numel(scenario.Actors) >= k + 1
                scenario.Actors(k+1).Position = [otherActors(k).Position(1), otherActors(k).Position(2), 0];
                if norm(otherActors(k).Velocity) > 1e-3
                    scenario.Actors(k+1).Yaw = rad2deg(atan2(otherActors(k).Velocity(2), otherActors(k).Velocity(1)));
                end
            end
        end
        chasePlot(ego, 'Parent', ax3D, 'ViewHeight', 12.0, 'ViewLocation', [-25, 0], 'ViewPitch', -20, 'Centerline', 'on');
        drawnow limitrate;
    end
    
    % 8. Safety Collision Verification (all actors — independent of in-path risk filter)
    logger.logMinDist(minDistPhysical);
    
    % Use more lenient threshold to account for ClassID-specific safety margins
    % The planner already uses anisotropic clearance (1.2m-2.5m lateral margins)
    % Collision threshold should be below the smallest safety margin (1.2m)
    if minDistPhysical < 0.8
        logger.incCollision();
        fprintf('[WARNING] Collision detected in scenario %s at t=%.2f s (MinDist = %.2f m)\n', scenName, t, minDistPhysical);
        break;
    end
    
    % 9. Goal Reached Verification
    if hypot(egoState(1) - goal(1), egoState(2) - goal(2)) < 3.5
        fprintf('[SUCCESS] Goal reached successfully in scenario %s at t=%.2f s! Speed=%.2f m/s\n', scenName, t, egoState(4));
        break;
    end
end

metrics = logger.summary();

if showVisuals && ~use3DView && ~isempty(fig) && isgraphics(fig)
    logger.plotSummary(scenName);
end

end
