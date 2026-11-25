function [bestPath, bestCmd, bestCost] = localPlannerDWA_refined(egoState, tracks, goal, params)
% LOCALPLANNERDWA_REFINED High-performance, vectorized Dynamic Window Approach local planner.
% Features Per-Obstacle Anisotropic Clearance based on obstacle ClassID taxonomy:
%   - ClassID 6 (Cattle/Animals)     : 2.5 m lateral margin
%   - ClassID 4 (Pedestrians)        : 2.0 m lateral margin
%   - ClassID 5 (Pushcarts/Bicycles) : 1.8 m lateral margin
%   - ClassID 1-3 (Vehicles/Trucks)  : 1.2 m lateral margin
%
% Inputs:
%   egoState - [x, y, yaw, v] or [x, y, yaw, v, w]
%   tracks   - Array of tracked objects (from kalmanTracker)
%   goal     - [x_goal, y_goal] target location
%   params   - Planner parameters structure
%
% Outputs:
%   bestPath - [T x 2] matrix of sampled best trajectory points
%   bestCmd  - [vCmd, wCmd] optimal linear and angular velocity commands
%   bestCost - Minimum total cost value

% Default Parameter Setup
if nargin < 4, params = struct(); end
if ~isfield(params, 'vMin'),           params.vMin = 0.0; end
if ~isfield(params, 'vMax'),           params.vMax = 12.0; end
if ~isfield(params, 'wMin'),           params.wMin = -1.2; end
if ~isfield(params, 'wMax'),           params.wMax = 1.2; end
if ~isfield(params, 'maxAccel'),       params.maxAccel = 3.5; end
if ~isfield(params, 'maxDecel'),       params.maxDecel = 6.0; end
if ~isfield(params, 'maxAlpha'),       params.maxAlpha = 2.5; end
if ~isfield(params, 'horizon'),        params.horizon = 2.0; end
if ~isfield(params, 'dt'),             params.dt = 0.05; end
if ~isfield(params, 'Nv'),             params.Nv = 11; end
if ~isfield(params, 'Nw'),             params.Nw = 21; end
if ~isfield(params, 'collisionRadius'),params.collisionRadius = 1.2; end
if ~isfield(params, 'targetSpeed'),    params.targetSpeed = params.vMax; end
if ~isfield(params, 'yLimit'),         params.yLimit = 3.5; end
if ~isfield(params, 'isIntersection'), params.isIntersection = false; end

% Parse Ego State
x0  = egoState(1);
y0  = egoState(2);
th0 = egoState(3);
v0  = egoState(4);
w0  = 0;
if numel(egoState) >= 5
    w0 = egoState(5);
end

% 1. Dynamic Window Calculation (Kinematic Constraints)
dt      = params.dt;
horizon = params.horizon;
T = max(2, round(horizon / dt));

vMin_DW = max(params.vMin, v0 - params.maxDecel * dt);
vMax_DW = min(params.vMax, v0 + params.maxAccel * dt);
wMin_DW = max(params.wMin, w0 - params.maxAlpha * dt);
wMax_DW = min(params.wMax, w0 + params.maxAlpha * dt);

% Ensure valid non-empty dynamic window bounds (always allow some speed)
if vMax_DW < vMin_DW, vMax_DW = vMin_DW; end
if wMax_DW < wMin_DW, wMax_DW = wMin_DW; end

% CRITICAL: Force non-zero speed evaluation before allowing steering changes
% Minimum speed threshold: 5 km/h (1.39 m/s) to prevent vehicle from locking up
MIN_SPEED_THRESHOLD = 1.39; % 5 km/h
vMin_DW_actual = max(vMin_DW, MIN_SPEED_THRESHOLD);
vMax_DW_actual = max(vMax_DW, max(MIN_SPEED_THRESHOLD, params.vMax * 0.3)); % At least 30% of max speed

hasTracks = ~isempty(tracks) && isstruct(tracks) && numel(tracks) > 0;

% Dynamic Free-Space Corridor Selection (relative to road boundaries +/- yLimit)
corridor_blocked = false;
target_corridor_y = 0.0;
if hasTracks && ~params.isIntersection
    blocking_lead_found = false;
    lead_obs_y = 0;
    min_d_fwd = inf;
    for tk = 1:numel(tracks)
        if isfield(tracks(tk), 'state') && ~isempty(tracks(tk).state)
            ox = tracks(tk).state(1); oy = tracks(tk).state(3);
            relX = ox - x0; relY = oy - y0;
            % Transform to ego frame
            d_fwd = relX * cos(th0) + relY * sin(th0);
            d_side = -relX * sin(th0) + relY * cos(th0);
            if d_fwd > 0.5 && d_fwd < 20.0 && abs(d_side) <= 1.0
                if d_fwd < min_d_fwd
                    min_d_fwd = d_fwd;
                    blocking_lead_found = true;
                    lead_obs_y = oy;
                end
            end
        end
    end
    
    if blocking_lead_found
        corridor_blocked = true;
        % Calculate real-time drivable lateral corridors relative to road boundary (yLimit = +/-3.5m)
        % Y_left is from lead_obs_y to +yLimit; Y_right is from -yLimit to lead_obs_y
        yLim = params.yLimit;
        width_left = max(0, yLim - lead_obs_y);
        width_right = max(0, lead_obs_y - (-yLim));
        
        % Shift the local goal candidate toward the widest available free-space corridor (+1.8m or -1.8m)
        if width_left >= width_right
            target_corridor_y = 1.8;
        else
            target_corridor_y = -1.8;
        end
    end
end

% Prioritize lateral offsets: wider and finer steering grid when blocked or obstacles present
if corridor_blocked
    % Allow wider angular velocities so vehicle can actively divert into free road space
    wMin_DW = max(params.wMin, min(wMin_DW, -0.6));
    wMax_DW = min(params.wMax, max(wMax_DW, 0.6));
    Nw_enhanced = min(params.Nw * 2, 41);
    w_grid = linspace(wMin_DW, wMax_DW, Nw_enhanced);
elseif hasTracks
    Nw_enhanced = min(params.Nw * 2, 41);
    w_grid = linspace(wMin_DW, wMax_DW, Nw_enhanced);
else
    w_grid = linspace(wMin_DW, wMax_DW, params.Nw);
end

% Reduce speed resolution to focus computational budget on steering evaluation
Nv_reduced = max(round(params.Nv / 2), 5);
v_grid = linspace(vMin_DW_actual, vMax_DW_actual, Nv_reduced);
[V_mesh, W_mesh] = meshgrid(v_grid, w_grid);

v_samples = V_mesh(:); % N x 1
w_samples = W_mesh(:); % N x 1
N = numel(v_samples);

% 2. Vectorized Trajectory Generation (Closed-form Analytical Kinematics & 3D Tensor)
t_vec = (1:T) * dt; % 1 x T

X_trajs  = zeros(N, T);
Y_trajs  = zeros(N, T);
TH_trajs = zeros(N, T);

w_abs = abs(w_samples);
zero_w_mask    = (w_abs < 1e-4);
nonzero_w_mask = ~zero_w_mask;

% Straight trajectories (w = 0)
if any(zero_w_mask)
    v_z = v_samples(zero_w_mask);
    X_trajs(zero_w_mask, :)  = x0 + (v_z * cos(th0)) * t_vec;
    Y_trajs(zero_w_mask, :)  = y0 + (v_z * sin(th0)) * t_vec;
    TH_trajs(zero_w_mask, :) = th0;
end

% Curved steering trajectories (w ~= 0)
if any(nonzero_w_mask)
    v_nz = v_samples(nonzero_w_mask);
    w_nz = w_samples(nonzero_w_mask);
    
    TH_nz = th0 + w_nz * t_vec;
    TH_trajs(nonzero_w_mask, :) = TH_nz;
    
    R_nz = v_nz ./ w_nz; % turning radius
    X_trajs(nonzero_w_mask, :) = x0 + R_nz .* (sin(TH_nz) - sin(th0));
    Y_trajs(nonzero_w_mask, :) = y0 - R_nz .* (cos(TH_nz) - cos(th0));
end

% Compute candidate elevation points Z_sample(t) across prediction horizon -> [N x T x 3] trajectory tensor
Z_trajs = getTerrainElevation(X_trajs, Y_trajs);

% Output path shape: [T x 3] trajectory tensor when selected
trajs3D = cat(3, X_trajs, Y_trajs, Z_trajs);

% 3. Drivable Road-Boundary Constraint Evaluation
road_boundary_cost = zeros(N, 1);
road_boundary_admissible = true(N, 1);

if params.isIntersection
    dist_horiz_road = max(0, abs(Y_trajs) - 3.5);
    dist_vert_road  = max(0, abs(X_trajs - 50.0) - 3.5);
    out_of_road = min(dist_horiz_road, dist_vert_road);
    max_road_viol = max(out_of_road, [], 2);
    
    road_boundary_admissible(max_road_viol > 0.8) = false;
    road_boundary_cost = 30.0 * max_road_viol.^2;
else
    max_y_dev = max(abs(Y_trajs), [], 2);
    % Only hard-reject trajectories clearly outside drivable area
    road_boundary_admissible(max_y_dev > params.yLimit + 0.3) = false;
    road_boundary_cost = 80.0 * max(0, max_y_dev - (params.yLimit - 0.3)).^2;
end

% 4. Multi-Hypothesis Obstacle Trajectory Predictions & ClassIDs
HYPOTHESES_PER_OBSTACLE = 3;
[~, predArray, predClassIDs] = predictTrajectories(tracks, horizon, dt, HYPOTHESES_PER_OBSTACLE, [x0, y0]);
P = size(predArray, 1);
M = numel(tracks);
K = HYPOTHESES_PER_OBSTACLE;
if M > 0
    K = max(1, round(P / M));
end

% 5. Vectorized 3D Per-Class Ellipsoidal Clearance & Dynamic TTC Evaluation
minDist_per_sample = inf(N, 1);
minTTC_per_sample  = inf(N, 1);
collision_cost     = zeros(N, 1);

if P > 0
    % Construct per-obstacle 3D safety bounds (a_lat, c_vert)
    r_lat_vec  = zeros(1, P);
    r_vert_vec = zeros(1, P);
    obs_velocities = zeros(P, 2); % Store obstacle velocities for directional risk
    obs_z_vec  = zeros(1, P);
    
    for p = 1:P
        cID = predClassIDs(p);
        switch cID
            case 6 % Cattle / Animal
                r_lat_vec(p)  = 2.5;
                r_vert_vec(p) = 2.0; % c = 2.0 m
            case 4 % Pedestrian
                r_lat_vec(p)  = 2.0;
                r_vert_vec(p) = 1.8; % c = 1.8 m
            case 5 % Pushcart / Bicycle
                r_lat_vec(p)  = 1.8;
                r_vert_vec(p) = 1.5; % c = 1.5 m
            otherwise % Vehicles / Trucks (ClassID 1-3)
                r_lat_vec(p)  = 1.2;
                r_vert_vec(p) = 1.5; % c = 1.5 m
        end
        
        % Adaptive Speed-Dependent Clearance Buffers
        if ~params.isIntersection
            speed_ratio = max(0, min(1.0, v0 / params.vMax));
            dynamic_scale = 0.6 + 0.4 * speed_ratio;
            r_lat_vec(p) = r_lat_vec(p) * dynamic_scale;
        end
        
        % Extract obstacle velocity and Z position from tracks using obstacleIdx = floor((p-1)/K) + 1
        obstacleIdx = floor((p - 1) / K) + 1;
        if isstruct(tracks) && numel(tracks) >= obstacleIdx
            if isfield(tracks(obstacleIdx), 'state') && ~isempty(tracks(obstacleIdx).state)
                obs_velocities(p, :) = [tracks(obstacleIdx).state(2), tracks(obstacleIdx).state(4)];
                % Extract 3D Z position if present (e.g. state(5) or Z field)
                if isfield(tracks(obstacleIdx), 'Z') && ~isempty(tracks(obstacleIdx).Z)
                    obs_z_vec(p) = tracks(obstacleIdx).Z;
                elseif numel(tracks(obstacleIdx).state) >= 6
                    obs_z_vec(p) = tracks(obstacleIdx).state(5);
                end
            end
        end
    end
    
    r_long = params.collisionRadius; % a = 1.2m longitudinal safety margin
    
    for ti = 1:T
        x_t  = X_trajs(:, ti);  % N x 1
        y_t  = Y_trajs(:, ti);  % N x 1
        z_t  = Z_trajs(:, ti);  % N x 1
        th_t = TH_trajs(:, ti); % N x 1
        
        ox_t = predArray(:, ti, 1)'; % 1 x P
        oy_t = predArray(:, ti, 2)'; % 1 x P
        
        % Pairwise displacement matrix: N x P
        dx = bsxfun(@minus, x_t, ox_t); % N x P
        dy = bsxfun(@minus, y_t, oy_t); % N x P
        dz = bsxfun(@minus, z_t, obs_z_vec); % N x P (vertical offset Delta Z)
        
        cos_th = cos(th_t); % N x 1
        sin_th = sin(th_t); % N x 1
        
        % Longitudinal and lateral relative displacements in ego body frame
        d_long = bsxfun(@times, dx, cos_th) + bsxfun(@times, dy, sin_th); % N x P
        d_lat  = bsxfun(@times, -dx, sin_th) + bsxfun(@times, dy, cos_th); % N x P
        
        % Directional/Asymmetrical Risk Mask: Inflate risk in front of obstacle heading
        directional_risk_factor = ones(N, P);
        for p = 1:P
            obs_vx = obs_velocities(p, 1);
            obs_vy = obs_velocities(p, 2);
            obs_speed = hypot(obs_vx, obs_vy);
            
            if obs_speed > 0.5 % Only apply directional risk for moving obstacles
                obs_heading = atan2(obs_vy, obs_vx);
                angles_to_traj = atan2(dy(:, p), dx(:, p)); % N x 1
                angle_diff = abs(mod(angles_to_traj - obs_heading + pi, 2*pi) - pi);
                
                front_mask = (angle_diff < pi/3);
                directional_risk_factor(front_mask, p) = 1.5; % 50% higher risk in front
                
                back_mask = (angle_diff > 2*pi/3);
                directional_risk_factor(back_mask, p) = 0.7; % 30% lower risk behind
            end
        end
        
        % In-Path Vertical Corridor Filter:
        % Bypass longitudinal speed reduction penalties / risk for objects completely clear of ego swept height (Delta Z > 2.5 m)
        vertical_clear_mask = (abs(dz) > 2.5); % N x P
        
        % 3D Ellipsoidal clearance evaluation:
        % dist_ellip3d^2 = (d_long / a)^2 + (d_lat / b)^2 + (dz / c)^2
        % dist_ellip3d = sqrt(...)
        term_long = (d_long / r_long).^2;
        term_lat  = bsxfun(@rdivide, d_lat, r_lat_vec).^2;
        term_vert = bsxfun(@rdivide, dz, r_vert_vec).^2;
        
        % Apply directional risk factor to horizontal plane component
        d_ellip3d_sq = (term_long + term_lat) .* (directional_risk_factor.^2) + term_vert;
        d_ellip3d    = sqrt(d_ellip3d_sq);
        
        % For objects completely vertically clear (Delta Z > 2.5m), set clearance to safe distance
        d_ellip3d(vertical_clear_mask) = 5.0;
        
        d_euclid = sqrt(dx.^2 + dy.^2 + dz.^2); % N x P
        d_euclid(vertical_clear_mask) = 10.0;
        
        min_d_t     = min(d_euclid,   [], 2); % N x 1
        min_ellip_t = min(d_ellip3d,  [], 2); % N x 1
        
        minDist_per_sample = min(minDist_per_sample, min_d_t);
        
        % TTC proxy: time to reach obstacle at current speed
        ttc_t = min_d_t ./ (v_samples + 1e-3);
        minTTC_per_sample = min(minTTC_per_sample, ttc_t);
        
        % Hard collision boundary check (dist_ellip3d < 0.95 = inside safety ellipsoid)
        % MUST be marked inadmissible across all fallback stages
        collision_mask = (min_ellip_t < 0.95);
        collision_cost(collision_mask) = collision_cost(collision_mask) + 5e4;
    end
end

% 6. Admissibility Filter & Dynamic Clearance Scaling
% Only reject if trajectory actually passes inside collision ellipse many times
admissible_mask = (collision_cost < 1e4) & road_boundary_admissible;

% Anti-stall fallback: relax road-boundary only — NEVER bypass collision_mask
% Trajectories with collision_cost >= 1e4 remain inadmissible (hard safety boundary)
if ~any(admissible_mask)
    admissible_mask = (collision_cost < 1e4);
    % If still none: all trajectories violate collision ellipse → emergency brake below
end

% 7. Multi-Objective Cost Function Evaluation
end_x  = X_trajs(:, end);
end_y  = Y_trajs(:, end);
end_th = TH_trajs(:, end);

% Goal progress cost with dynamic centerline relaxation when corridor is blocked
% If forward corridor is blocked, shift the effective goal target laterally toward the widest available free-space corridor (Y_target = +1.8m or -1.8m)
% Decompose goal distance into longitudinal progress and lateral deviation relative to eff_goal_y
eff_goal_x = goal(1);
eff_goal_y = goal(2);
if corridor_blocked && ~params.isIntersection
    eff_goal_y = target_corridor_y;
    dx_goal = abs(end_x - eff_goal_x);
    dy_goal = abs(end_y - eff_goal_y);
    dist_to_goal = hypot(dx_goal, 0.25 * dy_goal);
else
    dist_to_goal = hypot(end_x - eff_goal_x, end_y - eff_goal_y);
end

% Goal heading error at trajectory end (aligning with open free-space corridor when blocked)
goal_heading = atan2(eff_goal_y - end_y, eff_goal_x - end_x);
heading_err  = abs(localAngdiff(end_th, goal_heading));

% Velocity cost (prefer target cruise speed)
vel_cost = (params.targetSpeed - v_samples).^2;

% Zero-velocity penalty cost (heavy penalty for stopping in active roadway zones)
% Prevents vehicle from defaulting to v=0 when lateral evasion is possible
stop_cost = zeros(N, 1);
STOP_THRESHOLD = 1.39; % 5 km/h minimum speed threshold
stop_mask = (v_samples < STOP_THRESHOLD);
stop_cost(stop_mask) = 50.0 * (STOP_THRESHOLD - v_samples(stop_mask)).^2; % Heavy penalty for near-zero speeds

% Continuous Forward Velocity Cost Reward:
% Cost discount for trajectory samples that maintain continuous forward motion (v >= 2.5 m/s)
% via lateral nudging, making swerving around obstacles cheaper than slowing down directly behind them.
overtake_reward = zeros(N, 1);
if corridor_blocked && ~params.isIntersection
    % Target the selected corridor direction (sign matches target_corridor_y)
    lat_nudge_mask = (sign(end_y) == sign(target_corridor_y) | abs(target_corridor_y) < 1e-3) & (abs(end_y) >= 0.8) & (abs(end_y) <= 2.5);
    fast_divert_mask = (v_samples >= 2.5) & lat_nudge_mask & (collision_cost < 1e4);
    overtake_reward(fast_divert_mask) = -20.0 * (v_samples(fast_divert_mask) - 2.0);
end

% Smoothness cost
smooth_cost = 0.8 * abs(w_samples) + 0.4 * abs(w_samples - w0);

% Lateral offset preference cost (prioritize lateral movement over longitudinal deceleration)
% Encourages Frenet frame d-axis shifts when obstacle collision is predicted
lateral_offset_cost = zeros(N, 1);
if P > 0 && ~params.isIntersection && (min(minDist_per_sample) < 5.0 || corridor_blocked) % Apply when obstacles nearby or blocked on regular roadways
    % Calculate lateral deviation from centerline for each trajectory
    max_lat_dev = max(abs(Y_trajs), [], 2); % N x 1
    % Prefer moderate lateral offsets (0.8m - 2.2m) over staying centered when obstacles present
    lateral_offset_cost = 4.0 * max(0, 0.8 - max_lat_dev).^2; % Penalty for staying too centered
end

% Anisotropic clearance penalty cost (soft exponential penalty for clearance < 4.0m)
clearance_cost = zeros(N, 1);
close_mask = (minDist_per_sample < 4.0) & (minDist_per_sample > 0);
clearance_cost(close_mask) = 15.0 * (4.0 - minDist_per_sample(close_mask)).^2; % Increased penalty for better clearance

% Inverse TTC cost
ttc_cost = zeros(N, 1);
critical_ttc = (minTTC_per_sample < 3.0);
ttc_cost(critical_ttc) = 30.0 * max(0, 3.0 - minTTC_per_sample(critical_ttc));

% Combined Cost Aggregation
w_goal     = 4.0;
w_head     = 2.5;
w_vel      = 0.8;
w_stop     = 8.0; % High weight to prevent stopping
w_clear    = 3.5; % Increased clearance weight for safer operation
w_lateral  = 2.0; % Weight for lateral offset preference
w_ttc      = 4.0;
w_smooth   = 1.0;

totalCost = w_goal     * dist_to_goal    ...
          + w_head     * heading_err    ...
          + w_vel      * vel_cost       ...
          + w_stop     * stop_cost      ...
          + w_clear    * clearance_cost ...
          + w_lateral  * lateral_offset_cost ...
          + w_ttc      * ttc_cost       ...
          + w_smooth   * smooth_cost    ...
          + overtake_reward            ...
          + road_boundary_cost         ...
          + collision_cost;

% Apply admissibility mask
totalCost(~admissible_mask) = inf;

% 8. Best Trajectory & Control Command Selection
[bestCost, bestIdx] = min(totalCost);

if isinf(bestCost) || isempty(bestIdx)
    % Emergency brake: no collision-free trajectory exists — stop, do not bypass mask
    bestCmd  = [0, 0];
    fallbackX = x0 + (0:dt:dt*T)' * cos(th0);
    fallbackY = y0 + (0:dt:dt*T)' * sin(th0);
    fallbackZ = getTerrainElevation(fallbackX, fallbackY);
    bestPath = [fallbackX, fallbackY, fallbackZ];
    bestCost = 1e6;
else
    bestCmd  = [v_samples(bestIdx), w_samples(bestIdx)];
    bestPath = [X_trajs(bestIdx, :)', Y_trajs(bestIdx, :)', Z_trajs(bestIdx, :)'];
end

end

% Helper function for angular difference [-pi, pi]
function d = localAngdiff(th1, th2)
    d = mod(th1 - th2 + pi, 2*pi) - pi;
end
