function [preds, predArray, predClassIDs] = predictTrajectories(tracks, horizon, dt, K, egoPos)
% PREDICTTRAJECTORIES Multi-hypothesis trajectory prediction for tracked objects.
% Class-dependent maneuver hypotheses & proximity-based K scaling.
% Inputs:
%   tracks   - Struct array of tracks (with .state = [x; vx; y; vy] and .classID)
%   horizon  - Prediction horizon in seconds (default: 2.0)
%   dt       - Timestep in seconds (default: 0.05)
%   K        - Base number of hypothesis trajectories per obstacle (default: 3)
%   egoPos   - [x, y] optional ego vehicle position for proximity scaling
% Outputs:
%   preds        - M x 1 cell array, where preds{i} is K_i x T x 2 matrix
%   predArray    - TotalK x T x 2 matrix for fast tensor operations
%   predClassIDs - TotalK x 1 vector mapping each prediction row to obstacle ClassID

if nargin < 4 || isempty(K), K = 3; end
if nargin < 3 || isempty(dt), dt = 0.05; end
if nargin < 2 || isempty(horizon), horizon = 2.0; end
if nargin < 5, egoPos = []; end

T = round(horizon / dt);
M = numel(tracks);

if M == 0
    preds = cell(0, 1);
    predArray = zeros(0, T, 2);
    predClassIDs = zeros(0, 1);
    return;
end

% Pre-calculate total hypotheses count across all obstacles
K_per_obs = zeros(M, 1);
for i = 1:M
    k_i = K;
    cID = 1;
    if isstruct(tracks) && isfield(tracks(i), 'classID') && ~isempty(tracks(i).classID)
        cID = tracks(i).classID;
    elseif isstruct(tracks) && isfield(tracks(i), 'ClassID') && ~isempty(tracks(i).ClassID)
        cID = tracks(i).ClassID;
    end
    
    if ~isempty(egoPos) && (cID == 4 || cID == 6 || cID == 5)
        if isstruct(tracks) && isfield(tracks(i), 'state') && ~isempty(tracks(i).state)
            dEgo = hypot(tracks(i).state(1) - egoPos(1), tracks(i).state(3) - egoPos(2));
            if dEgo < 8.0
                k_i = K + 2; % Scale up hypotheses when close to high-risk classes
            end
        end
    end
    K_per_obs(i) = k_i;
end

totalK = sum(K_per_obs);

% Pre-allocate outputs
preds = cell(M, 1);
predArray = zeros(totalK, T, 2);
predClassIDs = zeros(totalK, 1);

idxCount = 0;
for i = 1:M
    k_i = K_per_obs(i);
    cID = 1;
    if isstruct(tracks) && isfield(tracks(i), 'classID') && ~isempty(tracks(i).classID)
        cID = tracks(i).classID;
    elseif isstruct(tracks) && isfield(tracks(i), 'ClassID') && ~isempty(tracks(i).ClassID)
        cID = tracks(i).ClassID;
    end
    
    % Class-dependent maneuver hypothesis spreads
    if cID == 4 || cID == 6 % Pedestrian / Cattle (high maneuver uncertainty)
        wMax = 0.35; aMin = -1.5; aMax = 1.0;
    elseif cID == 5 % Pushcart
        wMax = 0.25; aMin = -1.0; aMax = 0.8;
    else % Vehicles / Trucks (ClassID 1-3)
        wMax = 0.15; aMin = -1.0; aMax = 0.5;
    end
    
    if k_i == 3
        wOffsets = [0, wMax, -wMax];
        aOffsets = [0, aMin, aMax];
    else
        wOffsets = linspace(-wMax, wMax, k_i);
        aOffsets = linspace(aMin, aMax, k_i);
    end
    
    if isstruct(tracks) && isfield(tracks(i), 'state') && ~isempty(tracks(i).state)
        s = tracks(i).state;
        px = s(1); vx = s(2); py = s(3); vy = s(4);
    else
        px = 0; vx = 0; py = 0; vy = 0;
    end
    
    speed = hypot(vx, vy);
    heading = atan2(vy, vx);
    
    trajs = zeros(k_i, T, 2);
    for k = 1:k_i
        sp = speed;
        h = heading;
        x = px;
        y = py;
        w_k = wOffsets(min(k, numel(wOffsets)));
        a_k = aOffsets(min(k, numel(aOffsets)));
        
        for t = 1:T
            sp = max(0, sp + a_k * dt);
            h = h + w_k * dt;
            x = x + sp * cos(h) * dt;
            y = y + sp * sin(h) * dt;
            trajs(k, t, 1) = x;
            trajs(k, t, 2) = y;
        end
    end
    
    preds{i} = trajs;
    predArray(idxCount+1 : idxCount+k_i, :, :) = trajs;
    predClassIDs(idxCount+1 : idxCount+k_i, 1) = cID;
    idxCount = idxCount + k_i;
end
end
