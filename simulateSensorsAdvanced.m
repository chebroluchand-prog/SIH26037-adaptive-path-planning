function sensors = simulateSensorsAdvanced(egoState, otherActors, params)
% SIMULATESENSORSADVANCED Pre-allocated multi-sensor simulation (Camera, LiDAR, Radar).
% Inputs:
%   egoState    - [x, y, yaw, v]
%   otherActors - Struct array of dynamic actors with Position, Velocity, ClassID
%   params      - Range and resolution parameters
% Outputs:
%   sensors     - Struct with fields .camera, .lidar, .radar

if nargin < 3 || isempty(params)
    params.range = 60;
    params.lidarPoints = 400;
end

N_actors = numel(otherActors);

% Pre-allocate Camera detections with all 3D fields
camera = struct('ClassID', {}, 'Confidence', {}, 'bbox', {}, 'Position3D', {}, 'BoundingBox3D', {});
camIdx = 0;

for i = 1:N_actors
    pos = otherActors(i).Position;
    if numel(pos) < 3, pos = [pos(1:2), 0.0]; end
    dx = pos(1) - egoState(1);
    dy = pos(2) - egoState(2);
    r = hypot(dx, dy);
    
    cid = otherActors(i).ClassID;
    % Default height Z = 0.5m for elevated obstacles (pedestrians/cattle) and 0.0m for ground actors (vehicles/carts)
    if cid == 4 || cid == 6
        z_height = 0.5;
        lwh = [0.8, 0.8, 1.7]; % [Length, Width, Height]
    elseif cid == 5
        z_height = 0.0;
        lwh = [1.2, 1.0, 1.2];
    else
        z_height = 0.0;
        lwh = [4.5, 1.8, 1.5];
    end
    
    if r < params.range
        camIdx = camIdx + 1;
        det.ClassID = cid;
        det.Confidence = 0.75 + 0.25 * rand();
        det.Position3D = [pos(1), pos(2), z_height];
        det.BoundingBox3D = lwh;
        det.bbox = [pos(1)-0.5, pos(2)-0.5, 1.0, 1.0];
        camera(camIdx, 1) = det;
    end
end

% Pre-allocate LiDAR point cloud
lidar = zeros(0, 3);
if N_actors > 0
    ptsPerActor = round(params.lidarPoints / max(1, N_actors));
    lidarBuffer = zeros(N_actors * ptsPerActor, 3);
    lidarCount = 0;
    
    for i = 1:N_actors
        pos = otherActors(i).Position;
        if numel(pos) < 3, pos = [pos(1:2), 0.0]; end
        r = hypot(pos(1) - egoState(1), pos(2) - egoState(2));
        cid = otherActors(i).ClassID;
        z_base = 0.0;
        if cid == 4 || cid == 6, z_base = 0.5; end
        
        if r < params.range
            theta = rand(ptsPerActor, 1) * 2 * pi;
            rad = 0.3 + 0.2 * rand(ptsPerActor, 1);
            xs = pos(1) + rad .* cos(theta);
            ys = pos(2) + rad .* sin(theta);
            zs = z_base + 0.3 * rand(ptsPerActor, 1);
            
            lidarBuffer(lidarCount+1 : lidarCount+ptsPerActor, :) = [xs, ys, zs];
            lidarCount = lidarCount + ptsPerActor;
        end
    end
    lidar = lidarBuffer(1:lidarCount, :);
end

% Pre-allocate Radar targets
radar = struct('range', {}, 'azimuth', {}, 'rrRate', {}, 'ClassID', {}, 'Position3D', {}, 'BoundingBox3D', {});
radarIdx = 0;

for i = 1:N_actors
    pos = otherActors(i).Position;
    if numel(pos) < 3, pos = [pos(1:2), 0.0]; end
    vel = [0, 0, 0];
    if isfield(otherActors(i), 'Velocity')
        vTmp = otherActors(i).Velocity;
        if numel(vTmp) == 2, vel = [vTmp, 0]; else, vel = vTmp(1:3); end
    end
    
    dx = pos(1) - egoState(1);
    dy = pos(2) - egoState(2);
    r = hypot(dx, dy);
    cid = otherActors(i).ClassID;
    if cid == 4 || cid == 6, z_height = 0.5; lwh = [0.8, 0.8, 1.7];
    elseif cid == 5, z_height = 0.0; lwh = [1.2, 1.0, 1.2];
    else, z_height = 0.0; lwh = [4.5, 1.8, 1.5]; end
    
    if r < params.range
        radarIdx = radarIdx + 1;
        az = atan2(dy, dx) - egoState(3);
        rr = r + 0.1 * randn();
        rrRate = (vel(1) * cos(az) + vel(2) * sin(az)) + 0.1 * randn();
        
        target.range = rr;
        target.azimuth = az;
        target.rrRate = rrRate;
        target.ClassID = cid;
        target.Position3D = [pos(1), pos(2), z_height];
        target.BoundingBox3D = lwh;
        radar(radarIdx, 1) = target;
    end
end

sensors.camera = camera;
sensors.lidar = lidar;
sensors.radar = radar;
end
