function [vCmd, wCmd, bestPath] = stepDWAPlanner3D(egoState3D, trackStates, numTracks, goal) %#codegen
% STEPDWAPLANNER3D Simulink C-Coder compliant wrapper for 3D DWA Local Planner.
% %#codegen directive ensures Zero Dynamic Memory Allocation for Embedded Coder export.
%
% Inputs:
%   egoState3D  - [7x1] double: [x, y, z, pitch, yaw, v, w]
%   trackStates - [30x6] double: fixed pre-allocated track state matrix
%   numTracks   - double scalar: active track count (<= 30)
%   goal        - [2x1] double: target ENU coordinates
%
% Outputs:
%   vCmd        - double scalar: linear velocity command (m/s)
%   wCmd        - double scalar: yaw rate steering command (rad/s)
%   bestPath    - [40x3] double: candidate 3D trajectory points

coder.inline('never');

% Pre-allocate fixed C-array bounds
params = struct('horizon', 2.0, 'dt', 0.05, 'Nv', 11, 'Nw', 17, ...
                'vMin', 0, 'vMax', 15.0, 'wMin', -1.2, 'wMax', 1.2, ...
                'maxAccel', 3.5, 'maxDecel', 6.0, 'maxAlpha', 2.5, ...
                'collisionRadius', 1.2, 'targetSpeed', 10.0, ...
                'yLimit', 3.5, 'isIntersection', false);

% Fixed-size static struct array allocation
MAX_TRACKS = 30;
activeCount = min(max(0, floor(numTracks)), MAX_TRACKS);

% Pre-allocate fixed tracks struct array
tracks = struct('state', repmat({zeros(4,1)}, MAX_TRACKS, 1), ...
                'classID', repmat({1}, MAX_TRACKS, 1), ...
                'Z', repmat({0.0}, MAX_TRACKS, 1));

for k = 1:MAX_TRACKS
    if k <= activeCount
        tracks(k).state = [trackStates(k,1); trackStates(k,2); trackStates(k,3); trackStates(k,4)];
        tracks(k).classID = 1;
        tracks(k).Z = trackStates(k,5);
    end
end

% Extract 2D state projection for localPlannerDWA_refined
dwaEgoState = [egoState3D(1), egoState3D(2), egoState3D(5), egoState3D(6), egoState3D(7)];

% Call 3D DWA planner
[rawPath, bestCmd, ~] = localPlannerDWA_refined(dwaEgoState, tracks(1:activeCount), goal, params);

vCmd = bestCmd(1);
wCmd = bestCmd(2);

% Format bestPath as fixed-size [40x3] C-matrix
bestPath = zeros(40, 3);
nP = min(size(rawPath, 1), 40);
for i = 1:nP
    if size(rawPath, 2) >= 3
        bestPath(i, :) = rawPath(i, 1:3);
    else
        bestPath(i, 1:2) = rawPath(i, 1:2);
    end
end

end
