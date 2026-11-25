function [lateralAvailable, approaching] = checkLateralCorridor(egoState, otherActors, roadWidth)
% CHECKLATERALCORRIDOR Evaluate lateral evasion space availability and obstacle approach direction.
%
% Inputs:
%   egoState - [x, y, yaw, v, w] ego vehicle state
%   otherActors - Struct array of dynamic actors with Position, Velocity, ClassID
%   roadWidth - Total road width (default: 7.0m for 2-lane road)
%
% Outputs:
%   lateralAvailable - Boolean indicating if lateral evasion space exists
%   approaching - Boolean indicating if any obstacle is approaching ego vehicle
%
% Logic:
%   - Lateral available if ego not centered AND sufficient space on either side
%   - Approaching if obstacle moving toward ego (negative relative velocity in ego frame)

if nargin < 3 || isempty(roadWidth), roadWidth = 7.0; end

egoX = egoState(1);
egoY = egoState(2);
egoYaw = egoState(3);
egoV = egoState(4);

% Ego vehicle width approximation (2.0m)
egoWidth = 2.0;
lateralMargin = (roadWidth - egoWidth) / 2; % Available lateral space on each side

% Check if ego has lateral space for evasion
egoY_road = egoY; % Assume road center at y=0
leftSpace = egoY_road + lateralMargin;
rightSpace = lateralMargin - egoY_road;

lateralAvailable = (leftSpace > 1.5) || (rightSpace > 1.5); % Need at least 1.5m lateral space

% Check if any obstacle is approaching ego
approaching = false;
numA = numel(otherActors);

for k = 1:numA
    obsPos = otherActors(k).Position;
    obsVel = otherActors(k).Velocity;
    
    % Relative position and velocity
    dx = obsPos(1) - egoX;
    dy = obsPos(2) - egoY;
    dist = hypot(dx, dy);
    
    % Only consider obstacles within 30m range
    if dist > 30 || dist < 1.0
        continue;
    end
    
    % Transform obstacle velocity to ego body frame
    cos_yaw = cos(egoYaw);
    sin_yaw = sin(egoYaw);
    
    obsVx_body = obsVel(1) * cos_yaw + obsVel(2) * sin_yaw;
    obsVy_body = -obsVel(1) * sin_yaw + obsVel(2) * cos_yaw;
    
    % Obstacle is approaching if it has negative longitudinal velocity in ego frame
    % AND is within lateral corridor (|dy| < 3.0m)
    if obsVx_body < -0.5 && abs(dy) < 3.0
        approaching = true;
        break;
    end
end

end