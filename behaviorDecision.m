function behavior = behaviorDecision(egoState, riskMap, minDist, ttc, lateralCorridorAvailable, obstacleApproaching)
% BEHAVIORDECISION Stateflow-compatible behavioral decision state machine.
% Evaluates safety thresholds and intrusion risks in Indian traffic scenarios.
%
% Behaviors:
%   "EMERGENCY_STOP" - Immediate full deceleration (ONLY if TTC < 1.0s AND no lateral corridors)
%   "EVADE_LATERAL" - Active lateral evasion at creep velocity (10-15 km/h) when lateral space available
%   "PACING"        - Match speed of leading obstacle while maintaining safe following distance
%   "AVOID"         - Active lateral nudging/evasive steering (minDist < 3.0m or TTC < 2.2s)
%   "CAUTION"       - Reduced cruise speed, enhanced clearance monitoring (minDist < 6.0m or TTC < 4.0s)
%   "CRUISE"        - Nominal path tracking at cruise target speed
%
% Inputs:
%   egoState - [x, y, yaw, v, w] ego vehicle state
%   riskMap - Risk assessment structure with maxRisk field
%   minDist - Minimum distance to any obstacle
%   ttc - Minimum time-to-collision
%   lateralCorridorAvailable - Boolean indicating if lateral evasion space exists (optional, default: true)
%   obstacleApproaching - Boolean indicating if obstacle is approaching ego (optional, default: false)

if nargin < 4 || isempty(ttc), ttc = inf; end
if nargin < 3 || isempty(minDist), minDist = inf; end
if nargin < 2 || isempty(riskMap), riskMap.maxRisk = 0; end
if nargin < 5 || isempty(lateralCorridorAvailable), lateralCorridorAvailable = true; end
if nargin < 6 || isempty(obstacleApproaching), obstacleApproaching = false; end

v = egoState(4);
maxRisk = 0;
if isfield(riskMap, 'maxRisk')
    maxRisk = riskMap.maxRisk;
end

% CREEP_VELOCITY: 10-15 km/h (2.78-4.17 m/s) for lateral evasion
CREEP_VELOCITY_MIN = 2.78; % 10 km/h
CREEP_VELOCITY_MAX = 4.17; % 15 km/h

% 1. Emergency Stop State Trigger (ONLY when absolutely unavoidable)
% Only trigger if TTC < 1.0s AND no lateral corridors available
if (ttc < 1.0 && ~lateralCorridorAvailable) || minDist < 0.8 || maxRisk > 0.98
    behavior = "EMERGENCY_STOP";

% 2. EVADE_LATERAL State Trigger (Lateral evasion at creep velocity)
% Trigger when obstacle approaching AND lateral corridors available AND moderate risk
elseif obstacleApproaching && lateralCorridorAvailable && (minDist < 4.0 || ttc < 3.0 || maxRisk > 0.5)
    behavior = "EVADE_LATERAL";

% 3. PACING State Trigger (Match speed of leading obstacle)
% Trigger when obstacle ahead in same lane AND lateral evasion not preferred
elseif ~obstacleApproaching && (minDist < 5.0 && minDist > 2.0) && (ttc > 2.0 && ttc < 5.0)
    behavior = "PACING";

% 4. Avoidance / Evasive Lateral Nudging State Trigger
% Allows dynamic lateral nudging around dynamic objects when lateral space permits
elseif minDist < 3.0 || ttc < 2.2 || maxRisk > 0.65
    behavior = "AVOID";

% 5. Caution State Trigger (Reduced speed, active monitoring)
elseif minDist < 6.0 || ttc < 4.0 || maxRisk > 0.35 || v > 10.0
    behavior = "CAUTION";

% 6. Nominal Cruise State
else
    behavior = "CRUISE";
end

end