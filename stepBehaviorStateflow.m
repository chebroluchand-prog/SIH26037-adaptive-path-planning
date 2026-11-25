function [behaviorState, targetSpeed] = stepBehaviorStateflow(egoState3D, minDist, minTTC, latAvailable, obsApproaching) %#codegen
% STEPBEHAVIORSTATEFLOW Simulink Stateflow C-Coder compliant wrapper for behavioral state machine.
% %#codegen directive ensures Zero Dynamic Memory Allocation for Embedded Coder export.
%
% Inputs:
%   egoState3D     - [7x1] double: [x, y, z, pitch, yaw, v, w]
%   minDist        - double scalar: minimum physical clearance (m)
%   minTTC         - double scalar: minimum time-to-collision (s)
%   latAvailable   - logical: true if lateral evasion corridor is clear
%   obsApproaching - logical: true if obstacle is moving toward ego
%
% Outputs:
%   behaviorState  - int32: 1=CRUISE, 2=CAUTION, 3=PACING, 4=AVOID, 5=EVADE_LATERAL, 6=EMERGENCY_STOP
%   targetSpeed    - double scalar: target speed command (m/s)

coder.inline('never');

riskMap = struct('maxRisk', 1.0 / (1.0 + exp(minDist - 2.5)));
dwaEgoState = [egoState3D(1), egoState3D(2), egoState3D(5), egoState3D(6), egoState3D(7)];

strState = behaviorDecision(dwaEgoState, riskMap, minDist, minTTC, latAvailable, obsApproaching);

switch strState
    case "EMERGENCY_STOP"
        behaviorState = int32(6);
        targetSpeed = 0.0;
    case "EVADE_LATERAL"
        behaviorState = int32(5);
        targetSpeed = 3.5;
    case "AVOID"
        behaviorState = int32(4);
        targetSpeed = 2.0;
    case "PACING"
        behaviorState = int32(3);
        targetSpeed = max(1.5, egoState3D(6) * 0.8);
    case "CAUTION"
        behaviorState = int32(2);
        targetSpeed = 3.0;
    otherwise % CRUISE
        behaviorState = int32(1);
        targetSpeed = 10.0;
end

end
