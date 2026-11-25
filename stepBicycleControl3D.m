function nextEgoState3D = stepBicycleControl3D(egoState3D, throttleCmd, steerCmd, dt) %#codegen
% STEPBICYCLECONTROL3D Simulink C-Coder compliant wrapper for 3D Vehicle Dynamics Model.
% %#codegen directive ensures Zero Dynamic Memory Allocation for Embedded Coder export.
%
% Inputs:
%   egoState3D     - [7x1] double: [x, y, z, pitch, yaw, v, w]
%   throttleCmd    - double scalar: acceleration command (m/s^2)
%   steerCmd       - double scalar: front wheel steering angle (rad)
%   dt             - double scalar: simulation timestep (s)
%
% Outputs:
%   nextEgoState3D - [1x7] double: updated 3D pose state vector

coder.inline('never');

paramsVehicle = struct('L', 2.7, 'maxSteer', 0.6, 'maxAccel', 3.5, 'maxDecel', 6.0, 'maxSpeed', 30.0, 'terrainMode', 'flat');

nextEgoState3D = bicycleModel(egoState3D', throttleCmd, steerCmd, dt, paramsVehicle);

end
