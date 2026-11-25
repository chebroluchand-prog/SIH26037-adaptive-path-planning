function nextState = bicycleModel(state, throttle, delta, dt, params)
% BICYCLEMODEL Kinematic bicycle vehicle dynamics model with 3D pose & terrain elevation.
% state    = [x, y, yaw, v] or [x, y, yaw, v, w] or 3D state [x, y, z, pitch, yaw, v, w]
% throttle = acceleration command (m/s^2)
% delta    = front wheel steering angle (rad)
% dt       = timestep (s)
% params   = vehicle parameters struct (L, maxSteer, maxAccel, maxSpeed)

if nargin < 5 || isempty(params)
    params.L = 2.7;          % Wheelbase (m)
    params.maxSteer = 0.6;   % Max steer angle (rad) ~34 deg
    params.maxAccel = 3.5;   % Max acceleration (m/s^2)
    params.maxDecel = 6.0;   % Max deceleration (m/s^2)
    params.maxSpeed = 30.0;  % Max speed (m/s)
end

% Apply physical saturation limits
delta = max(-params.maxSteer, min(params.maxSteer, delta));
if throttle < 0
    throttle = max(-params.maxDecel, throttle);
else
    throttle = min(params.maxAccel, throttle);
end

% Parse input state flexible dimensions
if numel(state) >= 7
    x = state(1); y = state(2); z = state(3);
    psi = state(5); v = state(6);
elseif numel(state) == 5 || numel(state) == 4
    x = state(1); y = state(2); psi = state(3); v = state(4);
    z = 0.0;
else
    x = state(1); y = state(2); psi = state(3); v = state(4);
    z = 0.0;
end

% Compute terrain elevation and gradients dz/dx, dz/dy
terrainMode = 'flat';
if isfield(params, 'terrainMode')
    terrainMode = params.terrainMode;
end
[zVal, dzdx, dzdy] = getTerrainElevation(x, y, terrainMode);

% Compute dynamic road pitch angle: theta_pitch = atan2(dzdx*cos(psi) + dzdy*sin(psi), 1)
pitch = atan2(dzdx * cos(psi) + dzdy * sin(psi), 1.0);

% Kinematic velocity and slip angle update
vNext = max(0, min(params.maxSpeed, v + throttle * dt));
beta = atan(0.5 * tan(delta)); % Side slip angle approximation

effectiveHeading = psi + beta;

% 3D Spatial updates:
% x_dot = v * cos(psi) * cos(pitch)
% y_dot = v * sin(psi) * cos(pitch)
% z_dot = v * sin(pitch)
xNext = x + vNext * cos(effectiveHeading) * cos(pitch) * dt;
yNext = y + vNext * sin(effectiveHeading) * cos(pitch) * dt;
zNext = zVal + vNext * sin(pitch) * dt;

psiNext = psi + (vNext / params.L) * sin(beta) * dt;
psiNext = mod(psiNext + pi, 2*pi) - pi; % Wrap yaw to [-pi, pi]

wNext = (vNext / params.L) * sin(beta);

% Return 3D pose state vector: [x, y, z, pitch, yaw, v, w]
nextState = [xNext, yNext, zNext, pitch, psiNext, vNext, wNext];
end
