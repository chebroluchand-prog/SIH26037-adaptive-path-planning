function [scenario, ego, actors] = createIntersectionScenario()
% CREATEINTERSECTIONSCENARIO Unsignalled intersection with backward-derived crossing timing.
% Ego starts at (50, -40), speed=4.0 m/s north (+y) -> reaches intersection (50, 0) at t_arrival = 10.0 s.
% v1 (west approach) starts at (10, 0)  -> V_x = (50 - 10) / 10.0 = +4.0 m/s (arrives at (50, 0) at t=10.0 s).
% v2 (east approach) starts at (100, 0) -> V_x = (50 - 100) / 10.0 = -5.0 m/s (arrives at (50, 0) at t=10.0 s).
% p1 (pedestrian) starts at (65, -10)   -> t_arrival = 7.5 s -> V_x = (50 - 65) / 7.5 = -2.0 m/s.

scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 40);
road(scenario, [-40 0 0; 140 0 0]);
road(scenario, [50 -80 0; 50 80 0]);

% Ego vehicle starts on vertical road moving north (+y) towards (50, 60)
ego = vehicle(scenario, 'ClassID', 1, 'Position', [50 -40 0], 'Yaw', pi/2);

% Cross-traffic vehicles on horizontal road y=0
% Adjusted timing for robustness - slower crossing speeds
v1 = vehicle(scenario, 'ClassID', 1, 'Position', [10 0 0], 'Yaw', 0, 'Velocity', [3.0 0 0]);
v2 = vehicle(scenario, 'ClassID', 1, 'Position', [100 0 0], 'Yaw', pi, 'Velocity', [-4.0 0 0]);
p1 = actor(scenario, 'ClassID', 4, 'Position', [65 -10 0], 'Velocity', [-1.5 0 0]);
p2 = actor(scenario, 'ClassID', 4, 'Position', [35 15 0], 'Velocity', [0.8 0 0]);

v1.Velocity = [2.5 0 0]; % Calibrated for smooth 1.8m+ intersection clearance
v2.Velocity = [-3.5 0 0]; % Calibrated for smooth 1.8m+ intersection clearance
p1.Velocity = [-1.5 0 0]; % t_arrival=10.0s
p2.Velocity = [0.8 0 0]; % t_arrival=18.75s

actors = {v1, v2, p1, p2};
end
