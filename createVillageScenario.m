function [scenario, ego, actors] = createVillageScenario()
% CREATEVILLAGESCENARIO Unmarked village road with backward-derived crossing timing.
%
% Ego start: x=10, speed=5.0 m/s
% Pedestrian (ClassID=4): x=40, y_start=3.5 (road edge)
%   t_arrival = (40 - 10) / 5.0 = 6.0 s
%   V_y = -3.5 / 6.0 = -0.583 m/s, V_x = 0
%
% Pushcart (ClassID=5): x=80, y_start=-3.5 (opposite edge)
%   t_arrival = (80 - 10) / 5.0 = 14.0 s
%   V_y = +3.5 / 14.0 = +0.250 m/s, V_x = 0

scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 40);
roadCenters = [0 0 0; 160 0 0];
road(scenario, roadCenters, 'Lanes', lanespec(1));

ego = vehicle(scenario, 'ClassID', 1, 'Position', [10 0 0], 'Yaw', 0);

% Pedestrian crossing from right shoulder at road edge
% Adjusted timing: slower crossing for robustness
% t_arrival=8.0s, V_y=-3.5/8.0=-0.438 m/s
a1 = actor(scenario, 'ClassID', 4, 'Position', [40 3.5 0]);
a1.Velocity = [0 -0.438 0];

% Slow moving vehicle ahead staying in lane
a2 = vehicle(scenario, 'ClassID', 1, 'Position', [60 -1.0 0], 'Yaw', 0);
a2.Velocity = [2.0 0 0];

% Pushcart crossing from left shoulder
% Adjusted timing: slower crossing for robustness
% t_arrival=18.0s, V_y=+3.5/18.0=+0.194 m/s
a3 = actor(scenario, 'ClassID', 5, 'Position', [80 -3.5 0]);
a3.Velocity = [0 0.194 0];

% Oncoming auto-rickshaw
a4 = vehicle(scenario, 'ClassID', 1, 'Position', [110 0.5 0], 'Yaw', pi);
a4.Velocity = [-1.5 0 0];

actors = {a1, a2, a3, a4};
end
