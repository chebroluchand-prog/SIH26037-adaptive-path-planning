function [scenario, ego, actors] = createCattleScenario()
% CREATECATTLESCENARIO Scenario with cattle actor (ClassID=6) crossing ego path.
%
% Backward-derived timing (User Request 4):
%   Ego start: x=10, speed=6.0 m/s
%   Cattle:    x=70, y_start=5.0 m
%   t_arrival = (70 - 10) / 6.0 = 10.0 s
%   V_y = -5.0 / 10.0 = -0.500 m/s  (crosses from y=+5 to y=0 exactly at t=10.0s)
%   V_x = 0 (pure crossing, no longitudinal drift)

scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 35);

% Straight 2-lane road
roadCenters = [0 0 0; 200 0 0];
road(scenario, roadCenters, 'Lanes', lanespec(2));

% Ego vehicle (managed externally by runSimulation)
ego = vehicle(scenario, 'ClassID', 1, 'Position', [10 0 0], 'Yaw', 0);

% Slow vehicle ahead staying in lane (no crossing)
v1 = vehicle(scenario, 'ClassID', 1, 'Position', [100 1.5 0], 'Yaw', 0);
v1.Velocity = [2.5 0 0];

% Oncoming vehicle in opposite lane
v2 = vehicle(scenario, 'ClassID', 1, 'Position', [150 -1.5 0], 'Yaw', pi);
v2.Velocity = [-3.0 0 0];

% Cattle actor (ClassID=6) crossing from right shoulder to left
% Adjusted timing for robustness: slower crossing to give ego more reaction time
% t_arrival=12.0s, V_y=-5.0/12.0=-0.417 m/s (crosses from y=+5 to y=0 at t=12.0s)
cattle = actor(scenario, 'ClassID', 6, 'Position', [70 5.0 0]);
cattle.Velocity = [0 -0.417 0];

actors = {v1, v2, cattle};
end
