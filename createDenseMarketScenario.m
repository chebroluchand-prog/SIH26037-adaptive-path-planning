function [scenario, ego, actors] = createDenseMarketScenario()
% CREATEDENSEMARKETSCENARIO Dense market street with backward-derived crossing timing.
%
% Ego start: x=5, speed=4.0 m/s
% a1 (Pedestrian, ClassID=4) at x=25, y_start=3.5 -> t_arrival=(25-5)/4.0=5.0s  -> V_y=-3.5/5.0=-0.700 m/s
% a2 (Pedestrian, ClassID=4) at x=45, y_start=-3.5-> t_arrival=(45-5)/4.0=10.0s -> V_y=+3.5/10.0=+0.350 m/s
% a3 (Vehicle,    ClassID=1) at x=65, y_start=3.5 -> t_arrival=(65-5)/4.0=15.0s -> V_y=-3.5/15.0=-0.233 m/s
% a4 (Pushcart,   ClassID=5) at x=85, y_start=-3.5-> t_arrival=(85-5)/4.0=20.0s -> V_y=+3.5/20.0=+0.175 m/s
% a5 (Vehicle,    ClassID=1) at x=105, slow longitudinal motion

scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 45);
road(scenario, [0 0 0; 150 0 0], 'Lanes', lanespec(1));

ego = vehicle(scenario, 'ClassID', 1, 'Position', [5 0 0], 'Yaw', 0);

a1 = actor(scenario, 'ClassID', 4, 'Position', [25 3.5 0]);
a1.Velocity = [0 -0.500 0]; % Slower: t_arrival=7.0s

a2 = actor(scenario, 'ClassID', 4, 'Position', [45 -3.5 0]);
a2.Velocity = [0 0.250 0]; % Slower: t_arrival=14.0s

a3 = vehicle(scenario, 'ClassID', 1, 'Position', [65 3.5 0]);
a3.Velocity = [0 -0.175 0]; % Slower: t_arrival=20.0s

a4 = actor(scenario, 'ClassID', 5, 'Position', [85 -3.5 0]);
a4.Velocity = [0 0.125 0]; % Slower: t_arrival=28.0s

a5 = vehicle(scenario, 'ClassID', 1, 'Position', [105 -0.8 0], 'Yaw', 0);
a5.Velocity = [0.5 0 0];

actors = {a1, a2, a3, a4, a5};
end
