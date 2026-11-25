function [scenario, ego, actors] = createHighwayMergeScenario()
% CREATEHIGHWAYMERGESCENARIO Highway merge with backward-derived timing.
% Ego starts at x=30, speed=12.0 m/s -> reaches merge point x=160 at t_arrival = 10.833 s.
% Truck starts on ramp at (130, -18) -> V_x = 30 / 10.833 = 2.769 m/s, V_y = 18 / 10.833 = 1.662 m/s.

scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 50);
road(scenario, [0 0 0; 500 0 0], 'Lanes', lanespec(3));
road(scenario, [120 -20 0; 160 0 0]); % ramp

ego = vehicle(scenario, 'ClassID', 1, 'Position', [30 0 0], 'Yaw', 0);

v1 = vehicle(scenario, 'ClassID', 1, 'Position', [90 0 0], 'Velocity', [10.0 0 0]);
v2 = vehicle(scenario, 'ClassID', 1, 'Position', [200 0 0], 'Velocity', [8.0 0 0]);
% Merging truck from on-ramp timed to intersect at x=160, y=0
truck = vehicle(scenario, 'ClassID', 2, 'Position', [130 -18 0], 'Velocity', [2.769 1.662 0]);

v1.Velocity = [10.0 0 0];
v2.Velocity = [8.0 0 0];
truck.Velocity = [2.769 1.662 0];

actors = {v1, v2, truck};
end
