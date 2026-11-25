function detections = simulateSensors(scenario, egoPose, otherActors)
% SIMULATESENSORS Return simulated detections struct array.
% egoPose: [x y yaw vx]
% otherActors: array of actor structs with fields Position, Velocity, ClassID

% Parameters
maxRange = 50;
rng('shuffle');

detections = [];
for i=1:numel(otherActors)
    pos = otherActors(i).Position; % [x y z]
    dx = pos(1) - egoPose(1);
    dy = pos(2) - egoPose(2);
    dist = hypot(dx,dy);
    if dist <= maxRange
        % Simulate detection noise & classification
        az = atan2(dy,dx) - egoPose(3);
        relV = 0;
        if isfield(otherActors(i),'Velocity')
            relV = otherActors(i).Velocity(1)*cos(az) + otherActors(i).Velocity(2)*sin(az);
        end
        det.Position = [pos(1) pos(2)];
        det.Velocity = [otherActors(i).Velocity(1) otherActors(i).Velocity(2)];
        det.ClassID = otherActors(i).ClassID;
        det.MeasurementNoise = 0.5 + 0.5*rand; % meters
        det.Confidence = 0.6 + 0.4*rand;
        detections = [detections; det];
    end
end
end
