function detections = fuseSensorOutputs(sensors, egoState, prevDetections, dt)
% FUSESENSOROUTPUTS Fuses Camera, LiDAR, and Radar sensor data into tracker detections.
% Uses a lightweight two-frame velocity estimate when previous detections are provided.

if nargin < 3, prevDetections = []; end
if nargin < 4 || isempty(dt), dt = 0.05; end

N_cam = numel(sensors.camera);
N_rad = numel(sensors.radar);

emptyDet = struct('Position', [0 0], 'Velocity', [0 0], 'ClassID', 1, 'MeasurementNoise', 0.5, 'Confidence', 0.8);
maxDets = N_cam + N_rad;
detBuffer = repmat(emptyDet, maxDets, 1);
detCount = 0;

numPrev = numel(prevDetections);

% Primary Camera Detections
for i = 1:N_cam
    c = sensors.camera(i);
    cx = c.bbox(1) + c.bbox(3)/2;
    cy = c.bbox(2) + c.bbox(4)/2;
    
    detVel = [0, 0];
    if numPrev > 0
        minDistPrev = inf;
        bestPrevIdx = 0;
        for pIdx = 1:numPrev
            dxP = cx - prevDetections(pIdx).Position(1);
            dyP = cy - prevDetections(pIdx).Position(2);
            dPrev = hypot(dxP, dyP);
            if dPrev < minDistPrev
                minDistPrev = dPrev;
                bestPrevIdx = pIdx;
            end
        end
        if bestPrevIdx > 0 && minDistPrev < 2.0
            detVel = [(cx - prevDetections(bestPrevIdx).Position(1)) / dt, ...
                      (cy - prevDetections(bestPrevIdx).Position(2)) / dt];
        end
    end
    
    detCount = detCount + 1;
    detBuffer(detCount).Position = [cx, cy];
    detBuffer(detCount).Velocity = detVel;
    detBuffer(detCount).ClassID = c.ClassID;
    detBuffer(detCount).MeasurementNoise = 0.4;
    detBuffer(detCount).Confidence = c.Confidence;
end

% Radar-only Detections
for r = 1:N_rad
    az = sensors.radar(r).azimuth + egoState(3);
    rx = egoState(1) + sensors.radar(r).range * cos(az);
    ry = egoState(2) + sensors.radar(r).range * sin(az);
    
    isMatched = false;
    for cIdx = 1:N_cam
        cx = detBuffer(cIdx).Position(1);
        cy = detBuffer(cIdx).Position(2);
        if hypot(cx - rx, cy - ry) < 2.5
            isMatched = true;
            break;
        end
    end
    
    if ~isMatched
        detVel = [0, 0];
        if numPrev > 0
            minDistPrev = inf;
            bestPrevIdx = 0;
            for pIdx = 1:numPrev
                dxP = rx - prevDetections(pIdx).Position(1);
                dyP = ry - prevDetections(pIdx).Position(2);
                dPrev = hypot(dxP, dyP);
                if dPrev < minDistPrev
                    minDistPrev = dPrev;
                    bestPrevIdx = pIdx;
                end
            end
            if bestPrevIdx > 0 && minDistPrev < 2.0
                detVel = [(rx - prevDetections(bestPrevIdx).Position(1)) / dt, ...
                          (ry - prevDetections(bestPrevIdx).Position(2)) / dt];
            end
        end
        
        detCount = detCount + 1;
        detBuffer(detCount).Position = [rx, ry];
        detBuffer(detCount).Velocity = detVel;
        detBuffer(detCount).ClassID = sensors.radar(r).ClassID;
        detBuffer(detCount).MeasurementNoise = 0.8;
        detBuffer(detCount).Confidence = 0.6;
    end
end

detections = detBuffer(1:detCount, 1);
end
