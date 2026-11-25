classdef kalmanTracker < handle
% KALMANTRACKER Multi-target 3D Kalman tracker with pre-allocated track buffers.
% Uses Constant-Velocity (CV) dynamic model for 3D state [x; vx; y; vy; z; vz]
% and Mahalanobis-distance Nearest-Neighbor data association.
% C-code export compliant fixed-size struct array buffers.

properties
    tracks      % Struct array of tracks (id, state, P, age, lastUpdate, classID)
    nextID double = 1
    dt double = 0.05
    maxTracks double = 30
end

methods
    function obj = kalmanTracker(dt, maxTracks)
        if nargin >= 1 && ~isempty(dt), obj.dt = dt; end
        if nargin >= 2 && ~isempty(maxTracks), obj.maxTracks = maxTracks; end
        
        % Pre-allocate track buffer array with 6D 3D state [x; vx; y; vy; z; vz]
        emptyTrack = struct('id', 0, 'state', zeros(6,1), 'P', eye(6), 'age', 0, 'lastUpdate', 0, 'classID', 1);
        obj.tracks = repmat(emptyTrack, 0, 1);
    end

    function predictAll(obj)
        % 6x6 Constant Velocity Transition Matrix F
        F = [1 obj.dt 0      0 0      0; ...
             0      1 0      0 0      0; ...
             0      0 1 obj.dt 0      0; ...
             0      0 0      1 0      0; ...
             0      0 0      0 1 obj.dt; ...
             0      0 0      0 0      1];
        baseQ = 0.1;
        
        numT = numel(obj.tracks);
        for k = 1:numT
            % ClassID-dependent process noise scaling
            qScale = 1.0;
            if isfield(obj.tracks(k), 'classID') && ~isempty(obj.tracks(k).classID)
                cID = obj.tracks(k).classID;
                if cID == 4 || cID == 6 % Pedestrian / Cattle (high maneuverability/erratic)
                    qScale = 2.5;
                elseif cID == 5 % Pushcart / Bicycle
                    qScale = 1.5;
                end
            end
            Q = baseQ * qScale * eye(6);
            
            obj.tracks(k).state = F * obj.tracks(k).state;
            obj.tracks(k).P = F * obj.tracks(k).P * F' + Q;
            obj.tracks(k).age = obj.tracks(k).age + 1;
            obj.tracks(k).lastUpdate = obj.tracks(k).lastUpdate + 1;
        end
    end

    function update(obj, detections)
        if isempty(detections)
            obj.pruneTracks();
            return;
        end
        
        numD = numel(detections);
        numT = numel(obj.tracks);
        assigned = false(1, numD);
        
        % Extract 2D/3D measurement positions
        measPos = zeros(2, numD);
        for i = 1:numD
            measPos(:, i) = detections(i).Position(1:2)';
        end

        H = [1 0 0 0 0 0; 0 0 1 0 0 0]; % 2x6 measurement extraction matrix for 2D planner compatibility

        % Mahalanobis-distance Nearest-Neighbor data association
        for k = 1:numT
            trackPos = H * obj.tracks(k).state; % 2 x 1
            dists = inf(1, numD);
            
            for i = 1:numD
                rNoise = detections(i).MeasurementNoise;
                R = (rNoise^2) * eye(2);
                S = H * obj.tracks(k).P * H' + R; % 2 x 2 innovation covariance
                z = [detections(i).Position(1); detections(i).Position(2)];
                y = z - trackPos; % 2 x 1 innovation vector
                
                % Squared Mahalanobis distance y' * inv(S) * y
                mDistSq = y' * (S \ y);
                
                % Add penalty for ClassID mismatch to prevent cross-class identity swaps
                if isfield(detections(i), 'ClassID') && isfield(obj.tracks(k), 'classID') ...
                   && ~isempty(obj.tracks(k).classID) && detections(i).ClassID ~= obj.tracks(k).classID
                    mDistSq = mDistSq + 20.0;
                end
                
                dists(i) = mDistSq;
            end
            
            [mindSq, idx] = min(dists);
            
            % Chi-square gate threshold for 2 DOF (95% confidence limit = 5.99)
            if mindSq <= 5.99 && ~assigned(idx)
                rNoise = detections(idx).MeasurementNoise;
                R = (rNoise^2) * eye(2);
                
                z = [detections(idx).Position(1); detections(idx).Position(2)];
                y = z - H * obj.tracks(k).state;
                S = H * obj.tracks(k).P * H' + R;
                K = (obj.tracks(k).P * H') / S;
                
                obj.tracks(k).state = obj.tracks(k).state + K * y;
                obj.tracks(k).P = (eye(6) - K * H) * obj.tracks(k).P;
                obj.tracks(k).lastUpdate = 0;
                
                % Update classID if provided
                if isfield(detections(idx), 'ClassID')
                    obj.tracks(k).classID = detections(idx).ClassID;
                end
                
                assigned(idx) = true;
            end
        end

        % Create new 6D tracks for unassigned detections
        for i = 1:numD
            if ~assigned(i) && numel(obj.tracks) < obj.maxTracks
                detVel = [0; 0; 0];
                if isfield(detections(i), 'Velocity')
                    vTmp = detections(i).Velocity;
                    if numel(vTmp) == 2, detVel = [vTmp(1); 0; vTmp(2); 0; 0; 0];
                    else, detVel = [vTmp(1); 0; vTmp(2); 0; vTmp(3); 0]; end
                else
                    detVel = zeros(6, 1);
                end
                
                cID = 1;
                if isfield(detections(i), 'ClassID')
                    cID = detections(i).ClassID;
                end
                
                zVal = 0.0;
                if isfield(detections(i), 'Position3D') && numel(detections(i).Position3D) >= 3
                    zVal = detections(i).Position3D(3);
                elseif cID == 4 || cID == 6
                    zVal = 0.5;
                end
                
                s = [detections(i).Position(1); detVel(1); detections(i).Position(2); detVel(3); zVal; 0];
                newTrack.id = obj.nextID;
                newTrack.state = s;
                newTrack.P = eye(6);
                newTrack.age = 1;
                newTrack.lastUpdate = 0;
                newTrack.classID = cID;
                
                obj.tracks(end+1, 1) = newTrack;
                obj.nextID = obj.nextID + 1;
            end
        end
        
        obj.pruneTracks();
    end

    function pruneTracks(obj)
        if isempty(obj.tracks), return; end
        validMask = ([obj.tracks.lastUpdate] <= 5);
        obj.tracks = obj.tracks(validMask, :);
    end
end
end
