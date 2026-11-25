classdef Logger < handle
% LOGGER Performance metric tracking and visual summary generator for SIH26037.
% Tracks:
%   - Replanning Latency (s)
%   - Minimum Safety Distance (m)
%   - Path Smoothness (Curvature Proxy)
%   - Collision Counts
%   - Ego Trajectory History

properties
    replanningTimes double = []
    minDistances double = []
    pathHistory double = [] % N x 3 [x, y, t]
    pathSmoothness double = []
    behaviorHistory string = string.empty
    collisionCount double = 0
    timestamps datetime = datetime.empty
end

methods
    function logReplan(obj, t)
        obj.replanningTimes(end+1, 1) = t;
        obj.timestamps(end+1, 1) = datetime('now');
    end
    
    function logMinDist(obj, d)
        obj.minDistances(end+1, 1) = d;
    end
    
    function logPose(obj, x, y, t)
        obj.pathHistory(end+1, :) = [x, y, t];
    end
    
    function logSmooth(obj, curv)
        obj.pathSmoothness(end+1, 1) = curv;
    end
    
    function logBehavior(obj, b)
        obj.behaviorHistory(end+1, 1) = string(b);
    end
    
    function incCollision(obj)
        obj.collisionCount = obj.collisionCount + 1;
    end
    
    function s = summary(obj)
        avgRepl = mean(obj.replanningTimes, 'omitnan');
        maxRepl = max(obj.replanningTimes);
        minDist = min(obj.minDistances);
        avgMinDist = mean(obj.minDistances, 'omitnan');
        avgSmooth = mean(obj.pathSmoothness, 'omitnan');
        
        fprintf('\n=========================================\n');
        fprintf('       SIH26037 SIMULATION METRICS       \n');
        fprintf('=========================================\n');
        fprintf('Replanning Cycles   : %d\n', numel(obj.replanningTimes));
        fprintf('Mean Latency        : %.3f ms (Target < 100 ms)\n', avgRepl * 1000);
        fprintf('Peak Latency        : %.3f ms\n', maxRepl * 1000);
        fprintf('Collisions          : %d\n', obj.collisionCount);
        fprintf('Minimum Clearance   : %.3f m\n', minDist);
        fprintf('Average Clearance   : %.3f m\n', avgMinDist);
        fprintf('Avg Path Curvature  : %.4f rad/m\n', avgSmooth);
        fprintf('=========================================\n\n');
        
        s.avgReplanningTime = avgRepl;
        s.maxReplanningTime = maxRepl;
        s.collisions = obj.collisionCount;
        s.minClearance = minDist;
        s.avgClearance = avgMinDist;
        s.avgSmoothness = avgSmooth;
    end
    
    function plotSummary(obj, scenName)
        if nargin < 2, scenName = 'Scenario'; end
        
        fig = figure('Name', sprintf('Performance Summary - %s', scenName), 'Color', 'w', 'Position', [100 100 1000 700]);
        
        % Subplot 1: Replanning Latency
        subplot(2,2,1);
        if ~isempty(obj.replanningTimes)
            plot(obj.replanningTimes * 1000, 'b-', 'LineWidth', 1.5); hold on;
            yline(100, 'r--', '100ms Target Limit', 'LineWidth', 1.5);
            title('Replanning Latency (ms)', 'FontSize', 12, 'FontWeight', 'bold');
            xlabel('Step'); ylabel('Latency (ms)'); grid on;
        end
        
        % Subplot 2: Minimum Safety Distance
        subplot(2,2,2);
        if ~isempty(obj.minDistances)
            plot(obj.minDistances, 'g-', 'LineWidth', 1.5); hold on;
            yline(0.8, 'r--', 'Collision Threshold', 'LineWidth', 1.5);
            yline(1.2, 'b--', 'Safety Margin', 'LineWidth', 1.0);
            title('Minimum Clearance to Obstacles (m)', 'FontSize', 12, 'FontWeight', 'bold');
            xlabel('Step'); ylabel('Clearance (m)'); grid on;
        end
        
        % Subplot 3: Trajectory Path Smoothness
        subplot(2,2,3);
        if ~isempty(obj.pathSmoothness)
            plot(obj.pathSmoothness, 'm-', 'LineWidth', 1.5);
            title('Path Curvature Smoothness (rad/m)', 'FontSize', 12, 'FontWeight', 'bold');
            xlabel('Step'); ylabel('Curvature Proxy'); grid on;
        end
        
        % Subplot 4: Trajectory Traveled
        subplot(2,2,4);
        if ~isempty(obj.pathHistory)
            plot(obj.pathHistory(:,1), obj.pathHistory(:,2), 'b-', 'LineWidth', 2);
            title('Ego Vehicle Trajectory (m)', 'FontSize', 12, 'FontWeight', 'bold');
            xlabel('X (m)'); ylabel('Y (m)'); axis equal; grid on;
        end
    end
end
end
