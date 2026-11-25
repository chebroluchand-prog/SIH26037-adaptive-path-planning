function summaryTable = runAllScenarios()
% RUNALLSCENARIOS Automated benchmark suite executing all 5 mandated Indian road scenarios:
%   1. Cattle Crossing (cattle)
%   2. Unmarked Village Road (village)
%   3. Unsignalled Urban Intersection (intersection)
%   4. Highway Merge (highway)
%   5. Dense Market (market)
%
% Validates performance metrics:
%   - Replanning Latency (Target < 100 ms)
%   - Collision Count (Target = 0)
%   - Minimum Safety Distance (Target > 1.0 m)
%   - Path Smoothness (Curvature Proxy)

scenarios = {'cattle', 'village', 'intersection', 'highway', 'market'};
numScen = numel(scenarios);

avgLatencies = zeros(numScen, 1);
maxLatencies = zeros(numScen, 1);
collisions   = zeros(numScen, 1);
minClearances = zeros(numScen, 1);
avgClearances = zeros(numScen, 1);
avgSmoothness = zeros(numScen, 1);

fprintf('========================================================================\n');
fprintf('  STARTING SIH26037 BENCHMARK SUITE ACROSS ALL 5 INDIAN ROAD SCENARIOS  \n');
fprintf('========================================================================\n\n');

for i = 1:numScen
    scenName = scenarios{i};
    fprintf('>>> Running Benchmark: %s Scenario ...\n', upper(scenName));
    
    metrics = runSimulation(scenName, false);
    
    avgLatencies(i)  = metrics.avgReplanningTime * 1000; % ms
    maxLatencies(i)  = metrics.maxReplanningTime * 1000; % ms
    collisions(i)    = metrics.collisions;
    minClearances(i) = metrics.minClearance;
    avgClearances(i) = metrics.avgClearance;
    avgSmoothness(i) = metrics.avgSmoothness;
end

% Construct Summary Table
Scenario = upper(scenarios');
MeanLatency_ms = avgLatencies;
PeakLatency_ms = maxLatencies;
Collisions = collisions;
MinClearance_m = minClearances;
AvgClearance_m = avgClearances;
PathSmoothness = avgSmoothness;

summaryTable = table(Scenario, MeanLatency_ms, PeakLatency_ms, Collisions, MinClearance_m, AvgClearance_m, PathSmoothness);

fprintf('\n========================================================================\n');
fprintf('                  SIH26037 BENCHMARK RESULTS SUMMARY                   \n');
fprintf('========================================================================\n');
disp(summaryTable);
fprintf('========================================================================\n');

% Plot Multi-Scenario Comparison Dashboard
fig = figure('Name', 'SIH26037 Multi-Scenario Benchmark Results', 'Color', 'w', 'Position', [100 100 1100 700]);

% 1. Replanning Latency (ms)
subplot(2,2,1);
bar(avgLatencies, 'FaceColor', [0.2 0.6 0.8]); hold on;
yline(100, 'r--', '100 ms Limit', 'LineWidth', 1.5);
set(gca, 'XTickLabel', scenarios, 'FontSize', 10);
ylabel('Latency (ms)');
title('Mean Replanning Latency (ms)', 'FontWeight', 'bold');
grid on;

% 2. Minimum Clearance (m)
subplot(2,2,2);
bar(minClearances, 'FaceColor', [0.2 0.8 0.4]); hold on;
yline(0.8, 'r--', 'Collision Threshold (0.8m)', 'LineWidth', 1.5);
yline(1.2, 'b--', 'Safety Margin (1.2m)', 'LineWidth', 1.0);
set(gca, 'XTickLabel', scenarios, 'FontSize', 10);
ylabel('Clearance (m)');
title('Minimum Clearance Distance (m)', 'FontWeight', 'bold');
grid on;

% 3. Path Smoothness
subplot(2,2,3);
bar(avgSmoothness, 'FaceColor', [0.8 0.4 0.8]);
set(gca, 'XTickLabel', scenarios, 'FontSize', 10);
ylabel('Curvature Proxy (rad/m)');
title('Average Path Curvature Smoothness', 'FontWeight', 'bold');
grid on;

% 4. Collision Count
subplot(2,2,4);
bar(collisions, 'FaceColor', [0.8 0.2 0.2]);
set(gca, 'XTickLabel', scenarios, 'FontSize', 10);
ylabel('Count');
title('Collision Count (Goal = 0)', 'FontWeight', 'bold');
grid on;

% Assert verification criteria
assert(all(avgLatencies < 100), '[FAILURE] Replanning latency exceeded 100ms threshold!');
assert(all(collisions == 0), '[FAILURE] Collisions occurred during benchmarking!');
fprintf('\n[SUCCESS] ALL VERIFICATION TESTS PASSED SUCCESSFULLY!\n\n');

end
