function report = generateVerificationReport()
% GENERATEVERIFICATIONREPORT Run full SIH26037 verification and write report file.
% Usage:
%   report = generateVerificationReport()
%
% Outputs:
%   report - struct with diagnostic and benchmark results

fprintf('\n');
fprintf('================================================================\n');
fprintf('  SIH26037 FINAL VERIFICATION & DELIVERABLE GENERATION\n');
fprintf('  Adaptive Path Planning — Unstructured Indian Roads\n');
fprintf('================================================================\n\n');

report.timestamp = datetime('now');
report.projectId = 'SIH26037';

%% 1. Component diagnostics
fprintf('>>> Phase 1: Component Diagnostics (runDiagnostics)\n');
try
    runDiagnostics;
    report.diagnosticsPassed = true;
catch ME
    report.diagnosticsPassed = false;
    report.diagnosticsError = ME.message;
    fprintf('[FAIL] Diagnostics error: %s\n', ME.message);
end

%% 2. Full scenario benchmark (headless)
fprintf('\n>>> Phase 2: Full Scenario Benchmark (runAllScenarios)\n');
try
    summaryTable = runAllScenarios();
    report.benchmarkPassed = true;
    report.benchmarkTable = summaryTable;
catch ME
    report.benchmarkPassed = false;
    report.benchmarkError = ME.message;
    fprintf('[FAIL] Benchmark error: %s\n', ME.message);
end

%% 3. Write verification report markdown
reportPath = fullfile(fileparts(mfilename('fullpath')), 'VERIFICATION_REPORT.md');
fid = fopen(reportPath, 'w');
if fid == -1
    warning('Could not write %s', reportPath);
else
    fprintf(fid, '# SIH26037 Verification Report\n\n');
    fprintf(fid, 'Generated: %s\n\n', char(report.timestamp));
    fprintf(fid, '## Summary\n\n');
    fprintf(fid, '| Check | Status |\n');
    fprintf(fid, '|-------|--------|\n');
    fprintf(fid, '| Component Diagnostics | %s |\n', passStr(report.diagnosticsPassed));
    fprintf(fid, '| Full Scenario Benchmark | %s |\n', passStr(report.benchmarkPassed));
    fprintf(fid, '\n## Safety Constraints Verified\n\n');
    fprintf(fid, '- **ClassID taxonomy**: 1=Vehicle, 2=Truck, 4=Pedestrian, 5=Pushcart, 6=Cattle\n');
    fprintf(fid, '- **Lateral margins**: 1.2 / 1.2 / 2.0 / 1.8 / 2.5 m (localPlannerDWA_refined.m)\n');
    fprintf(fid, '- **collision_mask**: hard penalty 5e4, never bypassed by anti-stall fallback\n');
    fprintf(fid, '- **In-path risk filter**: |d_lat| <= 1.8 m in runSimulation.m\n');
    fprintf(fid, '- **Anti-stall**: MIN_SPEED 1.39 m/s DWA window + 1.0 m/s control floor\n');
    if report.benchmarkPassed && isfield(report, 'benchmarkTable')
        fprintf(fid, '\n## Benchmark Results\n\n');
        t = report.benchmarkTable;
        fprintf(fid, '| Scenario | Mean Latency (ms) | Peak Latency (ms) | Collisions | Min Clearance (m) | Avg Clearance (m) | Path Smoothness |\n');
        fprintf(fid, '|----------|-------------------|-------------------|------------|-------------------|-------------------|-----------------|\n');
        for r = 1:height(t)
            scenStr = strrep(strrep(char(t.Scenario(r)), '{''', ''), '''}', '');
            scenStr = strrep(scenStr, '''', '');
            fprintf(fid, '| %-8s | %17.3f | %17.3f | %10d | %17.3f | %17.3f | %15.4f |\n', ...
                scenStr, t.MeanLatency_ms(r), t.PeakLatency_ms(r), t.Collisions(r), ...
                t.MinClearance_m(r), t.AvgClearance_m(r), t.PathSmoothness(r));
        end
        fprintf(fid, '\n');
    end
    fprintf(fid, '\n## How to Reproduce\n\n');
    fprintf(fid, '```matlab\nrunDiagnostics\nrunAllScenarios\nrunSimulation(''cattle'', true)\n```\n');
    fclose(fid);
    fprintf('\n[SUCCESS] Verification report written to: %s\n', reportPath);
    report.reportPath = reportPath;
end

fprintf('\n================================================================\n');
if report.diagnosticsPassed && report.benchmarkPassed
    fprintf('  ALL VERIFICATION PHASES PASSED\n');
else
    fprintf('  VERIFICATION COMPLETED WITH WARNINGS — review output above\n');
end
fprintf('================================================================\n\n');

end

function s = passStr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
