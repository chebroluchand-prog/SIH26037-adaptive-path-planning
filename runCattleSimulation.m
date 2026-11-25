function runCattleSimulation()
% RUNCATTLESIMULATION Convenience wrapper to run cattle crossing simulation scenario.
runSimulation('cattle', true);
end
% In MATLAB, navigate to: c:\Users\chand chebrolu\OneDrive\Desktop\newsih

% STEP 1: Run diagnostics first
runDiagnostics

% STEP 2: Visual simulation (single scenario)
runSimulation('cattle')      % cattle crossing (10s timing)
runSimulation('village')     % pedestrian + pushcart crossing
runSimulation('intersection')% unsignalled intersection
runSimulation('highway')     % highway merge
runSimulation('market')      % dense market street

% STEP 3: Full benchmark suite (no visuals, generates table)
summaryTable = runAllScenarios()

% STEP 4: Export Simulink architecture guide
exportSimulinkComponents
