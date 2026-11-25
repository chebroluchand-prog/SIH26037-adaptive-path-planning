# Quick Start Guide - SIH26037

## MATLAB Environment Setup

Ensure MATLAB R2026a (or compatible) is installed with:
- Automated Driving Toolbox
- Navigation Toolbox

## Running the System

### 1. Quick Verification (Run First)
```matlab
runDiagnostics
```
This validates all components and ensures the system is working correctly.

### 2. Run Individual Scenarios
```matlab
% Cattle crossing scenario (recommended for testing ClassID 6)
runSimulation('cattle', true)

% Village road scenario (mixed ClassID 4,5)
runSimulation('village', true)

% Intersection scenario (complex geometry)
runSimulation('intersection', true)

% Highway merge scenario (high-speed)
runSimulation('highway', true)

% Dense market scenario (multi-actor)
runSimulation('market', true)
```

### 3. Run Full Benchmark
```matlab
runAllScenarios
```
This runs all scenarios and generates a performance summary table.

### 4. Headless Operation (No Graphics)
```matlab
runSimulation('cattle', false)
```
Useful for batch processing or when graphics are not available.

## Key Files

| File | Purpose |
|------|---------|
| `runSimulation.m` | Main simulation orchestrator |
| `localPlannerDWA_refined.m` | Core local planner with DWA |
| `behaviorDecision.m` | Behavioral state machine |
| `kalmanTracker.m` | Multi-target tracking |
| `predictTrajectories.m` | Trajectory prediction |
| `runDiagnostics.m` | Component verification |
| `runAllScenarios.m` | Benchmark suite |

## Safety Features

### ClassID Taxonomy
- **1**: Vehicle (1.2m margin)
- **2**: Heavy Truck (1.2m margin)  
- **4**: Pedestrian (2.0m margin)
- **5**: Pushcart/Bicycle (1.8m margin)
- **6**: Cattle/Animal (2.5m margin)

### Behavioral States
- **CRUISE**: Normal operation at target speed
- **CAUTION**: Reduced speed, enhanced monitoring
- **AVOID**: Active lateral nudging/evasive steering
- **EMERGENCY_STOP**: Immediate full deceleration

### Safety Thresholds
- Emergency Stop: minDist < 1.2m or TTC < 1.0s
- Avoidance: minDist < 3.0m or TTC < 2.2s
- Caution: minDist < 6.0m or TTC < 4.0s
- Collision Detection: 0.8m threshold (below smallest safety margin)
- Goal Achievement: 3.0m threshold

## Performance Metrics

- **Replanning Latency**: Target < 3ms
- **Safety Margins**: Class-specific (1.2m - 2.5m)
- **Horizon**: 2.0 seconds prediction
- **Update Rate**: 20 Hz (dt = 0.05s)

## Troubleshooting

### Vehicle Not Moving
- Check anti-stall mechanisms in localPlannerDWA_refined.m:62-64
- Verify speed floor in runSimulation.m:244-246
- Ensure goal position is reachable

### Collision Detected
- Review safety margins in localPlannerDWA_refined.m:139-148
- Check scenario timing in create*Scenario.m files
- Verify ClassID assignments are correct

### Performance Issues
- Reduce Nv/Nw parameters in plannerParams
- Shorten horizon if needed
- Check for memory constraints

## Customization

### Adjust Safety Margins
Edit localPlannerDWA_refined.m lines 139-148:
```matlab
case 6 % Cattle / Animal
    r_lat_vec(p) = 2.5;  % Adjust this value
```

### Modify Behavioral Thresholds
Edit behaviorDecision.m lines 22-32:
```matlab
if minDist < 1.0 || ttc < 0.8 || maxRisk > 0.95
    behavior = "EMERGENCY_STOP";
```

### Create New Scenarios
1. Copy existing scenario file (e.g., createCattleScenario.m)
2. Modify road geometry and actor positions
3. Add to runSimulation.m switch statement (lines 16-39)
4. Add to runDiagnostics.m scenario list (lines 97-99)

## Visualization Features

The simulation includes a professional Bird's-Eye View display with:
- Road network and lane boundaries
- Sensor FOV cones (LiDAR + Radar)
- Ego vehicle footprint and heading
- Dynamic obstacles with class-specific colors
- Velocity vectors for all actors
- DWA planner trajectory ribbon
- Real-time telemetry HUD

## Export Capabilities

### Simulink Components
```matlab
exportSimulinkComponents
```
Exports the system architecture for Simulink implementation.

### Architecture Documentation
```matlab
exportSimulinkArchitecture
```
Generates text-based architecture documentation.

## Support

For issues or questions:
1. Run `runDiagnostics` to verify component health
2. Check console output for specific error messages
3. Review SIH26037_DELIVERABLE.md for detailed documentation