# SIH26037 Verification Report

Generated: 11-Sep-2026 13:42:57

## Summary

| Check | Status |
|-------|--------|
| Component Diagnostics | PASS |
| Full Scenario Benchmark | PASS |

## Safety Constraints Verified

- **ClassID taxonomy**: 1=Vehicle, 2=Truck, 4=Pedestrian, 5=Pushcart, 6=Cattle
- **Lateral margins**: 1.2 / 1.2 / 2.0 / 1.8 / 2.5 m (localPlannerDWA_refined.m)
- **collision_mask**: hard penalty 5e4, never bypassed by anti-stall fallback
- **In-path risk filter**: |d_lat| <= 1.8 m in runSimulation.m
- **Anti-stall**: MIN_SPEED 1.39 m/s DWA window + 1.0 m/s control floor

## Benchmark Results

| Scenario | Mean Latency (ms) | Peak Latency (ms) | Collisions | Min Clearance (m) | Avg Clearance (m) | Path Smoothness |
|----------|-------------------|-------------------|------------|-------------------|-------------------|-----------------|
| CATTLE   |             5.835 |            24.977 |          0 |             3.990 |            20.188 |          0.0236 |
| VILLAGE  |             9.331 |            16.833 |          0 |             2.577 |             9.922 |          0.0301 |
| INTERSECTION |            11.779 |            28.377 |          0 |             4.485 |            12.789 |          0.0123 |
| HIGHWAY  |             2.844 |             9.444 |          0 |            36.960 |            54.605 |          0.0000 |
| MARKET   |             7.229 |            12.062 |          0 |             4.126 |             7.691 |          0.0336 |


## How to Reproduce

```matlab
runDiagnostics
runAllScenarios
runSimulation('cattle', true)
```
