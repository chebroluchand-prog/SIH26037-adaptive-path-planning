# Adaptive Path Planning for Unstructured Indian Roads

**Smart India Hackathon 2026** · Problem **SIH26037**  
**3rd prize** — Vishnu Institute of Technology, Bhimavaram

MATLAB simulation of adaptive path planning and collision avoidance for autonomous vehicles on unstructured Indian roads: mixed traffic, missing lane markings, cattle, pedestrians, and dense markets.

## Problem

> Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

Indian roads are not highway-style structured lanes. This project plans locally in real time while treating different road users with different safety margins — cattle get more clearance than a car — and never relaxing the collision mask to “unstick” the planner.

## What it does



- **Dynamic Window Approach (DWA)** local planner with class-aware elliptical clearance
- **Six-state behavior**: CRUISE → CAUTION → AVOID → PACING → EVADE_LATERAL → EMERGENCY_STOP
- **Sensor fusion + Kalman tracking** (camera, LiDAR, radar) with ClassID preserved through tracks
- **Multi-hypothesis trajectory prediction** for other actors
- **Zero collisions** across cattle, village, intersection, highway, and dense-market scenarios (see `VERIFICATION_REPORT.md`)

### Obstacle classes and lateral margins
<img width="1798" height="897" alt="BENCHMARK" src="https://github.com/user-attachments/assets/cea413af-eeff-4450-a7f3-9a63d08a3472" />
Benchmark test results among various scenarios

| ClassID | Object | Lateral margin |
| --- | --- | --- |
| 1 | Vehicle | 1.2 m |
| 2 | Heavy truck | 1.2 m |
| 4 | Pedestrian | 2.0 m |
| 5 | Pushcart / bicycle | 1.8 m |
| 6 | Cattle / animals | 2.5 m |

## Run it (MATLAB)

Needs MATLAB with Automated Driving Toolbox and Navigation Toolbox.

```matlab
runDiagnostics                 % component checks
runSimulation('cattle', true)  % live bird's-eye view
runAllScenarios                % full benchmark
generateVerificationReport     % writes VERIFICATION_REPORT.md
```

Other scenarios: `'village'`, `'intersection'`, `'highway'`, `'market'`.

## Layout

| File | Role |
| --- | --- |
| `runSimulation.m` | Closed-loop simulation + HUD |
| `localPlannerDWA_refined.m` | Vectorized DWA planner |
| `behaviorDecision.m` | Behavioral state machine |
| `kalmanTracker.m` | Multi-target tracking |
| `predictTrajectories.m` | Obstacle motion hypotheses |
| `QUICK_START.md` | Setup and troubleshooting |
| `SIH26037_DELIVERABLE.md` | Full technical write-up |

## Team / venue

Built for Smart India Hackathon at **Vishnu Institute of Technology, Bhimavaram**. College-level result: **3rd prize**.
