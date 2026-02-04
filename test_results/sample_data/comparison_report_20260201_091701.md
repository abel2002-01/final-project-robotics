# 🤖 Navigation Planner Comparison Report

## Autonomous Delivery Robot: NavFn vs SmacPlanner2D

**Generated:** 2026-02-01 09:17:01

---

## 📋 Executive Summary

This report presents a comprehensive comparison of two Nav2 global path planning algorithms:

| Algorithm | Description |
|-----------|-------------|
| **NavFn** | Classical grid-based planner using Dijkstra/A* wavefront expansion |
| **SmacPlanner2D** | Modern cost-aware A* planner with optimized search |

**Test runs analyzed:** 3 NavFn, 3 SmacPlanner2D

**Overall Winner:** SMAC
- Metrics won - NavFn: 0, SmacPlanner2D: 5, Tie: 1

---

## 🔬 Methodology

### Test Environment
- **World:** Custom office environment with rooms, corridors, and furniture
- **Robot:** Differential-drive delivery robot with 2D LiDAR
- **Waypoints:** Reception (2, 1) → Storage (4, 7.5) → Office (12.5, 3)

### Metrics Collected
| Metric | Description | Unit |
|--------|-------------|------|
| Total Time | Time to complete entire delivery route | seconds |
| Distance Traveled | Actual distance robot moved (from odometry) | meters |
| Path Length | Planned path length (from planner) | meters |
| Min Obstacle Distance | Closest approach to any obstacle | meters |
| Success Rate | Percentage of waypoints reached | % |
| Path Efficiency | Ratio of actual distance to planned path | % |

---

## 📊 Detailed Results

### NavFn Planner

**Number of test runs:** 3

| Metric | Mean | Std Dev | Min | Max |
|--------|------|---------|-----|-----|
| Time | 45.87 | 1.40 | 44.56 | 47.81 |
| Distance | 18.53 | 0.45 | 18.02 | 19.12 |
| Path Length | 17.97 | 0.34 | 17.65 | 18.45 |
| Obstacle Distance | 0.32 | 0.03 | 0.28 | 0.35 |
| Efficiency | 103.07 | 0.69 | 102.10 | 103.60 |
| Success Rate | 100.00 | 0.00 | 100.00 | 100.00 |

### SmacPlanner2D

**Number of test runs:** 3

| Metric | Mean | Std Dev | Min | Max |
|--------|------|---------|-----|-----|
| Time | 41.68 | 0.58 | 40.92 | 42.34 |
| Distance | 17.24 | 0.28 | 16.89 | 17.58 |
| Path Length | 16.96 | 0.23 | 16.68 | 17.25 |
| Obstacle Distance | 0.41 | 0.02 | 0.38 | 0.44 |
| Efficiency | 101.67 | 0.26 | 101.30 | 101.90 |
| Success Rate | 100.00 | 0.00 | 100.00 | 100.00 |

---

## ⚖️ Head-to-Head Comparison

| Metric | NavFn | SmacPlanner2D | Δ% | Winner |
|--------|-------|---------------|-----|--------|
| Total Time (s) | 45.87 | 41.68 | -9.1% | 🏆 SMAC |
| Distance Traveled (m) | 18.53 | 17.24 | -7.0% | 🏆 SMAC |
| Path Length (m) | 17.97 | 16.96 | -5.6% | 🏆 SMAC |
| Min Obstacle Distance (m) | 0.32 | 0.41 | +29.3% | 🏆 SMAC |
| Path Efficiency (%) | 103.07 | 101.67 | -1.4% | 🏆 SMAC |
| Success Rate (%) | 100.00 | 100.00 | +0.0% | 🤝 TIE |

---

## 📈 Analysis & Interpretation

### Total Time (s)

SmacPlanner2D completed 4.19s faster (9.1% improvement).

### Distance Traveled (m)

Robots traveled 1.29m less with SmacPlanner2D (7.0% more efficient).

### Path Length (m)

SmacPlanner2D generated paths 1.01m shorter on average.

### Min Obstacle Distance (m)

SmacPlanner2D maintained 0.09m greater clearance from obstacles.

### Success Rate (%)

NavFn had 0.0% lower success rate.

---

## 📊 EDA Summary

### Time Comparison (seconds)
```
NavFn        │██████████████████████████████│ 45.87s
SmacPlanner  │███████████████████████████   │ 41.68s
```

### Distance Traveled (meters)
```
NavFn        │██████████████████████████████│ 18.53m
SmacPlanner  │███████████████████████████   │ 17.24m
```

### Success Rate (%)
```
NavFn        │██████████████████████████████│ 100%
SmacPlanner  │██████████████████████████████│ 100%
```

---

## 🎯 Conclusions

**Overall Winner: SmacPlanner2D** - The cost-aware A* planner outperformed NavFn in the majority of metrics.
SmacPlanner2D is **9.1% faster** on average, completing the delivery route more quickly.
SmacPlanner2D generates **shorter paths**, suggesting better optimization of the global plan.
SmacPlanner2D maintains **larger clearance** from obstacles, enhancing safety in cluttered environments.
Both planners achieved **100% success rate**, demonstrating high reliability for the test scenarios.

### Recommendations
- Use **SmacPlanner2D** for complex environments where path optimality and obstacle avoidance are critical.
- SmacPlanner2D's cost-aware search makes it better suited for cluttered indoor spaces.

---

## 📝 Notes

- Results may vary based on simulation conditions and hardware performance
- For production use, consider running additional tests with varied environments
- Both planners use the same local controller (DWB) for trajectory tracking

---

*Report generated automatically by Navigation Analysis Tool*
*Analysis timestamp: 2026-02-01T09:17:01.938230*