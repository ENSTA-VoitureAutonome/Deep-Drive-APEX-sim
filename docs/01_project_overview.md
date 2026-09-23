# Project Overview

## Objective

Deep Drive APEX develops an autonomous RC vehicle with ROS 2. The repository supports two separate operational environments:

- Repeatable Gazebo simulation with synthetic sensors and ground truth.
- A physical vehicle with LiDAR, Arduino state, Ackermann odometry, and SLAM.

Older implementations are preserved for reference without being mixed into active workspaces.

## Current Recommended Workflow

While the physical vehicle is unavailable, active development should use:

```text
simulation/ -> Gazebo -> simulated sensors -> estimation/SLAM -> control -> metrics
```

Main entry point:

```bash
./simulation/tools/sim/apex_sim_up.sh --scenario baseline --rviz
```

## Simulation and Real Vehicle

| Environment | Directory | Primary stack | Current use |
| --- | --- | --- | --- |
| Simulation | `simulation/` | `rc_sim_description` plus simulation copies of `apex_telemetry` and `voiture_system` | Recommended now |
| Real vehicle | `real_vehicle/` | `voiture_system` plus `slam_toolbox` | Preserved until the car is available |
| No-SLAM legacy | `archive/legacy/full_soft/` | Python and direct drivers | Historical |

## Major Subsystems

- Gazebo vehicle model, tracks, and sensors.
- Gazebo-to-ROS bridges.
- Simulated or physical IMU and LiDAR.
- Odometry and estimation.
- SLAM and map publication.
- Planning and tracking.
- Ackermann control and actuation.
- Recording, analysis, and ground-truth comparison.

## Technology Stack

- Ubuntu 24.04 and WSL2.
- ROS 2 Jazzy.
- Gazebo Sim.
- Python 3.
- `colcon`, `rosdep`, and RViz.
- `slam_toolbox` and optional Nav2.
- Docker for auxiliary real-vehicle APEX workflows.

## Workspace Separation

Never build `simulation/ros2_ws` and `real_vehicle/ros2_ws` into the same build directory. Both contain packages with repeated names. Enter the correct workspace, build there, and open a fresh terminal when switching environments.

## Related Documentation

- [Repository Structure](02_repository_structure.md)
- [Quick Start](05_quick_start.md)
- [System Architecture](06_system_architecture.md)
- [Gazebo Simulation](08_simulation_gazebo.md)
- [SLAM-Enabled Real Vehicle](09_blue_vehicle_real_system.md)
