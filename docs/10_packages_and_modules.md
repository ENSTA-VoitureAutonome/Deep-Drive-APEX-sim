# Packages and Modules

## Summary

| Package or module | Primary location | Role |
| --- | --- | --- |
| `rc_sim_description` | `simulation/ros2_ws/src/rc_sim_description` | Gazebo simulation. |
| `voiture_system` | `real_vehicle/ros2_ws/src/voiture_system` | SLAM-enabled physical vehicle. |
| `apex_telemetry` | Separate copy in both workspaces | APEX pipeline and experiments. |
| Simulation tools | `simulation/tools/sim` | Startup, mapping, and capture. |
| Real tools | `real_vehicle/tools` | Hardware, capture, and analysis. |
| Legacy `full_soft` | `archive/legacy/full_soft` | Old Python stack without SLAM. |

## `rc_sim_description`

It contains `apex_sim.launch.py`, the vehicle Xacro, world files, scenarios, Gazebo bridges, ground truth, recorders, reconstruction, and teleoperation helpers. It uses `ament_cmake`.

## `voiture_system`

The real copy contains the SLAM-enabled launch, RPLIDAR driver, Arduino serial input, Ackermann odometry, motor/steering driver, adaptive controller, and `slam_toolbox`/Nav2 configuration.

The simulation copy adds `bringup_sim.launch.py` and bridge nodes. The recommended simulator entry point remains `rc_sim_description/apex_sim.launch.py`.

## `apex_telemetry`

It groups IMU, kinematic odometry, IMU-LiDAR fusion, curve and recognition-tour planning, tracking, command-to-actuation bridging, and session management. The simulation copy uses simulated backends; the real copy preserves auxiliary hardware and capture flows.

## Tooling

### `simulation/tools/sim`

- `apex_sim_up.sh`
- `apex_arm_recognition_tour.sh`
- `apex_recognition_tour_sim_capture.sh`
- `apex_manual_mapping_up.sh`
- `apex_manual_mapping_finish.sh`

### `real_vehicle/tools`

- `hardware/`: sensor checks.
- `firmware/`: firmware upload.
- `capture/`: run recording.
- `analysis/`: plots and metrics.
- `core/`: APEX Docker and services.
- `pc/`: monitoring from a workstation.

## Legacy Code

`archive/legacy/full_soft` is the old no-SLAM Python implementation. `archive/legacy/Lidar` stores older distributed LiDAR utilities. `simulation/legacy/Simulateur` stores the earlier simulator. None belongs in active `colcon` base paths.

## Related Documentation

- [Repository Structure](02_repository_structure.md)
- [ROS Architecture](07_ros_architecture.md)
- [Known Limitations](16_known_limitations_and_legacy_parts.md)
