# Repository Structure

## Top-Level Tree

```text
.
├── archive/
│   ├── artifacts/
│   ├── generated/
│   └── legacy/
│       ├── full_soft/
│       ├── Lidar/
│       └── voiture_system_sim_source/
├── docs/
├── real_vehicle/
│   ├── arduino/
│   ├── docker/
│   ├── data/
│   ├── ros2_ws/src/
│   │   ├── voiture_system/
│   │   └── apex_telemetry/
│   ├── systemd/
│   └── tools/
├── simulation/
│   ├── data/
│   ├── legacy/Simulateur/
│   ├── ros2_ws/src/
│   │   ├── rc_sim_description/
│   │   ├── apex_telemetry/
│   │   └── voiture_system/
│   ├── rviz/
│   └── tools/
└── README.md
```

## Directory Roles

| Path | Role | Executable workflow? |
| --- | --- | --- |
| `simulation/` | Self-contained Gazebo environment for development without the car. | Yes |
| `real_vehicle/` | Physical car; `voiture_system` is the primary SLAM-enabled stack. | Only on the vehicle |
| `archive/` | Legacy code, historical copies, and generated artifacts. | Not as a primary workflow |
| `docs/` | Shared documentation. | No |

## ROS Packages

| Package | Simulation location | Real location | Role |
| --- | --- | --- | --- |
| `rc_sim_description` | `simulation/ros2_ws/src/rc_sim_description` | — | Gazebo, URDF, worlds, bridges, and ground truth. |
| `apex_telemetry` | `simulation/ros2_ws/src/apex_telemetry` | `real_vehicle/ros2_ws/src/apex_telemetry` | Environment-specific APEX pipeline. |
| `voiture_system` | `simulation/ros2_ws/src/voiture_system` | `real_vehicle/ros2_ws/src/voiture_system` | Simulation compatibility and the physical SLAM stack. |

Repeated packages are environment-specific copies. Do not combine their overlays.

## Primary Real Vehicle

```text
real_vehicle/ros2_ws/src/voiture_system/
└── launch/bringup_real_slam_nav.launch.py
```

This launch enables `slam_toolbox` by default and can optionally enable Nav2.

## Legacy Stack Without SLAM

```text
archive/legacy/full_soft/
└── code/main.py
```

This is the Python implementation from before the current ROS 2 architecture. It contains direct LiDAR, motor, steering, and camera interfaces plus a `--simulation` mode.

## Simulation Assets

| Path | Content |
| --- | --- |
| `simulation/ros2_ws/src/rc_sim_description/worlds/` | Gazebo tracks. |
| `simulation/ros2_ws/src/rc_sim_description/urdf/` | Vehicle model. |
| `simulation/ros2_ws/src/rc_sim_description/config/` | Scenarios, SLAM, and fusion. |
| `simulation/tools/sim/` | Startup, mapping, and capture wrappers. |
| `simulation/data/` | Runs and analysis results. |
| `simulation/rviz/` | RViz layouts. |

## Auxiliary Real-Vehicle Areas

`real_vehicle/tools/` and `real_vehicle/ros2_ws/src/apex_telemetry/` preserve capture, analysis, and no-SLAM APEX workflows. They are useful for hardware diagnostics but do not replace the primary `voiture_system` SLAM stack.

## Related Documentation

- [Packages and Modules](10_packages_and_modules.md)
- [Launch Files and Flows](11_launch_files_and_execution_flows.md)
- [Known Limitations and Legacy Code](16_known_limitations_and_legacy_parts.md)
