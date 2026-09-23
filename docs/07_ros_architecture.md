# ROS Architecture

## Workspace Layout

```text
simulation/ros2_ws/src/
├── rc_sim_description/
├── apex_telemetry/
└── voiture_system/

real_vehicle/ros2_ws/src/
├── voiture_system/
└── apex_telemetry/
```

Each workspace has its own `build/`, `install/`, and `log/` directories.

## ROS 2 Packages

| Package | Build type | Responsibility |
| --- | --- | --- |
| `rc_sim_description` | `ament_cmake` | Gazebo model, worlds, launches, bridges, ground truth, and recording. |
| `voiture_system` | `ament_python` | Primary physical SLAM stack plus simulation compatibility. |
| `apex_telemetry` | `ament_python` | APEX sensors, estimation, planning, tracking, and actuation bridge. |

## Simulation Package Hierarchy

```text
rc_sim_description/
├── launch/apex_sim.launch.py
├── config/apex_sim_scenarios.json
├── urdf/rc_car.urdf.xacro
├── worlds/
└── scripts/

apex_telemetry/
├── imu/
├── odometry/
├── estimation/
├── perception/
├── control/
└── actuation/
```

## Real-Vehicle Package Hierarchy

```text
voiture_system/
├── launch/bringup_real_slam_nav.launch.py
├── config/slam_toolbox_online_async.yaml
├── config/nav2_ackermann.yaml
└── voiture_system/
    ├── rplidar_publisher_node.py
    ├── serial_state_node.py
    ├── ackermann_odometry_node.py
    ├── ackermann_drive_node.py
    └── adaptive_track_controller_node.py
```

## Primary Real-Vehicle Nodes

| Node | Input | Output |
| --- | --- | --- |
| `rplidar_publisher_node` | Serial RPLIDAR | `/lidar/scan` |
| `serial_state_node` | Serial Arduino | `/vehicle/speed_mps`, `/measured_wheelspeed` |
| `ackermann_odometry_node` | Speed and steering | `/odom` and TF |
| `adaptive_track_controller_node` | LiDAR and map | `/cmd_vel` |
| `ackermann_drive_node` | `/cmd_vel` | Motor and steering commands |
| `slam_toolbox` | `/lidar/scan` and TF | `/map` |

## Main Simulation Nodes

- `apex_gz_vehicle_bridge.py`: converts control output into Gazebo motion.
- `apex_ground_truth_node.py`: publishes reference odometry, path, and map.
- `apex_sim_run_recorder.py`: stores run data.
- APEX IMU and LiDAR nodes use simulated backends.
- `imu_lidar_planar_fusion_node`: APEX estimation.
- Recognition-tour planner and tracker: planning and control.
- `cmd_vel_to_apex_actuation_node`: simulated PWM backend.

## Key Topics

### Real Vehicle

- `/lidar/scan`
- `/vehicle/speed_mps`
- `/vehicle/steering_angle_cmd_rad`
- `/odom`
- `/map`
- `/cmd_vel`

### APEX Simulation

- `/apex/sim/scan`
- `/apex/sim/imu`
- `/apex/sim/pwm/motor_dc`
- `/apex/sim/pwm/steering_dc`
- `/apex/sim/ground_truth/odom`
- `/apex/odometry/imu_lidar_fused`
- `/apex/planning/recognition_tour_local_path`
- `/apex/cmd_vel_track`

## TF and Frames

| Frame | Use |
| --- | --- |
| `map` | Global SLAM reference. |
| `odom` | Continuous odometry reference. |
| `base_link` | Vehicle body. |
| `laser` | LiDAR. |
| `imu_link` | APEX IMU. |
| `rear_axle` | Ackermann/APEX control reference. |

The real launch publishes the static `base_link -> laser` transform. `slam_toolbox` requires a coherent chain from `map` through `odom` to `base_link`.

## Parameter Locations

- Real vehicle: `real_vehicle/ros2_ws/src/voiture_system/config/`.
- Simulation: `simulation/ros2_ws/src/rc_sim_description/config/`.
- Simulated APEX: `simulation/ros2_ws/src/apex_telemetry/config/apex_params.yaml`.
- Auxiliary real APEX: `real_vehicle/ros2_ws/src/apex_telemetry/config/apex_params.yaml`.

## Working Rules

1. Open a fresh shell when switching workspaces.
2. Build inside the correct `ros2_ws`.
3. Do not copy simulation parameters to hardware without validation.
4. Inspect topics and TF before enabling control.
5. Use ground truth for evaluation, not to hide estimator errors.

## Related Documentation

- [Topics and Parameters](12_topics_services_actions_parameters.md)
- [Configuration Reference](17_configuration_reference.md)
- [Glossary](20_glossary_ros_terms.md)
