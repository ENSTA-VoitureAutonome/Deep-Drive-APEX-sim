# System Architecture

## High-Level View

```text
                      control and SLAM logic
                               |
             +-----------------+-----------------+
             |                                   |
     simulation/                           real_vehicle/
  Gazebo + bridges                    LiDAR + Arduino
 sensors + ground truth               Ackermann + SLAM
```

The two environments use separate workspaces. They share concepts and selected code, but their overlays must not be loaded together.

## Main Runtime Paths

| Flow | Purpose | Entry point |
| --- | --- | --- |
| APEX simulation | Develop and validate without the car. | `simulation/tools/sim/apex_sim_up.sh` |
| SLAM-enabled real vehicle | Run physical hardware and build `/map`. | `voiture_system bringup_real_slam_nav.launch.py` |
| Auxiliary real APEX | Capture sensors and diagnose hardware. | `real_vehicle/tools/capture/` |
| Legacy Python | Inspect the old no-SLAM algorithm. | `archive/legacy/full_soft/code/main.py` |

## Simulation Flow

```text
Gazebo world + URDF
  -> ros_gz_bridge
  -> /apex/sim/scan + /apex/sim/imu
  -> perception and estimation
  -> planner and tracker
  -> /apex/cmd_vel_track
  -> simulated PWM bridge
  -> Gazebo vehicle dynamics
```

Ground truth runs in parallel so pose, path, and map can be evaluated without becoming mandatory estimator inputs.

## Real-Vehicle Flow

```text
RPLIDAR -> /lidar/scan -> slam_toolbox -> /map
Arduino -> /vehicle/speed_mps -> ackermann_odometry_node -> /odom
/map + /odom + /lidar/scan -> adaptive_track_controller_node
  -> /cmd_vel -> ackermann_drive_node -> motor and steering
```

`bringup_real_slam_nav.launch.py` enables SLAM by default and can optionally enable Nav2.

## Subsystem Responsibilities

| Subsystem | Simulation | Real vehicle |
| --- | --- | --- |
| Environment | Gazebo worlds | Physical track |
| LiDAR | Gazebo sensor and bridge | Serial RPLIDAR |
| Motion | Physics model | ESC, motor, and servo |
| Odometry | Ground truth or estimators | Serial state plus Ackermann model |
| SLAM | `slam_toolbox` or APEX estimation | `slam_toolbox` |
| Control | Planner/tracker or gamepad | Adaptive controller and `/cmd_vel` |
| Ground truth | Available | Unavailable |
| Physical safety | No mechanical risk | Mandatory procedures |

## Auxiliary Real APEX

`real_vehicle/ros2_ws/src/apex_telemetry` and `real_vehicle/tools` preserve a second workflow for IMU, LiDAR, fusion, capture, and PWM. Some configurations run without SLAM. The SLAM-enabled real-vehicle label belongs to `voiture_system`.

## Legacy Stack Without SLAM

`archive/legacy/full_soft` uses direct Python interfaces and does not depend on `slam_toolbox`. It is not part of an active ROS workspace.

## Design Principle

Validate algorithms against scenarios and ground truth in `simulation/` first. Port them explicitly to `real_vehicle/` only when hardware and safety testing are available.

## Related Documentation

- [ROS Architecture](07_ros_architecture.md)
- [Gazebo Simulation](08_simulation_gazebo.md)
- [SLAM-Enabled Real Vehicle](09_blue_vehicle_real_system.md)
- [Launch Files and Flows](11_launch_files_and_execution_flows.md)
