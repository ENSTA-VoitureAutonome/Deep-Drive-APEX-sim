# SLAM-Enabled Real Vehicle

This directory is reserved for the physical vehicle. Its primary stack is the ROS 2 `voiture_system` package, whose real launch integrates LiDAR, serial state, Ackermann odometry, vehicle control, and `slam_toolbox`.

## Component Roles

| Path | Purpose | Classification |
| --- | --- | --- |
| `ros2_ws/src/voiture_system/` | Primary real-vehicle stack with SLAM and optional Nav2. | Primary |
| `ros2_ws/src/apex_telemetry/` | APEX sensor, estimation, planning, tracking, and actuation pipeline. | Auxiliary/experimental |
| `docker/` | Container and device access for auxiliary APEX workflows. | Auxiliary |
| `tools/` | Capture, analysis, firmware, hardware checks, and PC utilities. | Auxiliary |
| `arduino/` | Sensor firmware used by the vehicle. | Hardware |
| `data/` | Captures and results obtained from the vehicle. | Data |

The old Python stack without SLAM is not stored here. It is preserved at `../archive/legacy/full_soft/`.

## Primary Stack: `voiture_system`

The primary launch file is:

```text
ros2_ws/src/voiture_system/launch/bringup_real_slam_nav.launch.py
```

Its defaults are `use_slam:=true`, `use_nav2:=false`, `use_auto_track:=true`, and `use_rviz:=true`.

```text
RPLIDAR -> /lidar/scan -> slam_toolbox -> /map
Arduino -> /vehicle/speed_mps -> ackermann_odometry_node -> /odom
adaptive_track_controller_node -> /cmd_vel
/cmd_vel -> ackermann_drive_node -> motor and steering
```

## Build

```bash
cd ~/AiAtonomousRc/real_vehicle/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select voiture_system
source install/setup.bash
```

## Run on the Vehicle

```bash
ros2 launch voiture_system bringup_real_slam_nav.launch.py \
  use_slam:=true \
  use_nav2:=false \
  use_auto_track:=true \
  use_rviz:=true
```

| Parameter | Default |
| --- | --- |
| `lidar_port` | `/dev/ttyUSB0` |
| `lidar_baudrate` | `256000` |
| `arduino_port` | `/dev/ttyACM0` |
| `arduino_baudrate` | `115200` |
| `max_steering_deg` | `18.0` |
| `speed_limit_pct` | `40.0` |

Always verify device paths and the actual LiDAR baud rate before launching.

## Safety

The real launch opens sensors and can issue actuator commands. Run it only on the vehicle, initially lifted or restrained, with a physical way to cut power. Do not use it as a desktop smoke test.

## Auxiliary APEX Tools Without SLAM

This directory also preserves a minimal APEX diagnostic and capture flow. It can start IMU, LiDAR, raw odometry, and controlled pulses without SLAM:

```bash
cd ~/AiAtonomousRc/real_vehicle
./tools/core/apex_core_down.sh
APEX_SKIP_BUILD=1 ./tools/capture/apex_raw_capture_up.sh
```

This auxiliary flow does not replace the primary SLAM-enabled `voiture_system` stack.

## Legacy Python Stack Without SLAM

The historical implementation independent of ROS 2 is stored at `archive/legacy/full_soft/`. Its old entry point is `code/main.py`, which accepts a `--simulation` option. Keep it as a reference; use `simulation/` or the real `voiture_system` stack for new work.

## Related Documentation

- [Real Vehicle System](../docs/09_blue_vehicle_real_system.md)
- [Hardware Interfaces](../docs/19_hardware_interfaces.md)
- [Launch Files and Flows](../docs/11_launch_files_and_execution_flows.md)
- [Known Limitations and Legacy Code](../docs/16_known_limitations_and_legacy_parts.md)
