# SLAM-Enabled Real Vehicle System

## Scope

The primary real vehicle in this layout is the `voiture_system` stack under `real_vehicle/`. It is the stack that includes SLAM and must be identified as the real vehicle.

## Recommended Real-Vehicle Stack

| Layer | Component |
| --- | --- |
| Workspace | `real_vehicle/ros2_ws` |
| Package | `real_vehicle/ros2_ws/src/voiture_system` |
| Launch | `bringup_real_slam_nav.launch.py` |
| SLAM | `slam_toolbox` |
| Optional navigation | Nav2 |
| Control | `adaptive_track_controller_node` |
| Odometry | `ackermann_odometry_node` |
| Actuation | `ackermann_drive_node` |
| LiDAR | `rplidar_publisher_node` |
| Vehicle state | `serial_state_node` |

## Expected Hardware

- Raspberry Pi.
- RPLIDAR on `/dev/ttyUSB0`.
- Arduino/state serial link on `/dev/ttyACM0`.
- ESC, motor, and steering servo.
- Encoder or speed state reported by the Arduino.
- Battery and a physical emergency cutoff.

Verify the actual hardware before accepting defaults.

## Build

```bash
cd ~/AiAtonomousRc/real_vehicle/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select voiture_system
source install/setup.bash
```

## Startup

```bash
ros2 launch voiture_system bringup_real_slam_nav.launch.py \
  use_slam:=true \
  use_nav2:=false \
  use_auto_track:=true \
  use_rviz:=true
```

| Argument | Default |
| --- | --- |
| `use_slam` | `true` |
| `use_nav2` | `false` |
| `use_auto_track` | `true` |
| `use_rviz` | `true` |
| `lidar_port` | `/dev/ttyUSB0` |
| `lidar_baudrate` | `256000` |
| `arduino_port` | `/dev/ttyACM0` |
| `arduino_baudrate` | `115200` |
| `speed_limit_pct` | `40.0` |

## Sensor and SLAM Flow

```text
RPLIDAR -> rplidar_publisher_node -> /lidar/scan -> slam_toolbox -> /map
Arduino -> serial_state_node -> /vehicle/speed_mps
  -> ackermann_odometry_node -> /odom
```

The launch publishes the static `base_link -> laser` transform. Odometry must provide a coherent TF chain for `slam_toolbox`.

## Control Flow

```text
/lidar/scan + /map -> adaptive_track_controller_node
  -> /cmd_vel -> ackermann_drive_node -> motor and steering
```

Nav2 is optional and should only be enabled after map, TF, and parameters are validated.

## Differences from Simulation

| Concern | Simulation | Real vehicle |
| --- | --- | --- |
| Sensors | Synthetic and repeatable | Real noise, dropouts, and calibration |
| Reference pose | Ground truth available | Unavailable |
| Actuation | Gazebo model | PWM and physical hardware |
| Time | May use `/clock` | Wall time |
| Risk | No physical movement | Safety procedures required |
| Workspace | `simulation/ros2_ws` | `real_vehicle/ros2_ws` |

## Safety Checklist

1. Lift or restrain the car.
2. Verify ESC neutral.
3. Verify steering center and direction.
4. Test LiDAR and Arduino separately.
5. Check `/odom`, TF, and `/map`.
6. Keep Nav2 disabled until SLAM is stable.
7. Prepare a physical power cutoff.
8. Keep people away from the test area.

## Auxiliary APEX Workflow

`real_vehicle/ros2_ws/src/apex_telemetry`, Docker, and `real_vehicle/tools/` contain additional IMU, LiDAR, fusion, and capture flows. The minimal mode can run without SLAM. It is not the primary real-vehicle stack.

## Legacy Stack Without SLAM

`archive/legacy/full_soft/` is the old Python stack. Do not confuse it with `voiture_system` or use it as the current hardware configuration.

## Related Documentation

- [Hardware Interfaces](19_hardware_interfaces.md)
- [ROS Architecture](07_ros_architecture.md)
- [Configuration Reference](17_configuration_reference.md)
- [Troubleshooting](15_troubleshooting.md)
