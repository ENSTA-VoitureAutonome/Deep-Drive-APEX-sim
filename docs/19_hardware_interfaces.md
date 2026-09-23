# Hardware Interfaces

## Scope

This page describes the SLAM-enabled real vehicle and separates auxiliary APEX hardware tools.

## Real `voiture_system` Stack

| Interface | Default | Node |
| --- | --- | --- |
| RPLIDAR | `/dev/ttyUSB0`, `256000` baud | `rplidar_publisher_node` |
| Arduino | `/dev/ttyACM0`, `115200` baud | `serial_state_node` |
| Motor and steering | Linux PWM | `ackermann_drive_node` |
| LiDAR topic | `/lidar/scan` | SLAM and control |
| Speed topic | `/vehicle/speed_mps` | Odometry |
| Command topic | `/cmd_vel` | Ackermann driver |

Defaults are declared in `bringup_real_slam_nav.launch.py`.

## SLAM and TF

`slam_toolbox` requires:

```text
map -> odom -> base_link -> laser
```

The launch publishes `base_link -> laser`. Verify physical offsets and signs on the car.

## PWM and Safety

Before testing, restrain the vehicle, verify ESC neutral, check steering center and sign, limit speed, and prepare a physical power cutoff. Never run real actuation nodes as a workstation smoke test.

## Serial Permissions

```bash
sudo usermod -aG dialout "$USER"
ls -l /dev/ttyUSB0 /dev/ttyACM0
```

Log out after changing group membership.

## Auxiliary APEX Hardware

`real_vehicle/docker/docker-compose.yml` and `real_vehicle/tools/hardware` use another interface set whose defaults can differ, including RPLIDAR at `115200`. Validate the actual sensor instead of copying values between stacks.

Auxiliary APEX support includes Nano IMU serial input, I2C/UART checks, sysfs PWM, LiDAR/IMU capture, and PC-side controller bridges.

## ROS 2 Networking

When Raspberry Pi, WSL, and Windows exchange topics, align `ROS_DOMAIN_ID`, RMW middleware, DDS discovery, firewall rules, and network interfaces.

## Simulation Equivalents

| Hardware | Simulation |
| --- | --- |
| RPLIDAR | Gazebo LiDAR and `ros_gz_bridge` |
| Arduino/encoder | Model state and bridges |
| ESC/servo | PWM topics and vehicle bridge |
| Physical track | SDF world |
| External reference pose | Ground truth |

## Pre-Run Checklist

1. Confirm device paths.
2. Confirm the LiDAR baud rate.
3. Verify power and common ground.
4. Inspect topics before allowing movement.
5. Validate TF and `/odom`.
6. Enable SLAM and inspect `/map`.
7. Enable control last.

## Related Documentation

- [Real Vehicle](09_blue_vehicle_real_system.md)
- [Configuration Reference](17_configuration_reference.md)
- [Troubleshooting](15_troubleshooting.md)
