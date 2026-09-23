# Configuration Reference

## Main Configuration Files

| File | Environment | Purpose |
| --- | --- | --- |
| `simulation/ros2_ws/src/rc_sim_description/config/apex_sim_scenarios.json` | Simulation | World, spawn, vehicle, noise, and overrides. |
| `simulation/ros2_ws/src/apex_telemetry/config/apex_params.yaml` | APEX simulation | Sensors, fusion, planning, tracking, and simulated actuation. |
| `simulation/ros2_ws/src/rc_sim_description/config/slam_toolbox_sim.yaml` | Simulation | Standard SLAM. |
| `simulation/ros2_ws/src/rc_sim_description/config/slam_toolbox_sim_ideal.yaml` | Simulation | SLAM with ideal inputs. |
| `simulation/ros2_ws/src/rc_sim_description/config/slam_toolbox_sim_rf2o_ekf.yaml` | Simulation | RF2O/EKF SLAM. |
| `real_vehicle/ros2_ws/src/voiture_system/config/slam_toolbox_online_async.yaml` | Real | Vehicle SLAM. |
| `real_vehicle/ros2_ws/src/voiture_system/config/nav2_ackermann.yaml` | Real | Nav2. |
| `real_vehicle/ros2_ws/src/voiture_system/config/controllers.yaml` | Real/alternate | ROS controllers. |
| `real_vehicle/ros2_ws/src/apex_telemetry/config/apex_params.yaml` | Auxiliary real APEX | APEX pipeline. |
| `real_vehicle/docker/docker-compose.yml` | Auxiliary real APEX | Docker, devices, and flags. |

## Simulation Scenarios

`apex_sim_scenarios.json` defines the world, spawn pose, steering and speed limits, motor/servo model, sensor noise and latency, and pipeline overrides. Select one with `--scenario` or `scenario:=NAME`.

## Real Launch Parameters

`bringup_real_slam_nav.launch.py` declares:

- Feature flags: `use_slam`, `use_nav2`, `use_auto_track`, and `use_rviz`.
- LiDAR port, baud, frame, offset, FOV, and range.
- Arduino port, baud, tick conversion, and rate.
- Wheelbase and Ackermann geometry.
- Speed, steering, rate, and timeout limits.
- Odometry frames and TF settings.
- SLAM and Nav2 YAML paths.

## Important Real Defaults

- `use_slam=true`
- `use_nav2=false`
- `lidar_port=/dev/ttyUSB0`
- `lidar_baudrate=256000`
- `arduino_port=/dev/ttyACM0`
- `arduino_baudrate=115200`
- `max_steering_deg=18.0`
- `speed_limit_pct=40.0`

Do not assume these values still match the vehicle after a sensor change.

## Simulated APEX Parameters

Important groups include simulated IMU and LiDAR backends, planar fusion, map and odometry topics, planner/tracker settings, `sim_pwm_topic` actuation, and session recording.

## Configuration Rule

Do not share one absolute-path YAML between real and simulation environments. Keep environment-specific defaults and document parameters that intentionally stay synchronized.

## Related Documentation

- [Topics and Parameters](12_topics_services_actions_parameters.md)
- [Gazebo Simulation](08_simulation_gazebo.md)
- [Real Vehicle](09_blue_vehicle_real_system.md)
