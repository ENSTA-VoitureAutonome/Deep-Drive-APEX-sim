# Topics, Services, Actions, and Parameters

This reference separates the APEX simulation interfaces from the real `voiture_system` interfaces.

## SLAM-Enabled Real Vehicle

| Topic | Type | Purpose |
| --- | --- | --- |
| `/lidar/scan` | `sensor_msgs/msg/LaserScan` | RPLIDAR scan for SLAM and control. |
| `/measured_wheelspeed` | `std_msgs/msg/Float64` | Measured wheel speed. |
| `/vehicle/speed_mps` | `std_msgs/msg/Float64` | Linear vehicle speed. |
| `/vehicle/steering_angle_cmd_rad` | `std_msgs/msg/Float64` | Commanded steering angle. |
| `/odom` | `nav_msgs/msg/Odometry` | Ackermann odometry. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM map. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Motion command. |

## Simulated APEX Sensors

| Topic | Type | Purpose |
| --- | --- | --- |
| `/apex/sim/scan` | `sensor_msgs/msg/LaserScan` | Raw Gazebo LiDAR. |
| `/apex/sim/imu` | `sensor_msgs/msg/Imu` | Gazebo IMU. |
| `/apex/imu/data_raw` | `sensor_msgs/msg/Imu` | IMU delivered to the APEX pipeline. |
| `/lidar/scan_localization` | `sensor_msgs/msg/LaserScan` | Scan used by estimation and planning. |

## APEX Estimation and Planning

| Topic | Type | Purpose |
| --- | --- | --- |
| `/apex/odometry/imu_lidar_fused` | `nav_msgs/msg/Odometry` | Fused odometry. |
| `/apex/estimation/path` | `nav_msgs/msg/Path` | Estimated path. |
| `/apex/estimation/live_map_points` | `sensor_msgs/msg/PointCloud2` | Local map. |
| `/apex/estimation/full_map_points` | `sensor_msgs/msg/PointCloud2` | Accumulated map. |
| `/apex/planning/recognition_tour_route` | `nav_msgs/msg/Path` | Global route. |
| `/apex/planning/recognition_tour_local_path` | `nav_msgs/msg/Path` | Local route. |
| `/apex/tracking/arm` | `std_msgs/msg/Bool` | Tracker arm state. |
| `/apex/cmd_vel_track` | `geometry_msgs/msg/Twist` | APEX motion command. |

## Simulated Actuation

- `/apex/sim/pwm/motor_dc`
- `/apex/sim/pwm/steering_dc`
- `/apex/vehicle/applied_speed_pct`
- `/apex/vehicle/applied_steering_deg`

## Ground Truth

- `/apex/sim/ground_truth/odom`
- `/apex/sim/ground_truth/path`
- `/apex/sim/ground_truth/perfect_map_points`
- `/apex/sim/ground_truth/status`
- `/clock`

## Services and Actions

The APEX pipeline exposes `std_srvs/srv/Trigger` services for kinematics reset and recalibration. Resolve their exact names with `ros2 service list`. No custom actions are defined; Nav2 actions appear only when `use_nav2:=true`.

## Parameter Files

| Environment | File |
| --- | --- |
| Simulated APEX | `simulation/ros2_ws/src/apex_telemetry/config/apex_params.yaml` |
| Scenarios | `simulation/ros2_ws/src/rc_sim_description/config/apex_sim_scenarios.json` |
| Simulated SLAM | `simulation/ros2_ws/src/rc_sim_description/config/slam_toolbox_sim*.yaml` |
| Real SLAM | `real_vehicle/ros2_ws/src/voiture_system/config/slam_toolbox_online_async.yaml` |
| Real Nav2 | `real_vehicle/ros2_ws/src/voiture_system/config/nav2_ackermann.yaml` |

## Related Documentation

- [ROS Architecture](07_ros_architecture.md)
- [Configuration Reference](17_configuration_reference.md)
- [Hardware Interfaces](19_hardware_interfaces.md)
