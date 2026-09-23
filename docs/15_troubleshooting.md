# Troubleshooting

## First Checks

```bash
pwd
git status --short --branch
echo "$ROS_DISTRO"
ros2 pkg list | grep -E 'rc_sim_description|apex_telemetry|voiture_system'
echo "$AMENT_PREFIX_PATH" | tr ':' '\n'
```

The simulation and real install prefixes must not appear together.

## Build Issues

| Symptom | Cause | Fix |
| --- | --- | --- |
| Package not found | Workspace not sourced | Source ROS and that workspace's `install/setup.bash`. |
| Wrong package loaded | Overlays mixed | Open a fresh shell. |
| Dependency missing | Incomplete `rosdep` install | Run `rosdep install --from-paths src --ignore-src -r -y`. |
| Python import failure | Stale build | Rebuild with `--symlink-install`. |

## Gazebo Issues

| Symptom | Action |
| --- | --- |
| Vehicle missing | Rebuild `rc_sim_description` and source the simulation workspace. |
| No LiDAR or IMU | Inspect `ros_gz_bridge` and `/apex/sim/*` topics. |
| Vehicle does not move | Inspect simulated PWM and `apex_gz_vehicle_bridge`. |
| RViz has no data | Check fixed frame, TF, and `use_sim_time`. |
| Old processes interfere | Let `apex_sim_up.sh` clean them or stop the old session. |

## Real-Vehicle SLAM Issues

| Symptom | Action |
| --- | --- |
| No `/map` | Confirm `use_slam:=true` and install `slam_toolbox`. |
| Missing TF | Verify `odom -> base_link -> laser`. |
| Distorted map | Check LiDAR baud/FOV, odometry, and sensor geometry. |
| No `/odom` | Check Arduino, `serial_state_node`, and `ackermann_odometry_node`. |
| No motion | Check `/cmd_vel`, limits, and PWM access while the car is restrained. |

## Serial Devices

```bash
ls -l /dev/ttyUSB0 /dev/ttyACM0
groups
```

The user normally needs the `dialout` group. The real launch defaults to LiDAR at `256000` and Arduino at `115200`, but actual hardware may differ.

## Windows and WSL2

Use WSLg for Gazebo/RViz, check firewall and mirrored networking, and use the Windows Xbox bridge if WSL cannot see the controller. Do not run physical drivers from PowerShell.

## Legacy Code

If `archive/legacy/full_soft` does not start, treat it as historical dependency debt. Do not replace the modern environment with the old stack.

## Related Documentation

- [Linux Installation](03_installation_linux.md)
- [Gazebo Simulation](08_simulation_gazebo.md)
- [Real Vehicle](09_blue_vehicle_real_system.md)
- [Hardware Interfaces](19_hardware_interfaces.md)
