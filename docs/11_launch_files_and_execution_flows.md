# Launch Files and Execution Flows

## Recommended Entry Points

| Environment | Entry point | Use |
| --- | --- | --- |
| Simulation | `./simulation/tools/sim/apex_sim_up.sh --scenario baseline --rviz` | Current development without the car. |
| Direct simulation | `ros2 launch rc_sim_description apex_sim.launch.py` | Launch debugging. |
| Real vehicle | `ros2 launch voiture_system bringup_real_slam_nav.launch.py` | Physical hardware with SLAM. |
| Minimal real APEX | `real_vehicle/tools/capture/apex_raw_capture_up.sh` | No-SLAM diagnostics and capture. |

## Simulation Flow

```text
apex_sim_up.sh
  -> build simulation/ros2_ws
  -> apex_sim.launch.py
     -> Gazebo and vehicle spawn
     -> sensor bridges
     -> APEX pipeline
     -> motion bridge
     -> ground truth
     -> optional RViz, SLAM, and recorder
```

```bash
./simulation/tools/sim/apex_sim_up.sh \
  --scenario tight_right_saturation --rviz --slam --arm
```

## Real-Vehicle Flow

```text
bringup_real_slam_nav.launch.py
  -> rplidar_publisher_node
  -> serial_state_node
  -> ackermann_odometry_node
  -> base_link to laser TF
  -> slam_toolbox
  -> adaptive_track_controller_node
  -> ackermann_drive_node
  -> RViz
  -> optional Nav2
```

Inspect without running hardware:

```bash
cd real_vehicle/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch voiture_system bringup_real_slam_nav.launch.py --show-args
```

Run only on the vehicle:

```bash
ros2 launch voiture_system bringup_real_slam_nav.launch.py \
  use_slam:=true use_nav2:=false use_auto_track:=true
```

## Simulation Capture Flows

| Goal | Script |
| --- | --- |
| Recognition tour | `simulation/tools/sim/apex_recognition_tour_sim_capture.sh` |
| Manual mapping | `simulation/tools/sim/apex_manual_mapping_up.sh` |
| Finalize and reconstruct | `simulation/tools/sim/apex_manual_mapping_finish.sh` |
| Publish a general map | `simulation/tools/sim/apex_general_map_publisher_up.sh` |

## Auxiliary Real APEX Flows

Scripts under `real_vehicle/tools/capture/` start different sensor, fusion, and capture combinations. Some can generate PWM. Review their arguments and use physical safety precautions. They are not the primary SLAM launch.

## Alternate and Historical Entries

- `spawn_rc_car.launch.py`: simple Gazebo experiments.
- `simulation/ros2_ws/src/voiture_system/launch/bringup_sim.launch.py`: Classic Gazebo and `ros2_control` path.
- `apex_pipeline.launch.py`: APEX node composition.
- `archive/legacy/full_soft/code/main.py`: historical Python entry point, not a ROS launch.

## Process Cleanup

The simulation wrapper can terminate stale Gazebo/ROS processes. Set `APEX_SIM_KILL_STALE=0` if multiple sessions must coexist.

## Related Documentation

- [Quick Start](05_quick_start.md)
- [Gazebo Simulation](08_simulation_gazebo.md)
- [Real Vehicle](09_blue_vehicle_real_system.md)
- [Mapping and Recording](18_mapping_and_recording_pipeline.md)
