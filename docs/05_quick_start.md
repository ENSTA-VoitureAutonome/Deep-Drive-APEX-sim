# Quick Start

This page gives the shortest path for each environment without mixing them.

## Build the Simulation Once

```bash
cd ~/AiAtonomousRc/simulation/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select rc_sim_description apex_telemetry voiture_system
source install/setup.bash
```

## Start the Recommended Simulation

```bash
cd ~/AiAtonomousRc
./simulation/tools/sim/apex_sim_up.sh --scenario baseline --rviz --skip-build
```

Expected result:

- Gazebo loads the selected track.
- The vehicle model is spawned.
- Bridges publish LiDAR, IMU, and state.
- The control pipeline uses simulated backends.
- RViz displays maps, trajectory, and estimation.

Checks:

```bash
ros2 topic echo /apex/sim/scan --once
ros2 topic echo /apex/sim/imu --once
ros2 topic echo /apex/sim/ground_truth/odom --once
```

Arm the recognition tour:

```bash
./simulation/tools/sim/apex_arm_recognition_tour.sh
```

Or start armed:

```bash
./simulation/tools/sim/apex_sim_up.sh --scenario baseline --rviz --arm
```

## Direct Launch

```bash
cd ~/AiAtonomousRc/simulation/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export APEX_SIM_ROOT=~/AiAtonomousRc/simulation
ros2 launch rc_sim_description apex_sim.launch.py \
  scenario:=baseline rviz:=true
```

## SLAM-Enabled Real Vehicle

Run only on the physical vehicle:

```bash
cd ~/AiAtonomousRc/real_vehicle/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select voiture_system
source install/setup.bash
ros2 launch voiture_system bringup_real_slam_nav.launch.py \
  use_slam:=true use_nav2:=false use_rviz:=true
```

This is the stack identified as the real vehicle. Restrain the car, verify LiDAR and Arduino, and prepare a physical power cutoff before launching.

## Auxiliary APEX Flow Without SLAM

```bash
cd ~/AiAtonomousRc/real_vehicle
APEX_SKIP_BUILD=1 ./tools/capture/apex_raw_capture_up.sh
```

This is a diagnostic/capture flow, not the primary SLAM launch.

## Legacy Python Stack Without SLAM

```bash
cd ~/AiAtonomousRc/archive/legacy/full_soft/code
python3 main.py --simulation
```

Its old dependencies may no longer be reproducible. Treat it as a reference.

## Useful Commands

| Task | Command |
| --- | --- |
| Show simulation arguments | `ros2 launch rc_sim_description apex_sim.launch.py --show-args` |
| Show real launch arguments | `ros2 launch voiture_system bringup_real_slam_nav.launch.py --show-args` |
| Inspect SLAM map | `ros2 topic echo /map --once` |
| Inspect real odometry | `ros2 topic echo /odom --once` |
| Inspect simulation ground truth | `ros2 topic echo /apex/sim/ground_truth/odom --once` |

## Related Documentation

- [Gazebo Simulation](08_simulation_gazebo.md)
- [SLAM-Enabled Real Vehicle](09_blue_vehicle_real_system.md)
- [Launch Files and Flows](11_launch_files_and_execution_flows.md)
- [Troubleshooting](15_troubleshooting.md)
