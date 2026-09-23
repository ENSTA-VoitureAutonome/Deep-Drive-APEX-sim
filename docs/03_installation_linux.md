# Installation on Linux

## Expected Environment

- Ubuntu 24.04.
- ROS 2 Jazzy.
- Gazebo Sim and `ros_gz` bridges.
- `colcon` and `rosdep`.
- RViz.
- Docker only for auxiliary real-vehicle APEX workflows.

## Prerequisites

```bash
sudo apt update
sudo apt install -y \
  build-essential curl git gnupg2 lsb-release \
  python3-colcon-common-extensions python3-pip python3-rosdep \
  python3-vcstool software-properties-common
```

Install ROS 2 Jazzy using the official ROS instructions, then run:

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init
rosdep update
```

If `rosdep init` was already run, use only `rosdep update`.

## ROS and Gazebo Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro ros-jazzy-rviz2 ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  python3-numpy python3-scipy python3-serial python3-yaml python3-pygame
```

`rf2o_laser_odometry` is optional and required only for the `rf2o_ekf` mode.

## Build the Simulation

```bash
cd ~/AiAtonomousRc/simulation/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select rc_sim_description apex_telemetry voiture_system
source install/setup.bash
```

## Validate the Simulation

```bash
ros2 pkg list | grep -E 'rc_sim_description|apex_telemetry|voiture_system'
ros2 launch rc_sim_description apex_sim.launch.py --show-args
```

From the repository root:

```bash
./simulation/tools/sim/apex_sim_up.sh --scenario baseline --rviz --skip-build
```

## Build the SLAM-Enabled Real Vehicle

Run this on the Raspberry Pi or only inspect the launch on a development machine:

```bash
cd ~/AiAtonomousRc/real_vehicle/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select voiture_system
source install/setup.bash
ros2 launch voiture_system bringup_real_slam_nav.launch.py --show-args
```

Do not execute the real launch as a desktop test. It opens `/dev/ttyUSB0`, `/dev/ttyACM0`, and can issue actuator commands.

## Common Issues

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| `ros2: command not found` | ROS is not sourced. | `source /opt/ros/jazzy/setup.bash`. |
| Package not found | Wrong workspace or unsourced install. | Enter its `ros2_ws`, rebuild, and source `install/setup.bash`. |
| Wrong package loaded | Real and simulation overlays were mixed. | Open a fresh terminal and source only one workspace. |
| Gazebo vehicle missing | Assets were not installed or sourced. | Rebuild `rc_sim_description` with `--symlink-install`. |
| `slam_toolbox` missing | Dependency not installed. | Install `ros-jazzy-slam-toolbox`. |

## Related Documentation

- [Windows Installation](04_installation_windows.md)
- [Quick Start](05_quick_start.md)
- [Troubleshooting](15_troubleshooting.md)
