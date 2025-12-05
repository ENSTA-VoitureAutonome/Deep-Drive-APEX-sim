# RC Simulation Description

Description package for a simple RC car in ROS 2 with Gazebo (gz). Instructions are tailored for WSL2 on Windows with Ubuntu 24.04 (Noble) and ROS Jazzy.

## Package layout
- `urdf/`: `rc_car.urdf.xacro` with steerable front wheels.
- `launch/`: `spawn_rc_car.launch.py` starts Gazebo (gz) and spawns the robot.
- `worlds/`: `basic_track.world` with a rectangular track and arrow marker (local ground and light, no `model://`).
- `config/rviz/`: quick RViz config (optional).
- `meshes/`: placeholder for custom meshes.

## Installation (WSL2, Ubuntu 24.04, ROS Jazzy)
1) Base tools:
```
sudo apt update
sudo apt install -y curl gnupg2 lsb-release software-properties-common \
  build-essential git python3-colcon-common-extensions python3-vcstool
```

2) Add ROS 2 Jazzy apt repo:
```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
```

3) Install ROS desktop and runtime deps:
```
sudo apt install -y ros-jazzy-desktop \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher ros-jazzy-xacro
```

4) Install rosdep (on Noble use `python3-rosdep`), init and update:
```
sudo apt install -y python3-rosdep
sudo rosdep init        # first time only
rosdep update
```

5) (Optional) Confirm `gz` is on PATH:
```
gz --help
```

6) Build the workspace:
```
cd ~/AiAtonomousRc
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Run
Launch Gazebo and spawn the car:
```
ros2 launch rc_sim_description spawn_rc_car.launch.py z:=0.15
```

Useful arguments:
- `x`, `y`, `z`: initial pose (keep `z` > 0 to avoid spawn collision).
- `world`: SDF world path (defaults to `worlds/basic_track.world`).
- `rviz:=true` to open RViz (default `false`, Gazebo only).

## WSL notes
- Use WSLg or an X/Wayland server to view the Gazebo GUI; on Windows 11 with WSLg it usually works out of the box.
- If you switch ROS or Gazebo distros, adjust package names accordingly.
