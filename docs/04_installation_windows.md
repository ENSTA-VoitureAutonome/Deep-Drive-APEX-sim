# Installation on Windows

## Support Level

The recommended path is Windows 11 with WSL2, Ubuntu 24.04, and WSLg. ROS 2 and Gazebo run inside WSL. Native PowerShell is used only for utilities such as the gamepad bridge.

## Prepare WSL2

Run in an elevated PowerShell:

```powershell
wsl --install -d Ubuntu-24.04
```

Inside Ubuntu, follow [Installation on Linux](03_installation_linux.md).

## Build the Simulation

```bash
cd ~/AiAtonomousRc/simulation/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select rc_sim_description apex_telemetry voiture_system
source install/setup.bash
```

Run from the repository root:

```bash
./simulation/tools/sim/apex_sim_up.sh --scenario baseline --rviz
```

WSLg should display Gazebo and RViz. If it does not, check `echo $DISPLAY`, update WSL, and avoid mixing Windows GUI packages with Ubuntu packages.

## Xbox Controller from Windows

Build the bridge once:

```bash
./simulation/tools/windows/build_apex_xbox_bridge_sim.sh
```

Run on Windows:

```text
simulation/tools/windows/dist/apex_xbox_bridge_sim.exe
```

Then launch in WSL:

```bash
./simulation/tools/sim/apex_sim_up.sh \
  --scenario precision_fusion \
  --control-mode manual_windows_bridge \
  --rviz
```

Use `manual_xbox` only when Linux/WSL can see the device directly through `pygame`.

## Real Vehicle from Windows

The SLAM-enabled real stack runs on the Raspberry Pi. Windows/WSL can be used to synchronize code, open SSH, build `real_vehicle/ros2_ws`, and monitor ROS. Execute the hardware launch only beside a safely restrained car.

## ROS 2 Networking

Align `ROS_DOMAIN_ID`, DDS middleware, and firewall rules between WSL, Windows, and the Raspberry Pi. WSL mirrored networking usually improves multicast discovery.

## Related Documentation

- [Linux Installation](03_installation_linux.md)
- [Gazebo Simulation](08_simulation_gazebo.md)
- [Hardware Interfaces](19_hardware_interfaces.md)
