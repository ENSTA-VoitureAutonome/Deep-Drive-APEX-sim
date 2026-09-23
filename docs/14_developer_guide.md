# Developer Guide

## Development Principles

- Develop in `simulation/` while the car is unavailable.
- Keep `simulation/ros2_ws` and `real_vehicle/ros2_ws` separate.
- Use stable ROS interfaces between sensing, estimation, and control.
- Evaluate against ground truth without feeding it into the estimator.
- Keep legacy code under `archive/` out of active dependencies.

## Where to Add Code

| Change | Location |
| --- | --- |
| Gazebo world, model, or bridge | `simulation/ros2_ws/src/rc_sim_description` |
| Simulated APEX algorithm | `simulation/ros2_ws/src/apex_telemetry` |
| SLAM-enabled real driver/control | `real_vehicle/ros2_ws/src/voiture_system` |
| Scenario parameters | `simulation/ros2_ws/src/rc_sim_description/config` |
| Simulation wrapper | `simulation/tools/sim` |
| Data analysis | `simulation/tools/analysis` or `real_vehicle/tools/analysis` |

## Recommended Workflow

1. Select or create a scenario.
2. Change code under `simulation/ros2_ws/src`.
3. Build with `--symlink-install`.
4. Run a headless smoke test.
5. Run with RViz.
6. Record a run.
7. Compare against ground truth.
8. Document parameters and limitations.
9. Port to the real vehicle only when hardware and safety testing are available.

## Minimum Validation

```bash
cd simulation/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select rc_sim_description apex_telemetry voiture_system
source install/setup.bash
python3 -m compileall -q src
```

```bash
timeout 45s ros2 launch rc_sim_description apex_sim.launch.py \
  scenario:=baseline rviz:=false gazebo_gui:=false
```

## Shared Changes

`apex_telemetry` and `voiture_system` have environment-specific copies. If a change belongs in both, apply and test it explicitly in each copy; do not create a combined overlay.

## Documentation Updates

When paths, topics, launch arguments, or parameters change, update the environment README and the relevant page under `docs/`. Keep historical reports intact when they describe earlier layouts.

## Working with Legacy Code

Port small, tested ideas from `archive/legacy/full_soft`. Do not add the whole directory to `PYTHONPATH` or an active ROS workspace.

## Related Documentation

- [Repository Structure](02_repository_structure.md)
- [Configuration Reference](17_configuration_reference.md)
- [Troubleshooting](15_troubleshooting.md)
