# Known Limitations and Legacy Code

## Current Classification

| Path | Classification | Reason |
| --- | --- | --- |
| `simulation/` | Active and currently recommended | Supports work without the car. |
| `real_vehicle/ros2_ws/src/voiture_system` | Primary real vehicle | Includes SLAM and optional Nav2. |
| `real_vehicle/ros2_ws/src/apex_telemetry` | Auxiliary/experimental | Capture, fusion, and APEX control; can run without SLAM. |
| `archive/legacy/full_soft` | Legacy without SLAM | Old Python code and direct drivers. |
| `simulation/legacy/Simulateur` | Legacy simulation | Earlier simulator. |
| `archive/generated` | Non-operational | Archived builds, caches, and runtime files. |

## Simulation Limitations

- ESC and servo dynamics are approximate.
- Noise and latency are configurable models, not exact hardware replicas.
- Worlds are not guaranteed CAD copies of the physical track.
- Ground truth is unavailable on the car and must not become a dependency.
- RF2O mode requires an additional package that may not be in Jazzy apt repositories.

## Real-Vehicle Limitations

- Launch defaults do not replace physical calibration.
- LiDAR baud defaults may not match every sensor.
- Nav2 is available but disabled by default.
- No native ground truth is available.
- PWM and serial access depend on system permissions.

## Duplicate Packages

`apex_telemetry` and `voiture_system` exist in more than one workspace. This is intentional environment separation, but their overlays must never be mixed.

## Legacy `full_soft`

- It does not use SLAM.
- Its historical entry point is `code/main.py`.
- It provides `--simulation`.
- Dependencies and paths may be obsolete.
- It is not the source of truth for current hardware parameters.
- It must remain under `archive/`.

## Overlapping Launch Paths

`apex_sim.launch.py` is the recommended Gazebo entry. `spawn_rc_car.launch.py`, `bringup_sim.launch.py`, and `simulation/legacy/Simulateur` are alternate or historical paths.

## Historical Reports

PDF and LaTeX reports under `docs/reports` and `simulation/docs/reports` may mention the earlier layout. Root Markdown documentation describes the current redistribution.

## Recommendations

1. Continue development in `simulation/`.
2. Keep `real_vehicle/` intact until the car is available.
3. Port changes to hardware only with targeted safety tests.
4. Keep `archive/` out of builds and imports.
5. Reduce package duplication gradually with regression tests.

## Related Documentation

- [Repository Structure](02_repository_structure.md)
- [Developer Guide](14_developer_guide.md)
- [Troubleshooting](15_troubleshooting.md)
