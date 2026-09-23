# Mapping and Recording Pipeline

## Purpose

Record inputs, estimates, commands, and references so behavior can be analyzed and compared across changes.

## Simulation

### Recognition Tour

```bash
./simulation/tools/sim/apex_recognition_tour_sim_capture.sh \
  --scenario tight_right_saturation --timeout-s 60
```

### Manual Mapping

```bash
./simulation/tools/sim/apex_manual_mapping_up.sh \
  --scenario precision_fusion --rviz
./simulation/tools/sim/apex_manual_mapping_finish.sh
```

## Data Flow

```text
Gazebo LiDAR + IMU + pose
  -> ROS bridges
  -> estimation or SLAM
  -> recorder
  -> CSV and JSON
  -> offline reconstruction
  -> metrics and plots
```

Ground truth is recorded for evaluation.

## Output

Simulation runs are stored under `simulation/data/`. The repository contains an example at:

```text
simulation/data/rc_sim_description/runs/lap_manual_01/
└── offline_reconstruction/
    └── reconstruction_overview.png
```

## Real Vehicle

The `voiture_system` launch publishes `/map` and `/odom` but does not include the same APEX recorder by default. Hardware capture scripts remain under `real_vehicle/tools/capture/`. Some can enable sensing, fusion, or actuation, so use them only with a safe vehicle.

## Comparison Rules

Normalize frames, units, initial pose, sampling rates, track geometry, vehicle parameters, and duration. Never present simulated ground truth as a measurement available on the real vehicle.

## Recommended Practices

1. Use a unique run ID.
2. Save the scenario and parameters.
3. Record the commit and dirty state.
4. Preserve raw inputs.
5. Store derived output in subdirectories.
6. Never overwrite runs.
7. Add a readable summary.

## Related Documentation

- [Data and Runs](13_data_and_runs.md)
- [Gazebo Simulation](08_simulation_gazebo.md)
- [Configuration Reference](17_configuration_reference.md)
