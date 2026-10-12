"""Autonomy of the real car: learned odometry -> SLAM -> race driver -> adaptive control.

The driver, the lap-1 reactive planner, the race planner and the adaptive layer
are *copies* of the simulation modules under
``simulation/ros2_ws/src/apex_fusion_research/apex_fusion_research/core/``, kept
byte-identical so that what Gazebo validated is what the car runs.
``test/test_core_parity.py`` is the gate that proves it; ``core/sim_paths.py``
and ``core/vehicle_io.py`` are the only two files allowed to differ.

What the car must *never* learn is enforced by ``test/test_knowledge_firewall.py``:
no track definition, no speed calibration, no ground truth, no simulation tree.
"""
