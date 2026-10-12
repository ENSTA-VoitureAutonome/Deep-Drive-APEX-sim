"""Where ``pose_dataset`` comes from on the car (ROS-independent).

The simulation's module of this name puts ``simulation/tools`` on ``sys.path``
so the nodes, the offline tools and the tests share the pose-dataset package.
There is no simulation tree on the car, so this shim points the same import at
``apex_race/vendor``, which holds copies of the four objects the driving logic
uses (``Path2D``, ``path_from_points``, ``PurePursuit``, ``_forward_backward``).

Keeping the module name and the function name is deliberate: it lets
``core/race_driver.py`` and ``core/race_planner.py`` stay byte-identical to the
simulation's, which is what ``test/test_core_parity.py`` checks. This file and
``core/vehicle_io.py`` are the only two the parity test allows to differ.
"""

from __future__ import annotations

from pathlib import Path
import sys


def sim_root() -> Path:
    """There is no simulation tree on the car; nothing may ask for one."""
    raise RuntimeError("no simulation tree on the car (apex_race is self-contained)")


def ensure_tools_path() -> Path:
    """Make ``pose_dataset`` importable from ``apex_race/vendor``; return it."""
    vendor = Path(__file__).resolve().parent.parent / "vendor"
    if str(vendor) not in sys.path:
        sys.path.insert(0, str(vendor))
    return vendor
