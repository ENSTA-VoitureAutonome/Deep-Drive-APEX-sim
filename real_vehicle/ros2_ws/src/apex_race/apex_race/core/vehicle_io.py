"""Reading the car's own description (ROS-independent).

``core/race_config.vehicle_limits()`` takes the parsed ``vehicle.yaml`` mapping.
In the simulation that mapping is produced by ``pose_dataset.vehicle``, a module
that also builds the Gazebo SDF from the xacro and draws the plant mismatch of a
run -- neither of which may reach the car. Reading the file is the only part the
car needs, and it is one line.

The car's file is ``config/vehicle_real.yaml``: sections ``model``, ``esc``,
``servo`` and ``controller`` only. It carries no ``calibration`` section (that is
per-track knowledge) and no ``xacro`` path.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def load_vehicle_config(path: str | Path) -> dict[str, Any]:
    """Parse the car's description file."""
    return yaml.safe_load(Path(path).read_text())
