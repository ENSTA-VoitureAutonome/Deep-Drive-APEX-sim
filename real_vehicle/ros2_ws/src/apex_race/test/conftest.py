"""Make the package importable without building or sourcing the ROS workspace.

The car runs these modules with ``PYTHONPATH=/work/ros2_ws/src/apex_race``;
this does the same for pytest, so the suite runs from anywhere in the repo.
"""

import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_ROOT.parents[3]
SIM_ROOT = REPO_ROOT / "simulation"
SIM_TEST = SIM_ROOT / "ros2_ws" / "src" / "apex_fusion_research" / "test"

sys.path.insert(0, str(PKG_ROOT))
if SIM_TEST.is_dir():  # the parity and identifiability tests are PC-side gates
    sys.path.insert(0, str(SIM_TEST))


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: closed-loop identification runs (~10 s)")
