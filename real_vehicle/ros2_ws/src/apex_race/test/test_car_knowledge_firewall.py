"""The car may only use what it has: its odometry model, its own limits and its
noisy sensors (through the estimator and the SLAM).

This is the simulation's firewall test
(``simulation/ros2_ws/src/apex_fusion_research/test/test_knowledge_firewall.py``)
applied to the car's package, plus the three things that only matter here:

* there is no simulation tree on the car, so nothing may reach for one;
* ``pose_dataset`` is a two-module copy and must stay one -- in particular
  ``pose_dataset.vehicle`` (which builds the Gazebo model and draws the plant
  mismatch of a run) must not be imported, unlike on the simulation side;
* the whole package is scanned, not a hand-maintained list, so a new module
  cannot slip past.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

# Paths, computed here and not imported from `conftest`: pytest keeps one
# module named `conftest` per session, so a suite that also collects the
# simulation's tests would import the wrong one.
PKG_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_ROOT.parents[3]
SIM_ROOT = REPO_ROOT / "simulation"
SIM_TEST = SIM_ROOT / "ros2_ws" / "src" / "apex_fusion_research" / "test"

CAR_PKG = PKG_ROOT / "apex_race"
SOURCES = sorted(p for p in CAR_PKG.rglob("*.py") if "__pycache__" not in p.parts)

# The plant a simulated run draws (its real wheelbase, the position of its
# axles, the gain and centre of its servo, the command its motor starts at, the
# charge of its battery) is written into run_config.json for the evaluation.
# The car identifies all of that from its own odometry, so reading any of it
# would make the identification meaningless. The same goes for the track: the
# driver has to find it with its own LiDAR.
FORBIDDEN = ("run_setup", "load_track", "speed_calibration", "/apex/sim/", "ground_truth", "truth", "race_eval", "clearance import",
             "_pose_tools", "gz_runner", "tracks_index", "plant_true", "draw_plant_mismatch", "perturb",
             # car-only additions
             "APEX_SIM_ROOT", "simulation/", "pose_dataset.vehicle", "pose_dataset.sensors", "pose_dataset.geometry")

ALLOWED_POSE_DATASET = {"pose_dataset.controller": {"PurePursuit", "_forward_backward"},
                        "pose_dataset.tracks": {"Path2D", "path_from_points"},
                        ".tracks": {"Path2D"}}


def _code_without_docstrings(path: Path) -> str:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef, ast.AsyncFunctionDef)) and node.body:
            first = node.body[0]
            if isinstance(first, ast.Expr) and isinstance(getattr(first, "value", None), ast.Constant) and isinstance(first.value.value, str):
                node.body = node.body[1:] or [ast.Pass()]
    return ast.unparse(tree)


def test_the_package_is_not_empty():
    assert len(SOURCES) >= 18, "the firewall is scanning the wrong directory"


@pytest.mark.parametrize("path", SOURCES, ids=lambda p: str(p.relative_to(CAR_PKG)))
def test_no_truth_track_or_simulation_access(path: Path):
    code = _code_without_docstrings(path)
    for word in FORBIDDEN:
        assert word not in code, f"{path.relative_to(CAR_PKG)} uses {word!r}"


def test_race_driver_declares_no_track_parameters():
    tree = ast.parse((CAR_PKG / "nodes" / "race_driver_node.py").read_text(encoding="utf-8"))
    declared = {n.args[0].value for n in ast.walk(tree)
                if isinstance(n, ast.Call) and getattr(n.func, "id", "") == "dp" and n.args and isinstance(n.args[0], ast.Constant)}
    assert declared.isdisjoint({"track", "motion", "seed", "v_ref", "start_pose", "truth_topic", "pose_source"})
    assert "laps" in declared
    # The real car's own additions: nothing may command it unarmed.
    assert {"arm_topic", "require_arm", "arm_timeout_s"} <= declared


@pytest.mark.parametrize("path", SOURCES, ids=lambda p: str(p.relative_to(CAR_PKG)))
def test_only_the_vendored_pose_dataset_objects_are_imported(path: Path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom) or not node.module:
            continue
        if not (node.module.startswith("pose_dataset") or (node.module == ".tracks" and "vendor" in path.parts)):
            continue
        allowed = ALLOWED_POSE_DATASET.get(node.module)
        assert allowed is not None, f"{path.relative_to(CAR_PKG)} imports {node.module}"
        names = {a.name for a in node.names}
        assert names <= allowed, f"{path.relative_to(CAR_PKG)} imports {names - allowed} from {node.module}"


def test_the_vendored_package_defines_only_the_four_reused_objects():
    vendor = CAR_PKG / "vendor" / "pose_dataset"
    defined = set()
    for path in sorted(vendor.glob("*.py")):
        for node in ast.parse(path.read_text(encoding="utf-8")).body:
            if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
                defined.add(node.name)
    assert defined == {"Path2D", "path_from_points", "_forward_backward", "PurePursuit"}, f"vendored surface drifted: {sorted(defined)}"


def test_nothing_resolves_a_path_into_the_simulation():
    """`sim_paths.sim_root()` must raise, not search upwards for a sim tree."""
    from apex_race.core.sim_paths import ensure_tools_path, sim_root  # noqa: PLC0415

    with pytest.raises(RuntimeError):
        sim_root()
    vendor = ensure_tools_path()
    assert vendor == CAR_PKG / "vendor" and vendor.is_dir()
