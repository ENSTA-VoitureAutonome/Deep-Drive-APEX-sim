"""The plant a run draws must never reach the dataset or the car.

``Actuators`` gained the knobs that make a simulated car differ from the
description the controller is given. The pose dataset the odometry network was
trained on was generated with that same class, so every knob has to default to
the identity: if it does not, regenerating the dataset would silently produce a
different car and the network's domain would move under it.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
import pytest

SIM = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(SIM / "tools"))

from pose_dataset.vehicle import Actuators, draw_plant_mismatch, load_vehicle_config  # noqa: E402

CFG = load_vehicle_config()


def _trajectory(act: Actuators, commands) -> list[tuple[float, float]]:
    out = []
    for speed, steer in commands:
        act.set_command(speed, steer)
        for _ in range(50):
            act.step(0.001)
        out.append((act.speed, act.steer_deg))
    return out


COMMANDS = [(0.0, 0.0)] * 5 + [(1.5, 8.0)] * 40 + [(3.0, -12.0)] * 40 + [(0.05, 0.0)] * 20 + [(0.0, 0.0)] * 10


def test_the_dataset_actuators_are_untouched():
    plain = _trajectory(Actuators(CFG), COMMANDS)
    empty = _trajectory(Actuators(CFG, {}), COMMANDS)
    assert plain == empty
    # And the identity is reachable from the draw itself.
    zero = draw_plant_mismatch(np.random.default_rng(1), scale=0.0)
    assert _trajectory(Actuators(CFG, zero), COMMANDS) == plain


def test_a_drawn_plant_actually_differs():
    drawn = draw_plant_mismatch(np.random.default_rng(6))
    moved = _trajectory(Actuators(CFG, drawn), COMMANDS)
    assert moved != _trajectory(Actuators(CFG), COMMANDS)


def test_the_draw_is_reproducible_and_scales_around_the_datasheet():
    a = draw_plant_mismatch(np.random.default_rng(11))
    b = draw_plant_mismatch(np.random.default_rng(11))
    assert a == b
    nominal = {"wheelbase_m": 0.30, "steer_gain": 1.0, "esc_gain": 1.0, "right_ratio": 0.96,
               "steer_trim_deg": 0.0, "stiction_mps": 0.0, "esc_sag_per_min": 0.0}
    half = draw_plant_mismatch(np.random.default_rng(11), scale=0.5)
    for key, value in nominal.items():
        assert abs(half[key] - value) == pytest.approx(0.5 * abs(a[key] - value), abs=1e-12)


def test_the_car_can_reverse_only_when_the_run_allows_it():
    """The dataset car cannot reverse at all, which is why the key defaults to
    zero; a run that needs to back out of a dead end asks for it."""
    forward_only = Actuators(CFG)
    forward_only.set_command(-0.5, 0.0)
    assert forward_only.speed_cmd == 0.0
    backing = Actuators(CFG, {"reverse_top_speed_mps": 1.0})
    backing.set_command(-0.5, 0.0)
    assert backing.speed_cmd == pytest.approx(-0.5)
    for _ in range(2000):
        backing.step(0.001)
    assert backing.speed < -0.3


def test_the_start_threshold_holds_the_car_and_then_lets_go():
    act = Actuators(CFG, {"stiction_mps": 0.20})
    act.set_command(0.15, 0.0)
    for _ in range(1500):
        act.step(0.001)
    assert act.speed == pytest.approx(0.0, abs=1e-6), "below the threshold the car must not move"
    act.set_command(0.25, 0.0)
    for _ in range(2000):
        act.step(0.001)
    assert act.speed > 0.2
    # Once rolling it holds on with less than it took to break away.
    act.set_command(0.15, 0.0)
    for _ in range(2000):
        act.step(0.001)
    assert act.speed > 0.1


def test_the_battery_sags_over_the_run():
    act = Actuators(CFG, {"esc_sag_per_min": 0.04})
    assert act.battery_gain() == pytest.approx(1.0)
    act.set_command(3.0, 0.0)
    for _ in range(180_000):  # three minutes
        act.step(0.001)
    assert act.battery_gain() == pytest.approx(0.88, abs=0.01)
    assert act.speed < 2.75
