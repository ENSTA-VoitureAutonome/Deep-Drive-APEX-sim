"""Can this car identify its own plant at the envelope it is allowed to drive?

The adaptive controller is only worth deploying if its three Kalman blocks ever
accept a measurement. They do not accept them at any speed: the lever arm needs
``adaptive.v_min_update_mps``, the curvature map needs ``adaptive.v_min_steer_mps``
and the speed map needs a command above ``dead + deadband_margin_mps``
(``plant_id.py``). With the bridge as it shipped -- 0.24 m/s at full scale --
none of the three can be met, so the identified model stays at the prior, the
confidence envelope stays at its floor and the car drives exactly as it did
before the feature existed.

This test closes the loop on a synthetic plant that includes **the bridge's own
speed map**, at the two envelopes, and asserts the difference. It is the cheapest
place where the central claim of the real-car port is either true or false, and
it needs no hardware, no ROS and no simulator.

The plant and the driving loop are the simulation's unit-test stubs
(``test/_plant_stub.py``), so this measures the same identifier the simulation
measured, not a re-implementation.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest

# Paths, computed here and not imported from `conftest`: pytest keeps one
# module named `conftest` per session, so a suite that also collects the
# simulation's tests would import the wrong one.
PKG_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_ROOT.parents[3]
SIM_ROOT = REPO_ROOT / "simulation"
SIM_TEST = SIM_ROOT / "ros2_ws" / "src" / "apex_fusion_research" / "test"

if not SIM_TEST.is_dir():  # pragma: no cover - the car has no simulation tree
    pytest.skip("no simulation tree (the identifiability gate is a PC-side check)", allow_module_level=True)

sys.path.insert(0, str(PKG_ROOT.parents[1] / "src" / "apex_telemetry"))

from _plant_stub import Plant, drive  # noqa: E402
from apex_race.core.adaptive import AdaptiveShaper  # noqa: E402
from apex_race.core.config_io import dataclass_from_flat, flatten_dataclass  # noqa: E402
from apex_race.core.plant_id import PlantIdentifier  # noqa: E402
from apex_race.core.race_config import RaceDriverConfig, vehicle_limits  # noqa: E402
from apex_race.core.vehicle_io import load_vehicle_config  # noqa: E402
from apex_telemetry.actuation.speed_map import SpeedMap  # noqa: E402

CONFIG_DIR = PKG_ROOT / "config"

# What the ESC is believed to do: it starts at 17 % and the span 17 -> 25 %
# covered 0 -> 0.24 m/s, i.e. ~0.030 m/s per percent (apex_params.yaml, the
# legacy profile). Whether that slope is right is exactly what the staged
# motion test re-measures; the point here is the *structure* of the map.
ESC_THRESHOLD_PCT = 17.0
ESC_MPS_PER_PCT = 0.030

OPTION_B = SpeedMap(esc_zero_pct=15.0, pct_per_mps=33.0, max_pct=45.0)
# The shipped bridge, expressed as the closest affine map there is: a step to
# 17 % and a ceiling 8 points above it.
OPTION_A = SpeedMap(esc_zero_pct=17.0, pct_per_mps=1.0 / ESC_MPS_PER_PCT, max_pct=25.0)


def real_config(**overrides: float) -> RaceDriverConfig:
    """The configuration the car will actually run, from its own YAML files."""
    cfg = RaceDriverConfig()
    if overrides:
        flat = flatten_dataclass(cfg)
        flat.update(overrides)
        cfg = dataclass_from_flat(RaceDriverConfig, flat)
    cfg.vehicle = vehicle_limits(load_vehicle_config(CONFIG_DIR / "vehicle_real.yaml"), cfg.vehicle.steer_plan_deg)
    return cfg


def race_driver_yaml_overrides() -> dict[str, float]:
    """The deltas of ``config/race_driver.yaml``, as flat config keys."""
    import yaml  # noqa: PLC0415

    params = yaml.safe_load((CONFIG_DIR / "race_driver.yaml").read_text())["apex_race_driver"]["ros__parameters"]
    flat: dict[str, float] = {}
    for section, values in params.items():
        if isinstance(values, dict):
            for key, value in values.items():
                flat[f"{section}.{key}"] = value
    return flat


class BridgePlant(Plant):
    """The synthetic car *behind the bridge*: commands in m/s, ESC percent inside.

    ``Plant`` already models a start threshold and a speed gain in command
    units. Here the command first goes through the bridge's affine map and then
    through the ESC's own threshold and slope, which is what the real chain does
    and what makes the residual deadband and the identified ``v_gain`` mean
    something.
    """

    def __init__(self, speed_map: SpeedMap, **kw) -> None:
        super().__init__(**kw)
        self.map = speed_map

    def step(self, dt: float, kappa_cmd: float, v_cmd: float) -> None:
        pct = self.map.pct(v_cmd)
        delivered = math.copysign(max(0.0, abs(pct) - ESC_THRESHOLD_PCT) * ESC_MPS_PER_PCT, pct) if pct else 0.0
        # `Plant` applies its own v_gain/v_off/dead on top; they are 1/0/0 here
        # because the whole longitudinal plant is the chain above.
        super().step(dt, kappa_cmd, delivered)


def cruise_and_weave(v: float, kappa: float, period: float = 6.0):
    """A command a lap-1 driver would produce: cruise, with the lane's curvature."""
    def cmd(t: float) -> tuple[float, float]:
        if t < 1.0:  # the launch, which is where the threshold is bracketed
            return 0.0, v
        return kappa * math.sin(2.0 * math.pi * (t - 1.0) / period), v
    return cmd


class LiveModel:
    """The model the shaper sees is the one identified so far, as on the car.

    The node calls ``ident.model()`` on every control tick; the simulation's
    ``drive()`` helper takes a single object, so this forwards to the current
    one and is refreshed from the command callback.
    """

    def __init__(self, ident: PlantIdentifier) -> None:
        self._ident = ident
        self.refresh()

    def refresh(self) -> None:
        self._model = self._ident.model()

    def __getattr__(self, name):  # noqa: ANN001
        return getattr(self._model, name)


def run_identification(speed_map: SpeedMap, v: float, kappa: float, cfg: RaceDriverConfig, duration: float = 40.0):
    """Drive the loop with the shaper in it; return the identifier and the plant."""
    ident = PlantIdentifier(cfg)
    shaper = AdaptiveShaper(cfg)
    plant = BridgePlant(speed_map, gain_l=1.15, gain_r=1.05, trim=0.02, arm=0.148, v_gain=1.0, v_off=0.0, dead=0.0,
                        tau_k=0.09, tau_v=0.22)
    live = LiveModel(ident)
    command = cruise_and_weave(v, kappa)

    def stepping_cmd(t: float) -> tuple[float, float]:
        live.refresh()
        return command(t)

    drive(ident, plant, duration, stepping_cmd, shaper=shaper, model=live, rng=np.random.default_rng(11))
    return ident, plant


# --------------------------------------------------------------------- option B
@pytest.mark.slow
def test_option_b_identifies_the_plant():
    cfg = real_config(**race_driver_yaml_overrides())
    assert cfg.vehicle.steer_max_deg == pytest.approx(12.5)
    kappa = 0.6 * math.tan(math.radians(cfg.vehicle.steer_max_deg)) / cfg.vehicle.wheelbase_m
    ident, plant = run_identification(OPTION_B, cfg.reactive.v_explore_mps, kappa, cfg)
    st = ident.status()
    model = ident.model()
    assert st["n"]["steer"] >= 40, st
    assert st["n"]["lever"] >= 30, st
    assert st["n"]["long"] >= 20, st
    assert not st["sign_fault"]
    assert st["trips"] == 0
    assert st["confidence"] > 0.8, st
    assert st["conf_lever"] > 0.8, st
    # The identified numbers must be the plant's, not the prior's.
    assert model.gain_left == pytest.approx(plant.gain_l, abs=0.08)
    assert abs(model.rear_offset_m() + plant.arm) < 0.02
    assert model.v_gain == pytest.approx(ESC_MPS_PER_PCT * OPTION_B.pct_per_mps, abs=0.10)


@pytest.mark.slow
def test_option_b_leaves_only_a_small_residual_deadband():
    """The gauge offset sits below the threshold, so the residual is a creep."""
    residual = OPTION_B.residual_deadband_mps(ESC_THRESHOLD_PCT)
    assert 0.0 < residual < 0.12, residual
    cfg = real_config(**race_driver_yaml_overrides())
    assert residual < cfg.adaptive.dead_max_mps, "the identifier could not even bracket the threshold"
    # ... and the prior the car ships with is the right order of magnitude.
    assert abs(residual - cfg.vehicle.min_effective_mps) < 0.05


# --------------------------------------------------------------------- option A
@pytest.mark.slow
def test_the_shipped_bridge_cannot_identify_anything():
    """Why the bridge had to change: at 0.24 m/s every gate refuses."""
    cfg = real_config(**race_driver_yaml_overrides())
    ident, _ = run_identification(OPTION_A, 0.20, 0.5, cfg, duration=40.0)
    st = ident.status()
    assert st["n"]["steer"] == 0, st["n"]
    assert st["n"]["lever"] == 0, st["n"]
    assert st["confidence"] == 0.0
    assert st["rejected"]["slow"] > 0, st["rejected"]


def test_the_two_lateral_gates_are_below_the_cruise_speed():
    """The arithmetic the whole envelope rests on, as an assertion."""
    cfg = real_config(**race_driver_yaml_overrides())
    # With margin, not merely below: the lap-1 planner floors the speed at
    # v_min_mps when the proven lane is short, and a gate sitting exactly at the
    # floor refuses half of those samples.
    assert cfg.adaptive.v_min_update_mps <= cfg.reactive.v_min_mps - 0.03, "the lever arm is gated at the speed floor"
    assert cfg.adaptive.v_min_steer_mps <= cfg.reactive.v_explore_mps - 0.10, "the curvature map is gated at the cruise speed"
    assert cfg.reactive.v_min_mps >= 0.3, "below the odometry network's training floor"
    assert cfg.race.v_max_mps <= OPTION_B.full_scale_mps() + 1e-9, "the driver may ask for more than the bridge can deliver"


def test_the_envelope_never_exceeds_the_nominal_configuration():
    """At zero confidence everything is slower; at full confidence it is nominal."""
    cfg = real_config(**race_driver_yaml_overrides())
    shaper = AdaptiveShaper(cfg)
    prior = PlantIdentifier(cfg).prior()
    env = shaper.envelope(prior)
    assert env.v_max_mps <= cfg.race.v_max_mps
    assert env.a_lat_mps2 <= cfg.race.a_lat_mps2
    assert 0.0 < env.lap1_v_scale <= 1.0
    assert env.kappa_limit <= math.tan(math.radians(cfg.vehicle.steer_max_deg)) / cfg.vehicle.wheelbase_m + 1e-9
