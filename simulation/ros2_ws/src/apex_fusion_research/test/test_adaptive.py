"""The shaper must be the unmodified driver at the prior, and better after."""

from __future__ import annotations

import math

import numpy as np
import pytest

from _plant_stub import Plant, drive
from apex_fusion_research.core.adaptive import AdaptiveShaper
from apex_fusion_research.core.plant_id import PlantIdentifier
from apex_fusion_research.core.race_config import RaceDriverConfig


def legacy_command(cfg: RaceDriverConfig, steer: float) -> float:
    """The curvature the driver published before the adaptive layer existed:
    ``steer_command`` divided right turns by the linkage ratio, and the node
    then published ``v * tan(steer) / wheelbase``."""
    veh = cfg.vehicle
    cmd = steer / veh.right_ratio if steer < 0.0 else steer
    lim = math.radians(veh.steer_max_deg)
    return math.tan(max(-lim, min(lim, cmd))) / veh.wheelbase_m


def test_at_the_prior_the_chain_is_the_unmodified_driver():
    cfg = RaceDriverConfig()
    veh = cfg.vehicle
    shaper = AdaptiveShaper(cfg)
    model = PlantIdentifier(cfg).prior()
    env = shaper.envelope(model)
    worst_rel, worst_left = 0.0, 0.0
    for deg in np.linspace(-veh.steer_max_deg, veh.steer_max_deg, 73):
        steer = math.radians(deg)
        kappa_des = math.tan(steer) / veh.wheelbase_m
        shaper.reset()
        shaper.shape(0.0, kappa_des, 1.5, 1.5, model, env)
        got = shaper.shape(0.02, kappa_des, 1.5, 1.5, model, env)[1]
        expect = legacy_command(cfg, steer)
        if abs(expect) > env.kappa_limit:
            continue
        if steer >= 0.0:  # no asymmetry on the left: it must be exact
            worst_left = max(worst_left, abs(got - expect))
        elif abs(expect) > 1e-9:
            worst_rel = max(worst_rel, abs(got - expect) / abs(expect))
    assert worst_left < 1e-12, f"the left-hand chain moved by {worst_left:.2e}"
    # On the right the linkage asymmetry is now a gain on the curvature rather
    # than on the angle. That is the correct place for it (and the identifier
    # replaces it with what it measures), and it is the same controller to
    # within the tan nonlinearity: 0.3 % of command at full lock.
    assert worst_rel < 0.005, f"the right-hand chain differs by {100 * worst_rel:.2f} %"


def test_the_speed_command_at_the_prior_is_the_request():
    cfg = RaceDriverConfig()
    shaper = AdaptiveShaper(cfg)
    model = PlantIdentifier(cfg).prior()
    env = shaper.envelope(model)
    assert shaper.shape(0.0, 0.0, 0.0, 0.0, model, env)[0] == 0.0
    v, _ = shaper.shape(0.02, 0.0, 2.0, 2.0, model, env)
    assert v == pytest.approx(2.0, abs=1e-9)


def test_the_inverse_of_the_identified_map_is_the_identity():
    cfg = RaceDriverConfig()
    ident = PlantIdentifier(cfg)
    drive(ident, Plant(gain_l=1.3, gain_r=0.8, trim=0.05), 90.0,
          lambda t: (0.5 * math.sin(2.0 * math.pi * t / 6.0), 1.6 + 1.0 * math.sin(0.13 * t)))
    model = ident.model()
    for kappa in (-0.6, -0.2, 0.0, 0.2, 0.6):
        for speed in (1.0, 2.0, 3.5):
            assert model.kappa_out(model.kappa_in(kappa, speed), speed) == pytest.approx(kappa, abs=1e-9)


def test_the_envelope_is_never_more_daring_than_the_nominal_car():
    cfg = RaceDriverConfig()
    shaper = AdaptiveShaper(cfg)
    ident = PlantIdentifier(cfg)
    prior = ident.prior()
    last = None
    for c in np.linspace(0.0, 1.0, 11):
        env = shaper.envelope(prior.__class__(**{**prior.__dict__, "confidence": float(c)}))
        assert env.v_max_mps <= cfg.race.v_max_mps + 1e-12
        assert env.a_lat_mps2 <= cfg.race.a_lat_mps2 + 1e-12
        assert env.lap1_v_scale <= 1.0 + 1e-12
        assert env.blind_margin_add_m >= -1e-12
        if last is not None:  # monotone in the confidence, in the safe direction
            assert env.v_max_mps >= last.v_max_mps - 1e-12
            assert env.blind_margin_add_m <= last.blind_margin_add_m + 1e-12
        last = env
    full = shaper.envelope(prior.__class__(**{**prior.__dict__, "confidence": 1.0}))
    assert full.v_max_mps == pytest.approx(cfg.race.v_max_mps)
    assert full.blind_margin_add_m == pytest.approx(0.0)


def test_the_lag_lead_is_capped_and_vanishes_in_the_steady_state():
    cfg = RaceDriverConfig()
    shaper = AdaptiveShaper(cfg)
    model = PlantIdentifier(cfg).prior()
    env = shaper.envelope(model)
    t = 0.0
    for _ in range(40):  # a step of curvature: the lead must saturate, not explode
        t += 0.02
        shaper.shape(t, 0.8, 2.0, 2.0, model, env)
        assert abs(shaper.lead) <= cfg.adaptive.lead_max + 1e-12
    for _ in range(60):
        t += 0.02
        shaper.shape(t, 0.8, 2.0, 2.0, model, env)
    assert abs(shaper.lead) < 0.01


def test_the_launch_ramp_is_bounded_and_clears_the_start_threshold():
    cfg = RaceDriverConfig()
    shaper = AdaptiveShaper(cfg)
    model = PlantIdentifier(cfg).prior()
    env = shaper.envelope(model)
    t, last = 0.0, 0.0
    for _ in range(300):  # 6 s of a car that refuses to move
        t += 0.02
        last = shaper.shape(t, 0.0, 0.5, 0.0, model, env)[0]
    assert last > model.dead_mps
    assert last <= 0.5 + cfg.adaptive.launch_ramp_mps2 * 0.5 + model.dead_mps + 0.1
    moving = shaper.shape(t + 0.02, 0.0, 0.5, 0.6, model, env)[0]
    assert shaper.launch_extra == 0.0 and moving == pytest.approx(0.5, abs=0.15)


def test_adaptation_tracks_a_mismatched_plant_better_than_the_datasheet():
    """The point of the whole exercise, on the quantity the tracking cares
    about: the curvature the car actually delivers for the one requested."""
    cfg = RaceDriverConfig()
    request = lambda t: (0.55 * math.sin(2.0 * math.pi * t / 7.0), 1.8 + 1.0 * math.sin(0.11 * t))  # noqa: E731

    def run(enabled: bool) -> float:
        cfg_run = RaceDriverConfig()
        cfg_run.adaptive.enabled = enabled
        ident = PlantIdentifier(cfg_run)
        shaper = AdaptiveShaper(cfg_run)
        plant = Plant(gain_l=1.32, gain_r=0.78, trim=0.07, arm=0.20, v_gain=0.82, tau_k=0.15)
        err: list[float] = []
        t, applied = 0.0, (0.0, 0.0)
        for step in range(int(120.0 / 0.02)):
            kappa_des, v_des = request(t)
            model = ident.model() if enabled else ident.prior()
            v_cmd, kappa_cmd = shaper.shape(t, kappa_des, v_des, plant.v, model, shaper.envelope(model))
            ident.push_command(t, kappa_cmd, v_cmd)
            for _ in range(10):
                plant.step(0.002, *applied)
            applied = (kappa_cmd, v_cmd)
            t += 0.02
            if step % 5 == 4:
                ident.on_estimate(t, plant.report(), (0.0032, 0.0032, 0.0009))
                if t > 60.0:  # score the second half, once it has had time to learn
                    err.append(plant.k - kappa_des)
        return float(np.sqrt(np.mean(np.square(err))))

    adapted, nominal = run(True), run(False)
    assert adapted < 0.7 * nominal, f"adapted rms {adapted:.4f} vs nominal {nominal:.4f}"


def test_the_control_rate_cost_fits_the_pi_budget():
    """What runs at 50 Hz is the shaper and one command into the history.

    The estimator is already at the Pi's limit (``pi4_budget.py``) and the whole
    control loop is budgeted 0.30 of a core, so this layer has to be invisible.
    The Pi 5 runs the odometry about 3-4x slower than this host, so a budget of
    100 us here is a few per cent of one 20 ms tick there.
    """
    import time

    cfg = RaceDriverConfig()
    ident = PlantIdentifier(cfg)
    shaper = AdaptiveShaper(cfg)
    model = ident.prior()
    env = shaper.envelope(model)
    n, t = 4000, 0.0
    for i in range(200):  # warm up the interpreter on both paths
        shaper.shape(0.02 * i, 0.3, 1.5, 1.5, model, env)
        ident.push_command(0.02 * i, 0.3, 1.5)
    t0 = time.perf_counter()
    for i in range(n):
        t += 0.02
        v, kappa = shaper.shape(t, 0.4 * math.sin(0.1 * i), 2.0, 1.9, model, env)
        ident.push_command(t, kappa, v)
    per_tick_us = 1e6 * (time.perf_counter() - t0) / n
    assert per_tick_us < 100.0, f"{per_tick_us:.1f} us per control tick"


def test_the_envelope_of_an_unsure_car_is_the_careful_one():
    """A plain reading of the schedule: at no confidence the car asks for less
    speed, less grip and more margin, and looks further ahead."""
    cfg = RaceDriverConfig()
    shaper = AdaptiveShaper(cfg)
    prior = PlantIdentifier(cfg).prior()
    unsure = shaper.envelope(prior)
    sure = shaper.envelope(prior.__class__(**{**prior.__dict__, "confidence": 1.0}))
    assert unsure.v_max_mps < sure.v_max_mps
    assert unsure.a_lat_mps2 < sure.a_lat_mps2
    assert unsure.lap1_v_scale < sure.lap1_v_scale
    assert unsure.blind_margin_add_m > sure.blind_margin_add_m
    assert unsure.lookahead_add_m > sure.lookahead_add_m
    assert unsure.kappa_limit < sure.kappa_limit
