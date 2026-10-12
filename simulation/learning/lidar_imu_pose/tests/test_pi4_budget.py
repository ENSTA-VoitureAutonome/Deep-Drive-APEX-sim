"""The Pi 4 compute budget: queue, freshness inversion, verdicts, cost
split, MAC counting, the IMU dead reckoning and the pair-width knob. No
dataset needed (the v5 checkpoint test is skipped without it)."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pi4_budget as pb  # noqa: E402
from live_odometry import propagate_imu  # noqa: E402
from streaming_model import PAIR_WIDTHS, PairEncoder, StreamingPoseNet, load_checkpoint  # noqa: E402

C = pb.Constants(queue_scans=4000)
NO_IMU = {"c0": 0.0, "c1_per_sample": 0.0, "ros_us": 0.0}
UNIT = pb.Factors(1, 1, 1, 1, 1)


def test_lindley_closed_form_matches_recursion():
    rng = np.random.default_rng(0)
    ready = np.cumsum(rng.uniform(0.05, 0.15, 500))
    service = rng.uniform(0.0, 0.2, 500)
    t, ref = -np.inf, []
    for r, s in zip(ready, service):
        t = max(r, t) + s
        ref.append(t)
    assert np.allclose(pb.lindley(ready, service), ref)


def test_queue_drains_below_and_diverges_above_the_period():
    stamps, ready = pb.arrival_model(3000, C)
    for s, grows in ((0.08, False), (0.12, True)):
        lat = pb.lindley(ready, np.full(len(ready), s)) - stamps
        first, last = lat[:300].mean(), lat[-300:].mean()
        assert (last > first + 1.0) == grows


def test_arrival_model():
    stamps, ready = pb.arrival_model(20000, pb.Constants())
    assert 0.985 < len(stamps) / 20000 < 0.995  # 1 % lost revolutions
    gap = ready - stamps  # revolution + coverage wait
    assert gap.min() > 0.09 and gap.max() < 0.1 + 0.05 + 0.03


def test_imu_share_grows_with_latency():
    cb = {"c0": 100.0, "c1_per_sample": 2.0, "ros_us": 50.0}
    u = [pb.imu_share(L, cb, 8.0, 1.0, C) for L in (0.1, 0.2, 0.4)]
    assert u[0] < u[1] < u[2]
    n = C.imu_rate_hz * (0.2 + 0.05)
    assert math.isclose(u[1], C.imu_rate_hz * (100 + 2 * n + 50) * 8 * 1e-6)


def _dr(e95):
    ages = [round(0.1 + 0.02 * k, 2) for k in range(36)]
    return {"ages_s": ages, "strata": {"v_ge_2p5": {"p95_xt_m": [e95(a) for a in ages]}}}


def test_r2_inversion():
    dr = _dr(lambda a: 0.1 * a * a)  # e95 = 0.1 A^2
    r = pb.latency_max_r2(dr, 0.2, C)
    a_ref = 0.3
    a_max = math.sqrt((0.1 * a_ref**2 + C.xt_budget_m) / 0.1)
    assert math.isclose(r["age_max_s"], a_max, rel_tol=2e-2)
    assert math.isclose(r["latency_p95_max_s"], a_max - C.scan_period_s, rel_tol=3e-2)
    # a larger budget allows a larger latency; the reference always passes
    r2 = pb.latency_max_r2(dr, 0.2, replace(C, xt_share=1.0))
    assert r2["latency_p95_max_s"] > r["latency_p95_max_s"] > 0.2


def test_bound_is_the_utilisation_cap_when_it_binds():
    trace = np.full(100, 10.0)  # ms
    b = pb.derive_budget(trace, 0.0, NO_IMU, UNIT, 10.0, C)
    assert b["feasible"] and b["binding"] == ["R1_utilisation"]
    assert math.isclose(b["service_max_ms"]["mean"], 1e3 * C.rho_max * C.scan_period_s, rel_tol=1e-3)


def test_bound_shrinks_with_the_imu_callback_and_contention():
    trace = np.full(100, 10.0)
    base = pb.derive_budget(trace, 0.0, NO_IMU, UNIT, 10.0, C)["service_max_ms"]["mean"]
    with_imu = pb.derive_budget(trace, 0.0, {"c0": 100.0, "c1_per_sample": 1.0, "ros_us": 0.0}, UNIT, 10.0, C)["service_max_ms"]["mean"]
    with_cont = pb.derive_budget(trace, 0.0, NO_IMU, replace(UNIT, contention=1.2), 10.0, C)["service_max_ms"]["mean"]
    assert with_imu < base and math.isclose(with_cont, base / 1.2, rel_tol=1e-3)


def test_latency_limit_can_bind():
    trace = np.full(100, 10.0)
    b = pb.derive_budget(trace, 0.0, NO_IMU, UNIT, 0.16, C)  # p95 latency <= 160 ms
    assert b["binding"] == ["R2_freshness"]
    assert b["operating_point"]["latency_ms"]["p95"] <= 160.0 + 1e-6


def test_verdicts():
    def entry(ok):
        return {"current": {"meets": {"R1_utilisation": ok, "R2_freshness": True, "G_scan_age": True}}}
    assert pb.verdict({"low": entry(True), "nominal": entry(True), "high": entry(True)}) == "PASS"
    assert pb.verdict({"low": entry(False), "nominal": entry(False), "high": entry(False)}) == "FAIL"
    assert pb.verdict({"low": entry(True), "nominal": entry(False), "high": entry(False)}) == "INDETERMINATE"
    assert pb.verdict({"low": entry(True), "high": entry(True), "measured": entry(False)}) == "FAIL"


def test_periodic_factor_from_a_given_live_run():
    """A candidate's own live run (the first 10 scans are warm-up) over the p50 of its replay."""
    k, src = pb.periodic_factor({"wall_ms_trace": [2.0] * 50}, {"service_ms_trace": [9.0] * 10 + [3.0] * 40})
    assert math.isclose(k, 1.5) and "given" in src


def test_component_split_accounts_for_the_wall_time():
    census = {"ms": {"compute": 1.0, "mem_cache": 2.0, "mem_dram": 3.0, "small": 50.0}, "top_ops": 1000}
    s = pb.component_split(10.0, census, 2.0)
    assert math.isclose(sum(s.values()), 10.0)
    assert s["dispatch"] == 2.0 and math.isclose(s["python"], 2.0)
    s = pb.component_split(3.0, census, 2.0)  # profiler inflation: work capped by the wall time
    assert math.isclose(sum(s.values()), 3.0) and s["dispatch"] == 0.0
    f = pb.Factors(compute=2, mem_cache=3, mem_dram=4, dispatch=5, python=6, contention=1.5)
    split = {"compute": 1, "mem_cache": 1, "mem_dram": 1, "dispatch": 1, "python": 1}
    assert math.isclose(pb.project_split(split, f), 20)  # idle Pi: contention enters the operating point


def test_spec_band_is_ordered():
    band = pb.spec_band(30.0)
    for cls in pb.CLASSES:
        assert band["low"].of(cls) < band["nominal"].of(cls) < band["high"].of(cls)
    gb6 = pb.GB6_SINGLE_PC / pb.GB6_SINGLE_PI4
    assert band["low"].dispatch < gb6 < band["high"].dispatch


def test_macs_by_hooks_match_the_formula():
    import latency_profile as lp
    enc = PairEncoder(128).eval()
    x = torch.rand(1, 1, 8, 360)
    macs = lp.macs_by_hooks(enc, lambda: enc(x))
    conv, length, cin = 0, 360, 8
    for w, (k, stride) in zip(PAIR_WIDTHS, ((7, 1), (5, 2), (5, 2), (3, 2))):
        length = (length + 2 * (k // 2) - k) // stride + 1
        conv += w * cin * k * length
        cin = w
    expected = conv + 128 * 32 * 9 + 32 * 9 * 128  # 1x1 bottleneck on 9 sectors + projection
    assert macs["total"] == expected


def test_propagate_imu_circle_and_line():
    v, w, dt = 2.0, 0.5, 0.002
    n = 500
    samples = [(int((k + 1) * dt * 1e9), np.array([0.0, v * w, 9.8, 0.0, 0.0, w])) for k in range(n)]
    _, pose, vel, rate = propagate_imu(np.zeros(3), np.array([v, 0.0]), np.zeros(3), 0, samples)
    t = n * dt
    r = v / w
    assert np.allclose(pose[:2], [r * math.sin(w * t), r * (1 - math.cos(w * t))], atol=5e-3)
    assert math.isclose(pose[2], w * t) and math.isclose(rate, w) and np.allclose(vel, [v, 0.0], atol=1e-2)
    samples = [(int((k + 1) * dt * 1e9), np.array([1.0, 0.0, 9.8, 0.0, 0.0, 0.0])) for k in range(n)]
    _, pose, vel, _ = propagate_imu(np.zeros(3), np.array([1.0, 0.0]), np.zeros(3), 0, samples)
    assert math.isclose(pose[0], t + 0.5 * t * t, rel_tol=2e-3) and math.isclose(vel[0], 1.0 + t)


def test_default_pair_widths_keep_the_layout():
    a = StreamingPoseNet(128, hybrid=True).state_dict()
    b = StreamingPoseNet(128, hybrid=True, pair_widths=PAIR_WIDTHS).state_dict()
    assert list(a) == list(b) and all(a[k].shape == b[k].shape for k in a)
    small = StreamingPoseNet(64, hybrid=True, pair_widths=(16, 24, 32, 48))
    assert sum(p.numel() for p in small.parameters()) < sum(v.numel() for v in a.values())
    with pytest.raises(ValueError):
        PairEncoder(128, widths=(10, 24, 32, 48))


CKPT = Path(__file__).resolve().parents[2] / "outputs" / "real2sim_v5_10hz" / "hybrid_submap_v4" / "best_model.pt"


@pytest.mark.skipif(not CKPT.exists(), reason="needs the v5 checkpoint")
def test_v5_checkpoint_loads_strictly():
    model, ck = load_checkpoint(CKPT)
    assert model.pair_widths == PAIR_WIDTHS and sum(p.numel() for p in model.parameters()) == 488_753
