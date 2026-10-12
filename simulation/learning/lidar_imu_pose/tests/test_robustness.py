"""Per-run perturbations of the stored streams (robustness.py): reproducible,
identity for variant 0, and the labels move with the inputs."""

from __future__ import annotations

from dataclasses import asdict
import math
from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robustness import RobustConfig, RunPerturbation, concat_splits  # noqa: E402

CONFIG = Path(__file__).resolve().parents[1] / "config" / "robust_real.yaml"


def _stream(seconds: float = 30.0, hz: float = 104.0):
    t = (np.arange(int(seconds * hz)) * (1e9 / hz)).astype(np.int64) + 5_000_000_000
    rng = np.random.default_rng(0)
    ga = rng.normal(0.0, 0.01, (len(t), 6))
    labels = rng.normal(0.0, 0.01, (len(t), 3))
    return t, ga, labels


def test_config_reads_the_thermal_block_of_the_sensor_profile():
    cfg = RobustConfig.load(CONFIG)
    assert cfg.thermal_params["gyro_coeff_std"] > 0.0
    assert cfg.tag() == RobustConfig.load(CONFIG).tag()
    assert cfg.tag() != RobustConfig(**{**asdict(cfg), "seed": cfg.seed + 1}).tag()


def test_variant_zero_is_the_identity_and_draws_are_reproducible():
    cfg = RobustConfig.load(CONFIG)
    t, ga, lab = _stream()
    t0, g0, l0 = RunPerturbation(cfg, 7, 0).apply_imu(t, ga, lab)
    assert t0 is t and g0 is ga and l0 is lab
    assert RunPerturbation(cfg, 7, 0).scan_keep(500).all()
    a, b = RunPerturbation(cfg, 7, 1).apply_imu(t, ga, lab), RunPerturbation(cfg, 7, 1).apply_imu(t, ga, lab)
    assert all(np.array_equal(x, y) for x, y in zip(a, b))
    c = RunPerturbation(cfg, 8, 1).apply_imu(t, ga, lab)
    assert not np.array_equal(a[1][:50], c[1][:50])


def test_labels_move_with_the_measurements_and_stamps_increase():
    cfg = RobustConfig.load(CONFIG)
    t, ga, lab = _stream()
    for run in range(30):
        p = RunPerturbation(cfg, run, 1)
        ts, meas, labels = p.apply_imu(t, ga, lab)
        keep = p.imu_keep(len(t))
        add = p.additive_bias(t)[keep]
        assert np.allclose(meas - ga[keep], add)
        assert np.allclose(labels - lab[keep], add[:, [2, 3, 4]])  # gz, ax, ay
        assert np.all(np.diff(ts) > 0)
        # Stamps: true instant + the run's constant offset + jitter (a few std at most).
        err = (ts - t[keep] - p.offset_ns) * 1e-6
        assert np.abs(err).max() < 6.0 * max(p.jitter_ns * 1e-6, 1e-3) + 1e-3
        if p.half_rate:
            assert 0.45 < keep.mean() < 0.51
        else:
            assert keep.mean() > 0.95


def test_thermal_ramp_has_the_profile_magnitude():
    cfg = RobustConfig.load(CONFIG)
    t = (np.arange(0, 3600 * 104) * (1e9 / 104)).astype(np.int64)
    p = RunPerturbation(cfg, 3, 1)
    b = p.additive_bias(t) - p.bias
    expect_end = p.coeff * p.rise * (1.0 - math.exp(-(p.t0 + 3600.0) / p.tau))
    assert np.allclose(b[-1], expect_end)
    assert np.allclose(b[0], p.coeff * p.rise * (1.0 - math.exp(-p.t0 / p.tau)))


def test_lost_revolutions_are_isolated():
    cfg = RobustConfig.load(CONFIG)
    lost_total = 0
    for run in range(40):
        keep = RunPerturbation(cfg, run, 2).scan_keep(1500)
        assert keep[0] and keep[-1]
        assert not np.any(~keep[1:] & ~keep[:-1])
        lost_total += int((~keep).sum())
    assert lost_total > 0


def test_concat_splits_shifts_offsets_and_run_indices():
    def part(n_runs: int, n: int) -> dict:
        return {"split": "x", "target": torch.zeros(n_runs * n, 3), "range_max": torch.ones(n_runs),
                "runs": [{"offset": i * n, "n": n} for i in range(n_runs)], "run_index": torch.arange(n_runs).repeat_interleave(n)}
    out = concat_splits([part(2, 5), part(3, 4)])
    assert [r["offset"] for r in out["runs"]] == [0, 5, 10, 14, 18]
    assert out["run_index"].tolist() == [0] * 5 + [1] * 5 + [2] * 4 + [3] * 4 + [4] * 4
    assert out["target"].shape[0] == 22 and out["range_max"].shape[0] == 5
