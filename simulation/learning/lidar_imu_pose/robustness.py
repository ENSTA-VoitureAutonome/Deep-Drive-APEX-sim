"""Per-run perturbations of the stored dataset for a robust odometry (v3).

The v2 dataset (Gazebo-native sensors + realism layer, profile ``APEX_real``)
models the hardware at its correct timing and with the turn-on biases of the
two real IMU recordings. What the real car can add, and what the network must
tolerate without a new Gazebo campaign, is applied here on the stored
streams, per run and per variant, reproducibly:

* gyro / accelerometer bias: a constant top-up that brings the turn-on bias
  to the datasheet spread (the profile in ``sensors.yaml`` holds the widened
  values; the stored runs were drawn with the old ones), and the warm-up
  drift of the board (``imu.thermal`` of the sensor profile, the same model
  as ``apex_fusion_research.core.imu_chip``);
* IMU arrival: a constant stamp offset with respect to the LiDAR, host
  reception jitter, isolated and burst losses, and a lower output data rate
  (every second sample: 52 Hz, a firmware that does not reach 104 Hz);
* LiDAR arrival: lost revolutions (the interval then spans two periods).

The additive biases also shift the bias labels, and the labels of the
increments follow the scans that are kept, so inputs and targets stay
consistent. Variant 0 is always the unperturbed run.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
import torch
import yaml

HERE = Path(__file__).resolve().parent
TOOLS = HERE.parents[1] / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from pose_dataset.sensors import load_sensor_profiles  # noqa: E402


@dataclass(frozen=True)
class RobustConfig:
    profile: str = "APEX_real"  # sensor profile whose ``imu.thermal`` block drives the warm-up drift
    thermal: bool = True
    gyro_bias_topup_std: tuple[float, float, float] = (0.0, 0.0, 0.0)  # rad/s, x y z
    accel_bias_topup_std: tuple[float, float, float] = (0.0, 0.0, 0.0)  # m/s^2
    imu_offset_ms: tuple[float, float] = (0.0, 0.0)  # uniform, constant per run (IMU stamps - LiDAR stamps)
    imu_jitter_std_ms: tuple[float, float] = (0.0, 0.0)  # uniform range of the per-run jitter std
    imu_dropout_prob: tuple[float, float] = (0.0, 0.0)  # uniform range of the isolated loss probability
    imu_burst_prob: tuple[float, float] = (0.0, 0.0)
    imu_burst_len: tuple[int, int] = (2, 4)
    imu_half_rate_prob: float = 0.0  # probability that the run keeps every second IMU sample
    scan_dropout_prob: tuple[float, float] = (0.0, 0.0)
    seed: int = 101
    thermal_params: dict[str, Any] = field(default_factory=dict)  # filled from the sensor profile

    @staticmethod
    def load(path: str | Path) -> "RobustConfig":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        tup = {k: tuple(v) if isinstance(v, list) else v for k, v in raw.items()}
        cfg = RobustConfig(**tup)
        th = load_sensor_profiles()[cfg.profile]["imu"].get("thermal", {}) if cfg.thermal else {}
        return RobustConfig(**{**asdict(cfg), "thermal_params": dict(th)})

    def tag(self) -> str:
        return hashlib.sha1(json.dumps(asdict(self), sort_keys=True, default=list).encode()).hexdigest()[:10]


class RunPerturbation:
    """The perturbation of one run and variant (all draws from its own seed)."""

    def __init__(self, cfg: RobustConfig, run_id: int, variant: int) -> None:
        self.cfg, self.run_id, self.variant = cfg, int(run_id), int(variant)
        self.identity = variant == 0
        rng = np.random.default_rng(np.random.SeedSequence((cfg.seed, self.run_id, self.variant)))
        self.bias = np.concatenate((rng.normal(0.0, cfg.gyro_bias_topup_std), rng.normal(0.0, cfg.accel_bias_topup_std)))  # gx gy gz ax ay az
        th = cfg.thermal_params
        self.thermal_on = bool(th) and (float(th.get("gyro_coeff_std", 0.0)) > 0.0 or float(th.get("accel_coeff_std", 0.0)) > 0.0)
        self.coeff = np.concatenate((rng.normal(0.0, float(th.get("gyro_coeff_std", 0.0)), 3), rng.normal(0.0, float(th.get("accel_coeff_std", 0.0)), 3)))
        self.rise = float(rng.uniform(*th.get("rise_c", (10.0, 25.0))))
        self.tau = float(rng.uniform(*th.get("tau_s", (300.0, 900.0))))
        frac = min(float(rng.uniform(*th.get("start_frac", (0.0, 1.0)))), 0.999)
        self.t0 = -self.tau * math.log1p(-frac)
        self.offset_ns = int(round(rng.uniform(*cfg.imu_offset_ms) * 1e6))
        self.jitter_ns = float(rng.uniform(*cfg.imu_jitter_std_ms)) * 1e6
        self.p_drop = float(rng.uniform(*cfg.imu_dropout_prob))
        self.p_burst = float(rng.uniform(*cfg.imu_burst_prob))
        self.half_rate = bool(rng.random() < cfg.imu_half_rate_prob)
        self.half_phase = int(rng.integers(0, 2))
        self.p_scan = float(rng.uniform(*cfg.scan_dropout_prob))
        self._seed_imu = int(rng.integers(0, 2**31))
        self._seed_scan = int(rng.integers(0, 2**31))

    # --------------------------------------------------------------- scans
    def scan_keep(self, n: int) -> np.ndarray:
        """Revolutions that reach the host (first and last always kept; a lost
        one never follows another, a gap is at most two periods)."""
        keep = np.ones(n, dtype=bool)
        if self.identity or self.p_scan <= 0.0 or n < 3:
            return keep
        lost = np.random.default_rng(self._seed_scan).random(n) < self.p_scan
        lost[[0, -1]] = False
        lost[1:] &= ~lost[:-1]
        return ~lost

    # ----------------------------------------------------------------- IMU
    def additive_bias(self, t_ns: np.ndarray) -> np.ndarray:
        """[N, 6] bias added to (gx, gy, gz, ax, ay, az) at the true sample instants."""
        out = np.broadcast_to(self.bias, (len(t_ns), 6)).copy()
        if self.thermal_on and len(t_ns):
            t = (t_ns - t_ns[0]) * 1e-9 + self.t0
            out += self.coeff[None, :] * self.rise * (1.0 - np.exp(-t / self.tau))[:, None]
        return out

    def imu_keep(self, n: int) -> np.ndarray:
        """Samples that reach the host (rate reduction, isolated and burst losses)."""
        keep = np.ones(n, dtype=bool)
        if self.identity:
            return keep
        if self.half_rate:
            keep[:] = False
            keep[self.half_phase::2] = True
        rng = np.random.default_rng(self._seed_imu)
        if self.p_drop > 0.0:
            keep &= rng.random(n) >= self.p_drop
        if self.p_burst > 0.0:
            starts = np.flatnonzero(rng.random(n) < self.p_burst)
            lo, hi = self.cfg.imu_burst_len
            for s, length in zip(starts, rng.integers(lo, hi + 1, size=len(starts))):
                keep[s : s + int(length)] = False
        keep[[0, -1]] = True
        return keep

    def imu_stamps(self, t_ns: np.ndarray) -> np.ndarray:
        """Reported stamps of the kept samples: offset + jitter, strictly increasing."""
        if self.identity:
            return t_ns
        rng = np.random.default_rng(self._seed_imu + 1)
        jitter = rng.normal(0.0, self.jitter_ns, len(t_ns)) if self.jitter_ns > 0.0 else np.zeros(len(t_ns))
        out = (t_ns + self.offset_ns + np.round(jitter)).astype(np.int64)
        for i in range(1, len(out)):  # drivers never publish out-of-order stamps
            if out[i] <= out[i - 1]:
                out[i] = out[i - 1] + 1000
        return out

    def apply_imu(self, t_ns: np.ndarray, gyro_accel: np.ndarray | None = None, bias_cols: np.ndarray | None = None
                  ) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
        """The stored IMU stream of the run as the host would see it.

        ``t_ns`` [N] true stamps; ``gyro_accel`` [N, 6] (gx gy gz ax ay az)
        measurements; ``bias_cols`` [N, 3] bias labels (gz, ax, ay). Returns
        the kept samples with the perturbed stamps, measurements and labels."""
        if self.identity:
            return t_ns, gyro_accel, bias_cols
        add = self.additive_bias(t_ns)
        keep = self.imu_keep(len(t_ns))
        meas = None if gyro_accel is None else (gyro_accel + add)[keep]
        labels = None if bias_cols is None else (bias_cols + add[:, [2, 3, 4]])[keep]
        return self.imu_stamps(t_ns[keep]), meas, labels

    def realization(self) -> dict[str, Any]:
        return {"variant": self.variant, "bias": self.bias.tolist(), "thermal_coeff": self.coeff.tolist() if self.thermal_on else None,
                "rise_c": self.rise, "tau_s": self.tau, "t0_s": self.t0, "imu_offset_ms": self.offset_ns * 1e-6, "imu_jitter_ms": self.jitter_ns * 1e-6,
                "imu_dropout": self.p_drop, "imu_burst": self.p_burst, "imu_half_rate": self.half_rate, "scan_dropout": self.p_scan}


class Perturber:
    """Factory of the run perturbations of one variant (hook of the loaders)."""

    def __init__(self, cfg: RobustConfig, variant: int) -> None:
        self.cfg, self.variant = cfg, int(variant)

    @property
    def tag(self) -> str:
        return f"_robust{self.cfg.tag()}v{self.variant}"

    def run(self, run_id: int) -> RunPerturbation:
        return RunPerturbation(self.cfg, run_id, self.variant)


# ------------------------------------------------------------------ splits
def concat_splits(parts: list[dict[str, Any]]) -> dict[str, Any]:
    """One split made of several loaded splits (run offsets and indices shifted)."""
    if len(parts) == 1:
        return parts[0]
    n_totals = [int(p["target"].shape[0]) for p in parts]
    out: dict[str, Any] = {}
    for k, v in parts[0].items():
        if k in ("runs", "run_index", "split"):
            continue
        if torch.is_tensor(v):
            out[k] = torch.cat([p[k] for p in parts])
        else:
            out[k] = v
    runs, run_index, off, nrun = [], [], 0, 0
    for p, n in zip(parts, n_totals):
        runs += [{**r, "offset": r["offset"] + off} for r in p["runs"]]
        run_index.append(p["run_index"] + nrun)
        off += n
        nrun += len(p["runs"])
    out["runs"] = runs
    out["run_index"] = torch.cat(run_index)
    out["split"] = parts[0]["split"]
    return out


def load_split_variants(db: Path, cache: Path, split: str, imu_delay_comp: bool, cfg: RobustConfig | None, variants: list[int]) -> dict[str, Any]:
    """A split made of the given variants of every run (0 = unperturbed);
    every run of a perturbed variant is tagged with it in ``runs[i]["variant"]``."""
    from sqlite_streams import load_split  # local: sqlite_streams imports nothing from here

    parts = []
    for v in variants:
        data = load_split(db, cache, split, imu_delay_comp, perturb=None if (cfg is None or v == 0) else Perturber(cfg, v))
        data["runs"] = [{**r, "variant": v} for r in data["runs"]]
        parts.append(data)
    return concat_splits(parts)


def variants_tag(cfg: RobustConfig | None, variants: list[int]) -> str:
    """Cache suffix of a split made of ``variants`` ('' for the plain split)."""
    if cfg is None or list(variants) == [0]:
        return ""
    return f"_robust{cfg.tag()}v{''.join(str(v) for v in variants)}"
