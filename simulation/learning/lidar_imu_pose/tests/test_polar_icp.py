"""The polar ICP (``polar_icp.py``) is the torch classical odometry, compiled:
its nearest-neighbour search is exact within the gate, and a whole stream of
intervals matches ``IcpStream`` run in float64 (the float32 torch version
differs only by its cdist rounding). Synthetic scans, no dataset needed."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytest.importorskip("numba")
import polar_icp as pi  # noqa: E402
from icp_odometry import ICP_VARIANTS, IcpStream  # noqa: E402

N = 360
LIDAR_XY = (0.18, 0.0)
# a room with two pillars (x0, y0, x1, y1): structure in both directions
BOXES = ((-4.0, -2.5, 14.0, 2.5), (3.0, 0.8, 3.6, 1.4), (7.0, -1.6, 7.5, -1.0))


def _ray(ox, oy, dx, dy) -> float:
    """First hit of a ray on the walls of the room (inside) and the pillars (outside)."""
    best = math.inf
    for k, (x0, y0, x1, y1) in enumerate(BOXES):
        for axis, lo, hi, o_a, d_a, o_b, d_b, b0, b1 in ((0, x0, x1, ox, dx, oy, dy, y0, y1), (1, y0, y1, oy, dy, ox, dx, x0, x1)):
            if abs(d_a) < 1e-12:
                continue
            for wall in (lo, hi):
                t = (wall - o_a) / d_a
                if t <= 1e-6:
                    continue
                b = o_b + t * d_b
                if b0 <= b <= b1:
                    best = min(best, t)
    return best


def _scan(x, y, yaw, speed, rate, dt):
    """A rolling scan: beam i fired at i * dt / N while the car moves."""
    r = np.empty(N, dtype=np.float32)
    for i in range(N):
        tau = i * dt / N
        yaw_i = yaw + rate * tau
        xi = x + speed * tau * math.cos(yaw + 0.5 * rate * tau)
        yi = y + speed * tau * math.sin(yaw + 0.5 * rate * tau)
        lx = xi + LIDAR_XY[0] * math.cos(yaw_i)
        ly = yi + LIDAR_XY[0] * math.sin(yaw_i)
        th = yaw_i - math.pi + i * 2 * math.pi / N
        r[i] = _ray(lx, ly, math.cos(th), math.sin(th))
    valid = (r > 0.15) & (r < 12.0) & np.isfinite(r)
    return np.where(valid, r, 0.0).astype(np.float32), valid


def _stream(steps=14, dt=0.1, speed=2.0, rate=0.3):
    """Consecutive intervals as ``StreamData.gather`` gives them."""
    poses = [(0.0, 0.0, 0.0)]
    for _ in range(steps + 1):
        x, y, yaw = poses[-1]
        poses.append((x + speed * dt * math.cos(yaw + 0.5 * rate * dt), y + speed * dt * math.sin(yaw + 0.5 * rate * dt), yaw + rate * dt))
    scans = [_scan(*p, speed, rate, dt) for p in poses]
    prof = np.linspace(0.0, rate * dt, 17)
    out = []
    for k in range(1, steps + 1):
        f = lambda v: torch.tensor([v], dtype=torch.float32)  # noqa: E731
        out.append({"prev_ranges": torch.from_numpy(scans[k - 1][0])[None], "prev_valid": torch.from_numpy(scans[k - 1][1])[None],
                    "cur_ranges": torch.from_numpy(scans[k][0])[None], "cur_valid": torch.from_numpy(scans[k][1])[None],
                    "w_prev": f(rate), "w_cur": f(rate), "dt": f(dt), "dt_next": f(dt), "gyro_integral": f(rate * dt),
                    "accel": torch.tensor([[0.0, speed * rate]]), "accel_std": f(0.5),
                    "prof_prev": torch.tensor(prof, dtype=torch.float32)[None], "prof_cur": torch.tensor(prof, dtype=torch.float32)[None]})
    return out


def _geo(dtype=torch.float32):
    return (torch.tensor([0.1 / N], dtype=dtype), torch.tensor([LIDAR_XY], dtype=dtype), torch.tensor([-math.pi], dtype=dtype),
            torch.tensor([2 * math.pi / N], dtype=dtype), None)


def test_nearest_is_exact_within_the_gate():
    rng = np.random.default_rng(0)
    pts = np.concatenate((rng.uniform(-6, 6, (700, 2)), np.full((20, 2), pi.SENTINEL)))
    ox, oy = 0.18, 0.0
    order, start = np.empty(len(pts), np.int64), np.empty(pi.NBINS + 1, np.int64)
    pi._build_index(pts, ox, oy, order, start)
    real = np.abs(pts).max(1) < 1e3
    for gate in (0.05, 0.4, 3.0):
        for q in np.concatenate((rng.uniform(-6, 6, (400, 2)), [[ox + 0.05, oy], [-5.9, 0.01], [-5.9, -0.01]])):
            d = np.hypot(*(pts[real] - q).T)
            j_ref = np.flatnonzero(real)[np.argmin(d)] if d.min() < gate else -1
            j = pi._nearest(q[0], q[1], pts, order, start, ox, oy, gate)
            assert j == j_ref
    empty = np.full((5, 2), pi.SENTINEL)
    pi._build_index(empty, ox, oy, order[:5], start)
    assert pi._nearest(1.0, 1.0, empty, order[:5], start, ox, oy, 1.0) == -1


@pytest.mark.parametrize("variant", ["submap_v4", "submap_v3", "scan"])
def test_stream_matches_torch_float64(variant):
    cfg = ICP_VARIANTS[variant]
    sig = torch.tensor([[0.003, 0.005]])
    old = torch.get_default_dtype()
    torch.set_default_dtype(torch.float64)
    try:
        ref = IcpStream(cfg, _geo(torch.float64), sig.double(), N, torch.device("cpu"))
        pol = pi.PolarIcpStream(replace(cfg, association="polar"), _geo(), sig, N, torch.device("cpu"))
        act = torch.ones(1, dtype=torch.bool)
        for b in _stream():
            with torch.no_grad():
                rt = ref.step({k: (v.double() if v.is_floating_point() else v) for k, v in b.items()}, act)
            rp = pol.step(b, act)
            for key in ("pred", "sigma", "gyro_bias", "yaw_icp", "pairs", "map_used"):
                assert torch.allclose(rt[key].double(), rp[key].double(), atol=1e-6, rtol=1e-5), (key, rt[key], rp[key])
            assert torch.equal(rt["prev_points_valid"], rp["prev_points_valid"])
    finally:
        torch.set_default_dtype(old)


def test_polar_variants_are_registered():
    for name, cfg in list(ICP_VARIANTS.items()):
        if not name.endswith("_polar"):
            assert ICP_VARIANTS[f"{name}_polar"] == replace(cfg, association="polar")
