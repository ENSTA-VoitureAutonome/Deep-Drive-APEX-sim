"""Line normals of the ICP on dense and sparse scans (icp_odometry.line_normals)."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from icp_odometry import line_normals  # noqa: E402

INC = torch.tensor([math.radians(1.0)])


def _wall_scan(keep_every: int = 1, x_wall: float = 2.0):
    """Beams of 1-degree bins hitting the wall x = x_wall (|angle| < 60 deg); every ``keep_every``-th bin valid."""
    th = np.radians(np.arange(360) - 180.0 + 0.5)
    r = np.where(np.abs(th) < math.radians(60.0), x_wall / np.cos(th), 0.0).astype(np.float32)
    valid = r > 0
    valid &= (np.arange(360) % keep_every) == 0
    r = np.where(valid, r, 0.0).astype(np.float32)
    pts = torch.tensor(np.stack((r * np.cos(th), r * np.sin(th)), -1), dtype=torch.float32)[None]
    return pts, torch.tensor(valid)[None], torch.tensor(r)[None]


def test_dense_scan_keeps_the_historical_normals():
    pts, v, r = _wall_scan()
    n_old, ok_old = line_normals(pts, v, r, INC, 0.05)
    n_new, ok_new = line_normals(pts, v, r, INC, 0.05, window=3, min_points=4)
    assert ok_old.sum() > 100
    both = ok_old & ok_new
    assert both.sum() >= ok_old.sum() - 2
    # Same line direction (normals may differ in sign).
    assert torch.allclose((n_old[both] * n_new[both]).sum(-1).abs(), torch.ones(int(both.sum())), atol=1e-4)
    assert torch.allclose(n_old[ok_old][:, 0].abs(), torch.ones(int(ok_old.sum())), atol=1e-3)  # wall x = const: normal along x


def test_sparse_scan_gets_normals_only_with_the_patch_rule():
    pts, v, r = _wall_scan(keep_every=2)  # a 2 kHz-like scan: every second bin empty
    _, ok_old = line_normals(pts, v, r, INC, 0.05)
    n, ok = line_normals(pts, v, r, INC, 0.05, window=3, min_points=3)
    assert ok_old.sum() == 0
    assert ok.sum() > 50
    assert torch.allclose(n[ok][:, 0].abs(), torch.ones(int(ok.sum())), atol=1e-3)


def test_patch_does_not_bridge_a_depth_jump():
    th = np.radians(np.arange(360) - 180.0 + 0.5)
    r = np.zeros(360, np.float32)
    near = np.abs(th) < math.radians(10.0)
    far = (th >= math.radians(10.0)) & (th < math.radians(40.0))
    r[near] = 1.0 / np.cos(th[near])  # wall x = 1 m
    r[far] = 3.0 / np.cos(th[far])  # wall x = 3 m behind it
    pts = torch.tensor(np.stack((r * np.cos(th), r * np.sin(th)), -1), dtype=torch.float32)[None]
    v = torch.tensor(r > 0)[None]
    n, ok = line_normals(pts, v, torch.tensor(r)[None], INC, 0.05, window=3, min_points=3)
    edge = int(np.flatnonzero(near)[-1])
    # The last near beam keeps a normal along x (its far neighbours are not on its surface).
    assert ok[0, edge] and abs(float(n[0, edge, 0])) > 0.999
