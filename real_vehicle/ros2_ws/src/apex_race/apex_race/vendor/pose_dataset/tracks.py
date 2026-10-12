"""Closed arc-length polyline, vendored from the simulation.

Copied verbatim from ``simulation/tools/pose_dataset/tracks.py`` (lines 53-104):
``Path2D`` and ``path_from_points``, the two objects ``core/race_planner.py``
imports. Nothing else of that module may reach the car -- ``Track``,
``load_track`` and the Gazebo geometry are track knowledge, and the knowledge
firewall test forbids them. ``test_core_parity.py`` compares each object here
against its original, so this file is never edited by hand.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

@dataclass
class Path2D:
    """Closed, arc-length parametrised polyline."""

    xy: np.ndarray  # (N, 2)
    s: np.ndarray  # (N,)
    heading: np.ndarray  # (N,)
    curvature: np.ndarray  # (N,)
    length: float

    @property
    def normal(self) -> np.ndarray:
        return np.column_stack((-np.sin(self.heading), np.cos(self.heading)))

    def interp(self, s_query: np.ndarray | float) -> np.ndarray:
        sq = np.mod(np.asarray(s_query, dtype=float), self.length)
        s_ext = np.append(self.s, self.length)
        x_ext = np.append(self.xy[:, 0], self.xy[0, 0])
        y_ext = np.append(self.xy[:, 1], self.xy[0, 1])
        return np.stack((np.interp(sq, s_ext, x_ext), np.interp(sq, s_ext, y_ext)), axis=-1)

    def interp_scalar(self, values: np.ndarray, s_query: np.ndarray | float) -> np.ndarray:
        sq = np.mod(np.asarray(s_query, dtype=float), self.length)
        return np.interp(sq, np.append(self.s, self.length), np.append(values, values[0]))

    def reversed(self) -> "Path2D":
        xy = self.xy[::-1].copy()
        return path_from_points(xy)

    def rotated_start(self, s0: float) -> "Path2D":
        """Same closed path with the arc-length origin moved to ``s0``."""
        idx = int(np.searchsorted(self.s, np.mod(s0, self.length)))
        xy = np.roll(self.xy, -idx, axis=0)
        return path_from_points(xy)


def path_from_points(xy: np.ndarray) -> Path2D:
    """Arc length, heading and curvature (finite differences) of a closed polyline."""
    xy = np.asarray(xy, dtype=float)
    nxt = np.roll(xy, -1, axis=0)
    seg = np.linalg.norm(nxt - xy, axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg[:-1])))
    length = float(np.sum(seg))
    d = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    heading = np.arctan2(d[:, 1], d[:, 0])
    dh = np.angle(np.exp(1j * (np.roll(heading, -1) - np.roll(heading, 1))))
    ds = np.roll(s, -1) - np.roll(s, 1)
    ds = np.where(ds <= 0, ds + length, ds)
    curvature = dh / np.maximum(ds, 1e-9)
    return Path2D(xy, s, heading, curvature, length)


