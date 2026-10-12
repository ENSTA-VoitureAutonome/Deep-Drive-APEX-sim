"""Pure-pursuit tracker and speed pass, vendored from the simulation.

Copied verbatim from ``simulation/tools/pose_dataset/controller.py``:
``_forward_backward`` (lines 76-86) and ``PurePursuit`` (lines 155-200), the
two objects ``core/race_driver.py`` and ``core/race_planner.py`` import. The
rest of that module is dataset generation (motion profiles, seeded speed plans,
driving paths over a known track) and must not reach the car.
``test_core_parity.py`` compares each object here against its original.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from .tracks import Path2D

def _forward_backward(v_cap: np.ndarray, ds: float, a_acc: float, a_dec: float, v_start: float = 0.0, v_end: float = 0.0, zero_idx: list[int] | None = None) -> np.ndarray:
    v = v_cap.copy()
    for i in zero_idx or []:
        v[i] = 0.0
    v[0] = min(v[0], v_start)
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_acc * ds))
    v[-1] = min(v[-1], v_end)
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_dec * ds))
    return v


class PurePursuit:
    def __init__(self, path: Path2D, wheelbase: float, cfg: dict[str, Any], gain_scale: float = 1.0) -> None:
        self.path = path
        self.wheelbase = wheelbase
        self.l_min = float(cfg["lookahead_min_m"])
        self.l_time = float(cfg["lookahead_time_s"]) * gain_scale
        self.l_base = float(cfg["lookahead_base_m"]) * gain_scale
        self.idx = 0
        self.s_total = 0.0  # unwrapped progress along the path [m]
        self._last_s = 0.0
        n = len(path.s)
        self._window = max(20, int(2.0 / max(path.length / n, 1e-3)))

    def reset(self, x: float, y: float) -> None:
        """Restart the progress at the path point nearest to (x, y) (global search)."""
        self.idx = int(np.argmin(np.hypot(self.path.xy[:, 0] - x, self.path.xy[:, 1] - y)))
        self._last_s = float(self.path.s[self.idx])
        self.s_total = 0.0

    def locate(self, x: float, y: float) -> tuple[float, float]:
        """Update progress with the reference point; return (s_total, lateral error)."""
        n = len(self.path.s)
        ids = (self.idx + np.arange(-self._window // 4, self._window)) % n
        d = np.hypot(self.path.xy[ids, 0] - x, self.path.xy[ids, 1] - y)
        k = int(np.argmin(d))
        self.idx = int(ids[k])
        s_now = float(self.path.s[self.idx])
        ds = s_now - self._last_s
        if ds < -0.5 * self.path.length:
            ds += self.path.length
        elif ds > 0.5 * self.path.length:
            ds -= self.path.length
        self.s_total += ds
        self._last_s = s_now
        h = self.path.heading[self.idx]
        px, py = self.path.xy[self.idx]
        lateral = -math.sin(h) * (x - px) + math.cos(h) * (y - py)
        return self.s_total, lateral

    def steering(self, x: float, y: float, yaw: float, speed: float) -> float:
        """Steering angle [rad] of the equivalent centre wheel."""
        ld = max(self.l_min, self.l_base + self.l_time * max(speed, 0.0))
        tx, ty = self.path.interp(self.path.s[self.idx] + ld)
        alpha = math.atan2(ty - y, tx - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        return math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)
