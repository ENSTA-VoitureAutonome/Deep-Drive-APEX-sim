#!/usr/bin/env python3
"""Map a sensor's own clock to the host (ROS) clock.

The learned odometry needs every sample stamped at the instant the sensor
measured it (``real_vehicle/docs/sensor_timing_requirements.md``). A stamp
taken when the host reads the serial port adds the transfer time plus a USB
and scheduling latency that jitters by milliseconds.

Both sensors carry a clock of their own: the Nano sends ``micros()`` with
every IMU sample, and the RPLIDAR measures at a fixed sample period, so the
index of a sample is a time. For every packet

    host_arrival = device_time + offset + latency,   latency >= 0.

The packets that arrived with the smallest latency lie on the lower envelope
of ``host_arrival - device_time``. A line fitted to that envelope over a
sliding window maps any device time to the host clock without the reception
jitter; its slope absorbs the drift between the two oscillators. The
constant minimum latency (a fraction of a millisecond on USB) stays in the
offset.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np


class CounterUnwrapper:
    """Unwrap a free-running unsigned counter, e.g. the 32-bit ``micros()``.

    A value far below the previous one is a wrap (every 71.6 min for
    ``micros()``). A small step back is not: it is returned as is, so the
    caller sees time go backwards (a device reset) and can resynchronise.
    """

    def __init__(self, bits: int = 32) -> None:
        self._modulus = 1 << bits
        self._last: int | None = None
        self._base = 0

    def reset(self) -> None:
        self._last = None
        self._base = 0

    def __call__(self, value: int) -> int:
        value = int(value) % self._modulus
        if self._last is not None and value < self._last and (self._last - value) > self._modulus // 2:
            self._base += self._modulus
        self._last = value
        return self._base + value


@dataclass(frozen=True)
class ClockMapperStats:
    pairs: int
    drift_ppm: float
    latency_p50_ms: float  # reception latency above the envelope
    latency_p95_ms: float
    resets: int
    rejected: int


class DeviceClockMapper:
    """Map device times (s) to host times (ns) through the latency envelope.

    Parameters
    ----------
    window_s: the envelope is fitted over this much device time.
    chunks: the window is split in this many chunks; the fastest packet of
        each chunk is one point of the envelope.
    refit_every_s: device time between two fits.
    max_drift: bound on the fitted slope (relative rate error between the
        clocks, 1e-3 = 1000 ppm).
    early_outlier_s: once the envelope has seen ``converge_s`` of data, a
        packet that arrives earlier than it by more than this is impossible
        for a correct device time (a miscounted packet); it is rejected.
    reset_jump_s: a packet later than the envelope by more than this is a
        stall, unless ``reset_after`` of them come in a row (a host clock
        step or a device that restarted without going backwards): then the
        mapper starts again.
    """

    def __init__(
        self,
        *,
        window_s: float = 10.0,
        chunks: int = 10,
        refit_every_s: float = 0.5,
        max_drift: float = 5.0e-3,
        early_outlier_s: float = 2.0e-3,
        reset_jump_s: float = 0.25,
        reset_after: int = 20,
        converge_s: float = 2.0,
    ) -> None:
        self._window_s = float(window_s)
        self._chunks = max(3, int(chunks))
        self._refit_every_s = float(refit_every_s)
        self._max_drift = float(max_drift)
        self._early_outlier_s = float(early_outlier_s)
        self._reset_jump_s = float(reset_jump_s)
        self._reset_after = max(1, int(reset_after))
        self._converge_s = float(converge_s)
        self.resets = 0
        self.rejected = 0
        self._clear()

    def _clear(self) -> None:
        self._pairs: deque[tuple[float, float]] = deque()  # (x device s since ref, host - device offset s)
        self._ref_host_ns: int | None = None
        self._ref_dev_s = 0.0
        self._a = 0.0  # envelope: offset(x) = a + b * x
        self._b = 0.0
        self._fitted_at: float | None = None
        self._fits = 0
        self._last_x: float | None = None
        self._early_run = 0
        self._late_run = 0

    def reset(self) -> None:
        """Forget the mapping (device restarted, reconnection)."""
        if self._ref_host_ns is not None:
            self.resets += 1
        self._clear()

    @property
    def ready(self) -> bool:
        return self._ref_host_ns is not None

    @property
    def drift(self) -> float:
        return self._b

    def add(self, device_s: float, host_ns: int) -> int:
        """Record one packet and return the host time (ns) of ``device_s``.

        The result never exceeds ``host_ns``: a sample is not measured after
        it was received.
        """
        if self._ref_host_ns is None or (self._last_x is not None and device_s - self._ref_dev_s < self._last_x - 1e-6):
            if self._ref_host_ns is not None:
                self.reset()  # device time went backwards: the device restarted
            self._ref_host_ns = int(host_ns)
            self._ref_dev_s = float(device_s)
        x = float(device_s) - self._ref_dev_s
        o = (int(host_ns) - self._ref_host_ns) * 1e-9 - x
        if self._fits > 0 and x >= self._converge_s:
            residual = o - (self._a + self._b * x)
            if residual < -self._early_outlier_s:
                self._early_run += 1
                self._late_run = 0
                self.rejected += 1
                if self._early_run >= self._reset_after:
                    self.reset()
                    return self.add(device_s, host_ns)
                return min(self._to_host_ns(x), int(host_ns))
            if residual > self._reset_jump_s:
                self._late_run += 1
                if self._late_run >= self._reset_after:
                    self.reset()
                    return self.add(device_s, host_ns)
            else:
                self._late_run = 0
            self._early_run = 0
        self._pairs.append((x, o))
        while self._pairs and x - self._pairs[0][0] > self._window_s:
            self._pairs.popleft()
        if o < self._a + self._b * x:
            self._a = o - self._b * x  # a faster packet: the envelope is at least this low
        if self._fitted_at is None or x - self._fitted_at >= self._refit_every_s:
            self._refit(x)
        self._last_x = x
        return min(self._to_host_ns(x), int(host_ns))

    def map(self, device_s: float) -> int:
        """Host time (ns) of a device time, with the current envelope."""
        if self._ref_host_ns is None:
            raise RuntimeError("DeviceClockMapper.map() before the first packet")
        return self._to_host_ns(float(device_s) - self._ref_dev_s)

    def stats(self) -> ClockMapperStats:
        if not self._pairs:
            return ClockMapperStats(0, 0.0, float("nan"), float("nan"), self.resets, self.rejected)
        xs = np.fromiter((p[0] for p in self._pairs), dtype=float, count=len(self._pairs))
        os_ = np.fromiter((p[1] for p in self._pairs), dtype=float, count=len(self._pairs))
        lat = (os_ - (self._a + self._b * xs)) * 1e3
        return ClockMapperStats(
            pairs=len(xs),
            drift_ppm=self._b * 1e6,
            latency_p50_ms=float(np.percentile(lat, 50)),
            latency_p95_ms=float(np.percentile(lat, 95)),
            resets=self.resets,
            rejected=self.rejected,
        )

    def _to_host_ns(self, x: float) -> int:
        return self._ref_host_ns + int(round((x + self._a + self._b * x) * 1e9))

    def _refit(self, x_now: float) -> None:
        self._fitted_at = x_now
        n = len(self._pairs)
        xs = np.fromiter((p[0] for p in self._pairs), dtype=float, count=n)
        os_ = np.fromiter((p[1] for p in self._pairs), dtype=float, count=n)
        b = self._b
        if n >= 3 * self._chunks and xs[-1] - xs[0] >= 2.0 * self._refit_every_s:
            edges = np.linspace(xs[0], xs[-1], self._chunks + 1)
            idx = np.clip(np.searchsorted(edges, xs, side="right") - 1, 0, self._chunks - 1)
            cx, co = [], []
            for k in range(self._chunks):
                sel = np.flatnonzero(idx == k)
                if sel.size:
                    j = sel[np.argmin(os_[sel])]
                    cx.append(xs[j])
                    co.append(os_[j])
            if len(cx) >= 3:
                b = float(np.clip(np.polyfit(cx, co, 1)[0], -self._max_drift, self._max_drift))
        # The lowest line of slope b under every packet of the window.
        self._b = b
        self._a = float(np.min(os_ - b * xs))
        self._fits += 1
