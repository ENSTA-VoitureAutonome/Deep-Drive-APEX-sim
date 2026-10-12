#!/usr/bin/env python3
"""Lines of the Nano IMU serial stream (``arduino/nano33_iot_accel_stream``)."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Optional


@dataclass(frozen=True)
class ImuLine:
    """One IMU sample from the serial stream, SI units."""

    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    has_gyro: bool
    t_us: Optional[int] = None  # MCU micros() at data-ready (firmware v2)
    seq: Optional[int] = None  # MCU sample counter (firmware v2)


def parse_imu_line(line: str, allow_legacy_3axis: bool = False) -> Optional[ImuLine]:
    """Parse one serial line.

    Firmware v2: ``ax,ay,az,gx,gy,gz,t_us,seq``; v1: ``ax,ay,az,gx,gy,gz``;
    with ``allow_legacy_3axis`` also ``ax,ay,az``. Optional prefixes such as
    ``ACC:`` are accepted; ``INFO:`` / ``ERROR:`` lines are not samples.
    """
    text = line.strip()
    if not text:
        return None
    if ":" in text:
        text = text.split(":", 1)[1].strip()
    parts = [p for p in re.split(r"[,\s;]+", text) if p]
    if len(parts) < 3:
        return None
    try:
        ax, ay, az = (float(v) for v in parts[0:3])
        if len(parts) >= 6:
            gx, gy, gz = (float(v) for v in parts[3:6])
            if not all(math.isfinite(v) for v in (ax, ay, az, gx, gy, gz)):
                return None
            t_us = seq = None
            if len(parts) >= 8:
                t_us, seq = int(parts[6]), int(parts[7])
                if t_us < 0 or seq < 0:
                    return None
            return ImuLine(ax, ay, az, gx, gy, gz, True, t_us, seq)
        if not allow_legacy_3axis or not all(math.isfinite(v) for v in (ax, ay, az)):
            return None
        return ImuLine(ax, ay, az, 0.0, 0.0, 0.0, False)
    except ValueError:
        return None
