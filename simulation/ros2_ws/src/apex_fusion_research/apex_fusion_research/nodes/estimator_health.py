"""Is the estimator a channel to identify a plant from? (ROS-independent).

The driver passes one boolean to ``RaceDriverCore.on_odometry``: whether the
estimate may be used to identify the car's plant. An estimator whose sensors are
off *rate* is not such a channel -- the network integrates the gyro over each
revolution and expects the interval it was trained with -- while jitter around
the right rate only deserves a larger sigma, which the estimate already carries.

That is what the simulation node meant to express, and it did it by searching
the word ``rate`` in the estimator's warnings. None of the three warnings
``live_odometry._Health.report()`` can emit contains that substring:

    "IMU at 60.0 Hz, calibration 104.0 Hz"
    "LiDAR at 13.03 Hz, calibration 10.00 Hz"
    "LiDAR period jitter 45.9 ms"

so the test was always true and a de-rated sensor would have fed the identifier
unnoticed. The same report carries the measured rates as numbers, so the gate
belongs on the numbers. This module is the gate, kept pure so the three real
strings and the three real failures can be unit-tested without ROS.
"""

from __future__ import annotations

from typing import Any

# Default tolerance on the sensor rates. The estimator warns above 10 %; for
# *identification* 5 % is the right order: it accepts the measured 9.974 Hz of
# the A2M8 (-0.26 %) and a 100 Hz IMU read against a 104 Hz calibration
# (-3.8 %, inside the +-8 ms stamp robustness of the training), and refuses a
# 13 Hz legacy LiDAR mode or a half-rate IMU.
RATE_TOL = 0.05

# The blind sector behind the car (measured 142..-155 deg, 63 deg) is 17.5 % of
# the revolution, so a healthy scan fills at most ~0.82 of the 360 bins and the
# car measured 0.69 at rest. Below this a quadrant has gone dark.
MIN_VALID_BINS = 0.45


def estimator_ok(
    health: dict[str, Any] | None,
    imu_rate_hz: float,
    lidar_rate_hz: float,
    rate_tol: float = RATE_TOL,
    min_valid_bins: float = MIN_VALID_BINS,
) -> tuple[bool, str]:
    """``(usable_for_identification, reason)`` from an estimator health report.

    ``reason`` is empty when usable. A missing or still-empty report is *not*
    usable: the rates are reported as 0.0 until three samples have arrived, and
    refusing to identify from the first revolutions of a run is correct.
    """
    if not health:
        return False, "no health report"
    checks = (("IMU", "imu_rate_hz", imu_rate_hz), ("LiDAR", "lidar_rate_hz", lidar_rate_hz))
    for label, key, nominal in checks:
        measured = float(health.get(key, 0.0) or 0.0)
        if nominal <= 0.0:
            continue
        if measured <= 0.0:
            return False, f"{label} rate not measured yet"
        error = measured / nominal - 1.0
        if abs(error) > rate_tol:
            return False, f"{label} at {measured:.2f} Hz, calibration {nominal:.2f} Hz ({100.0 * error:+.1f} %)"
    valid = float(health.get("valid_bins_fraction", 0.0) or 0.0)
    if valid < min_valid_bins:
        return False, f"only {100.0 * valid:.0f} % of the scan bins carry a return"
    return True, ""
