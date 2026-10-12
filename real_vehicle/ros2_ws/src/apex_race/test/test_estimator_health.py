"""The health gate must refuse a de-rated sensor and ignore jitter.

Regression for the defect this module exists to fix: the simulation node tested
``"rate" in warning`` against warnings that never contain that substring, so
``estimator_ok`` was permanently true and a half-rate IMU would have fed the
plant identifier unnoticed.
"""

from __future__ import annotations

import pytest

from apex_race.nodes.estimator_health import estimator_ok

NOMINAL = (100.0, 10.0)  # WT61PC at 100 Hz, A2M8 Express at 10 Hz


def healthy(**over):
    """A report like the one the car produced at rest on 2026-10-09."""
    return {"imu_rate_hz": 100.0, "imu_period_jitter_ms": 0.3, "lidar_rate_hz": 9.974,
            "lidar_period_jitter_ms": 45.9, "valid_bins_fraction": 0.686, **over}


# --------------------------------------------------------------------- accepts
def test_the_measured_report_of_the_real_car_is_usable():
    ok, reason = estimator_ok(healthy(), *NOMINAL)
    assert ok, reason


def test_jitter_alone_never_disarms():
    """The three strings `_Health.report()` can emit, as data, not as words."""
    for jitter in (5.1, 13.4, 45.9, 200.0):
        ok, reason = estimator_ok(healthy(lidar_period_jitter_ms=jitter), *NOMINAL)
        assert ok, f"jitter {jitter} ms disarmed the identifier: {reason}"


def test_an_imu_read_against_the_old_calibration_still_passes():
    """100 Hz hardware against the 104 Hz LSM6DS3 profile: -3.8 %, inside 5 %."""
    ok, reason = estimator_ok(healthy(imu_rate_hz=100.0), 104.0, 10.0)
    assert ok, reason


def test_the_lidar_running_at_its_measured_rate_passes():
    for rate in (9.9, 9.974, 10.0, 10.1):
        ok, reason = estimator_ok(healthy(lidar_rate_hz=rate), *NOMINAL)
        assert ok, reason


# --------------------------------------------------------------------- refuses
@pytest.mark.parametrize(("over", "expect"), [
    ({"imu_rate_hz": 55.6}, "IMU"),          # the defective driver's rate, stamped on reception
    ({"imu_rate_hz": 52.0}, "IMU"),          # half rate: drift 0.90 -> 1.17 %
    ({"lidar_rate_hz": 13.03}, "LiDAR"),     # the legacy 2 kHz mode: the ICP finds no correspondences
    ({"lidar_rate_hz": 5.0}, "LiDAR"),
    ({"valid_bins_fraction": 0.30}, "bins"),  # a dark quadrant
])
def test_a_de_rated_sensor_refuses_identification(over, expect):
    ok, reason = estimator_ok(healthy(**over), *NOMINAL)
    assert not ok
    assert expect in reason


def test_a_report_that_has_not_measured_anything_yet_is_not_usable():
    """`rate()` returns 0.0 until three samples: refusing the first revolutions
    of a run is correct, and the driver does not need this flag to drive."""
    for health in (None, {}, {"imu_rate_hz": 0.0, "lidar_rate_hz": 0.0, "valid_bins_fraction": 0.0}):
        ok, _ = estimator_ok(health, *NOMINAL)
        assert not ok


def test_the_warning_strings_of_the_estimator_contain_no_usable_signal():
    """Why the gate moved to the numbers: the exact strings, verbatim."""
    warnings = [f"IMU at {60.0:.1f} Hz, calibration {104.0:.1f} Hz",
                f"LiDAR at {13.03:.2f} Hz, calibration {10.00:.2f} Hz",
                f"LiDAR period jitter {45.9:.1f} ms"]
    assert not any("rate" in w for w in warnings), "the old string gate would have worked after all"
