#!/usr/bin/env python3
"""Pack a run's recorded sensors into one file the real car can replay.

The car's static tests can exercise the sensors (the LiDAR turns, the IMU
reports) but not the driving: ``RaceDriverCore.on_scan`` returns immediately in
``wait``, and reaching ``planning`` needs a closed lap. The only way to drive
the whole state machine with the car switched off is to feed the graph a
recording of a complete run, in wall-clock time, on the real topics.

This writes that recording. Input is any real2sim run directory (its
``imu_raw.csv`` and ``lidar_scans.csv``, the same two files
``ros2_ws/scripts/capture/record_real2sim_sensors.py`` writes on the car);
output is a compressed ``.npz`` that ``real_vehicle/ros2_ws/scripts/replay_sensors.py``
publishes.

    python3 simulation/tools/analysis/export_replay_fixture.py \\
        simulation/data/real2sim/final_s13 \\
        real_vehicle/data/replay/final_s13_sensors.npz

What it deliberately does not carry: the truth tracks, the plan, the map, the
run configuration. The replayed car has to find the track again from the two
sensor streams, exactly as it would on the floor.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def read_imu(path: Path) -> tuple[np.ndarray, np.ndarray]:
    t, values = [], []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            t.append(int(row["stamp_sec"]) + 1e-9 * int(row["stamp_nanosec"]))
            values.append([float(row[k]) for k in ("ax_mps2", "ay_mps2", "az_mps2", "gx_rps", "gy_rps", "gz_rps")])
    return np.asarray(t, dtype=np.float64), np.asarray(values, dtype=np.float32)


def read_scans(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t, scan_time, ranges = [], [], []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        n_bins = sum(1 for c in header if c.startswith("r"))
        for row in reader:
            t.append(int(row[0]) * 1e-9)
            scan_time.append(float(row[1]))
            # A bin with no return is written empty by the simulation recorder
            # and as `inf` by the car's; REP-117 says +inf, and that is what the
            # metric embedding of the network treats as missing.
            ranges.append([math.inf if v == "" else float(v) for v in row[2:2 + n_bins]])
    return (np.asarray(t, dtype=np.float64), np.asarray(scan_time, dtype=np.float32),
            np.asarray(ranges, dtype=np.float32))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path, help="a real2sim run directory, or a capture from the car")
    ap.add_argument("out", type=Path, help="the .npz to write")
    ap.add_argument("--start-s", type=float, default=0.0, help="drop this many seconds from the beginning")
    ap.add_argument("--duration-s", type=float, default=0.0, help="keep only this many seconds (0: all)")
    args = ap.parse_args()

    imu_t, imu = read_imu(args.run_dir / "imu_raw.csv")
    scan_t, scan_time, ranges = read_scans(args.run_dir / "lidar_scans.csv")
    if not len(imu_t) or not len(scan_t):
        raise SystemExit(f"{args.run_dir}: need both imu_raw.csv and lidar_scans.csv with data "
                         f"(got {len(imu_t)} IMU samples and {len(scan_t)} scans)")

    # One clock for both streams, starting at the first sample of either.
    t0 = min(float(imu_t[0]), float(scan_t[0])) + args.start_s
    imu_keep = imu_t >= t0
    scan_keep = scan_t >= t0
    if args.duration_s > 0.0:
        imu_keep &= imu_t <= t0 + args.duration_s
        scan_keep &= scan_t <= t0 + args.duration_s
    imu_t, imu = imu_t[imu_keep] - t0, imu[imu_keep]
    scan_t, scan_time, ranges = scan_t[scan_keep] - t0, scan_time[scan_keep], ranges[scan_keep]

    meta = {"source": str(args.run_dir), "beams": int(ranges.shape[1]),
            "duration_s": round(float(max(imu_t[-1], scan_t[-1])), 3),
            "imu_rate_hz": round(float(len(imu_t) / max(imu_t[-1], 1e-9)), 2),
            "lidar_rate_hz": round(float(len(scan_t) / max(scan_t[-1], 1e-9)), 3)}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, imu_t=imu_t, imu=imu, scan_t=scan_t, scan_time=scan_time, ranges=ranges,
                        meta=json.dumps(meta))
    size_mb = args.out.stat().st_size / 1e6
    print(f"{args.out} ({size_mb:.2f} MB): {len(imu_t)} IMU samples at {meta['imu_rate_hz']} Hz, "
          f"{len(scan_t)} scans at {meta['lidar_rate_hz']} Hz, {meta['duration_s']} s, {meta['beams']} bins")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
