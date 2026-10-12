#!/usr/bin/env python3
"""Check the car's LiDAR against what the learned odometry was trained for.

Reads ``/lidar/scan_raw`` (``rplidar_publisher_node``, backend
``rplidar_express``) and grades it against
``real_vehicle/docs/sensor_timing_requirements.md`` and the simulation
profile ``APEX_real``: 10 Hz +-1 %, complete revolutions stamped at their
first sample, 360 one-degree bins from -180 deg counter-clockwise, blind
sector behind the car (+145..-155 deg).

Modes:

* ``summary`` (default): collect ``--duration`` s with the car at rest in a
  room (walls within 12 m all around), then print PASS/FAIL per
  requirement and the measurements the simulation assumed (INFO).
* ``watch``: every second, the nearest object in the laser frame (x
  forward, angles counter-clockwise). An object ahead must read ~0 deg, on
  the left ~+90 deg. With ``--expect DEG`` (where the object really is) it
  prints the ``heading_offset_deg`` that would put it there.

Inside the LiDAR container (``tools/lidar/apex_lidar_only.sh check ...``):

    python3 /work/ros2_ws/scripts/lidar_bringup_check.py --duration 60
    python3 /work/ros2_ws/scripts/lidar_bringup_check.py watch --expect 0
"""

from __future__ import annotations

import argparse
import json
import math
import re
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

RATE_HZ = 10.0
RATE_TOL = 0.01
BEAMS = 360
BLIND_DEG = (145.0, 205.0)  # APEX_real occluded_sectors_deg, angles in [0, 360)
RANGE_MIN_M = 0.15
FILLABLE_BINS = BEAMS * (1.0 - (BLIND_DEG[1] - BLIND_DEG[0]) / 360.0)
MISSING_ASSUMED = 0.12  # APEX_real dropout_prob_base
DROPPED_REVS_MAX = 0.02  # the odometry is trained with 1 % lost revolutions
PARAMS_FILE = "/work/ros2_ws/src/apex_telemetry/config/apex_params.yaml"

BIN_DEG = -180.0 + np.arange(BEAMS) * 360.0 / BEAMS
BLIND = (np.mod(BIN_DEG, 360.0) >= BLIND_DEG[0]) & (np.mod(BIN_DEG, 360.0) <= BLIND_DEG[1])


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def current_heading_offset(path: str = PARAMS_FILE, default: int = 91) -> int:
    try:
        text = open(path, encoding="utf-8").read()
    except OSError:
        return default
    block = text.split("apex_rplidar_publisher:", 1)[-1]
    m = re.search(r"^\s*heading_offset_deg:\s*(-?\d+)", block, re.M)
    return int(m.group(1)) if m else default


def nearest_object(ranges: np.ndarray, min_m: float, max_m: float) -> tuple[float, float] | None:
    """(centroid angle deg, range m) of the nearest return cluster outside the blind sector."""
    r = np.where(np.isfinite(ranges) & ~BLIND & (ranges >= min_m) & (ranges <= max_m), ranges, np.inf)
    i = int(np.argmin(r))
    if not np.isfinite(r[i]):
        return None
    members = [i]
    for step in (1, -1):
        j = (i + step) % BEAMS
        while j != i and np.isfinite(r[j]) and r[j] <= r[i] + 0.10:
            members.append(j)
            j = (j + step) % BEAMS
    ang = np.radians(BIN_DEG[members])
    return math.degrees(math.atan2(np.sin(ang).mean(), np.cos(ang).mean())), float(r[i])


def largest_run(mask: np.ndarray) -> tuple[int, int] | None:
    """(first, length) of the longest circular run of True bins."""
    if mask.all():
        return 0, BEAMS
    if not mask.any():
        return None
    start = int(np.argmin(mask))  # begin the walk at a False bin
    best, cur_first, cur_len = (0, 0), 0, 0
    for k in range(1, BEAMS + 1):
        idx = (start + k) % BEAMS
        if mask[idx]:
            if cur_len == 0:
                cur_first = idx
            cur_len += 1
            if cur_len > best[1]:
                best = (cur_first, cur_len)
        else:
            cur_len = 0
    return best


class ScanRawCheck(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("lidar_bringup_check")
        self.scans: list[tuple[float, float, float, np.ndarray]] = []
        self.layout: dict | None = None
        self.create_subscription(LaserScan, topic, self._on_scan, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))

    def _on_scan(self, m: LaserScan) -> None:
        recv = self.get_clock().now().nanoseconds * 1e-9
        if self.layout is None:
            self.layout = {
                "beams": len(m.ranges),
                "angle_min": m.angle_min,
                "angle_increment": m.angle_increment,
                "range_min": m.range_min,
                "frame_id": m.header.frame_id,
            }
        stamp = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        self.scans.append((stamp, recv, float(m.scan_time), np.asarray(m.ranges, dtype=float)))


def summarize(node: ScanRawCheck) -> dict:
    checks: dict[str, dict] = {}
    info: dict[str, object] = {}

    def check(name: str, ok: bool, measured, required: str) -> None:
        checks[name] = {"pass": bool(ok), "measured": measured, "required": required}

    lay = node.layout or {}
    check(
        "layout",
        lay.get("beams") == BEAMS
        and abs(lay.get("angle_min", 0.0) + math.pi) < 1e-3
        and abs(lay.get("angle_increment", 0.0) - 2 * math.pi / BEAMS) < 1e-4
        and abs(lay.get("range_min", 0.0) - RANGE_MIN_M) < 1e-3,
        {k: (round(v, 5) if isinstance(v, float) else v) for k, v in lay.items()},
        "360 bins, angle_min -pi, 1 deg, range_min 0.15 m",
    )
    stamps = np.array([s[0] for s in node.scans])
    recv = np.array([s[1] for s in node.scans])
    scan_time = np.array([s[2] for s in node.scans])
    ranges = np.stack([s[3] for s in node.scans]) if node.scans and lay.get("beams") == BEAMS else None
    dt = np.diff(stamps)
    check("stamps_increasing", bool(np.all(dt > 0)), int(np.sum(dt <= 0)), "every stamp after the previous one")
    period = float(np.median(dt))
    rate = 1.0 / period
    check("rate_10hz", abs(rate / RATE_HZ - 1.0) <= RATE_TOL, round(rate, 3), f"{RATE_HZ:g} Hz +-{100 * RATE_TOL:g} %")
    p5, p95 = (float(x) for x in np.percentile(dt, [5, 95]))
    check("regular_revolutions", p5 > 0.9 / RATE_HZ and p95 < 1.1 / RATE_HZ, [round(p5 * 1e3, 1), round(p95 * 1e3, 1)],
          f"p5..p95 within {0.9e3 / RATE_HZ:.0f}..{1.1e3 / RATE_HZ:.0f} ms")
    partial = float(np.mean(dt < 0.8 * period))
    check("no_partial_revolutions", partial < 0.01, round(100 * partial, 2), "< 1 % of intervals shorter than 0.8 revolution")
    lost = float(np.sum(np.maximum(0, np.round(dt / period) - 1)))
    lost_frac = lost / (len(dt) + lost)
    check("dropped_revolutions", lost_frac <= DROPPED_REVS_MAX, round(100 * lost_frac, 2), f"<= {100 * DROPPED_REVS_MAX:g} % of revolutions lost")
    whole = np.abs(dt / period - 1.0) < 0.2
    mismatch = float(np.median(np.abs(scan_time[:-1][whole] - dt[whole]))) if whole.any() else float("nan")
    check("scan_time_is_the_period", mismatch < 1e-3, round(mismatch * 1e3, 3), "median |scan_time - stamp interval| < 1 ms")
    after_end = recv - stamps - scan_time
    a5, a50, a95 = (float(x) for x in np.percentile(after_end, [5, 50, 95]))
    check("stamp_at_first_sample", a5 > -0.002 and a95 < 0.05, [round(a5 * 1e3, 1), round(a50 * 1e3, 1), round(a95 * 1e3, 1)],
          "received 0..50 ms after the end of its revolution (p5, p50, p95 in ms)")
    if ranges is not None:
        finite = np.isfinite(ranges)
        valid = finite.sum(axis=1)
        need = 0.7 * FILLABLE_BINS
        check("valid_bins", float(np.median(valid)) >= need, float(np.median(valid)), f">= {need:.0f} of {FILLABLE_BINS:.0f} fillable 1-degree bins")
        empty = (~finite) & (ranges > 0)
        run = largest_run(empty.mean(axis=0) >= 0.9)
        if run is not None:
            first, length = run
            centre = wrap_deg(BIN_DEG[first] + (length - 1) / 2.0)
            check("blind_sector_behind", length >= 20 and abs(wrap_deg(centre - 180.0)) <= 15.0,
                  {"from_deg": round(float(wrap_deg(BIN_DEG[first])), 0), "to_deg": round(float(wrap_deg(BIN_DEG[first] + length - 1)), 0),
                   "centre_deg": round(float(centre), 0)},
                  "the longest always-empty sector is behind the car (centre 180 +-15 deg; profile +145..-155)")
        else:
            check("blind_sector_behind", False, None, "an always-empty sector behind the car")
        missing = float(np.mean(empty[:, ~BLIND]))
        info["missing_returns_outside_blind_pct"] = round(100 * missing, 1)
        info["missing_returns_assumed_by_APEX_real_pct"] = 100 * MISSING_ASSUMED
        info["returns_inside_blind_sector_pct"] = round(100 * float(np.mean(finite[:, BLIND])), 1)
        close = np.where(np.mean(np.isneginf(ranges), axis=0) >= 0.5)[0]
        info["bins_always_below_0.15m_deg"] = [int(round(BIN_DEG[k])) for k in close]
    info["revolutions"] = len(node.scans)
    info["mean_rate_hz"] = round(float(1.0 / np.mean(scan_time)), 3)
    return {"passed": all(c["pass"] for c in checks.values()), "checks": checks, "info": info}


def run_summary(node: ScanRawCheck, duration_s: float, json_path: str | None) -> int:
    t0 = time.monotonic()
    last_print, last_count = t0, 0
    node.get_logger().info(f"Collecting /lidar/scan_raw for {duration_s:.0f} s; keep the car still")
    while rclpy.ok() and time.monotonic() - t0 < duration_s:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - last_print >= 10.0:
            recent = node.scans[last_count:]
            if len(recent) >= 2:
                st = [s[0] for s in recent]
                print(f"  {time.monotonic() - t0:4.0f} s: {(len(st) - 1) / (st[-1] - st[0]):.2f} Hz over the last 10 s", flush=True)
            last_print, last_count = time.monotonic(), len(node.scans)
    if len(node.scans) < 20:
        print(f"FAILED: only {len(node.scans)} scans on /lidar/scan_raw. Is the LiDAR node running (backend rplidar_express)?")
        return 2
    res = summarize(node)
    for name, c in res["checks"].items():
        print(f"{'PASS' if c['pass'] else 'FAIL'}  {name:24s} measured {c['measured']}  (required {c['required']})")
    for name, v in res["info"].items():
        print(f"INFO  {name:40s} {v}")
    print("PASSED" if res["passed"] else "FAILED")
    if json_path:
        with open(json_path, "w", encoding="utf-8") as fh:
            json.dump(res, fh, indent=1)
    return 0 if res["passed"] else 1


def run_watch(node: ScanRawCheck, every_s: float, expect: float | None, offset: int, min_m: float, max_m: float) -> int:
    print(f"Nearest object between {min_m:.2f} and {max_m:.2f} m, outside the blind sector. "
          f"Laser frame: 0 ahead, +90 left, -90 right. heading_offset_deg now {offset}. Ctrl+C to stop.")
    last = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - last < every_s:
            continue
        last = time.monotonic()
        recent = node.scans[-10:]
        del node.scans[:-10]
        if not recent:
            print("  no scan received")
            continue
        found = [o for o in (nearest_object(s[3], min_m, max_m) for s in recent) if o is not None]
        r = recent[-1][3]

        def sector(c: float) -> str:
            sel = np.abs([wrap_deg(a - c) for a in BIN_DEG]) <= 15.0
            v = r[sel][np.isfinite(r[sel])]
            return f"{v.min():.2f}" if v.size else "  -  "

        sectors = f"front {sector(0)}  left {sector(90)}  right {sector(-90)}"
        if not found:
            print(f"  no object in range | {sectors}", flush=True)
            continue
        ang = np.radians([a for a, _ in found])
        mean = math.degrees(math.atan2(np.sin(ang).mean(), np.cos(ang).mean()))
        spread = float(np.std([wrap_deg(a - mean) for a, _ in found]))
        line = f"  object at {mean:+6.1f} deg (+-{spread:.1f}) {np.mean([d for _, d in found]):.2f} m | {sectors}"
        if expect is not None:
            err = wrap_deg(mean - expect)
            line += f" | error {err:+.1f} deg -> heading_offset_deg {offset} -> {int(round(offset + err))}"
            if abs(err) > 30.0:
                line += " (large: fix the front first; if then the left object reads -90, the scan is mirrored)"
        print(line, flush=True)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", nargs="?", choices=("summary", "watch"), default="summary")
    ap.add_argument("--topic", default="/lidar/scan_raw")
    ap.add_argument("--duration", type=float, default=60.0, help="summary: seconds to collect")
    ap.add_argument("--json", default=None, help="summary: also write the result here")
    ap.add_argument("--expect", type=float, default=None, help="watch: true angle of the object, deg (0 ahead, 90 left)")
    ap.add_argument("--heading-offset", type=int, default=None, help="watch: heading_offset_deg the node runs with (default: apex_params.yaml)")
    ap.add_argument("--min-range", type=float, default=0.2)
    ap.add_argument("--max-range", type=float, default=1.5)
    ap.add_argument("--every", type=float, default=1.0)
    args = ap.parse_args()
    rclpy.init()
    node = ScanRawCheck(args.topic)
    try:
        if args.mode == "summary":
            return run_summary(node, args.duration, args.json)
        offset = args.heading_offset if args.heading_offset is not None else current_heading_offset()
        return run_watch(node, args.every, args.expect, offset, args.min_range, args.max_range)
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
