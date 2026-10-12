#!/usr/bin/env python3
"""How far does the learned odometry walk while the car is parked?

Two modes, both offline, both on the PC, neither needing the car:

``parked``
    A synthetic *parked* stream: the stationary prefix of a recorded run (the
    seconds before the race driver leaves WAIT) tiled for as long as asked,
    with fresh LiDAR noise on every revolution. This is the regime the training
    set does not contain -- its stationary prefix is 1.5 s, the training chunks
    are 3.2 s and the ``age`` input saturates at 5 s -- and the one the real car
    sits in between a bring-up check and a race.

``run``
    The recording as it was captured, which is the non-regression: the hold must
    not touch a single driving revolution.

``--compare`` runs the same stream through two estimators at once, one with the
standstill hold and one without, and prints both. That is the only honest form
of the before/after, because the LiDAR noise and the ICP are shared.

    standstill_drift.py parked --seconds 180 --imu real --compare
    standstill_drift.py run --compare

With ``--truth`` the ``run`` mode also scores both variants against the
simulator's own pose at every revolution (relative pose error over 1 s and 10 s
horizons, which is what an odometry is for, plus the end-to-end drift). That is
the only measurement that says which of the two is *better* rather than merely
different.

``--imu real`` replaces the simulated IMU's static signature with the real
car's: the WT61PC auto-zeroes its gyro at rest (exactly 0.000 rad/s, which the
simulated sensor never produces) and the chassis sits at the measured 2.2 deg
of tilt.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys
import time

import numpy as np

SIM = Path(__file__).resolve().parents[2]
ROOT = SIM.parent
sys.path.insert(0, str(SIM / "learning" / "lidar_imu_pose"))
sys.path.insert(0, str(SIM / "tools"))

from live_odometry import LidarCalibration, LiveOdometry  # noqa: E402
from pose_dataset.sensors import load_sensor_profiles  # noqa: E402
from sqlite_windows import imu_group_delay_s  # noqa: E402
from odometry_metrics import relative_errors  # noqa: E402
from standstill import StandstillConfig  # noqa: E402

SENSORS_YAML = SIM / "tools" / "pose_dataset" / "config" / "sensors.yaml"
FIXTURE = ROOT / "real_vehicle" / "data" / "replay" / "final_s13_sensors.npz"
CKPT = SIM / "learning" / "outputs" / "real2sim_v7e_standstill" / "hybrid_submap_v4" / "best_model.pt"
FIELDS = ("t", "x", "y", "yaw", "vx", "vy", "bias_gz", "bias_ax", "bias_ay", "dx", "dy", "dyaw",
          "icp_dx", "icp_dy", "icp_dyaw", "still", "hold", "eig_ratio", "sigma_x", "compute_ms")


def pingpong(i: int, n: int) -> int:
    """Tile a block forwards then backwards: no value jump at the seam.

    Plain cycling puts a step in the specific force once per period, which
    spikes ``accel_std`` and drops the ICP's standstill flag -- an artefact of
    the tiling, not of the car.
    """
    if n < 2:
        return 0
    k = i % (2 * n - 2)
    return k if k < n else 2 * n - 2 - k


def build_estimator(args: argparse.Namespace, prof: dict, hold: bool) -> LiveOdometry:
    cal = LidarCalibration.from_profile(prof["lidar"])
    cfg = StandstillConfig(enable=hold, confirm_scans=args.confirm_scans, max_step_m=args.max_step_m)
    return LiveOdometry(args.checkpoint, cal, device="cpu", imu_wait_s=10.0, imu_rate_hz=float(prof["imu"]["rate_hz"]),
                        imu_group_delay_s=imu_group_delay_s(prof["imu"]), icp_backend="polar", standstill=cfg)


def row(est, t: float) -> dict:
    return dict(t=t, x=est.pose[0], y=est.pose[1], yaw=est.pose[2], vx=est.velocity[0], vy=est.velocity[1],
                bias_gz=est.bias[0], bias_ax=est.bias[1], bias_ay=est.bias[2],
                dx=est.delta[0], dy=est.delta[1], dyaw=est.delta[2],
                icp_dx=est.icp.get("icp_dx", float("nan")), icp_dy=est.icp.get("icp_dy", float("nan")),
                icp_dyaw=est.icp.get("icp_dyaw", float("nan")), still=est.icp.get("standstill", float("nan")),
                hold=est.icp.get("hold", 0.0), eig_ratio=est.icp.get("eig_ratio", float("nan")),
                sigma_x=est.sigma[0], compute_ms=est.compute_ms)


def parked_stream(args: argparse.Namespace, d, cal: LidarCalibration):
    """(scan stamps, ranges, scan_time, imu samples) of a car that never moves."""
    scan_t, ranges, scan_time, imu_t, imu = d["scan_t"], d["ranges"], d["scan_time"], d["imu_t"], d["imu"]
    ns, ni = int(np.searchsorted(scan_t, args.static_s)), int(np.searchsorted(imu_t, args.static_s))
    base_r, base_imu = ranges[:ns].copy(), imu[:ni].copy()
    dt_scan = float(np.median(np.diff(scan_t[:ns])))
    dt_imu = float(np.median(np.diff(imu_t[:ni])))
    if args.imu != "sim":
        # The real car's two peculiarities at rest, separable on purpose: the
        # chassis sits at a measured 2.2 deg of tilt (a specific force the
        # simulated turn-on bias draws differently), and the WT61PC auto-zeroes
        # its gyro, reporting exactly 0.000 rad/s with no noise at all -- which
        # no simulated sensor ever produces.
        tilt = math.radians(2.2)
        if args.imu in ("real", "real_accel"):
            base_imu[:, :3] = (base_imu[:, :3] - base_imu[:, :3].mean(axis=0)) + np.array(
                [9.859 * math.sin(tilt) * 0.3, -9.859 * math.sin(tilt) * 0.95, 9.851], dtype=np.float32)
        if args.imu in ("real", "real_gyro"):
            base_imu[:, 3:] = 0.0
    rng = np.random.default_rng(args.seed)
    n_scans = int(args.seconds / dt_scan)
    scans, imus = [], []
    for k in range(n_scans):
        r = base_r[k % len(base_r)].copy()
        fin = np.isfinite(r)
        sig = cal.sigma_const_m + cal.sigma_prop * np.where(fin, r, 0.0)
        r[fin] = r[fin] + rng.normal(0.0, sig[fin]).astype(np.float32)
        scans.append((int(round(k * dt_scan * 1e9)), r, float(scan_time[k % len(base_r)])))
    for j in range(int(args.seconds / dt_imu) + 20):
        s = base_imu[pingpong(j, len(base_imu))]
        imus.append((int(round(j * dt_imu * 1e9)), s))
    return scans, imus


def recorded_stream(d):
    scan_t, ranges, scan_time, imu_t, imu = d["scan_t"], d["ranges"], d["scan_time"], d["imu_t"], d["imu"]
    scans = [(int(round(float(scan_t[k]) * 1e9)), ranges[k], float(scan_time[k])) for k in range(len(scan_t))]
    imus = [(int(round(float(imu_t[j]) * 1e9)), imu[j]) for j in range(len(imu_t))]
    return scans, imus


def summarise(name: str, rows: list[dict], seconds: float) -> dict:
    a = {k: np.array([r[k] for r in rows]) for k in rows[0]}
    dist = float(np.hypot(a["x"][-1], a["y"][-1]))
    held = float(a["hold"].mean())
    print(f"[{name}] {len(rows)} revolutions, {a['t'][-1]:.1f} s of stream")
    print(f"[{name}]   ICP standstill flag {100 * a['still'].mean():5.1f} % | hold active {100 * held:5.1f} % "
          f"({int(a['hold'].sum())} revolutions)")
    print(f"[{name}]   per-revolution translation: network {1e3 * np.hypot(a['dx'], a['dy']).mean():6.2f} mm, "
          f"ICP {1e3 * np.hypot(a['icp_dx'], a['icp_dy']).mean():6.2f} mm")
    print(f"[{name}]   final pose x {a['x'][-1]:+8.3f} y {a['y'][-1]:+8.3f} m, yaw {math.degrees(a['yaw'][-1]):+7.2f} deg "
          f"-> {dist:.3f} m from the origin")
    for lo, hi in ((0.0, 30.0), (30.0, 60.0), (60.0, 120.0), (120.0, max(121.0, seconds))):
        m = (a["t"] >= lo) & (a["t"] <= hi)
        if m.sum() < 5:
            continue
        i, f = np.flatnonzero(m)[0], np.flatnonzero(m)[-1]
        rate = 60.0 * math.hypot(a["x"][f] - a["x"][i], a["y"][f] - a["y"][i]) / (a["t"][f] - a["t"][i])
        print(f"[{name}]     {lo:5.0f}-{hi:5.0f} s: {rate:7.3f} m/min")
    print(f"[{name}]   end state v ({a['vx'][-1]:+.4f}, {a['vy'][-1]:+.4f}) m/s, "
          f"bias (gz {a['bias_gz'][-1]:+.4f}, ax {a['bias_ax'][-1]:+.4f}, ay {a['bias_ay'][-1]:+.4f})")
    still = a["still"] > 0.5
    if still.any():
        print(f"[{name}]   conditioning: eig_ratio parked p01 {np.quantile(a['eig_ratio'][still], 0.01):.4f} "
              f"median {np.median(a['eig_ratio'][still]):.4f}", end="")
    if (~still).any():
        print(f" | moving p01 {np.quantile(a['eig_ratio'][~still], 0.01):.4f} "
              f"median {np.median(a['eig_ratio'][~still]):.4f}", end="")
    print()
    return {"name": name, "rows": rows, "dist_m": dist, "hold_frac": held, "arr": a}


def score(name: str, rows: list[dict], truth_csv: Path) -> None:
    """Relative pose error against the simulator's pose, at two horizons."""
    tr = np.genfromtxt(truth_csv, delimiter=",", names=True)
    t_truth = (tr["stamp_ns"] - tr["stamp_ns"][0]) * 1e-9
    gt = np.column_stack((tr["x"], tr["y"], tr["yaw"]))
    t_est = np.array([r["t"] for r in rows])
    idx = np.searchsorted(t_truth, t_est)
    keep = (idx < len(t_truth)) & (np.abs(t_truth[np.clip(idx, 0, len(t_truth) - 1)] - t_est) < 0.02)
    est = np.column_stack(([r["x"] for r in rows], [r["y"] for r in rows], [r["yaw"] for r in rows]))[keep]
    gt = gt[idx[keep]]
    # Both trajectories to a common origin: the odometry starts at the identity.
    c, s_ = math.cos(gt[0, 2]), math.sin(gt[0, 2])
    d = gt[:, :2] - gt[0, :2]
    gt = np.column_stack((c * d[:, 0] + s_ * d[:, 1], -s_ * d[:, 0] + c * d[:, 1], gt[:, 2] - gt[0, 2]))
    n = len(est)
    out = []
    for horizon, step in (("1 s", 10), ("10 s", 100)):
        a = np.arange(0, n - step)
        te, ye = relative_errors(est, gt, a, a + step)
        out.append(f"RPE {horizon}: {1e3 * te.mean():6.1f} mm mean, {1e3 * np.quantile(te, 0.95):6.1f} p95, "
                   f"{math.degrees(ye.mean()):.3f} deg")
    path = float(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1).sum())
    final = float(np.linalg.norm(est[-1, :2] - gt[-1, :2]))
    print(f"[{name}]   {out[0]}")
    print(f"[{name}]   {out[1]}")
    print(f"[{name}]   end-to-end: {final:.3f} m off after {path:.1f} m of path ({100 * final / path:.2f} %), "
          f"heading {math.degrees(abs(est[-1, 2] - gt[-1, 2])):.2f} deg")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("parked", "run"))
    ap.add_argument("--seconds", type=float, default=180.0, help="parked mode: length of the stream")
    ap.add_argument("--imu", choices=("sim", "real", "real_accel", "real_gyro"), default="sim",
                    help="real: the car's whole static signature | real_accel / real_gyro: one half of it, to tell them apart")
    ap.add_argument("--static-s", type=float, default=4.4, help="stationary prefix of the recording to tile")
    ap.add_argument("--fixture", default=str(FIXTURE))
    ap.add_argument("--checkpoint", default=str(CKPT))
    ap.add_argument("--profile", default="APEX_real")
    ap.add_argument("--confirm-scans", type=int, default=StandstillConfig.confirm_scans)
    ap.add_argument("--max-step-m", type=float, default=StandstillConfig.max_step_m)
    ap.add_argument("--no-hold", dest="hold", action="store_false", help="the estimator as it was before the fix")
    ap.add_argument("--compare", action="store_true", help="run both, with and without the hold")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default="", help="CSV of the held/with-hold estimator")
    ap.add_argument("--truth", default="", help="run mode: truth_scans.csv of the recorded run, for RPE/ATE")
    args = ap.parse_args()

    d = np.load(args.fixture, allow_pickle=True)
    prof = load_sensor_profiles(SENSORS_YAML)[args.profile]
    cal = LidarCalibration.from_profile(prof["lidar"])
    scans, imus = parked_stream(args, d, cal) if args.mode == "parked" else recorded_stream(d)
    seconds = args.seconds if args.mode == "parked" else float(d["scan_t"][-1])
    variants = [True, False] if args.compare else [args.hold]
    print(f"[standstill] {args.mode} stream: {len(scans)} revolutions, {len(imus)} IMU samples, "
          f"profile {args.profile}, IMU {args.imu if args.mode == 'parked' else 'as recorded'}")

    results = []
    for hold in variants:
        live = build_estimator(args, prof, hold)
        rows: list[dict] = []
        j = 0
        t0 = time.perf_counter()
        for stamp, r, st in scans:
            end = stamp + int(round(st * 1e9))
            while j < len(imus) and imus[j][0] <= end + int(2e7):
                s = imus[j][1]
                live.add_imu(imus[j][0], np.asarray(s[3:], dtype=np.float64), np.asarray(s[:3], dtype=np.float64))
                j += 1
            live.add_scan(stamp, r, st)
            for est in live.process(end):
                rows.append(row(est, stamp * 1e-9))
        wall = time.perf_counter() - t0
        name = "with hold" if hold else "no hold  "
        print(f"[{name}] replayed in {wall:.0f} s wall ({live.hold.status()})")
        results.append(summarise(name, rows, seconds))
        if args.truth and args.mode == "run":
            score(name, rows, Path(args.truth))
        if args.out and hold == variants[0]:
            with open(args.out, "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(FIELDS))
                w.writeheader()
                w.writerows(rows)
            print(f"[standstill] wrote {args.out}")

    if len(results) == 2:
        a, b = results[0]["arr"], results[1]["arr"]
        n = min(len(a["x"]), len(b["x"]))
        sep = np.hypot(a["x"][:n] - b["x"][:n], a["y"][:n] - b["y"][:n])
        moving = (a["hold"][:n] < 0.5) & (b["still"][:n] < 0.5)
        print(f"[compare] drift {results[1]['dist_m']:.3f} m without the hold -> {results[0]['dist_m']:.3f} m with it "
              f"({results[1]['dist_m'] / max(results[0]['dist_m'], 1e-9):.0f}x)")
        print(f"[compare] the two trajectories separate by {sep[-1]:.3f} m at the end; "
              f"{int((a['hold'][:n] > 0.5).sum())} revolutions were held, of which "
              f"{int(((a['hold'][:n] > 0.5) & (b['still'][:n] < 0.5)).sum())} were not flagged still by the ICP")
        if moving.any():
            print(f"[compare] over the {int(moving.sum())} revolutions where neither held nor flagged still, the "
                  f"per-revolution increments differ by {1e3 * np.abs(np.hypot(a['dx'][:n][moving], a['dy'][:n][moving]) - np.hypot(b['dx'][:n][moving], b['dy'][:n][moving])).max():.3f} mm at most")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
