#!/usr/bin/env python3
"""Replay the lap-1 reactive driver offline on the scans a Gazebo run recorded.

``apex_real2sim.launch.py`` makes the race driver write ``driver_inputs.csv``:
every LiDAR revolution the reactive driver received during lap 1, with the
odometry poses, speed and travel it was given. This script feeds them again to
``RaceDriverCore.on_scan`` (same accumulator, same tracker and planners), with
the configuration of the run or a modified one (``--set``), and prints what
the driver decides at every revolution next to what it decided in Gazebo.

The replay is open loop: the car does not move differently when the decision
changes, so it answers "what would the new driver have commanded here, with
what it saw", not "would the car have made the corner". Use it to find why a
decision was wrong and to screen changes in seconds; confirm them in Gazebo.

  python3 tools/analysis/replay_reactive.py data/real2sim/<run> --from-travel 16 --to-travel 20 \\
      [--set reactive.cluster_eps_m=0.30 ...] [--plot 20.2 20.6 21.0]

``--plot`` draws, at the given node times [s], the car-frame cloud, the
followed wall, the chosen path, and the true walls (grey) and curbs (orange,
invisible to the LiDAR) placed with the true pose at that time.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

SIM = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM / "ros2_ws" / "src" / "apex_fusion_research"))
sys.path.insert(0, str(SIM / "tools"))

from apex_fusion_research.core.map_metrics import se2_apply, se2_inverse  # noqa: E402
from apex_fusion_research.core.race_config import RaceDriverConfig, vehicle_limits  # noqa: E402
from apex_fusion_research.core.race_driver import EXPLORE, RaceDriverCore  # noqa: E402
from apex_fusion_research.core.reactive import lane_known  # noqa: E402

N_HEAD = 16  # stamp_ns, now_ns, phase, angle_min, angle_inc, pose at stamp (3), pose now (3), speed, travel, laser (3); then the ranges


def load_config(run_dir: Path, overrides: list[str]) -> RaceDriverConfig:
    """The driver configuration of the run (vehicle.yaml + the launch's car overrides), then ``overrides``."""
    from pose_dataset.vehicle import load_vehicle_config  # noqa: PLC0415

    rc = json.loads((run_dir / "run_config.json").read_text(encoding="utf-8"))
    car = dict(rc.get("car", {}))
    cfg = RaceDriverConfig()
    items = [f"{k}={v}" for k, v in car.items() if "." in k] + list(overrides)
    for item in items:
        key, value = item.split("=", 1)
        sec, field = key.split(".", 1)
        obj = getattr(cfg, sec)
        cur = getattr(obj, field)
        setattr(obj, field, type(cur)(value) if not isinstance(cur, bool) else str(value).lower() in ("1", "true", "yes"))
    vy = car.get("vehicle_yaml")
    cfg.vehicle = vehicle_limits(load_vehicle_config(Path(vy)) if vy else load_vehicle_config(), cfg.vehicle.steer_plan_deg)
    return cfg


def read_inputs(run_dir: Path) -> list[dict]:
    rows = []
    with open(run_dir / "driver_inputs.csv", encoding="utf-8") as fh:
        rd = csv.reader(fh)
        next(rd)
        for r in rd:
            f = [float(v) for v in r[3:N_HEAD]]
            rows.append({"stamp_ns": int(r[0]), "now_ns": int(r[1]), "phase": r[2], "angle_min": f[0], "angle_inc": f[1],
                         "pose_stamp": tuple(f[2:5]), "pose_now": tuple(f[5:8]), "speed": f[8], "travel": f[9], "laser": tuple(f[10:13]),
                         "ranges": np.array([float(v) for v in r[N_HEAD:]])})
    return rows


def logged_decisions(run_dir: Path) -> tuple[np.ndarray, list[dict]]:
    rows = list(csv.DictReader(open(run_dir / "driver.csv", encoding="utf-8")))
    return np.array([float(r["t"]) for r in rows]), rows


def replay(run_dir: Path, cfg: RaceDriverConfig, inputs: list[dict]) -> list[dict]:
    core = RaceDriverCore(cfg, 1, None, 0.0)
    out = []
    for r in inputs:
        core.base_to_laser = r["laser"]
        core.phase = EXPLORE if r["phase"] != "wait" else "wait"
        core.travel = r["travel"]
        core.on_scan(r["stamp_ns"] * 1e-9, r["ranges"], r["angle_min"], r["angle_inc"], r["pose_stamp"], r["pose_now"], r["speed"])
        if core.phase == "wait" or core.cmd is None:
            continue
        c, s = core.cmd, core.state
        out.append({"t": r["now_ns"] * 1e-9, "travel": r["travel"], "mode": s.mode, "planner": c.diag.get("planner", ""), "k1": c.kappa, "v": c.v,
                    "d_wall": c.diag.get("d_wall", math.nan), "d_target": s.d_target, "feasible": c.feasible, "n_feas": c.diag.get("n_feasible", 0),
                    "path": c.path.copy(), "cloud": core.acc.cloud_in(r["pose_now"]),
                    "ref": None if s.ref is None else s.ref.points.copy(), "other": None if s.other is None else s.other.points.copy(),
                    "pose_now": r["pose_now"], "speed": r["speed"], "diag": dict(c.diag), "lane": list(s.lane), "d_hi": s.d_hi})
    return out


def truth_pose(run_dir: Path):  # noqa: ANN201
    tr = np.array([[float(r["t_ns"]) * 1e-9, float(r["x"]), float(r["y"]), float(r["yaw"])]
                   for r in csv.DictReader(open(run_dir / "truth_track.csv", encoding="utf-8"))])
    yaw = np.unwrap(tr[:, 3])
    return lambda t: (float(np.interp(t, tr[:, 0], tr[:, 1])), float(np.interp(t, tr[:, 0], tr[:, 2])), float(np.interp(t, tr[:, 0], yaw)))


def plot(run_dir: Path, rep: list[dict], times: list[float], out: Path, cfg: RaceDriverConfig) -> None:
    import matplotlib  # noqa: PLC0415

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415

    from apex_fusion_research.core.clearance import outline_points  # noqa: PLC0415
    from apex_fusion_research.nodes._pose_tools import run_setup  # noqa: PLC0415

    rc = json.loads((run_dir / "run_config.json").read_text(encoding="utf-8"))
    geom = run_setup(rc["track"], rc["motion"], int(rc["seed"]), int(float(rc["laps"])))["track"].geometry
    walls, _ = outline_points(geom, kinds=("wall", "obstacle"))
    curbs, _ = outline_points(geom, kinds=("curb",))
    pose_at = truth_pose(run_dir)
    ts = np.array([r["t"] for r in rep])
    fig, axes = plt.subplots(1, len(times), figsize=(5 * len(times), 5.6), squeeze=False)
    veh = cfg.vehicle
    for ax, t in zip(axes[0], times):
        r = rep[int(np.argmin(np.abs(ts - t)))]
        tp = pose_at(r["t"])
        inv = se2_inverse(tp)
        for pts, col in ((walls, "#9a978e"), (curbs, "#eb6834")):
            q = se2_apply(inv, pts)
            m = (q[:, 0] > -2.5) & (q[:, 0] < 5.0) & (np.abs(q[:, 1]) < 3.5)
            ax.scatter(q[m, 0], q[m, 1], s=1.5, c=col, linewidths=0)
        if r["lane"]:  # the proven lane (lane rule, lane_known)
            gx, gy = np.meshgrid(np.arange(-2.0, 4.5, 0.08), np.arange(-3.2, 3.2, 0.08), indexing="ij")
            g = np.stack((gx, gy), axis=-1)
            k = lane_known(g, r["lane"], r["d_hi"], cfg.reactive.both_max_width_m)
            ax.scatter(gx[k], gy[k], s=5, c="#a6dcaa", linewidths=0, zorder=0, label="carril comprobado")
        cl = r["cloud"]
        ax.scatter(cl[:, 0], cl[:, 1], s=4, c="#2a78d6", linewidths=0, label="nube del carro")
        if r["ref"] is not None:
            ax.scatter(r["ref"][:, 0], r["ref"][:, 1], s=7, facecolors="none", edgecolors="#1baf7a", linewidths=0.6, label="pared seguida")
        if r["other"] is not None:
            ax.scatter(r["other"][:, 0], r["other"][:, 1], s=7, facecolors="none", edgecolors="#b04ad6", linewidths=0.6, label="otra pared")
        p = r["path"]
        if len(p):
            ax.plot(p[:, 0], p[:, 1], c="k", lw=1.5, label="camino elegido (eje trasero)")
        box = np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5]]) * [veh.length_m, veh.width_m]
        ax.plot(box[:, 0], box[:, 1], c="r", lw=1)
        d = r["diag"]
        ax.set_title(f"t={r['t']:.2f} s, travel {r['travel']:.1f} m: {r['mode']} / {r['planner']}\n"
                     f"k1={r['k1']:+.2f} v={r['v']:.2f} d_wall={r['d_wall']:.2f} d_t={r['d_target']:.2f} feas={r['n_feas']} "
                     f"relaxed={d.get('bound_relaxed', '-')}", fontsize=8)
        ax.set_aspect("equal")
        ax.set_xlim(-2.0, 4.5)
        ax.set_ylim(-3.2, 3.2)
    axes[0][0].legend(fontsize=7, loc="lower left")
    fig.suptitle(f"{run_dir.name}: replay (grey: true walls, orange: true curbs, invisible to the LiDAR)", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    print(f"figure: {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--set", nargs="*", default=[], help="config overrides section.field=value")
    ap.add_argument("--from-travel", type=float, default=0.0)
    ap.add_argument("--to-travel", type=float, default=math.inf)
    ap.add_argument("--every", type=int, default=1, help="print one revolution in N")
    ap.add_argument("--plot", type=float, nargs="*", default=[], help="node times [s] to draw")
    ap.add_argument("--out", type=Path, default=None, help="figure path (default <run_dir>/replay.png)")
    a = ap.parse_args()
    cfg = load_config(a.run_dir, a.set)
    inputs = read_inputs(a.run_dir)
    rep = replay(a.run_dir, cfg, inputs)
    lt, lrows = logged_decisions(a.run_dir)
    print(f"{'t':>7s} {'travel':>6s} | replay: {'mode':4s} {'plan':5s} {'k1':>6s} {'v':>5s} {'d_wall':>6s} {'d_t':>5s} {'feas':>4s} {'known':>5s} "
          f"| gazebo: {'mode':4s} {'plan':5s} {'d_wall':>6s} {'steer':>6s}")
    shown = [r for r in rep if a.from_travel <= r["travel"] <= a.to_travel]
    for r in shown[:: max(1, a.every)]:
        g = lrows[int(np.argmin(np.abs(lt - r["t"])))]
        print(f"{r['t']:7.2f} {r['travel']:6.2f} | {r['mode']:>6s} {r['planner']:5s} {r['k1']:+6.2f} {r['v']:5.2f} {r['d_wall']:6.2f} {r['d_target']:5.2f} "
              f"{r['n_feas']:4d} {r['diag'].get('lane_known', math.nan):5.2f} | {g['mode']:>4s} {g['planner']:5s} {float(g['d_wall'] or 'nan'):6.2f} {float(g['steer_deg']):+6.1f}")
    if a.plot:
        plot(a.run_dir, rep, a.plot, a.out or a.run_dir / "replay.png", cfg)


if __name__ == "__main__":
    main()
