#!/usr/bin/env python3
"""Replay the plant identification over a run that is already on disk.

The estimator only ever sees two things: the pose increments the odometry
network published, with their sigma, and the commands the driver published. A
finished run has both, in ``estimator.csv`` and ``driver.csv``, so what the
identification *would* have found can be recomputed from any recording, in a
second, without a simulator -- and the same holds for a recording made on the
real car, which is the point: this is how the car's own plant gets measured
before anything is tuned on it.

    learning/.venv/bin/python tools/analysis/replay_adaptation.py data/real2sim/adapt_s6
    .../replay_adaptation.py data/real2sim/fin_s* --quiet      # the null test

With no plant drawn (``plant_scale 0``) the answer must be the datasheet: that
is the null test, and it is the cheapest way to catch a sign, a scale or a
sigma mistake in the estimator. With a plant drawn, ``run_config.json`` holds
what it was, and the table prints both.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve()
SIM = HERE.parents[2]
sys.path.insert(0, str(SIM / "ros2_ws" / "src" / "apex_fusion_research"))

from apex_fusion_research.core.plant_id import PlantIdentifier  # noqa: E402
from apex_fusion_research.core.race_config import RaceDriverConfig  # noqa: E402


def _rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def replay(run: Path, cfg: RaceDriverConfig | None = None) -> dict:
    """Identify the plant of ``run`` from its recorded estimates and commands."""
    cfg = cfg or RaceDriverConfig()
    ident = PlantIdentifier(cfg)
    config = json.loads((run / "run_config.json").read_text()) if (run / "run_config.json").exists() else {}
    veh = cfg.vehicle

    commands = []
    driver = run / "driver.csv"
    if driver.exists():
        for row in _rows(driver):
            try:
                t, v, steer = float(row["t"]), float(row["v_cmd"]), math.radians(float(row["steer_deg"]))
            except (KeyError, ValueError):
                continue
            # driver.csv records the angle that was published; the curvature it
            # carries on the wire is tan(steer) / wheelbase.
            commands.append((t, math.tan(steer) / veh.wheelbase_m, v))
    estimates = []
    pose = (0.0, 0.0, 0.0)
    for row in _rows(run / "estimator.csv"):
        try:
            t = float(row["stamp_ns"]) * 1e-9
            d = (float(row["dx"]), float(row["dy"]), float(row["dyaw"]))
            sigma = (float(row["sx"]), float(row["sy"]), float(row["syaw"]))
        except (KeyError, ValueError):
            continue
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pose = (pose[0] + c * d[0] - s * d[1], pose[1] + s * d[0] + c * d[1], pose[2] + d[2])
        estimates.append((t, pose, sigma))
    if not estimates:
        raise SystemExit(f"{run}: estimator.csv has no usable rows")

    track, i = [], 0
    for t, p, sigma in estimates:
        while i < len(commands) and commands[i][0] <= t:
            ident.push_command(*commands[i])
            i += 1
        ident.on_estimate(t, p, sigma)
        if len(track) < 1 or t - track[-1]["t"] >= 5.0:
            track.append({"t": t, **{k: ident.status()[k] for k in ("confidence", "gain_left", "gain_right", "trim", "arm_m", "v_gain")}})
    return {"run": run.name, "status": ident.status(), "track": track, "plant": config.get("plant_true", {}),
            "n_estimates": len(estimates), "n_commands": len(commands)}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", type=Path, nargs="+")
    ap.add_argument("--quiet", action="store_true", help="one line per run, no convergence table")
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args()

    out = []
    for run in args.runs:
        if not (run / "estimator.csv").exists():
            print(f"{run.name}: no estimator.csv, skipped")
            continue
        res = replay(run)
        out.append(res)
        st, plant = res["status"], res["plant"]
        print(f"\n== {res['run']}  ({res['n_estimates']} estimates, {res['n_commands']} commands)")
        if not args.quiet and res["track"]:
            print("    t      conf   g_left  g_right   trim      arm    g_v")
            for row in res["track"]:
                print("  %6.1f  %5.2f  %7.3f %7.3f  %+7.4f  %6.4f  %5.3f" % (
                    row["t"], row["confidence"], row["gain_left"], row["gain_right"], row["trim"], row["arm_m"], row["v_gain"]))
        if plant:
            print("  found : arm %.4f m  gain %.3f / %.3f  trim %+.4f  v_gain %.3f  dead %.3f  tau %.2f/%.2f  conf %.2f" % (
                st["arm_m"], st["gain_left"], st["gain_right"], st["trim"], st["v_gain"], st["dead_mps"],
                st["tau_steer_s"], st["tau_speed_s"], st["confidence"]))
            print("  drawn : arm %.4f m  steer gain %.3f  trim %+.2f deg  esc gain %.3f  start %.3f m/s  bridge L %.3f m  (scale %.1f)" % (
                plant.get("arm_true_m", float("nan")), plant.get("steer_gain", float("nan")), plant.get("steer_trim_deg", float("nan")),
                plant.get("esc_gain", float("nan")), plant.get("stiction_mps", float("nan")),
                plant.get("decode_wheelbase_m", float("nan")), plant.get("plant_scale", float("nan"))))
            print("  arm error %+.1f mm" % (1000.0 * (st["arm_m"] - plant.get("arm_true_m", float("nan")))))
        else:
            print("  found : arm %.4f m  gain %.3f / %.3f  trim %+.4f  conf %.2f (no plant recorded: the datasheet is the answer)" % (
                st["arm_m"], st["gain_left"], st["gain_right"], st["trim"], st["confidence"]))
        if st["rejected"]:
            print("  refused:", {k: v for k, v in st["rejected"].items() if v})
    if args.json:
        args.json.write_text(json.dumps(out, indent=1, default=float), encoding="utf-8")


if __name__ == "__main__":
    main()
