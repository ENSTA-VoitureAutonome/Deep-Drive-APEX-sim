"""Odometry of several checkpoints on the plain and the perturbed splits.

Every hybrid checkpoint runs with the classical odometry it was trained with
(``ck["config"]["icp_variant"]``), on the stored runs (variant 0) and on one
perturbed copy of every run (variant 1 of a ``robustness.py`` config: datasheet
gyro bias and warm-up drift, IMU stamp offset / jitter / losses / 52 Hz, lost
revolutions). The perturbed split is broken down by IMU rate and by the size
of the gyro-z bias.

    python evaluate_robust.py --db <v2 sqlite> --robust config/robust_real.yaml \
        --out ../outputs/real2sim_v3_robust/evaluation \
        --checkpoints v2_fast=../outputs/real2sim_v2_fast/hybrid_submap_v3/best_model.pt \
                      v3_robust=../outputs/real2sim_v3_robust/hybrid_submap_v3_real/best_model.pt
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import torch

import odometry_metrics as om
from robustness import RobustConfig, load_split_variants, variants_tag
from sqlite_streams import StreamData
from streaming_model import load_checkpoint
from train_streaming import CACHE, _bias_err, load_or_run_icp, stream_predict

KEYS = ("speed_err_cmps", "yawrate_err_dps", "rpe_1s_cm", "t_rel_pct", "r_rel_degpm")


def _label_runs(data: dict[str, Any]) -> None:
    """Per-run groups of the perturbed split: IMU rate and gyro-z bias size."""
    bias = data["bias_truth"].numpy()[:, 0]
    for r in data["runs"]:
        p = r.get("perturbation")
        r["imu_rate"] = "52 Hz" if p and p["imu_half_rate"] else "104 Hz"
        b = float(np.abs(np.median(bias[r["offset"] + 1 : r["offset"] + r["n"]])))
        r["gyro_bias"] = "|bz| > 0.1 rad/s" if b > 0.1 else ("0.05-0.1 rad/s" if b > 0.05 else "< 0.05 rad/s")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--db", type=Path, required=True)
    ap.add_argument("--robust", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--checkpoints", nargs="+", required=True, help="name=path")
    ap.add_argument("--splits", nargs="+", default=["validation", "test"])
    ap.add_argument("--icp-association", choices=("cdist", "polar"), default="cdist",
                    help="nearest-neighbour search of the classical odometry (polar: polar_icp.py, the same algorithm compiled)")
    ap.add_argument("--variants", type=int, nargs="+", default=[0, 1], help="0: stored runs, 1: one perturbed copy")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    rcfg = RobustConfig.load(args.robust)

    def log(msg: str) -> None:
        print(msg, flush=True)
        with open(args.out / "log_evaluate.txt", "a", encoding="utf-8") as fh:
            fh.write(msg + "\n")

    nets = {}
    for spec in args.checkpoints:
        name, path = spec.split("=", 1)
        model, ck = load_checkpoint(path, dev)
        nets[name] = (model, ck["config"])
    summary: dict[str, Any] = {"robust_config": str(args.robust), "robust_tag": rcfg.tag(), "checkpoints": {n: dict(c) for n, (_, c) in nets.items()}, "results": {}}
    for split in args.splits:
        for variant in args.variants:
            name = f"{split}_{'plain' if variant == 0 else 'perturbed'}"
            data = load_split_variants(args.db, CACHE, split, True, rcfg, [variant])
            _label_runs(data)
            sd = StreamData(data, dev)
            truth = om.split_truth(data)
            tag = variants_tag(rcfg, [variant])
            res = {}
            for net_name, (model, cfg) in nets.items():
                icp_variant = cfg["icp_variant"] + ("_polar" if args.icp_association == "polar" else "")
                icp = load_or_run_icp(split, sd, args.out, log, icp_variant, args.db, bool(cfg.get("imu_delay_comp", False)), tag)
                if f"classical_{icp_variant}" not in res:  # the classical odometry alone
                    mc = om.evaluate(icp["pred"].numpy().astype(np.float64), truth)
                    res[f"classical_{icp_variant}"] = mc
                    log(f"[{name}] classical {icp_variant}: " + ", ".join(f"{k} {mc['all'][k]:.3f}" for k in KEYS))
                sd.attach_icp(icp)
                est = stream_predict(model, sd)
                pred = est["pred"].numpy().astype(np.float64)
                m = om.evaluate(pred, truth, sigma=est["sigma"].numpy(), groups=("imu_rate", "gyro_bias", "motion") if variant else ("motion",))
                m["gyro_bias_error_dps"] = _bias_err(est["bias"].numpy()[:, 0], data, np.isfinite(pred).all(axis=1))
                # Invented motion while parked [mm per revolution]: the mean
                # translation reported where the car's true body speed is under
                # 1 cm/s. Standing still is the regime v5 was never shown, and
                # where it walks away from itself (README section 15); this is
                # the number a model has to bring down, on data it never saw.
                still = (torch.linalg.vector_norm(data["v_body"], dim=1) < 0.01).numpy() & np.isfinite(pred).all(axis=1)
                m["still_mm"] = float(1e3 * np.linalg.norm(pred[still, :2], axis=1).mean()) if still.any() else float("nan")
                m["still_n"] = int(still.sum())
                res[net_name] = m
                log(f"[{name}] {net_name}: " + ", ".join(f"{k} {m['all'][k]:.3f}" for k in KEYS)
                    + f", gyro bias err {m['gyro_bias_error_dps']:.3f} deg/s, parked {m['still_mm']:.3f} mm over {m['still_n']} intervals")
                if variant:
                    for g in ("imu_rate", "gyro_bias"):
                        log("    " + " | ".join(f"{g}={k}: t_rel {v['t_rel_pct']:.2f} %, r_rel {v['r_rel_degpm']:.3f}" for k, v in m[g].items()))
            summary["results"][name] = res
            del sd, data
            (args.out / "evaluation.json").write_text(json.dumps(summary, indent=1, default=float), encoding="utf-8")


if __name__ == "__main__":
    main()
