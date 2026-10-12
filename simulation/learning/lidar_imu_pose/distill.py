#!/usr/bin/env python3
"""v6: v5 with its IMU encoder distilled into a feed-forward perceptron. The
student imitates the teacher (v5); the ground truth is not used (README
section 14).

Why. On the Pi the network's cost is v5's IMU encoder, a GRU stepped over the
40 zero-padded samples of every interval: ~80 % of the ONNX Runtime time and
2217 of the 2968 graph nodes, for 15 % of the parameters. The encoder keeps
no state between intervals, so it is a function of the window alone, which
``streaming_model.WindowImuEncoder`` computes with two matrix products. The
rest of the student is v5, weight for weight, frozen: the scan-pair
convolutions, the fusion layer, the two recurrent cells (the memory), the
heads and the Kalman-style update. Only the new encoder is trained:

  prefit    regression of v5's IMU feature e_T on every window of the
            training split and on its mirrored copy (the lanes that drive a
            mirrored world), in the metric in which the rest of the network
            sees it: the fusion layer reads e only as u = W_imu e (the IMU
            columns of ``inp[0]``), so the loss is
                E |W_imu (e_S - e_T)|^2 / E |W_imu (e_T - mean e_T)|^2 = 1 - R^2(u).
            No sequences are needed. Writes ``prefit.pt``.
  distill   student and teacher run side by side on the training lanes
            (``train_streaming.Lanes``: truncated back-propagation through
            time, the same augmentation), each one with its own state, so the
            student is trained on its own trajectories, not only on the
            teacher's (as in DAgger [4]). Loss per interval:
                KL(N(mu_T, S_T) || N(mu_S, S_S))                               outputs
                + |h_S - h_T|^2 / 0.1^2 + |v_S - v_T|^2 / 0.05^2 + |b_S - b_T|^2 / b_u^2    state
            the closed-form KL of the two diagonal Gaussian predictions (delta
            and logvar, in the loss units) and the mismatch of the recurrent
            state [5]: the GRU cells, the body velocity and the biases (units of
            ``train_streaming.streaming_loss``). The gradient flows through the
            frozen core back to the encoder, so the encoder learns what the
            core needs over the chunk, not only at the fusion input. The model
            kept is the one closest to the teacher on validation (mean
            per-interval distance, the prefit student included), so the
            ground truth is used neither to train nor to select, only to
            report (``--select t_rel``: the lowest validation segment drift
            instead, as ``train_streaming.train``). Writes ``best_model.pt``
            (read by ``streaming_model.load_checkpoint``: the teacher's
            configuration + ``imu_encoder="window"``), ``history.json`` and
            ``log_distill.txt``.

Why the teacher and not the ground truth [1, 2]. The student's error is
bounded by its distance to the teacher, |S - y| <= |S - T| + |T - y|. With a
squared loss, E[(S - y)^2] = E[(S - E[y|x])^2] + E[Var(y|x)]; the teacher
approximates E[y|x], so imitating it removes the irreducible noise term from
the gradient. Why the imitation error does not pile up over the laps [3]:
the frozen core is contracting (largest Lyapunov exponent -0.02 to -0.05 1/s,
measured on v5), so a bounded input difference gives a bounded state
difference (incremental input-to-state stability).

    python distill.py prefit  --out ../outputs/real2sim_v6_imu_window/hybrid_submap_v4
    python distill.py distill --out ../outputs/real2sim_v6_imu_window/hybrid_submap_v4 --epochs 10

References
  [1] G. Hinton, O. Vinyals, J. Dean, "Distilling the Knowledge in a Neural
      Network", arXiv:1503.02531, 2015.
  [2] A. K. Menon, A. S. Rawat, S. J. Reddi, S. Kim, S. Kumar, "A Statistical
      Perspective on Distillation", ICML 2021.
  [3] F. Bonassi, M. Farina, R. Scattolini, "On the stability properties of
      Gated Recurrent Units neural networks", Systems & Control Letters 157,
      2021.
  [4] S. Ross, G. Gordon, D. Bagnell, "A Reduction of Imitation Learning and
      Structured Prediction to No-Regret Online Learning", AISTATS 2011.
  [5] A. Romero, N. Ballas, S. E. Kahou, A. Chassang, C. Gatta, Y. Bengio,
      "FitNets: Hints for Thin Deep Nets", ICLR 2015.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Any, Callable

import numpy as np
import torch
from torch import Tensor, nn

import odometry_metrics as om
from gridsearch_sqlite import DEFAULT_DB, ROOT
from sqlite_streams import StreamData
from streaming_model import IMU_MIRROR, StreamingPoseNet, load_checkpoint
from train_streaming import UNITS, Lanes, TrainConfig, _wrap, augment, load_or_run_icp, load_split_cfg, stream_predict, subset_runs

HERE = Path(__file__).resolve().parent
TEACHER = ROOT / "outputs" / "real2sim_v5_10hz" / "hybrid_submap_v4" / "best_model.pt"
OUT = ROOT / "outputs" / "real2sim_v6_imu_window" / "hybrid_submap_v4"
H_UNIT = 0.1  # GRU state (in (-1, 1))
V_UNIT = 0.05  # body velocity [m/s], as streaming_loss
BIAS_UNITS = (1.0e-3, 0.02, 0.02)  # gyro z [rad/s], accel x, y [m/s^2], as streaming_loss


@dataclass(frozen=True)
class DistillConfig:
    teacher: str = str(TEACHER)
    prefit_epochs: int = 40
    prefit_lr: float = 2.0e-3
    prefit_batch: int = 4096
    epochs: int = 10
    learning_rate: float = 3.0e-4
    state_weight: float = 1.0  # the state mismatch against the output KL
    eval_every: int = 2
    select: str = "teacher"  # "teacher": closest to the teacher on validation; "t_rel": lowest validation segment drift
    seed: int = 23


# ------------------------------------------------------------------ student
def student_from(teacher: StreamingPoseNet) -> StreamingPoseNet:
    """The teacher's architecture with ``WindowImuEncoder``; every other weight is the teacher's."""
    student = StreamingPoseNet(teacher.hidden, scales=teacher.scales, regularize=len(teacher.pair_encoder.cnn) > 13, hybrid=teacher.hybrid,
                               sweep_profile=teacher.sweep_profile, pair_widths=teacher.pair_widths, imu_encoder="window")
    own = {k: v for k, v in teacher.state_dict().items() if not k.startswith("imu_encoder.")}
    missing, unexpected = student.load_state_dict(own, strict=False)
    if unexpected or any(not k.startswith("imu_encoder.") for k in missing):
        raise RuntimeError(f"student and teacher differ outside the IMU encoder: missing {missing}, unexpected {unexpected}")
    return student.to(next(teacher.parameters()).device)


def freeze_all_but_encoder(model: StreamingPoseNet) -> list[nn.Parameter]:
    """Only the IMU encoder trains; returns its parameters."""
    for name, p in model.named_parameters():
        p.requires_grad_(name.startswith("imu_encoder."))
    return [p for p in model.parameters() if p.requires_grad]


def fusion_imu_weight(model: StreamingPoseNet) -> Tensor:
    """W_imu: the IMU columns of the fusion layer. The rest of the network reads the IMU feature e only as W_imu e."""
    return model.inp[0].weight[:, model.hidden : 2 * model.hidden].detach()


def gaussian_kl(mu_t: Tensor, lv_t: Tensor, mu_s: Tensor, lv_s: Tensor) -> Tensor:
    """KL(N(mu_t, exp lv_t) || N(mu_s, exp lv_s)) element-wise (diagonal Gaussians, lv = log variance)."""
    return 0.5 * (lv_s - lv_t + (torch.exp(lv_t) + (mu_t - mu_s).square()) * torch.exp(-lv_s) - 1.0)


def distill_loss(out_s: dict[str, Tensor], out_t: dict[str, Tensor], h_s: Tensor, h_t: Tensor, state_weight: float) -> tuple[Tensor, dict[str, float]]:
    """Output KL + recurrent-state mismatch of one chunk (module docstring)."""
    delta_s, delta_t = out_s["delta"], out_t["delta"]
    d = torch.cat((delta_s[..., :2] - delta_t[..., :2], _wrap(delta_s[..., 2:] - delta_t[..., 2:])), dim=-1) / delta_s.new_tensor(UNITS)
    kl = gaussian_kl(torch.zeros_like(d), out_t["logvar"], d, out_s["logvar"]).mean()
    s_h = ((h_s - h_t) / H_UNIT).square().mean()
    s_v = ((out_s["velocity"] - out_t["velocity"]) / V_UNIT).square().mean()
    s_b = ((out_s["bias"] - out_t["bias"]) / delta_s.new_tensor(BIAS_UNITS)).square().mean()
    total = kl + state_weight * (s_h + s_v + s_b)
    parts = {"loss": total, "kl": kl, "h": s_h, "v": s_v, "b": s_b,
             "dev_mm": 1e3 * torch.linalg.vector_norm(delta_s[..., :2] - delta_t[..., :2], dim=-1).mean()}
    return total, {k: float(v.detach()) for k, v in parts.items()}


class CellTap:
    """The output of every GRU cell at every step (forward hooks), for the state term."""

    def __init__(self, model: StreamingPoseNet) -> None:
        self.active, self.h = False, []
        self.handles = [c.register_forward_hook(self._hook) for c in model.cells]

    def _hook(self, module: nn.Module, inputs: Any, output: Tensor) -> None:  # noqa: ANN401
        if self.active:
            self.h.append(output)

    def take(self) -> Tensor:
        h, self.h = self.h, []
        return torch.stack(h)


# --------------------------------------------------------------------- data
def resolve_db(ck: dict[str, Any], db: Path | None) -> Path:
    """The teacher's database: the file named in its checkpoint, checked by size."""
    tag = ck.get("db") or {}
    path = Path(db) if db else DEFAULT_DB.parent / tag.get("name", DEFAULT_DB.name)
    if tag.get("bytes") and path.stat().st_size != tag["bytes"]:
        raise ValueError(f"{path} is not the database the teacher was trained on ({tag})")
    return path


def load_splits(tcfg: dict[str, Any], db: Path, icp_dir: Path, dev: torch.device, log: Callable[[str], None], splits: tuple[str, ...],
                icp: bool = True) -> tuple[dict[str, StreamData], dict[str, dict[str, Any]]]:
    """The teacher's splits (same sensors, robustness variants and IMU stamps) on the device, with its cached
    classical odometry attached (``icp``)."""
    delay = bool(tcfg.get("imu_delay_comp", False))
    sensors = tuple(tcfg.get("sensors") or ())
    robust = str(tcfg.get("robust", "") or "")
    if robust and not Path(robust).is_absolute() and not Path(robust).exists():
        robust = str(HERE / robust)
    sds, datas = {}, {}
    for s in splits:
        full, tag = load_split_cfg(db, s, delay, robust, int(tcfg.get("variants", 0)))
        data, sel = subset_runs(full, 1, sensors) if sensors else (full, None)
        del full
        sd = StreamData(data, dev)
        if icp and tcfg.get("hybrid", False):
            res = load_or_run_icp(s, None, icp_dir, log, tcfg["icp_variant"], db, delay, tag)
            sd.attach_icp(res if sel is None else {k: (v[sel] if torch.is_tensor(v) else v) for k, v in res.items()})
        sds[s], datas[s] = sd, data
        log(f"[data] {s}: {len(data['runs'])} runs, {sd.intervals} intervals")
    return sds, datas


def intervals_of(sd: StreamData) -> Tensor:
    """Global index of every interval of the split (scans 1 .. n-1 of every run)."""
    return torch.cat([torch.arange(o + 1, o + n, device=sd.device) for o, n in zip(sd.offset.tolist(), sd.length.tolist())])


def windows(sd: StreamData, idx: Tensor, mirror: Tensor) -> tuple[Tensor, Tensor]:
    """IMU windows [n, 1, 40, 6] as the model sees them (those of mirrored lanes flipped) and their lengths [n, 1]."""
    imu = sd.imu[idx].float()
    imu = torch.where(mirror[:, None, None], imu * imu.new_tensor(IMU_MIRROR), imu)
    return imu[:, None], sd.imu_len[idx][:, None]


def window_set(teacher: StreamingPoseNet, sd: StreamData, chunk: int = 16384) -> tuple[Tensor, Tensor, Tensor]:
    """Every interval of the split twice (plain, mirrored) with the teacher's IMU feature."""
    idx = intervals_of(sd)
    mirror = torch.cat((torch.zeros_like(idx, dtype=torch.bool), torch.ones_like(idx, dtype=torch.bool)))
    idx = torch.cat((idx, idx))
    with torch.no_grad():
        e = torch.cat([teacher.imu_encoder(*windows(sd, idx[i : i + chunk], mirror[i : i + chunk]))[:, 0] for i in range(0, len(idx), chunk)])
    return idx, mirror, e


@torch.no_grad()
def feature_r2(enc: nn.Module, sd: StreamData, ws: tuple[Tensor, Tensor, Tensor], w: Tensor, chunk: int = 16384) -> dict[str, float]:
    """R^2 of the student's IMU feature: in the metric of the fusion layer (u = W_imu e) and plain."""
    idx, mirror, e_t = ws
    e_s = torch.cat([enc(*windows(sd, idx[i : i + chunk], mirror[i : i + chunk]))[:, 0] for i in range(0, len(idx), chunk)])
    c = e_t - e_t.mean(0)
    return {"r2_u": float(1.0 - ((e_s - e_t) @ w.T).square().sum() / (c @ w.T).square().sum()),
            "r2_e": float(1.0 - (e_s - e_t).square().sum() / c.square().sum())}


def distance_mm(pred: Tensor, ref: Tensor) -> dict[str, float]:
    """Per-interval translation difference [mm] between two predictions of the same split."""
    ok = torch.isfinite(pred[:, 0]) & torch.isfinite(ref[:, 0])
    d = 1e3 * torch.linalg.vector_norm(pred[ok, :2] - ref[ok, :2], dim=-1)
    return {"mean_mm": float(d.mean()), "p99_mm": float(torch.quantile(d, 0.99)), "max_mm": float(d.max())}


# ------------------------------------------------------------------- stages
def prefit(args: argparse.Namespace, dcfg: DistillConfig, log: Callable[[str], None]) -> None:
    dev = torch.device("cuda")
    torch.manual_seed(dcfg.seed)
    ck = torch.load(dcfg.teacher, map_location="cpu", weights_only=False)
    teacher, _ = load_checkpoint(dcfg.teacher, dev)
    sds, _ = load_splits(ck["config"], resolve_db(ck, args.db), args.icp_dir, dev, log, ("train", "validation"), icp=False)
    enc = student_from(teacher).imu_encoder
    w = fusion_imu_weight(teacher)
    sets = {s: window_set(teacher, sd) for s, sd in sds.items()}
    idx, mirror, e_t = sets["train"]
    var_u = ((e_t - e_t.mean(0)) @ w.T).square().sum(1).mean()
    with torch.no_grad():
        enc.mlp[-1].bias.copy_(e_t.mean(0))  # start from the mean feature
    per_epoch = math.ceil(len(idx) / dcfg.prefit_batch)
    opt = torch.optim.AdamW(enc.parameters(), lr=dcfg.prefit_lr, weight_decay=0.0)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=dcfg.prefit_lr, total_steps=dcfg.prefit_epochs * per_epoch, pct_start=0.05)
    gen = torch.Generator(device=dev)
    gen.manual_seed(dcfg.seed)
    log(f"[prefit] {len(idx)} training windows (plain + mirrored), {sum(p.numel() for p in enc.parameters())} encoder parameters, {dcfg.prefit_epochs} epochs")
    t0 = time.time()
    for epoch in range(1, dcfg.prefit_epochs + 1):
        perm = torch.randperm(len(idx), device=dev, generator=gen)
        total = torch.zeros((), device=dev)
        for i in range(0, len(idx), dcfg.prefit_batch):
            b = perm[i : i + dcfg.prefit_batch]
            e_s = enc(*windows(sds["train"], idx[b], mirror[b]))[:, 0]
            loss = ((e_s - e_t[b]) @ w.T).square().sum(1).mean() / var_u
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            sched.step()
            total += loss.detach() * len(b)
        if epoch % 5 == 0 or epoch == dcfg.prefit_epochs:
            r2 = feature_r2(enc, sds["validation"], sets["validation"], w)
            log(f"[prefit] epoch {epoch}/{dcfg.prefit_epochs} {time.time() - t0:.0f}s train 1-R2(u) {float(total) / len(idx):.5f} | validation R2(u) {r2['r2_u']:.5f} R2(e) {r2['r2_e']:.5f}")
    r2 = {s: feature_r2(enc, sds[s], sets[s], w) for s in sets}
    torch.save({"imu_encoder": enc.state_dict(), "r2": r2, "config": asdict(dcfg)}, args.out / "prefit.pt")
    log(f"[prefit] done in {time.time() - t0:.0f} s: R2(u) train {r2['train']['r2_u']:.5f}, validation {r2['validation']['r2_u']:.5f} -> {args.out / 'prefit.pt'}")


def distill(args: argparse.Namespace, dcfg: DistillConfig, log: Callable[[str], None]) -> None:
    dev = torch.device("cuda")
    torch.manual_seed(dcfg.seed)
    ck = torch.load(dcfg.teacher, map_location="cpu", weights_only=False)
    tcfg = ck["config"]
    teacher, _ = load_checkpoint(dcfg.teacher, dev)
    for p in teacher.parameters():
        p.requires_grad_(False)
    student = student_from(teacher)
    pre = torch.load(args.out / "prefit.pt", map_location=dev, weights_only=False)
    student.imu_encoder.load_state_dict(pre["imu_encoder"])
    params = freeze_all_but_encoder(student)
    teacher.eval()
    student.eval()  # no dropout: the frozen parts compute exactly the teacher's function
    sds, datas = load_splits(tcfg, resolve_db(ck, args.db), args.icp_dir, dev, log, ("train", "validation"))
    truth_val = om.split_truth(datas["validation"])
    tc = TrainConfig(**{k: (tuple(v) if isinstance(v, list) else v) for k, v in tcfg.items() if k in TrainConfig.__dataclass_fields__})
    steps_per_epoch = math.ceil(sds["train"].intervals / (tc.lanes * tc.chunk))
    opt = torch.optim.AdamW(params, lr=dcfg.learning_rate, weight_decay=0.0)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=dcfg.learning_rate, total_steps=dcfg.epochs * steps_per_epoch, pct_start=0.1)
    lanes = Lanes(sds["train"], tc, np.random.default_rng(dcfg.seed))
    tap_t, tap_s = CellTap(teacher), CellTap(student)

    ref = stream_predict(teacher, sds["validation"])
    m_ref = om.evaluate(ref["pred"].numpy(), truth_val, sigma=ref["sigma"].numpy(), groups=())["all"]
    log(f"[distill] teacher {dcfg.teacher}: validation t_rel {m_ref['t_rel_pct']:.3f} % rpe1s {m_ref['rpe_1s_cm']:.2f} cm speed {m_ref['speed_err_cmps']:.2f} cm/s")

    def validate() -> tuple[dict[str, Any], dict[str, float]]:
        val = stream_predict(student, sds["validation"])
        return om.evaluate(val["pred"].numpy(), truth_val, sigma=val["sigma"].numpy(), groups=())["all"], distance_mm(val["pred"], ref["pred"])

    def fmt(m: dict[str, Any], dist: dict[str, float]) -> str:
        return (f"val t_rel {m['t_rel_pct']:.3f} % rpe1s {m['rpe_1s_cm']:.2f} cm speed {m['speed_err_cmps']:.2f} cm/s | "
                f"to teacher {dist['mean_mm']:.3f} / p99 {dist['p99_mm']:.3f} / max {dist['max_mm']:.2f} mm")

    config = {**tcfg, "imu_encoder": "window", "teacher": str(Path(dcfg.teacher).resolve()), "distill": asdict(dcfg), "prefit": {"r2": pre["r2"], **pre["config"]}}
    history: list[dict[str, Any]] = []
    best = (math.inf, -1)

    def keep(epoch: int, m: dict[str, Any], dist: dict[str, float]) -> None:
        nonlocal best
        score = dist["mean_mm"] if dcfg.select == "teacher" else m["t_rel_pct"]
        if score < best[0]:
            best = (score, epoch)
            torch.save({"model": student.state_dict(), "config": config, "epoch": epoch, "val": m, "val_teacher": m_ref, "distance_to_teacher": dist,
                        "db": ck.get("db")}, args.out / "best_model.pt")

    m, dist = validate()
    log(f"[distill] prefit student (epoch 0): {fmt(m, dist)}")
    history.append({"epoch": 0, **{f"val_{k}": v for k, v in m.items()}, **{f"to_teacher_{k}": v for k, v in dist.items()}})
    keep(0, m, dist)
    log(f"[distill] {len(params)} trainable tensors ({sum(p.numel() for p in params)} parameters) of {sum(p.numel() for p in student.parameters())}; "
        f"{steps_per_epoch} steps/epoch of {tc.lanes} lanes x {tc.chunk} intervals, {dcfg.epochs} epochs")
    st_t = teacher.initial_state(tc.lanes, dev)
    st_s = student.initial_state(tc.lanes, dev)
    t_start = time.time()
    for epoch in range(1, dcfg.epochs + 1):
        t0 = time.time()
        sums: dict[str, float] = {}
        tap_t.active = tap_s.active = True
        for _ in range(steps_per_epoch):
            idx, reset, mirror = lanes.next()
            st_t = teacher.reset(st_t, reset)
            st_s = student.reset({k: v.detach() for k, v in st_s.items()}, reset)
            batch = augment(sds["train"].gather(idx), mirror, tc.beam_dropout_max)
            with torch.no_grad():
                out_t, st_t = teacher(batch, st_t)
            out_s, st_s = student(batch, st_s)
            loss, parts = distill_loss(out_s, out_t, tap_s.take(), tap_t.take(), dcfg.state_weight)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(params, 5.0)
            opt.step()
            sched.step()
            for k, v in parts.items():
                sums[k] = sums.get(k, 0.0) + v / steps_per_epoch
        tap_t.active = tap_s.active = False
        rec = {"epoch": epoch, "seconds": time.time() - t0, **{f"train_{k}": v for k, v in sums.items()}}
        line = (f"[distill] epoch {epoch}/{dcfg.epochs} {rec['seconds']:.0f}s loss {sums['loss']:.4f} (kl {sums['kl']:.4f} h {sums['h']:.4f} "
                f"v {sums['v']:.4f} b {sums['b']:.4f}) to teacher {sums['dev_mm']:.3f} mm")
        if epoch % dcfg.eval_every == 0 or epoch == dcfg.epochs:
            m, dist = validate()
            rec.update({**{f"val_{k}": v for k, v in m.items()}, **{f"to_teacher_{k}": v for k, v in dist.items()}})
            keep(epoch, m, dist)
            line += " | " + fmt(m, dist)
        log(line)
        history.append(rec)
        (args.out / "history.json").write_text(json.dumps({"config": config, "val_teacher": m_ref, "best_epoch": best[1], "history": history}, indent=1, default=float),
                                               encoding="utf-8")
    crit = "mean distance to the teacher [mm]" if dcfg.select == "teacher" else "validation segment drift [%]"
    log(f"[distill] done in {time.time() - t_start:.0f} s, best epoch {best[1]} ({crit} {best[0]:.3f}; teacher drift {m_ref['t_rel_pct']:.3f} %)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=("prefit", "distill"))
    ap.add_argument("--teacher", type=Path, default=TEACHER)
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--db", type=Path, default=None, help="the teacher's database (default: the file named in its checkpoint, next to the default one)")
    ap.add_argument("--icp-dir", type=Path, default=None, help="the teacher's cached classical odometry (default: the output folder of its icp stage)")
    ap.add_argument("--epochs", type=int, default=None, help=f"default: prefit {DistillConfig.prefit_epochs}, distill {DistillConfig.epochs}")
    ap.add_argument("--lr", type=float, default=None, help=f"default: prefit {DistillConfig.prefit_lr}, distill {DistillConfig.learning_rate}")
    ap.add_argument("--state-weight", type=float, default=DistillConfig.state_weight)
    ap.add_argument("--select", choices=("teacher", "t_rel"), default=DistillConfig.select, help="distill: keep the epoch closest to the teacher, or the lowest drift")
    args = ap.parse_args()
    args.icp_dir = args.icp_dir or args.teacher.resolve().parents[1]
    base = DistillConfig()
    dcfg = DistillConfig(teacher=str(args.teacher), state_weight=args.state_weight, select=args.select,
                         prefit_epochs=args.epochs if args.stage == "prefit" and args.epochs else base.prefit_epochs,
                         prefit_lr=args.lr if args.stage == "prefit" and args.lr else base.prefit_lr,
                         epochs=args.epochs if args.stage == "distill" and args.epochs else base.epochs,
                         learning_rate=args.lr if args.stage == "distill" and args.lr else base.learning_rate)
    # Exact float32 products: the student imitates the teacher at the sub-millimetre level, below TF32's rounding.
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = True
    torch.set_num_threads(4)
    args.out.mkdir(parents=True, exist_ok=True)
    log_path = args.out / f"log_{args.stage}.txt"

    def log(msg: str) -> None:
        print(msg, flush=True)
        with open(log_path, "a", encoding="utf-8") as fh:
            fh.write(msg + "\n")

    (prefit if args.stage == "prefit" else distill)(args, dcfg, log)


if __name__ == "__main__":
    main()
