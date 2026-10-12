#!/usr/bin/env python3
"""Per-scan latency of the learned odometry on this machine (PC or Pi 4).

The measurement side of the Pi 4 compute budget (``pi4_budget.py`` turns
these measurements into the bound; README section 10). Portable: torch,
numpy and the standard library only (no ROS, no Gazebo, no matplotlib), so
the file runs unchanged on the car's Raspberry Pi 4 from the archive made
by ``bundle``. The SQLite dataset is only needed by ``export``, ``dr`` and
``simref`` (PC).

Subcommands:
  export     validation runs of the dataset -> replay .npz (scans, IMU,
             calibration, ground truth)                                      [PC]
  bundle     tar.gz with the sources, a replay .npz and the checkpoint       [PC]
  calibrate  micro-benchmarks of the cost classes (see below)
  profile    replay through ``LiveOdometry``: wall time per scan and per
             component, and a census of the torch operations
  sweep      latency against the network size (random weights, no
             training) and against the ICP settings
  live       real-time replay with the threads of the ROS node (IMU
             callback + scan worker sharing the GIL)
  dr         error of the IMU-propagated control pose against its age        [PC]
  simref     latency of the validated closed-loop simulation runs            [PC]

Cost classes. The time of one scan is split into
  compute    convolutions and matrix products with >= 4 FLOP per byte
             touched (weights reused along the scan): SIMD-bound;
  mem_cache  larger operations below 0.5 MB of working set (the GRU and
             linear layers at batch 1 are GEMVs: their weights stream from
             the cache every scan);
  mem_dram   operations above 0.5 MB (the cdist of the submap ICP: a
             360 x 1800 distance matrix of 2.6 MB), in the PC's 24 MB L3
             but past the Pi 4's shared 1 MB L2;
  dispatch   the fixed cost of every top-level torch call (argument parsing,
             dispatcher, allocation) = top-level small ops x the measured
             cost of a small op;
  python     the rest: interpreter and numpy between the torch calls.
A slower machine scales every class by its own factor; ``pi4_budget.py``
projects the PC measurement to the Pi with those factors (a spec band until
``calibrate`` + ``profile`` have been run on the Pi, then the measured ratios).
"""

from __future__ import annotations

import os

# One intra-op thread before torch is imported: the reference of the budget
# is one reserved core (the node's num_threads must be 1 on the Pi).
for _var in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse  # noqa: E402
from collections import defaultdict, deque  # noqa: E402
from dataclasses import replace  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
from pathlib import Path  # noqa: E402
import platform  # noqa: E402
import shutil  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402
import tarfile  # noqa: E402
import tempfile  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
from typing import Any, Callable, Iterator  # noqa: E402

import numpy as np  # noqa: E402
import torch  # noqa: E402
from torch import nn  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import icp_odometry  # noqa: E402
from icp_odometry import ICP_VARIANTS, IcpStream  # noqa: E402
import live_odometry  # noqa: E402
from live_odometry import LidarCalibration, LiveOdometry, propagate_imu  # noqa: E402
from sqlite_windows import imu_group_delay_s  # noqa: E402
from streaming_model import PAIR_WIDTHS, StreamingPoseNet, load_checkpoint  # noqa: E402

DEFAULT_CKPT = HERE.parent / "outputs" / "real2sim_v5_10hz" / "hybrid_submap_v4" / "best_model.pt"
DEFAULT_DB = HERE.parents[1] / "data" / "multiscenario_pose" / "pose_dataset_v4.sqlite3"
DATA_DIR = HERE / "config" / "pi4_budget"  # small measurement JSONs (tracked)
REPLAY_DIR = HERE.parent / "outputs" / "pi4_budget"  # replays and bundles (gitignored)
SIM_RUNS = HERE.parents[1] / "data" / "real2sim"

KB, MB = 1024.0, 1024.0 * 1024.0
DRAM_WS_BYTES = 0.5 * MB  # half of the Pi 4's shared 1 MB L2 (the other cores use the rest)
SMALL_WS_BYTES = 64 * KB  # below this an operation is its fixed cost (dispatch)
COMPUTE_INTENSITY = 4.0  # FLOP per byte from which an operation is SIMD-bound
CONV_OPS = ("aten::conv1d", "aten::convolution", "aten::_convolution", "aten::mkldnn_convolution", "aten::_slow_conv2d_forward",
            "aten::thnn_conv2d", "aten::_nnpack_spatial_convolution", "aten::slow_conv_dilated2d")
COMPONENTS = ("prep", "icp_scan", "icp_map", "icp_native", "icp_other", "network", "net_native", "post", "glue")


# ----------------------------------------------------------------- helpers
def pct(x: np.ndarray | list[float], q: float) -> float:
    x = np.asarray(x, dtype=float)
    return float(np.percentile(x, q)) if x.size else float("nan")


def stats_ms(x: np.ndarray | list[float]) -> dict[str, float]:
    x = np.asarray(x, dtype=float)
    return {"n": int(x.size), "mean": float(x.mean()) if x.size else float("nan"), "p50": pct(x, 50), "p95": pct(x, 95), "p99": pct(x, 99),
            "max": float(x.max()) if x.size else float("nan")}


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=1, default=float), encoding="utf-8")
    print(f"wrote {path}")


def set_threads(threads: int, cpu: int | None) -> None:
    torch.set_num_threads(threads)
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:  # already set in this process
        pass
    if cpu is not None and hasattr(os, "sched_setaffinity"):
        os.sched_setaffinity(0, {cpu})


def _read(path: str) -> str | None:
    try:
        return Path(path).read_text(encoding="utf-8").strip()
    except OSError:
        return None


def machine_info() -> dict[str, Any]:
    """What the numbers depend on: CPU, clock, governor, temperature,
    throttling (Pi), torch build."""
    cpuinfo = _read("/proc/cpuinfo") or ""
    fields = {}
    for ln in cpuinfo.splitlines():
        if ":" in ln:
            k, v = ln.split(":", 1)
            fields.setdefault(k.strip().lower(), v.strip())
    model = fields.get("model name") or fields.get("hardware") or fields.get("model") or platform.processor()  # x86 / Pi: "Model : Raspberry Pi 4 ..."
    info: dict[str, Any] = {
        "host": platform.node(), "machine": platform.machine(), "cpu_model": model, "board": _read("/proc/device-tree/model"),
        "cpus": os.cpu_count(), "python": platform.python_version(), "torch": torch.__version__, "numpy": np.__version__,
        "torch_threads": torch.get_num_threads(), "affinity": sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None,
        "governor": _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
        "max_freq_khz": _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq"),
        "cur_freq_khz": _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"),
        "temp_c": (float(t) / 1000.0 if (t := _read("/sys/class/thermal/thermal_zone0/temp")) else None),
        "wsl": "microsoft" in (_read("/proc/sys/kernel/osrelease") or "").lower(),
        "torch_config": torch.__config__.show().splitlines()[:40],
        "date": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    if shutil.which("vcgencmd"):  # Raspberry Pi: 0x0 = never throttled nor under-voltage since boot
        for key, cmd in (("throttled", ["vcgencmd", "get_throttled"]), ("arm_clock", ["vcgencmd", "measure_clock", "arm"])):
            try:
                info[key] = subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout.strip()
            except (OSError, subprocess.SubprocessError):
                info[key] = None
    return info


def timeit_us(fn: Callable[[], Any], min_time_s: float = 0.2, repeats: int = 7) -> float:
    """Median over ``repeats`` batches of the time per call [us]."""
    fn()
    n, t = 1, 0.0
    while True:  # calls per batch for ~min_time / repeats per batch
        t0 = time.perf_counter()
        for _ in range(n):
            fn()
        t = time.perf_counter() - t0
        if t > min_time_s / repeats or n > 1_000_000:
            break
        n *= 4
    out = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        for _ in range(n):
            fn()
        out.append((time.perf_counter() - t0) / n)
    return float(np.median(out) * 1e6)


# ------------------------------------------------------------------ replay
class Replay:
    """Recorded runs as a robot receives them (``export`` writes the file).

    Per run: scans (stamp = first sample of the revolution, ranges with inf
    for no return, exact revolution end = next stamp), raw IMU samples, the
    LiDAR / IMU calibration of the run and the ground truth (base_link pose
    and body velocity at ~114 Hz)."""

    def __init__(self, path: Path) -> None:
        z = np.load(path, allow_pickle=False)
        self.path = path
        self.meta = json.loads(str(z["meta"]))
        self.runs = self.meta["runs"]
        self.scan_t, self.scan_time, self.ranges = z["scan_t"], z["scan_time"], z["ranges"]
        self.imu_t, self.imu = z["imu_t"], z["imu"]
        self.gt_t, self.gt = z["gt_t"], z["gt"]

    def run(self, i: int) -> dict[str, Any]:
        r = self.runs[i]
        s, m, g = slice(r["scan_off"], r["scan_off"] + r["scan_n"]), slice(r["imu_off"], r["imu_off"] + r["imu_n"]), slice(r["gt_off"], r["gt_off"] + r["gt_n"])
        return {**r, "scan_t": self.scan_t[s], "scan_time": self.scan_time[s], "ranges": self.ranges[s], "imu_t": self.imu_t[m], "imu": self.imu[m],
                "gt_t": self.gt_t[g], "gt": self.gt[g]}

    def calibration(self, i: int) -> tuple[LidarCalibration, float]:
        r = self.runs[i]
        return LidarCalibration.from_profile(r["lidar_profile"]), imu_group_delay_s(r["imu_cfg"])


ICP_BACKEND = ""  # --icp-backend: "polar" (polar_icp.py, compiled) or "cdist"; empty: the checkpoint's
NET_BACKEND = ""  # --net-backend: "onnx" (onnx_net.py, ONNX Runtime) or "torch" / empty


def new_live(ckpt: Path, rp: Replay, i: int, imu_wait_s: float = 10.0) -> LiveOdometry:
    cal, delay = rp.calibration(i)
    return LiveOdometry(ckpt, cal, device="cpu", imu_wait_s=imu_wait_s, imu_rate_hz=float(rp.runs[i]["imu_cfg"].get("rate_hz", 104.0)),
                        imu_group_delay_s=delay, icp_backend=ICP_BACKEND, net_backend=NET_BACKEND)


def replay_run(live: LiveOdometry, run: dict[str, Any], on_scan: Callable[[int, list, float], None] | None = None,
               on_imu: Callable[[int, float], None] | None = None, max_scans: int | None = None) -> None:
    """Exact-end replay (as ``tests/test_live_odometry.py``): every IMU
    sample up to the end of revolution k (plus the next one, for the gyro at
    the boundary), then scan k, then ``process``. ``on_scan(k, estimates,
    seconds of process())``; ``on_imu(j, seconds of add_imu)`` after each
    ``add_imu``."""
    stamps, imu_t, imu = run["scan_t"], run["imu_t"], run["imu"]
    lag = live.imu_delay_ns
    j = 0
    n = len(stamps) if max_scans is None else min(len(stamps), max_scans)
    for k in range(n):
        t = int(stamps[k])
        end = t + int(round(run["scan_time"][k] * 1e9))
        while j < len(imu_t) and (j == 0 or imu_t[j - 1] - lag <= end):
            t0 = time.perf_counter()
            live.add_imu(int(imu_t[j]), imu[j, 3:6], imu[j, 0:3])
            if on_imu is not None:
                on_imu(j, time.perf_counter() - t0)
            j += 1
        live.add_scan(t, run["ranges"][k], float(run["scan_time"][k]), arrival_ns=end)
        t0 = time.perf_counter()
        est = live.process(now_ns=end)
        dt = time.perf_counter() - t0
        if on_scan is not None:
            on_scan(k, est, dt)


# ------------------------------------------------------ component spans
class Spans:
    """Wall time of the pipeline stages, by wrapping the functions the
    estimator calls (no change to the estimator). With ``annotate`` each
    stage is also a profiler range, so the op census can attribute every
    torch op to its stage."""

    def __init__(self, live: LiveOdometry, annotate: bool = False) -> None:
        self.acc: dict[str, float] = defaultdict(float)
        self.annotate = annotate
        self._restore: list[tuple[Any, str, Any]] = []
        for name in ("_metric", "_gyro_integral", "_gyro_profile", "_imu_between"):
            self._patch(live, name, "prep")
        self._patch(live, "_to_scan", "post")
        self._patch(live_odometry, "deskew", "post")  # the output de-skew (the ICP calls icp_odometry.deskew)
        self._patch(live.model, "forward", "network")
        if hasattr(live.model, "session"):  # ONNX Runtime: the compiled run is its own cost class, the rest is glue
            self._patch(live.model.session, "run", "net_native")
        if live.hybrid and live.icp_cfg.association == "polar":
            import polar_icp
            self._patch(live.icp, "step", "icp_total")
            self._patch(polar_icp, "step_kernel", "icp_native")  # compiled: its own cost class
        elif live.hybrid:
            self._patch(live.icp, "step", "icp_total")
            orig = icp_odometry._register

            def register(*a: Any, **k: Any) -> Any:
                return self._timed("icp_scan" if a[11] is None else "icp_map", orig, a, k)
            self._restore.append((icp_odometry, "_register", orig))
            icp_odometry._register = register

    def _timed(self, name: str, fn: Callable, a: tuple, k: dict) -> Any:
        ctx = torch.profiler.record_function(f"span::{name}") if self.annotate else None
        if ctx is not None:
            ctx.__enter__()
        t0 = time.perf_counter()
        try:
            return fn(*a, **k)
        finally:
            self.acc[name] += time.perf_counter() - t0
            if ctx is not None:
                ctx.__exit__(None, None, None)

    def _patch(self, obj: Any, attr: str, name: str) -> None:
        orig = getattr(obj, attr)
        had_instance_attr = attr in getattr(obj, "__dict__", {})
        self._restore.append((obj, attr, orig if had_instance_attr or isinstance(obj, type(sys)) else None))
        setattr(obj, attr, lambda *a, **k: self._timed(name, orig, a, k))

    def take(self, total_s: float) -> dict[str, float]:
        """Per-component ms of the last scan(s); resets the accumulator."""
        a = self.acc
        out = {"prep": a["prep"], "icp_scan": a["icp_scan"], "icp_map": a["icp_map"], "icp_native": a["icp_native"],
               "icp_other": max(0.0, a["icp_total"] - a["icp_scan"] - a["icp_map"] - a["icp_native"]),
               "network": max(0.0, a["network"] - a["net_native"]), "net_native": a["net_native"], "post": a["post"]}
        out["glue"] = max(0.0, total_s - sum(out.values()))
        self.acc = defaultdict(float)
        return {k: 1e3 * v for k, v in out.items()}

    def close(self) -> None:
        for obj, attr, orig in reversed(self._restore):
            if orig is None:
                delattr(obj, attr)
            else:
                setattr(obj, attr, orig)


# ------------------------------------------------------------- op census
def _numel(shape: Any) -> int:
    return int(np.prod(shape)) if isinstance(shape, (list, tuple)) and shape and all(isinstance(d, int) for d in shape) else 0


def _op_ws_bytes(name: str, shapes: list) -> float:
    """Working set of one operation: its largest tensor (float32 assumed),
    the output of the products included (not among the inputs). The
    distance matrix of a cdist sets it for every op of that cdist (product,
    clamp, sqrt, min)."""
    tens = [s for s in shapes if isinstance(s, (list, tuple)) and s and all(isinstance(d, int) for d in s)]
    elems = max((_numel(s) for s in tens), default=0)
    if name == "aten::bmm" and len(tens) >= 2 and len(tens[0]) == 3 and len(tens[1]) == 3:
        elems = max(elems, tens[0][0] * tens[0][1] * tens[1][2])
    elif name in ("aten::mm", "aten::addmm") and len(tens) >= 2:
        a, b = tens[-2], tens[-1]
        if len(a) == 2 and len(b) == 2:
            elems = max(elems, a[0] * b[1])
    return 4.0 * elems


def classify(name: str, flops: float, ws: float) -> str:
    """Cost class of one op (see the module docstring)."""
    if name in CONV_OPS or (flops > 0.0 and flops / max(ws, 1.0) >= COMPUTE_INTENSITY):
        return "compute"
    if ws < SMALL_WS_BYTES:
        return "small"
    return "mem_cache" if ws <= DRAM_WS_BYTES else "mem_dram"


def census(step: Callable[[], None], scans: int) -> dict[str, Any]:
    """Torch op census of ``scans`` calls of ``step`` (one scan each) under
    the profiler: per component, the number of top-level ops (called from
    Python; nested ones are counted apart) and the self time [ms/scan] of
    every class, plus the heaviest ops. The self times carry some profiler
    overhead; only the work classes (not the small ops) are used as times."""
    from torch.profiler import ProfilerActivity, profile
    with profile(activities=[ProfilerActivity.CPU], record_shapes=True, with_flops=True) as prof:
        for _ in range(scans):
            step()
    events = prof.events()
    comp = {c: {"top_ops": 0, "nested_ops": 0, "ms": defaultdict(float), "flops": 0.0} for c in COMPONENTS}
    heavy: dict[tuple[str, str, str], dict[str, float]] = {}
    max_ws = 0.0
    for ev in events:
        if ev.name.startswith("span::") or not ev.name.startswith("aten::"):
            continue
        # component = nearest span annotation; top-level = no aten parent
        parent, top, component = ev.cpu_parent, True, "glue"
        while parent is not None:
            if parent.name.startswith("span::"):
                span = parent.name[6:]
                component = "icp_other" if span == "icp_total" and component == "glue" else (span if component == "glue" else component)
                if span != "icp_total":
                    break
            elif parent.name.startswith("aten::"):
                top = False
            parent = parent.cpu_parent
        shapes = list(ev.input_shapes or [])
        ws = _op_ws_bytes(ev.name, shapes)
        flops = float(getattr(ev, "flops", 0) or 0)
        cls = classify(ev.name, flops, ws)
        c = comp[component]
        c["top_ops" if top else "nested_ops"] += 1
        # work classes are counted on the op itself (self time) whatever its depth
        c["ms"][cls] += ev.self_cpu_time_total / 1e3
        c["flops"] += flops
        if cls != "small":
            max_ws = max(max_ws, ws)
            key = (component, ev.name, str([s for s in shapes if s]))
            h = heavy.setdefault(key, {"calls": 0, "ms": 0.0, "ws_bytes": ws, "cls": cls})
            h["calls"] += 1
            h["ms"] += ev.self_cpu_time_total / 1e3
    out = {}
    for name, c in comp.items():
        out[name] = {"top_ops": c["top_ops"] / scans, "nested_ops": c["nested_ops"] / scans, "mflop": c["flops"] / scans / 1e6,
                     "ms": {k: v / scans for k, v in c["ms"].items()}}
    top = sorted(heavy.items(), key=lambda kv: -kv[1]["ms"])[:25]
    return {"scans": scans, "components": out, "max_ws_bytes": max_ws,
            "heavy_ops": [{"component": k[0], "op": k[1], "shapes": k[2], "calls_per_scan": v["calls"] / scans, "ms_per_scan": v["ms"] / scans,
                           "ws_mb": v["ws_bytes"] / MB, "class": v["cls"]} for k, v in top]}


def macs_by_hooks(model: nn.Module, run: Callable[[], Any]) -> dict[str, float]:
    """Multiply-accumulates of one forward, counted by hooks on the layers
    (the profiler does not count conv1d): Conv1d = C_out C_in/g K L_out,
    Linear = in out rows, GRU = 3 (in h + h h) per step, GRUCell the same per call."""
    acc: dict[str, float] = defaultdict(float)

    def hook(mod: nn.Module, inp: tuple, out: Any, name: str) -> None:
        if isinstance(mod, nn.Conv1d):
            acc[name] += out.shape[0] * mod.out_channels * (mod.in_channels // mod.groups) * mod.kernel_size[0] * out.shape[-1]
        elif isinstance(mod, nn.Linear):
            acc[name] += inp[0].numel() // mod.in_features * mod.in_features * mod.out_features
        elif isinstance(mod, nn.GRUCell):
            acc[name] += inp[0].shape[0] * 3 * (mod.input_size + mod.hidden_size) * mod.hidden_size
        elif isinstance(mod, nn.GRU):
            x = inp[0]
            steps = int(x.batch_sizes.sum()) if isinstance(x, nn.utils.rnn.PackedSequence) else x.shape[0] * x.shape[1]
            acc[name] += steps * 3 * (mod.input_size + mod.hidden_size) * mod.hidden_size

    handles = [m.register_forward_hook(lambda mod, i, o, n=n: hook(mod, i, o, n)) for n, m in model.named_modules()
               if isinstance(m, (nn.Conv1d, nn.Linear, nn.GRUCell, nn.GRU))]
    try:
        run()
    finally:
        for h in handles:
            h.remove()
    groups: dict[str, float] = defaultdict(float)
    for n, v in acc.items():
        groups[n.split(".")[0]] += v
    return {"total": float(sum(acc.values())), **{k: float(v) for k, v in groups.items()}}


def param_groups(model: nn.Module) -> dict[str, int]:
    out: dict[str, int] = defaultdict(int)
    for n, p in model.named_parameters():
        out[n.split(".")[0]] += p.numel()
    return {"total": int(sum(out.values())), **out}


# -------------------------------------------------------- micro-benchmarks
def _conv_stack(widths: tuple[int, ...] = PAIR_WIDTHS) -> nn.Module:
    from streaming_model import PairEncoder
    return PairEncoder(128, widths=widths).cnn.eval()


def micro_benchmarks() -> dict[str, dict[str, Any]]:
    """Time per call [us] of small kernels shaped like the estimator's, one
    or more per cost class. The ratio of the same entry on two machines is
    that class's slowdown factor."""
    g = torch.Generator().manual_seed(0)
    r = lambda *s: torch.rand(*s, generator=g)  # noqa: E731
    a1, b1 = r(1), r(1)
    idx_src, ridx, j = r(1, 1800, 2), torch.zeros(1, dtype=torch.long), torch.randint(0, 1800, (1, 360), generator=g)
    w360, jac = r(1, 360), r(1, 360, 3)
    h3 = torch.eye(3)[None] * 2.0 + 0.1 * r(1, 3, 3)
    g3 = r(1, 3, 1)
    y, tgt_scan, tgt_map = r(1, 360, 2) * 8, r(1, 360, 2) * 8, r(1, 1800, 2) * 8
    x_small, x_cache, x_dram = r(360), r(64 * 1024), r(650_000)  # 1.4 kB, 256 kB, 2.6 MB
    conv = _conv_stack()
    pair = r(1, 8, 360)
    cell = nn.GRUCell(128, 128).eval()
    hcell, xcell = r(1, 128), r(1, 128)
    lin = nn.Linear(287, 128).eval()
    xl = r(1, 287)
    samples = [(int(1e7 * (k + 1)), np.array([0.1, 0.2, 9.8, 0.0, 0.0, 0.3])) for k in range(20)]
    tq = np.linspace(0.0, 1.0, 20)
    gz = np.sin(tq)
    q = np.linspace(0.0, 1.0, 17)
    conv_flop = 2.0 * macs_by_hooks(conv, lambda: conv(pair))["total"]
    benches: dict[str, tuple[str, Callable[[], Any], float]] = {
        # dispatch: small top-level torch calls (the bulk of the ~11.6k ops per scan)
        "op_add_1": ("dispatch", lambda: torch.add(a1, b1), 0.0),
        "op_mul_360": ("dispatch", lambda: x_small * 1.5, 0.0),
        "op_where_360": ("dispatch", lambda: torch.where(x_small > 0.5, x_small, 0.0), 0.0),
        "op_cat_3x360": ("dispatch", lambda: torch.cat((x_small, x_small, x_small)), 0.0),
        "op_index_gather": ("dispatch", lambda: idx_src[ridx[:, None], j], 0.0),
        "op_einsum_jtwj": ("dispatch", lambda: torch.einsum("rn,rni,rnj->rij", w360, jac, jac), 0.0),
        "op_solve_3x3": ("dispatch", lambda: torch.linalg.solve(h3, g3), 0.0),
        # python: the interpreter and numpy between the torch calls
        "py_propagate_imu_20": ("python", lambda: propagate_imu(np.zeros(3), np.array([2.0, 0.0]), np.zeros(3), 0, samples), 0.0),
        "np_interp_17": ("python", lambda: np.interp(q, tq, gz), 0.0),
        "np_searchsorted": ("python", lambda: np.searchsorted(tq, 0.5), 0.0),
        # compute: the scan-pair convolutions (weights reused along 360 beams)
        "conv_stack_pair": ("compute", lambda: conv(pair), conv_flop),
        # mem_cache: batch-1 GEMV and mid-size element-wise work
        "gemv_grucell_128": ("mem_cache", lambda: cell(xcell, hcell), 2.0 * 3 * 256 * 128),
        "gemv_linear_287x128": ("mem_cache", lambda: lin(xl), 2.0 * 287 * 128),
        "elementwise_256k": ("mem_cache", lambda: x_cache * 1.5 + 0.5, 0.0),
        "cdist_min_360x360": ("mem_cache", lambda: torch.cdist(y, tgt_scan).min(dim=-1), 0.0),
        # mem_dram: the submap-ICP distance matrix and a 2.6 MB stream
        "cdist_min_360x1800": ("mem_dram", lambda: torch.cdist(y, tgt_map).min(dim=-1), 0.0),
        "elementwise_2p6MB": ("mem_dram", lambda: x_dram * 1.5 + 0.5, 0.0),
    }
    native = _native_bench()
    if native is not None:
        benches["polar_icp_step"] = ("native", native, 0.0)
    out = {}
    with torch.no_grad():
        for name, (cls, fn, flop) in benches.items():
            us = timeit_us(fn)
            out[name] = {"class": cls, "us": us, **({"gflops": flop / us / 1e3} if flop else {})}
    return out


def _native_bench() -> Callable[[], Any] | None:
    """One interval of the compiled polar ICP (polar_icp.py) on a synthetic
    corridor scan with a 5-keyframe map; None without numba."""
    try:
        import polar_icp as pi
    except ImportError:
        return None
    n = 360
    th = -math.pi + np.arange(n) * 2 * math.pi / n
    with np.errstate(divide="ignore"):
        r = np.minimum(np.abs(1.5 / np.sin(th)), np.abs(6.0 / np.cos(th)))
    r = np.minimum(r, 11.0)
    valid = np.ones(n, dtype=np.bool_)
    cfg = ICP_VARIANTS["submap_v4_polar"]
    geo = (torch.tensor([0.1 / n]), torch.tensor([[0.18, 0.0]]), torch.tensor([-math.pi]), torch.tensor([2 * math.pi / n]), None)
    s = pi.PolarIcpStream(cfg, geo, torch.tensor([[0.003, 0.005]]), n, torch.device("cpu"))
    sc = np.array([0.1, 0.01, 0.1, 0.1, 0.1, 0.2, 0.0, 0.5])
    prof = np.linspace(0.0, 0.01, 17)
    s.state.st[pi.ST_VX] = 2.0
    for _ in range(8):  # fill the keyframes (the car moves 0.2 m per scan)
        pi.run_step(s, r, valid, r, valid, sc, prof, prof)
        s.state.st[pi.ST_PX] += 0.3
    return lambda: pi.run_step(s, r, valid, r, valid, sc, prof, prof)


def reference_score() -> float:
    """Short fixed benchmark [us] recorded before and after a measurement:
    a drift > 5 % (turbo, thermal, other load) invalidates the run."""
    a, b = torch.rand(360), torch.rand(360)
    conv = _conv_stack()
    pair = torch.rand(1, 8, 360)
    with torch.no_grad():
        t1 = timeit_us(lambda: torch.where(a > b, a, b), 0.15)
        t2 = timeit_us(lambda: conv(pair), 0.15)
    return float(math.sqrt(t1 * t2))


def _stream_memory(stop: Any) -> None:  # background load of ``calibrate --background``
    x = np.ones(4 * 1024 * 1024)
    while not stop.is_set():
        x += 1.0


def cmd_calibrate(args: argparse.Namespace) -> None:
    set_threads(args.threads, args.cpu)
    procs, stop = [], None
    if args.background:
        import multiprocessing as mp
        stop = mp.Event()
        procs = [mp.Process(target=_stream_memory, args=(stop,), daemon=True) for _ in range(args.background)]
        for p in procs:
            p.start()
        time.sleep(1.0)
    try:
        before = reference_score()
        benches = micro_benchmarks()
        after = reference_score()
    finally:
        if stop is not None:
            stop.set()
            for p in procs:
                p.join(timeout=5)
    small = [v["us"] for v in benches.values() if v["class"] == "dispatch"]
    out = {"kind": "calibration", "machine": machine_info(), "background_streams": args.background,
           "reference_us": {"before": before, "after": after, "drift": after / before - 1.0},
           "small_op_us": float(np.median(small)), "benchmarks": benches}
    write_json(args.out or DATA_DIR / f"calibration_{args.tag}.json", out)
    if abs(after / before - 1.0) > 0.05:
        print(f"WARNING: reference drifted {100 * (after / before - 1):.1f} % during the run: repeat it")


# ----------------------------------------------------------------- profile
def profile_pipeline(ckpt: Path, rp: Replay, runs: list[int], warmup: int, census_scans: int, icp_cfg: Any = None,
                     max_scans: int | None = None) -> dict[str, Any]:
    """Wall time per scan (process(), which includes _metric) and per
    component over the replayed runs, then an op census of
    ``census_scans`` scans. ``icp_cfg`` replaces the checkpoint's ICP."""
    wall, comps, imu_cost = [], [], []
    for i in runs:
        live = new_live(ckpt, rp, i)
        if icp_cfg is not None and live.hybrid:
            live.icp_cfg = icp_cfg
            live.icp = IcpStream(icp_cfg, live.geo, torch.tensor([[live.lidar.sigma_const_m, live.lidar.sigma_prop]]), live.lidar.beams, live.device)
        spans = Spans(live)
        run = rp.run(i)
        seen = [0]

        def on_scan(k: int, est: list, dt: float) -> None:
            c = spans.take(dt)
            if est:
                seen[0] += 1
                if seen[0] > warmup:
                    wall.append(1e3 * dt)
                    comps.append(c)

        try:
            replay_run(live, run, on_scan, None, max_scans)
        finally:
            spans.close()
        imu_cost += predict_cost(live)
    comp_ms = {c: stats_ms([x[c] for x in comps]) for c in COMPONENTS}
    # IMU callback work: add_imu + predict() = c0 + c1 n [us], n = samples since the last scan
    n_arr = np.array([n for n, _ in imu_cost], dtype=float)
    t_arr = np.array([t for _, t in imu_cost], dtype=float)
    c1, c0 = np.polyfit(n_arr, t_arr, 1)
    # census on a fresh instance of the first run, after the same warm-up
    live = new_live(ckpt, rp, runs[0])
    if icp_cfg is not None and live.hybrid:
        live.icp_cfg = icp_cfg
        live.icp = IcpStream(icp_cfg, live.geo, torch.tensor([[live.lidar.sigma_const_m, live.lidar.sigma_prop]]), live.lidar.beams, live.device)
    run = rp.run(runs[0])
    it = _scan_steps(live, run)
    for _ in range(warmup + 1):
        next(it)
    spans = Spans(live, annotate=True)
    try:
        cen = census(lambda: next(it), census_scans)
    finally:
        spans.close()
    model = live.model
    if not isinstance(model, nn.Module):  # ONNX Runtime: the same weights, counted on the checkpoint
        from streaming_model import load_checkpoint  # noqa: PLC0415
        model = load_checkpoint(ckpt, "cpu")[0]
    params = param_groups(model)
    return {"wall_ms": stats_ms(wall), "wall_ms_trace": [round(x, 3) for x in wall], "components_ms": comp_ms,
            "imu_callback_us": {"c0": float(c0), "c1_per_sample": float(c1), "points": [[float(n), float(t)] for n, t in zip(n_arr, t_arr)]},
            "census": cen, "params": params, "hybrid": live.hybrid,
            "icp": None if live.icp_cfg is None else {k: getattr(live.icp_cfg, k) for k in ("iterations", "deskew_passes", "map_iterations", "submap_keyframes")}}


def predict_cost(live: LiveOdometry, ns: tuple[int, ...] = (0, 5, 10, 15, 20, 25, 30, 40)) -> list[tuple[int, float]]:
    """Work of one IMU callback of the node (``add_imu`` + ``predict``) on the
    estimator's final state (a full 3 s IMU buffer), with the last scan
    stamp placed n samples back: [(n, us)]. The copy keeps ``live`` intact."""
    t = list(live._imu_t)
    v = list(live._imu_v)
    out = []
    saved = live.last_stamp_ns, live._imu_t, live._imu_v
    try:
        for n in ns:
            if n + 2 > len(t):
                continue
            def call() -> None:
                live._imu_t, live._imu_v = deque(t[:-1]), deque(v[:-1])
                live.last_stamp_ns = t[-2 - n]
                t0 = time.perf_counter()
                live.add_imu(t[-1] + live.imu_delay_ns, v[-1][3:6], v[-1][0:3])
                live.predict()
                acc.append(time.perf_counter() - t0)
            acc: list[float] = []
            for _ in range(200):
                call()
            out.append((n, 1e6 * float(np.median(acc[20:]))))
    finally:
        live.last_stamp_ns, live._imu_t, live._imu_v = saved
    return out


def _scan_steps(live: LiveOdometry, run: dict[str, Any]) -> Iterator[None]:
    """One replayed scan (that yields an estimate) per ``next``."""
    stamps, imu_t, imu = run["scan_t"], run["imu_t"], run["imu"]
    lag, j = live.imu_delay_ns, 0
    for k in range(len(stamps)):
        t = int(stamps[k])
        end = t + int(round(run["scan_time"][k] * 1e9))
        while j < len(imu_t) and (j == 0 or imu_t[j - 1] - lag <= end):
            live.add_imu(int(imu_t[j]), imu[j, 3:6], imu[j, 0:3])
            j += 1
        live.add_scan(t, run["ranges"][k], float(run["scan_time"][k]), arrival_ns=end)
        if live.process(now_ns=end):
            yield None
    raise RuntimeError("replay exhausted during the census: use a longer run or fewer census scans")


def cmd_profile(args: argparse.Namespace) -> None:
    set_threads(args.threads, args.cpu)
    rp = Replay(args.replay)
    runs = args.runs or list(range(len(rp.runs)))
    before = reference_score()
    t0 = time.time()
    prof = profile_pipeline(args.ckpt, rp, runs, args.warmup, args.census_scans)
    after = reference_score()
    out = {"kind": "profile", "machine": machine_info(), "checkpoint": str(args.ckpt), "replay": str(args.replay), "icp_backend": ICP_BACKEND or "checkpoint",
           "net_backend": NET_BACKEND or "torch",
           "runs": [rp.runs[i]["run_key"] for i in runs], "threads": args.threads, "seconds": time.time() - t0,
           "reference_us": {"before": before, "after": after, "drift": after / before - 1.0}, **prof}
    write_json(args.out or DATA_DIR / f"profile_{args.tag}.json", out)
    w = prof["wall_ms"]
    print(f"per scan: p50 {w['p50']:.1f} ms, p95 {w['p95']:.1f}, p99 {w['p99']:.1f}, max {w['max']:.1f} (n={w['n']}); "
          f"top-level ops {sum(c['top_ops'] for c in prof['census']['components'].values()):.0f}/scan")
    for c in COMPONENTS:
        print(f"  {c:9s} {prof['components_ms'][c]['mean']:6.2f} ms")
    if abs(after / before - 1.0) > 0.05:
        print(f"WARNING: reference drifted {100 * (after / before - 1):.1f} % during the run: repeat it")


# ------------------------------------------------------------------- sweep
def capture_batches(ckpt: Path, rp: Replay, run_index: int, n: int) -> list[dict[str, torch.Tensor]]:
    """The network inputs of ``n`` consecutive real scans (ICP included)."""
    live = new_live(ckpt, rp, run_index)
    batches: list[dict[str, torch.Tensor]] = []
    orig = live.model.forward

    def grab(batch: dict[str, torch.Tensor], state: dict[str, torch.Tensor]) -> Any:
        batches.append({k: v.clone() for k, v in batch.items()})
        return orig(batch, state)
    live.model.forward = grab
    it = _scan_steps(live, rp.run(run_index))
    while len(batches) < n:
        next(it)
    return batches


def network_point(model: StreamingPoseNet, batches: list[dict[str, torch.Tensor]], warmup: int, census_n: int) -> dict[str, Any]:
    """Latency, census, parameters and MACs of one network on real inputs."""
    model.eval()
    state = model.initial_state(1, torch.device("cpu"))
    times = []
    with torch.no_grad():
        for k, b in enumerate(batches):
            t0 = time.perf_counter()
            _, state = model(b, state)
            if k >= warmup:
                times.append(1e3 * (time.perf_counter() - t0))
        it = iter(batches * 2)
        st = [state]

        def step() -> None:
            with torch.profiler.record_function("span::network"):
                _, st[0] = model(next(it), st[0])
        cen = census(step, census_n)
        macs = macs_by_hooks(model, lambda: model(batches[0], model.initial_state(1, torch.device("cpu"))))
    c = cen["components"]["network"]
    return {"params": param_groups(model), "macs": macs, "ms": stats_ms(times), "top_ops": c["top_ops"], "nested_ops": c["nested_ops"],
            "class_ms": c["ms"], "max_ws_mb": cen["max_ws_bytes"] / MB}


ICP_KNOBS = {  # latency only: every ICP change alters the hybrid's inputs (retraining)
    "baseline": {},
    "gn_iterations_4": {"iterations": 4},
    "deskew_passes_1": {"deskew_passes": 1},
    "map_iterations_2": {"map_iterations": 2},
    "keyframes_3": {"submap_keyframes": 3},
    "keyframes_2": {"submap_keyframes": 2},
    "scan_to_scan_only": {"submap_keyframes": 0},
    "lean_combo": {"iterations": 4, "deskew_passes": 1, "map_iterations": 2, "submap_keyframes": 3},
}


def cmd_sweep(args: argparse.Namespace) -> None:
    set_threads(args.threads, args.cpu)
    rp = Replay(args.replay)
    base_model, ck = load_checkpoint(args.ckpt, "cpu")
    cfg = ck["config"]
    before = reference_score()
    out: dict[str, Any] = {"kind": "sweep", "machine": machine_info(), "checkpoint": str(args.ckpt), "threads": args.threads}
    if "network" in args.parts:
        batches = capture_batches(args.ckpt, rp, args.runs[0] if args.runs else 0, args.batches)
        points = []
        grid = [(h, w) for w in args.widths for h in args.hidden]
        for h, w in grid:
            torch.manual_seed(0)
            m = StreamingPoseNet(h, hybrid=True, sweep_profile=bool(cfg.get("sweep_profile", False)), pair_widths=w)
            p = network_point(m, batches, args.warmup, args.census_scans)
            p.update(hidden=h, pair_widths=list(w), trained=False)
            points.append(p)
            print(f"  hidden {h:4d} widths {str(w):22s} params {p['params']['total']:9,d}  MACs {p['macs']['total'] / 1e6:7.2f} M  "
                  f"{p['ms']['p50']:6.2f} ms  top ops {p['top_ops']:.0f}")
        base = network_point(base_model, batches, args.warmup, args.census_scans)
        base.update(hidden=int(cfg.get("hidden", 128)), pair_widths=list(base_model.pair_widths), trained=True)
        out["network"] = {"batches": len(batches), "points": points, "checkpoint_point": base}
    if "icp" in args.parts:
        runs = args.runs or [0]
        variant = ICP_VARIANTS[cfg.get("icp_variant", "scan")]
        res = {}
        for name, knob in ICP_KNOBS.items():
            icp_cfg = replace(variant, **knob)
            p = profile_pipeline(args.ckpt, rp, runs, args.warmup, args.census_scans, icp_cfg=icp_cfg, max_scans=args.max_scans)
            p.pop("wall_ms_trace")
            res[name] = {"knobs": knob, **p}
            print(f"  ICP {name:18s} {p['wall_ms']['p50']:6.2f} ms p50  (icp_scan {p['components_ms']['icp_scan']['mean']:.2f}, "
                  f"icp_map {p['components_ms']['icp_map']['mean']:.2f}, network {p['components_ms']['network']['mean']:.2f})")
        # no ICP at all: a pure (non-hybrid) network of the same size, random weights
        with tempfile.TemporaryDirectory() as tmp:
            pure = StreamingPoseNet(int(cfg.get("hidden", 128)), hybrid=False, sweep_profile=bool(cfg.get("sweep_profile", False)))
            path = Path(tmp) / "pure.pt"
            torch.save({"model": pure.state_dict(), "config": {**cfg, "hybrid": False}}, path)
            p = profile_pipeline(path, rp, runs, args.warmup, args.census_scans, max_scans=args.max_scans)
            p.pop("wall_ms_trace")
            res["no_icp_pure_network"] = {"knobs": {"hybrid": False}, **p}
            print(f"  ICP {'none (pure net)':18s} {p['wall_ms']['p50']:6.2f} ms p50")
        out["icp"] = res
    after = reference_score()
    out["reference_us"] = {"before": before, "after": after, "drift": after / before - 1.0}
    write_json(args.out or DATA_DIR / f"sweep_{args.tag}.json", out)


# -------------------------------------------------------------------- live
def cmd_live(args: argparse.Namespace) -> None:
    """Real-time replay with the node's threading: one executor thread
    delivers the IMU samples (add_imu + predict, as ``_on_imu``) and the
    scans at their arrival time; a worker thread runs ``process`` (as
    ``_scan_worker``). ``--scale k`` busy-waits (k - 1) x the compute of
    each scan in Python, holding the GIL, to emulate a k-times slower CPU."""
    set_threads(args.threads, args.cpu)
    warm = None
    if args.keep_warm:  # a nice-19 spinner on the same CPU: the core never idles, so its clock never drops between scans
        if args.cpu is None:
            raise SystemExit("--keep-warm needs --cpu")
        warm = subprocess.Popen(["taskset", "-c", str(args.cpu), "nice", "-n", "19", sys.executable, "-c", "while True: pass"])
        time.sleep(1.0)
    rp = Replay(args.replay)
    i = args.runs[0] if args.runs else 0
    run = rp.run(i)
    live = new_live(args.ckpt, rp, i, imu_wait_s=0.03)
    t_data0 = int(run["scan_t"][0])
    speed = args.speed
    t_wall0 = time.perf_counter_ns() + 200_000_000

    def data_now() -> int:
        return t_data0 + int((time.perf_counter_ns() - t_wall0) * speed)

    # merged event list: (arrival data time, kind, index)
    events = [(int(t) + int(args.imu_transport_ms * 1e6), 0, j) for j, t in enumerate(run["imu_t"])]
    ends = run["scan_t"] + np.round(run["scan_time"] * 1e9).astype(np.int64)
    events += [(int(e) + int(args.lidar_publish_ms * 1e6), 1, k) for k, e in enumerate(ends)]
    events.sort()
    t_stop = t_data0 + int(args.seconds * 1e9)
    wake, done = threading.Event(), threading.Event()
    lat, service, pend, cb_delay, cb_cost = [], [], [], [], []
    arrival: dict[int, int] = {}

    def executor() -> None:
        for t_evt, kind, idx in events:
            if t_evt > t_stop:
                break
            while (d := t_evt - data_now()) > 0:  # one wake per message, as the rclpy executor (no polling)
                time.sleep(d / speed * 1e-9)
            now = data_now()
            t0 = time.perf_counter()
            if kind == 0:
                live.add_imu(int(run["imu_t"][idx]), run["imu"][idx, 3:6], run["imu"][idx, 0:3])
                wake.set()
                live.predict()
                cb_delay.append((now - t_evt) * 1e-6)
                cb_cost.append(1e3 * (time.perf_counter() - t0))
            else:
                t_scan = int(run["scan_t"][idx])
                arrival[t_scan] = now
                live.add_scan(t_scan, run["ranges"][idx], float(run["scan_time"][idx]), arrival_ns=now)
                wake.set()
        done.set()

    def worker() -> None:
        while not done.is_set() or live._pending:
            wake.wait(0.005)
            wake.clear()
            pend.append(len(live._pending))
            t0 = time.perf_counter()
            ests = live.process(data_now())
            s = time.perf_counter() - t0
            if ests and args.scale > 1.0:
                t_end = time.perf_counter() + (args.scale - 1.0) * s
                while time.perf_counter() < t_end:  # pure Python: holds the GIL
                    pass
            now = data_now()
            for e in ests:
                lat.append((now - e.stamp_ns) * 1e-6)
                service.append(1e3 * (time.perf_counter() - t0) / max(1, len(ests)))
            if done.is_set() and not live._pending:
                break

    th = [threading.Thread(target=executor, daemon=True), threading.Thread(target=worker, daemon=True)]
    for t in th:
        t.start()
    for t in th:
        t.join()
    if warm is not None:
        warm.kill()
    out = {"kind": "live", "machine": machine_info(), "checkpoint": str(args.ckpt), "run": run["run_key"], "threads": args.threads,
           "scale": args.scale, "speed": speed, "seconds": args.seconds, "keep_warm": bool(args.keep_warm), "icp_backend": ICP_BACKEND or "checkpoint",
           "net_backend": NET_BACKEND or "torch",
           "latency_ms": stats_ms(lat[args.warmup:]), "service_ms": stats_ms(service[args.warmup:]), "service_ms_trace": [round(x, 3) for x in service],
           "pending_max": int(max(pend) if pend else 0), "imu_callback_delay_ms": stats_ms(cb_delay), "imu_callback_ms": stats_ms(cb_cost)}
    write_json(args.out or DATA_DIR / f"live_{args.tag}.json", out)
    print(f"latency p50 {out['latency_ms']['p50']:.1f} p95 {out['latency_ms']['p95']:.1f} p99 {out['latency_ms']['p99']:.1f} ms; "
          f"service p50 {out['service_ms']['p50']:.1f} ms; pending max {out['pending_max']}; "
          f"IMU callback delay p99 {out['imu_callback_delay_ms']['p99']:.2f} ms")


# ---------------------------------------------------------------------- dr
DR_AGES_S = tuple(round(0.10 + 0.02 * k, 2) for k in range(36))  # 0.10 ... 0.80 s


def _interp_pose(gt_t: np.ndarray, gt: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Ground truth (x, y, unwrapped yaw, vx_body, vy_body) at the times ``t``."""
    return np.stack([np.interp(t, gt_t, gt[:, c]) for c in range(gt.shape[1])], axis=-1)


def dr_run(args: tuple[str, str, int, tuple[float, ...]]) -> dict[str, Any]:
    """Error of the control pose against its age A for one run.

    For every estimate (scan k, stamp t_k) and age A, the control pose at
    tau = t_k + A is what the driver computes at that instant: the estimate
    propagated with the IMU samples received by tau (``propagate_imu``,
    stamps moved back by the chip delay as the estimator does), then
    advanced at constant twist to tau (``race_driver_node._pose_now``). It
    is compared with the ground-truth motion over [t_k, tau] in the frame of
    scan k (local increments: the long-term drift, corrected by the SLAM,
    cancels). Cross-track = across the true direction of travel at tau."""
    ckpt, replay_path, i, ages = args
    torch.set_num_threads(1)
    rp = Replay(Path(replay_path))
    run = rp.run(i)
    live = new_live(Path(ckpt), rp, i)
    lag = live.imu_delay_ns
    imu_t_raw, imu = run["imu_t"], run["imu"]
    imu_t_shift = imu_t_raw - lag
    gt_t, gt = run["gt_t"].astype(np.float64), run["gt"].copy()
    gt[:, 2] = np.unwrap(gt[:, 2])
    ages_ns = (np.asarray(ages) * 1e9).astype(np.int64)
    rows: list[np.ndarray] = []  # speed, then (xt, along, yaw) per age
    count = [0]

    def on_scan(k: int, ests: list, dt: float) -> None:
        for e in ests:
            count[0] += 1
            if count[0] <= 10:  # ready_scans of the node: IMU bias settling
                continue
            t_k = e.stamp_ns
            taus = t_k + ages_ns
            truth = _interp_pose(gt_t, gt, np.concatenate(([t_k], taus)).astype(np.float64))
            p0 = truth[0]
            speed = float(math.hypot(p0[3], p0[4]))
            c0, s0 = math.cos(p0[2]), math.sin(p0[2])
            ce, se = math.cos(e.pose[2]), math.sin(e.pose[2])
            lo = int(np.searchsorted(imu_t_shift, t_k, side="right"))
            row = [speed]
            for a, tau in enumerate(taus):
                hi = int(np.searchsorted(imu_t_raw, tau, side="right"))  # received by tau
                samples = [(int(imu_t_shift[m]), imu[m]) for m in range(lo, hi)]
                t_last, pose, v, rate = propagate_imu(e.pose, e.velocity, e.bias, t_k, samples)
                age = max(0.0, (tau - t_last) * 1e-9)
                mid = pose[2] + 0.5 * rate * age
                x = pose[0] + v[0] * age * math.cos(mid)  # driver: twist.linear.x only
                y = pose[1] + v[0] * age * math.sin(mid)
                yaw = pose[2] + rate * age
                dx_e, dy_e = x - e.pose[0], y - e.pose[1]
                ex, ey = ce * dx_e + se * dy_e, -se * dx_e + ce * dy_e  # estimated increment, frame of scan k
                pt = truth[1 + a]
                dx_t, dy_t = pt[0] - p0[0], pt[1] - p0[1]
                tx, ty = c0 * dx_t + s0 * dy_t, -s0 * dx_t + c0 * dy_t
                dpsi = pt[2] - p0[2]
                ux, uy = math.cos(dpsi), math.sin(dpsi)
                row += [(ex - tx) * -uy + (ey - ty) * ux, (ex - tx) * ux + (ey - ty) * uy, (yaw - e.pose[2]) - dpsi]
            rows.append(np.array(row))

    replay_run(live, run, on_scan)
    return {"run_key": run["run_key"], "rows": np.array(rows)}


def cmd_dr(args: argparse.Namespace) -> None:
    import multiprocessing as mp
    rp = Replay(args.replay)
    runs = args.runs or list(range(len(rp.runs)))
    ages = DR_AGES_S
    t0 = time.time()
    with mp.get_context("spawn").Pool(args.workers) as pool:
        res = pool.map(dr_run, [(str(args.ckpt), str(args.replay), i, ages) for i in runs])
    rows = np.concatenate([r["rows"] for r in res if len(r["rows"])])
    speed = rows[:, 0]
    err = rows[:, 1:].reshape(len(rows), len(ages), 3)
    strata = {"all_moving": speed > 0.3, "v_0p3_1p0": (speed > 0.3) & (speed < 1.0), "v_1p0_2p5": (speed >= 1.0) & (speed < 2.5),
              "v_ge_2p5": speed >= 2.5, "v_ge_3p0": speed >= 3.0}
    out: dict[str, Any] = {"kind": "dr", "checkpoint": str(args.ckpt), "replay": str(args.replay), "runs": [r["run_key"] for r in res],
                           "ages_s": list(ages), "seconds": time.time() - t0, "strata": {}}
    for name, m in strata.items():
        e = err[m]
        out["strata"][name] = {"n": int(m.sum()), "speed_mean": float(speed[m].mean()) if m.any() else float("nan"),
                               **{f"{q}_{k}": [pct(np.abs(e[:, a, j]), p) for a in range(len(ages))]
                                  for j, k in enumerate(("xt_m", "along_m", "yaw_rad")) for q, p in (("p50", 50), ("p95", 95), ("p99", 99))}}
    write_json(args.out or DATA_DIR / "dr_curves.json", out)
    for name, s in out["strata"].items():
        if s["n"]:
            print(f"  {name:12s} n={s['n']:6d}  v={s['speed_mean']:.2f} m/s  xt p95 at A=0.2/0.3/0.5 s: "
                  f"{100 * s['p95_xt_m'][5]:.1f} / {100 * s['p95_xt_m'][10]:.1f} / {100 * s['p95_xt_m'][20]:.1f} cm")


# ------------------------------------------------------------------ simref
def cmd_simref(args: argparse.Namespace) -> None:
    """Latency (scan stamp -> estimate, sim clock) of the closed-loop runs
    that validated the race margins, from their estimator.csv (the first
    ``ready`` estimates are the start-up and are left out)."""
    import csv
    lat, per_run = [], {}
    for d in sorted(SIM_RUNS.glob(args.glob)):
        f = d / "estimator.csv"
        if not f.exists():
            continue
        with open(f, newline="", encoding="utf-8") as fh:
            vals = [float(r["latency_ms"]) for r in csv.DictReader(fh) if r.get("latency_ms")]
        vals = vals[args.skip:]
        if vals:
            per_run[d.name] = stats_ms(vals)
            lat += vals
    # CPU of the lap-1 reactive planner, which shares the Pi: react_ms of the
    # driver per phase (PC wall clock under Gazebo, logged at each control tick)
    import csv
    react: dict[str, list[float]] = defaultdict(list)
    for d in sorted(SIM_RUNS.glob(args.glob)):
        f = d / "driver.csv"
        if f.exists():
            with open(f, newline="", encoding="utf-8") as fh:
                for r in csv.DictReader(fh):
                    if r.get("react_ms") and float(r["react_ms"]) > 0.0:
                        react[r["phase"]].append(float(r["react_ms"]))
    out = {"kind": "simref", "glob": args.glob, "skip": args.skip, "latency_ms": stats_ms(lat), "runs": per_run,
           "react_ms": {k: stats_ms(v) for k, v in react.items()}}
    write_json(args.out or DATA_DIR / "simref.json", out)
    s = out["latency_ms"]
    print(f"{len(per_run)} runs: latency p50 {s['p50']:.1f} p95 {s['p95']:.1f} p99 {s['p99']:.1f} ms")


# ---------------------------------------------------------- export, bundle
def cmd_export(args: argparse.Namespace) -> None:
    import sqlite3
    tools = HERE.parents[1] / "tools"
    sys.path.insert(0, str(tools))
    from pose_dataset.blobs import decode_mask, decode_ranges
    conn = sqlite3.connect(f"file:{args.db}?mode=ro", uri=True)
    keys = [r[0] for r in conn.execute("SELECT run_key FROM runs WHERE split = ? AND status = 'completed' ORDER BY run_key", (args.split,))]
    if args.match:
        keys = [k for k in keys if any(m in k for m in args.match)]
    runs, st, stime, rng, it, iv, gt_t, gt = [], [], [], [], [], [], [], []
    so = io = go = 0
    for key in keys:
        run_id = conn.execute("SELECT run_id FROM runs WHERE run_key = ?", (key,)).fetchone()[0]
        scans = conn.execute("SELECT timestamp_ns, beam_count, ranges_encoding, ranges_blob, valid_mask_blob FROM lidar_scans WHERE run_id = ? ORDER BY seq",
                             (run_id,)).fetchall()
        imu = np.array(conn.execute("SELECT timestamp_ns, ax, ay, az, gx, gy, gz FROM imu_samples WHERE run_id = ? ORDER BY seq", (run_id,)).fetchall(),
                       dtype=np.float64)
        truth = np.array(conn.execute("SELECT timestamp_ns, x, y, yaw, vx_body, vy_body FROM ground_truth WHERE run_id = ? ORDER BY timestamp_ns",
                                      (run_id,)).fetchall(), dtype=np.float64)
        ext, noise = conn.execute("SELECT extrinsic_translation, noise_configuration_json FROM sensor_calibration WHERE run_id = ? AND sensor_name = 'lidar'",
                                  (run_id,)).fetchone()
        imu_cfg = json.loads(conn.execute("SELECT noise_configuration_json FROM sensor_calibration WHERE run_id = ? AND sensor_name = 'imu'",
                                          (run_id,)).fetchone()[0])
        prof = json.loads(noise)
        prof["extrinsic"] = {"xyz": json.loads(ext)}
        stamps = np.array([s[0] for s in scans], dtype=np.int64)
        rate = float(prof["rate_hz"])
        # exact end of revolution k = stamp k+1 (as the stored gyro rates)
        scan_time = np.append(np.diff(stamps) * 1e-9, 1.0 / rate)
        ranges = np.stack([np.where(decode_mask(m, b), decode_ranges(blob, enc, b), np.inf).astype(np.float32) for _, b, enc, blob, m in scans])
        runs.append({"run_key": key, "lidar_profile": prof, "imu_cfg": imu_cfg, "scan_off": so, "scan_n": len(stamps), "imu_off": io,
                     "imu_n": len(imu), "gt_off": go, "gt_n": len(truth)})
        so, io, go = so + len(stamps), io + len(imu), go + len(truth)
        st.append(stamps), stime.append(scan_time), rng.append(ranges)
        it.append(imu[:, 0].astype(np.int64)), iv.append(imu[:, 1:])
        gt_t.append(truth[:, 0].astype(np.int64)), gt.append(truth[:, 1:])
        print(f"  {key}: {len(stamps)} scans, {len(imu)} IMU, {len(truth)} truth")
    conn.close()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, meta=json.dumps({"db": str(args.db), "split": args.split, "runs": runs}), scan_t=np.concatenate(st),
                        scan_time=np.concatenate(stime), ranges=np.concatenate(rng), imu_t=np.concatenate(it), imu=np.concatenate(iv),
                        gt_t=np.concatenate(gt_t), gt=np.concatenate(gt))
    print(f"wrote {args.out} ({len(runs)} runs, {so} scans, {args.out.stat().st_size / MB:.1f} MB)")


BUNDLE_SOURCES = ("latency_profile.py", "pi4_budget.py", "ros_callback_bench.py", "polar_icp.py", "onnx_net.py", "live_odometry.py", "icp_odometry.py", "sqlite_streams.py",
                  "sqlite_windows.py", "streaming_model.py", "train_pose_fusion.py")
PI_INSTRUCTIONS = """# Pi 4 calibration of the odometry compute budget

Needs a 64-bit OS, Python 3.12 and the same torch as the PC (2.11, CPU wheel
for aarch64) plus numpy, numba (the compiled ICP) and onnxruntime (the exported network):
`python3 -m venv venv && venv/bin/pip install torch numpy numba onnxruntime`.
The first run compiles the polar ICP (cached in __pycache__ afterwards).
Stop the car's stack first (nothing else loading the CPU), fan on.

    tar xzf pi4_bundle.tar.gz && cd simulation/learning/lidar_imu_pose
    sudo sh -c 'echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor'   # optional, recorded either way
    ../../../venv/bin/python latency_profile.py calibrate --cpu 3 --tag pi4
    ../../../venv/bin/python latency_profile.py calibrate --cpu 3 --tag pi4_bg --background 3
    ../../../venv/bin/python latency_profile.py profile --cpu 3 --tag pi4 --runs 0 1 5 6      # ~1400 scans, ~5-10 min
    ../../../venv/bin/python latency_profile.py profile --cpu 3 --tag pi4_polar --icp-backend polar --runs 0 1 5 6
    ../../../venv/bin/python latency_profile.py profile --cpu 3 --tag pi4_polar_onnx --icp-backend polar --net-backend onnx --runs 0 1 5 6
    ../../../venv/bin/python latency_profile.py live --cpu 3 --tag pi4_polar_onnx_warm --icp-backend polar --net-backend onnx --runs 0 --seconds 25 --keep-warm
    ../../../venv/bin/python latency_profile.py live --cpu 3 --tag pi4 --runs 0 --seconds 25
    # inside the car's ROS Docker image (rclpy, no torch needed):
    python3 ros_callback_bench.py --tag pi4

Bring back `config/pi4_budget/*_pi4*.json` (and `ros_pi4.json`) to the PC and run
`python pi4_budget.py report`: the measured factors replace the spec band.
"""


def cmd_bundle(args: argparse.Namespace) -> None:
    """Archive for the Pi, with the directory layout the imports expect:
    simulation/learning/lidar_imu_pose/*.py, simulation/tools/pose_dataset/blobs.py."""
    sim = HERE.parents[1]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(args.out, "w:gz") as tar:
        for name in BUNDLE_SOURCES:
            tar.add(HERE / name, arcname=f"simulation/learning/lidar_imu_pose/{name}")
        for name in ("__init__.py", "blobs.py"):
            tar.add(sim / "tools" / "pose_dataset" / name, arcname=f"simulation/tools/pose_dataset/{name}")
        tar.add(args.replay, arcname="simulation/learning/outputs/pi4_budget/replay_pi.npz")
        tar.add(args.ckpt, arcname="simulation/learning/outputs/real2sim_v5_10hz/hybrid_submap_v4/best_model.pt")
        onnx_file = Path(args.ckpt).with_suffix(".onnx")  # onnx_net.py export (onnxruntime runs it; no onnx needed on the Pi)
        if onnx_file.exists():
            tar.add(onnx_file, arcname="simulation/learning/outputs/real2sim_v5_10hz/hybrid_submap_v4/best_model.onnx")
        pc = sorted(DATA_DIR.glob("*.json"))
        for f in pc:
            tar.add(f, arcname=f"simulation/learning/lidar_imu_pose/config/pi4_budget/{f.name}")
        import io
        info = tarfile.TarInfo("RUN_ON_PI.md")
        data = PI_INSTRUCTIONS.encode()
        info.size = len(data)
        tar.addfile(info, io.BytesIO(data))
    print(f"wrote {args.out} ({args.out.stat().st_size / MB:.1f} MB)")


# --------------------------------------------------------------------- CLI
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p: argparse.ArgumentParser, replay: bool = True) -> None:
        p.add_argument("--ckpt", type=Path, default=DEFAULT_CKPT)
        if replay:
            p.add_argument("--replay", type=Path, default=REPLAY_DIR / "replay_pi.npz")
            p.add_argument("--runs", type=int, nargs="*", default=[], help="run indices in the replay (default: all)")
        p.add_argument("--threads", type=int, default=1)
        p.add_argument("--cpu", type=int, default=None, help="pin to this CPU (avoid migrations; WSL: the P-core vCPUs)")
        p.add_argument("--tag", default=platform.node() or "host", help="suffix of the output JSON (e.g. pc, pi4)")
        p.add_argument("--icp-backend", choices=("", "cdist", "polar"), default="", help="nearest-neighbour search of the ICP (polar: polar_icp.py)")
        p.add_argument("--net-backend", choices=("", "torch", "onnx"), default="", help="network runtime (onnx: onnx_net.py, <checkpoint>.onnx)")
        p.add_argument("--out", type=Path, default=None)

    p = sub.add_parser("export", help="dataset runs -> replay .npz")
    p.add_argument("--db", type=Path, default=DEFAULT_DB)
    p.add_argument("--split", default="validation")
    p.add_argument("--match", nargs="*", default=[], help="keep run keys containing any of these")
    p.add_argument("--out", type=Path, default=REPLAY_DIR / "replay_validation.npz")
    p.set_defaults(fn=cmd_export)
    p = sub.add_parser("bundle", help="tar.gz for the Pi")
    p.add_argument("--ckpt", type=Path, default=DEFAULT_CKPT)
    p.add_argument("--replay", type=Path, default=REPLAY_DIR / "replay_pi.npz")
    p.add_argument("--out", type=Path, default=REPLAY_DIR / "pi4_bundle.tar.gz")
    p.set_defaults(fn=cmd_bundle)
    p = sub.add_parser("calibrate", help="micro-benchmarks of the cost classes")
    common(p, replay=False)
    p.add_argument("--background", type=int, default=0, help="memory-streaming processes during the benchmarks (L2 / DRAM contention)")
    p.set_defaults(fn=cmd_calibrate)
    p = sub.add_parser("profile", help="replay through LiveOdometry: time per scan and per component, op census")
    common(p)
    p.add_argument("--warmup", type=int, default=20)
    p.add_argument("--census-scans", type=int, default=40)
    p.set_defaults(fn=cmd_profile)
    p = sub.add_parser("sweep", help="latency against network size and ICP settings")
    common(p)
    p.add_argument("--parts", nargs="*", default=["network", "icp"], choices=("network", "icp"))
    p.add_argument("--hidden", type=int, nargs="*", default=[32, 64, 96, 128, 192, 256, 384, 512])
    p.add_argument("--widths", type=lambda s: tuple(int(v) for v in s.split(",")), nargs="*",
                   default=[(16, 24, 32, 48), (24, 32, 48, 64), (32, 48, 64, 96), (48, 64, 96, 128), (64, 96, 128, 192), (96, 128, 192, 256)])
    p.add_argument("--batches", type=int, default=240)
    p.add_argument("--warmup", type=int, default=20)
    p.add_argument("--census-scans", type=int, default=30)
    p.add_argument("--max-scans", type=int, default=260, help="ICP part: scans replayed per variant")
    p.set_defaults(fn=cmd_sweep)
    p = sub.add_parser("live", help="real-time replay with the node's threads")
    common(p)
    p.add_argument("--scale", type=float, default=1.0, help="emulate a k-times slower CPU (busy-wait holding the GIL)")
    p.add_argument("--speed", type=float, default=1.0, help="replay speed (1 = real time)")
    p.add_argument("--seconds", type=float, default=45.0)
    p.add_argument("--warmup", type=int, default=10)
    p.add_argument("--imu-transport-ms", type=float, default=1.0)
    p.add_argument("--lidar-publish-ms", type=float, default=3.0)
    p.add_argument("--keep-warm", action="store_true", help="keep the pinned core busy with a nice-19 spinner (no clock drop between scans)")
    p.set_defaults(fn=cmd_live)
    p = sub.add_parser("dr", help="error of the IMU-propagated control pose against its age")
    common(p)
    p.set_defaults(replay=REPLAY_DIR / "replay_validation.npz")
    p.add_argument("--workers", type=int, default=4)
    p.set_defaults(fn=cmd_dr)
    p = sub.add_parser("simref", help="latency of the validated closed-loop sim runs")
    p.add_argument("--glob", default="fin_s*")
    p.add_argument("--skip", type=int, default=20, help="start-up estimates left out per run")
    p.add_argument("--out", type=Path, default=None)
    p.set_defaults(fn=cmd_simref)
    args = ap.parse_args()
    global ICP_BACKEND, NET_BACKEND
    ICP_BACKEND = getattr(args, "icp_backend", "")
    NET_BACKEND = getattr(args, "net_backend", "")
    args.fn(args)


if __name__ == "__main__":
    main()
