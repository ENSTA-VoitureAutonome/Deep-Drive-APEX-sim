#!/usr/bin/env python3
"""Pi 4 compute budget of the learned odometry: the design bound and how it
is derived (README section 10).

The simulation runs the estimator on the PC, where one scan costs ~28 ms
against a 100 ms scan period: the latency it adds is harmless there. On the
car's Raspberry Pi 4 the same code is several times slower. This module
turns the measurements of ``latency_profile.py`` into the requirement a
model must meet on the Pi, in six steps. Every constant carries its source:

  [ds-...]      datasheet            [code:file:line]  this repository
  [meas-...]    a measurement JSON   [user]            decided with the user
  [assumption]  an engineering choice, to replace by a measurement when possible

1. Projection PC -> Pi (``project``). A scan's time on the PC splits into
   cost classes (``latency_profile`` docstring): compute, mem_cache,
   mem_dram (from the op census), dispatch (top-level small ops x the cost
   of a small op) and python (the rest). Each class scales by its own
   factor k_c = Pi time / PC time of the same work:
       S_pi = sum_c S_pc,c * k_c
   S_pi is the time on an idle Pi, i.e. what ``latency_profile.py profile``
   measures there; the memory traffic of the other processes (k_contention)
   and the clock (k_thermal) enter at step 3. The factors come from a spec band (``spec_band``: low / nominal / high)
   until the Pi has run ``latency_profile.py calibrate`` and ``profile``;
   then from the two profiles of the same workload (``measured_factors``).

2. The node's other work on the same GIL (``imu_share``). The IMU callback
   (104 Hz) runs add_imu + predict(), a Python loop over the n samples since
   the last scan stamp, plus the ROS message work:
       u_imu = f_imu * (c0 + c1 * n + c_ros) * k_python,  n = f_imu (L + T/2)
   and each estimate is published by ``_handle`` (S_handle). The GIL
   serialises them with the scan worker (conservative: aten kernels release
   it), so a scan's service time on the loaded Pi becomes
       S' = (S_pi k_contention + S_handle) k_thermal / (1 - u_imu).

3. Queue (``operating_point``). Scans wait in a FIFO that never drops
   (the network is recurrent, trained at dt = 0.1 s: skipping scans is out
   of distribution). Arrivals follow the sensor model (1 % rate jitter, 1 %
   lost revolutions, IMU coverage wait); the Lindley recursion
       start_k = max(ready_k, finish_{k-1}),  finish_k = start_k + S'_k
   gives the latency L_k = finish_k - stamp_k (stamp = first sample of the
   revolution) with the measured jitter of S. L and u_imu are solved as a
   fixed point. S is the service time in the node's periodic, threaded
   regime: ``profile`` replays scans back to back (hot caches, busy core),
   so its time is multiplied by the ratio ``live --keep-warm`` / ``profile``
   measured for the same ICP backend (``periodic_factor``, ~1.3-1.5 on the
   PC: caches and predictors cold at every scan, two threads on the GIL).

4. Requirements on that operating point:
   R1 utilisation  U = (S_pi k_cont + S_handle) k_th / T + u_imu <= rho_max = 0.70
      (Kingman: the mean wait grows as rho / (1 - rho): x2.3 at 0.7, x9 at
      0.9; the margin also covers rclpy, DDS and cold caches) [assumption].
   R2 freshness   the control pose, just before the next correction, is
      T + L old and dead-reckoned on the IMU. ``latency_profile dr``
      measured its cross-track error e95(A) against ground truth; the
      margins of the race planner were validated at the simulation's
      latency L_ref, so
          e95(T + L_p95) - e95(T + L_ref,p95) <= 0.025 m
      = half of what the 0.20 m blind margin leaves after the 0.10-0.15 m
      belief error [code:race_config.py:164-166].
   G  guards      the driver stops when the last scan-corrected estimate
      is older than 0.6 s: 2 T + L_p99 <= 0.6 s (one lost revolution).

5. The bound (``derive_budget``): the largest factor sigma applied to the
   projected service trace (its jitter shape kept) that meets R1, R2 and G.
   S_max = sigma * S_pi is the estimator's per-scan budget on the Pi (one
   thread, process() + _metric, measured idle as ``profile`` does), stated
   as mean / p95 / p99 in ms.

6. What it means for the model (``allocate``, ``pc_proxies``, ``check``):
   the Pi cost of every stage of the current pipeline against S_max, the
   network budget left by each ICP setting of the sweep, the network sizes
   (random-weight sweep) that fit, and PC-measurable proxies. ``check``
   gives PASS / FAIL / INDETERMINATE (the band straddles the bound) for any
   profile JSON.

Usage:
  python pi4_budget.py report                 # budget.json + budget_report.md from config/pi4_budget/*.json
  python pi4_budget.py check --profile X.json # verdict for a candidate pipeline profiled on the PC (or the Pi)
         [--live Y.json]                       # with the candidate's own live --keep-warm run (periodic factor)
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
DATA_DIR = HERE / "config" / "pi4_budget"
CLASSES = ("compute", "mem_cache", "mem_dram", "dispatch", "python", "native")
WORK_CLASSES = ("compute", "mem_cache", "mem_dram")
COMPONENTS = ("prep", "icp_scan", "icp_map", "icp_native", "icp_other", "network", "net_native", "post", "glue")
BANDS = ("low", "nominal", "high")


# --------------------------------------------------------------- 1. constants
@dataclass(frozen=True)
class Constants:
    # Sensors: the APEX_real profile of tools/pose_dataset/config/sensors.yaml.
    scan_period_s: float = 0.100  # [ds-a2m8][sensors.yaml:252] A2M8 Express 4 kHz at 10 Hz
    scan_rate_jitter: float = 0.01  # [sensors.yaml:253] per-revolution rate std (motor)
    lost_revolutions: float = 0.01  # [sensors.yaml:272]
    lidar_publish_s: float = 0.003  # [sensors.yaml:271] revolution end -> message
    imu_rate_hz: float = 104.0  # [ds-lsm6ds3][sensors.yaml:297]
    imu_publish_s: float = 0.002  # [sensors.yaml:301]
    imu_dlpf_hz: float = 33.0  # [sensors.yaml] gyro low-pass; group delay sqrt(2)/(2 pi f) (sqlite_windows.imu_group_delay_s)
    imu_wait_s: float = 0.03  # [code:learned_odometry_node.py:76] scan processed without full IMU coverage after this
    # Scheduling of the estimator's core.
    rho_max: float = 0.70  # [assumption] utilisation cap (Kingman: wait ~ rho / (1 - rho))
    # Control (race driver, apex_fusion_research).
    blind_margin_m: float = 0.20  # [code:race_config.py:166]
    belief_error_m: float = 0.15  # [code:race_config.py:164] belief error at 3 m/s the margin was sized for
    xt_share: float = 0.5  # [assumption] share of the leftover margin that compute latency may take
    max_scan_age_s: float = 0.6  # [code:race_driver_node.py:89] the car stops beyond this
    max_pose_age_s: float = 0.3  # [code:race_driver_node.py:88]
    dr_stratum: str = "v_ge_2p5"  # race laps: v_max 3.5 m/s [code:race_config.py:181]
    # Raspberry Pi 4 (BCM2711).
    pi_cores: int = 4  # [ds-bcm2711] Cortex-A72
    pi_clock_ghz: float = 1.5  # [ds-bcm2711][user] worst case (1.8 GHz on recent boards)
    pi_l2_bytes: float = 1024.0 * 1024.0  # [ds-bcm2711] shared by the 4 cores
    op_ws_max_bytes: float = 0.5 * 1024.0 * 1024.0  # [assumption] half of L2 per operation (the other cores use the rest)
    thermal: float = 1.0  # [user 2026-09-28] fan + heatsink: nominal clock sustained
    # Queue simulation.
    queue_scans: int = 20000
    seed: int = 7

    @property
    def xt_budget_m(self) -> float:
        """Extra cross-track error allowed to compute latency (R2)."""
        return self.xt_share * (self.blind_margin_m - self.belief_error_m)

    @property
    def imu_group_delay_s(self) -> float:
        return math.sqrt(2.0) / (2.0 * math.pi * self.imu_dlpf_hz)


# ------------------------------------------------------- 2. platform factors
@dataclass(frozen=True)
class Factors:
    """Pi time / PC time of the same work, per cost class, and the
    multipliers of the whole scan (other cores' memory traffic, clock)."""

    compute: float
    mem_cache: float
    mem_dram: float
    dispatch: float
    python: float
    native: float = 0.0  # compiled code (numba: polar_icp.py); 0 = same as python
    contention: float = 1.0
    thermal: float = 1.0
    source: str = ""

    def of(self, cls: str) -> float:
        if cls == "native" and self.native <= 0.0:
            return float(self.python)
        return float(getattr(self, cls))


# Public anchors of the spec band (September 2026).
PC_TURBO_GHZ = 4.8  # [ds-intel-155h] P-core max turbo
GB6_SINGLE_PC = 2400.0  # [gb6] Core Ultra 7 155H, 2384-2421 (Geekbench browser / aggregators)
GB6_SINGLE_PI4 = 340.0  # [gb6] Raspberry Pi 4 at 1.5 GHz, raspberrypi.com "Benchmarking Raspberry Pi 5"
A72_FP32_FLOP_PER_CYCLE = 8.0  # [ds-a72] one 128-bit FMA per cycle (FP64: 4; HPL measures 3.2, OpenBLAS #4149)


def spec_band(pc_conv_gflops: float, c: Constants = Constants()) -> dict[str, Factors]:
    """Factors from specifications, before the Pi is measured.

    scalar (dispatch, python): clock ratio x per-clock ratio of a wide
      out-of-order x86 core over the A72 on branchy interpreter code, 1.9 /
      2.35 / 3.1 [assumption]; the Geekbench 6 single-core ratio
      2400 / 340 = 7.1 = 3.2 x 2.2 [gb6] sits inside the band.
    compute: the PC's measured GFLOPS of the scan-pair convolutions
      (calibration) over the A72's FP32 peak (8 FLOP/cycle x clock) at 50 /
      35 / 25 % efficiency for small convolutions [assumption].
    mem_cache: clock x SIMD width (AVX2 256 bit / NEON 128 bit = 2) x
      load-port ratio 1.0 / 1.4 / 2.0 [assumption].
    mem_dram: mem_cache x a DRAM penalty 1.1 / 1.3 / 1.6: the Pi's single
      32-bit LPDDR4 channel (RAMspeed 5.9 GB/s read, 4.4 GB/s write, whole
      chip [raspberrypi.com]) against the PC's 24 MB L3 [assumption].
    native (compiled numba code, the polar ICP): clock x per-clock ratio on
      compiled scalar code, the same band; Geekbench 6 measures exactly that
      kind of code [assumption].
    contention: 1.0 / 1.1 / 1.3 for the memory traffic of SLAM and the
      drivers on the other cores [assumption; ``calibrate --background 3``
      on the Pi measures it]. thermal: fan + heatsink [user]."""
    clock = PC_TURBO_GHZ / c.pi_clock_ghz
    pi_peak = A72_FP32_FLOP_PER_CYCLE * c.pi_clock_ghz
    out = {}
    for band, ipc, eff, port, dram, cont in zip(BANDS, (1.9, 2.35, 3.1), (0.50, 0.35, 0.25), (1.0, 1.4, 2.0), (1.1, 1.3, 1.6), (1.0, 1.1, 1.3)):
        cache = clock * 2.0 * port
        out[band] = Factors(compute=pc_conv_gflops / (pi_peak * eff), mem_cache=cache, mem_dram=cache * dram, dispatch=clock * ipc, python=clock * ipc,
                            native=clock * ipc, contention=cont, thermal=c.thermal, source=f"spec band ({band})")
    return out


def measured_factors(pc: dict[str, Any], pi: dict[str, Any], contention: float = 1.0, c: Constants = Constants()) -> Factors:
    """Factors from the profiles of the same replay on the PC and on the Pi
    (``latency_profile.py profile``): per work class the ratio of the census
    times, for the scalar classes the ratio of the rest of the wall time."""
    sp, si = class_split(pc), class_split(pi)
    k = {cls: (si[cls] / sp[cls] if sp[cls] > 1e-6 else 1.0) for cls in WORK_CLASSES}
    scalar_pc, scalar_pi = sp["dispatch"] + sp["python"], si["dispatch"] + si["python"]
    k_scalar = scalar_pi / scalar_pc if scalar_pc > 0 else 1.0
    k_native = si["native"] / sp["native"] if sp["native"] > 1e-6 and si["native"] > 0 else 0.0
    return Factors(compute=k["compute"], mem_cache=k["mem_cache"], mem_dram=k["mem_dram"], dispatch=k_scalar, python=k_scalar, native=k_native,
                   contention=contention, thermal=c.thermal, source=f"measured ({pi.get('machine', {}).get('board') or pi.get('machine', {}).get('cpu_model')})")


def contention_from(cal: dict[str, Any], cal_bg: dict[str, Any]) -> float:
    """Slowdown of the memory-class benchmarks with memory-streaming
    processes on the other cores (``calibrate --background 3``)."""
    r = [cal_bg["benchmarks"][n]["us"] / v["us"] for n, v in cal["benchmarks"].items() if v["class"] in ("mem_cache", "mem_dram") and n in cal_bg["benchmarks"]]
    return float(max(1.0, np.exp(np.mean(np.log(r))))) if r else 1.0


# ------------------------------------------------------------- projection
def component_split(wall_ms: float, census: dict[str, Any], small_op_us: float, native: bool = False) -> dict[str, float]:
    """Cost classes of one stage [ms per scan]: the work classes from the
    census (profiler self times, capped by the stage's wall time), dispatch
    = top-level ops x the measured small-op cost, python = the rest. A
    compiled stage (``native``) is all native."""
    if native:
        return {**dict.fromkeys(CLASSES, 0.0), "native": wall_ms}
    work = {cls: float(census["ms"].get(cls, 0.0)) for cls in WORK_CLASSES}
    tot = sum(work.values())
    if tot > wall_ms and tot > 0:  # profiler inflation: keep the proportions
        work = {k: v * wall_ms / tot for k, v in work.items()}
        tot = wall_ms
    rest = wall_ms - tot
    dispatch = min(rest, census["top_ops"] * small_op_us * 1e-3)
    return {**work, "dispatch": dispatch, "python": rest - dispatch, "native": 0.0}


def class_split(profile: dict[str, Any], small_op_us: float | None = None) -> dict[str, float]:
    """Cost classes of the whole scan [ms, mean per scan]."""
    small = small_op_us if small_op_us is not None else profile.get("small_op_us", SMALL_OP_US_DEFAULT)
    out = dict.fromkeys(CLASSES, 0.0)
    for comp, split in stage_splits(profile, small).items():
        for cls, v in split.items():
            out[cls] += v
    return out


SMALL_OP_US_DEFAULT = 6.0  # used only when neither the profile nor a calibration gives it


def stage_splits(profile: dict[str, Any], small_op_us: float) -> dict[str, dict[str, float]]:
    comps = profile["census"]["components"]
    return {c: component_split(profile["components_ms"].get(c, {"mean": 0.0})["mean"], comps.get(c, {}), small_op_us, native=c in ("icp_native", "net_native"))
            for c in COMPONENTS if c in comps and c in profile["components_ms"]}


def project_split(split: dict[str, float], f: Factors) -> float:
    """Idle-Pi time [ms] of a class split measured on the PC."""
    return sum(split.get(cls, 0.0) * f.of(cls) for cls in CLASSES)


def project(profile: dict[str, Any], f: Factors, small_op_us: float) -> dict[str, Any]:
    """Pi projection of a pipeline profile: per stage and in total, and the
    per-scan service trace (the PC trace scaled by the overall ratio, so its
    jitter shape is kept)."""
    stages = {c: project_split(s, f) for c, s in stage_splits(profile, small_op_us).items()}
    pc_mean = sum(profile["components_ms"][c]["mean"] for c in stages)
    pi_mean = sum(stages.values())
    ratio = pi_mean / pc_mean
    trace = np.asarray(profile["wall_ms_trace"], dtype=float) * ratio if "wall_ms_trace" in profile else np.array([pi_mean])
    return {"stages_ms": stages, "mean_ms": pi_mean, "ratio": ratio, "trace_ms": trace, "pc_mean_ms": pc_mean}


# -------------------------------------------------- 2-3. GIL share and queue
def imu_share(latency_s: float, cb: dict[str, float], k_python: float, thermal: float, c: Constants) -> float:
    """Fraction of the estimator's core taken by the IMU callback:
    f_imu (c0 + c1 n + c_ros) k_python, n = f_imu (L + T/2) samples since
    the last scan stamp (predict() walks them in Python)."""
    n = c.imu_rate_hz * (latency_s + 0.5 * c.scan_period_s)
    us = (cb["c0"] + cb["c1_per_sample"] * n + cb.get("ros_us", 0.0)) * k_python * thermal
    return c.imu_rate_hz * us * 1e-6


def arrival_model(n: int, c: Constants) -> tuple[np.ndarray, np.ndarray]:
    """Stamps (first sample of each revolution) and the instant the scan
    can be processed: the end of its revolution plus the wait for the IMU
    sample that covers it (chip group delay, sampling phase, publication),
    at most the node's imu_wait after the scan message. Revolution periods
    jitter by 1 %, 1 % of the revolutions are lost."""
    rng = np.random.default_rng(c.seed)
    periods = c.scan_period_s * (1.0 + c.scan_rate_jitter * rng.standard_normal(n))
    stamps = np.concatenate(([0.0], np.cumsum(periods)[:-1]))
    keep = rng.random(n) >= c.lost_revolutions
    end = stamps + periods
    phase = rng.random(n) / c.imu_rate_hz
    wait = np.minimum(c.imu_group_delay_s + phase + c.imu_publish_s, c.lidar_publish_s + c.imu_wait_s)
    ready = end + np.maximum(c.lidar_publish_s, wait)
    return stamps[keep], ready[keep]


def lindley(ready: np.ndarray, service: np.ndarray) -> np.ndarray:
    """Finish times of a FIFO single server: finish_k = max(ready_k,
    finish_{k-1}) + s_k (Lindley recursion), in closed form
    finish_k = C_k + max_{j<=k} (ready_j - C_{j-1}),  C_k = s_0 + ... + s_k."""
    cum = np.cumsum(service)
    return cum + np.maximum.accumulate(ready - (cum - service))


def _pctl(x: np.ndarray) -> dict[str, float]:
    return {"mean": float(np.mean(x)), "p50": float(np.percentile(x, 50)), "p95": float(np.percentile(x, 95)), "p99": float(np.percentile(x, 99)),
            "max": float(np.max(x))}


def operating_point(service_ms: np.ndarray, handle_ms: float, cb: dict[str, float], f: Factors, c: Constants) -> dict[str, Any]:
    """Latency and utilisation of the estimator's core on the Pi for a
    per-scan service trace [ms, idle Pi] (cycled over ``queue_scans``
    arrivals), with the other cores' memory traffic, the clock and the
    node's publication. Fixed point between the latency and the IMU
    callback share."""
    stamps, ready = arrival_model(c.queue_scans, c)
    reps = int(math.ceil(len(stamps) / len(service_ms)))
    s = (np.tile(service_ms, reps)[: len(stamps)] * f.contention + handle_ms) * f.thermal * 1e-3
    u = 0.0
    lat = c.scan_period_s + float(np.mean(s))
    for _ in range(30):
        u = imu_share(lat, cb, f.python, f.thermal, c)
        if u >= 1.0:
            break
        finish = lindley(ready, s / (1.0 - u))
        latency = finish - stamps
        new = float(np.mean(latency))
        if abs(new - lat) < 1e-5 or new > 10.0:
            lat = new
            break
        lat = 0.5 * (lat + new)
    util = float(np.mean(s)) / c.scan_period_s + u
    stable = util < 1.0 and u < 1.0
    if u >= 1.0:
        latency = np.full(len(stamps), np.inf)
    backlog = np.searchsorted(finish, ready, side="right") if u < 1.0 else np.zeros(1)
    pending = np.arange(len(ready)) - backlog if u < 1.0 else np.zeros(1)
    return {"utilisation": util, "imu_share": u, "stable": bool(stable),
            "latency_ms": _pctl(latency * 1e3) if stable else {"mean": math.inf, "p50": math.inf, "p95": math.inf, "p99": math.inf, "max": math.inf},
            "pending_max": int(np.max(pending)) if stable else -1, "service_ms": _pctl(s * 1e3)}


# ----------------------------------------------------------- 4. freshness
def dr_e95(dr: dict[str, Any], stratum: str) -> tuple[np.ndarray, np.ndarray]:
    s = dr["strata"][stratum]
    return np.asarray(dr["ages_s"], dtype=float), np.asarray(s["p95_xt_m"], dtype=float)


def latency_max_r2(dr: dict[str, Any], l_ref_s: float, c: Constants) -> dict[str, float]:
    """Largest p95 latency whose control-pose age T + L adds at most
    ``xt_budget_m`` of p95 cross-track error over the validated latency."""
    ages, e95 = dr_e95(dr, c.dr_stratum)
    e95 = np.maximum.accumulate(e95)  # monotone envelope
    a_ref = c.scan_period_s + l_ref_s
    e_ref = float(np.interp(a_ref, ages, e95))
    target = e_ref + c.xt_budget_m
    if target >= e95[-1]:  # beyond the measured ages: extrapolate the last slope
        slope = (e95[-1] - e95[-2]) / (ages[-1] - ages[-2])
        a_max = ages[-1] + (target - e95[-1]) / max(slope, 1e-9)
    else:
        k = int(np.searchsorted(e95, target))
        a_max = float(np.interp(target, e95[k - 1 : k + 1], ages[k - 1 : k + 1])) if k > 0 else ages[0]
    return {"age_ref_s": a_ref, "e95_ref_m": e_ref, "e95_target_m": target, "age_max_s": float(a_max), "latency_p95_max_s": float(a_max - c.scan_period_s)}


def latency_max_guard(c: Constants) -> float:
    """p99 latency at which one lost revolution trips the scan-age guard."""
    return c.max_scan_age_s - 2.0 * c.scan_period_s


# ------------------------------------------------------------- 5. the bound
def meets(op: dict[str, Any], lim_r2_s: float, lim_guard_s: float, c: Constants) -> dict[str, bool]:
    lat = op["latency_ms"]
    return {"R1_utilisation": op["stable"] and op["utilisation"] <= c.rho_max, "R2_freshness": lat["p95"] * 1e-3 <= lim_r2_s,
            "G_scan_age": lat["p99"] * 1e-3 <= lim_guard_s}


def derive_budget(trace_ms: np.ndarray, handle_ms: float, cb: dict[str, float], f: Factors, lim_r2_s: float, c: Constants) -> dict[str, Any]:
    """Largest sigma such that sigma x trace meets every requirement
    (bisection: every requirement is monotone in sigma)."""
    lim_guard = latency_max_guard(c)
    base = np.asarray(trace_ms, dtype=float)

    def ok(sig: float) -> bool:
        return all(meets(operating_point(base * sig, handle_ms, cb, f, c), lim_r2_s, lim_guard, c).values())

    lo, hi = 0.0, 1.0
    while ok(hi) and hi < 1e3:
        lo, hi = hi, hi * 2.0
    if not ok(1e-6):
        return {"feasible": False, "reason": "the node's fixed work alone breaks a requirement"}
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
        if hi - lo < 1e-4 * hi:
            break
    sig = lo
    op = operating_point(base * sig, handle_ms, cb, f, c)
    # which requirement binds: the first to fail just above sigma
    above = meets(operating_point(base * sig * 1.01, handle_ms, cb, f, c), lim_r2_s, lim_guard, c)
    return {"feasible": True, "sigma": sig, "service_max_ms": _pctl(base * sig), "binding": [k for k, v in above.items() if not v],
            "operating_point": op, "limits": {"rho_max": c.rho_max, "latency_p95_max_ms": 1e3 * lim_r2_s, "latency_p99_max_ms": 1e3 * lim_guard}}


# --------------------------------------------------- 6. allocation, proxies
def project_network_point(p: dict[str, Any], f: Factors, small_op_us: float) -> float:
    census = {"ms": p["class_ms"], "top_ops": p["top_ops"]}
    return project_split(component_split(p["ms"]["mean"], census, small_op_us), f)


def allocate(budget_ms: float, profile: dict[str, Any], sweep: dict[str, Any] | None, f: Factors, small_op_us: float) -> dict[str, Any]:
    """Where the budget goes: the Pi cost of every stage now, the network
    budget each ICP setting of the sweep leaves, and the network sizes of
    the random-weight sweep that fit in it."""
    now = project(profile, f, small_op_us)
    out: dict[str, Any] = {"budget_mean_ms": budget_ms, "current_stages_ms": now["stages_ms"], "current_mean_ms": now["mean_ms"],
                           "network_budget_if_icp_unchanged_ms": budget_ms - (now["mean_ms"] - now["stages_ms"].get("network", 0.0))}
    if not sweep:
        return out
    scen = {}
    for name, v in sweep.get("icp", {}).items():
        pr = project(v, f, small_op_us)
        scen[name] = {"knobs": v["knobs"], "pc_mean_ms": pr["pc_mean_ms"], "pi_mean_ms": pr["mean_ms"], "pi_network_ms": pr["stages_ms"].get("network", 0.0),
                      "network_budget_ms": budget_ms - (pr["mean_ms"] - pr["stages_ms"].get("network", 0.0))}
    out["icp_scenarios"] = scen
    net = sweep.get("network")
    if net:
        pts = []
        for p in net["points"] + [net["checkpoint_point"]]:
            pts.append({"hidden": p["hidden"], "pair_widths": p["pair_widths"], "trained": p["trained"], "params": p["params"]["total"],
                        "macs_m": p["macs"]["total"] / 1e6, "top_ops": p["top_ops"], "pc_ms": p["ms"]["mean"], "pi_ms": project_network_point(p, f, small_op_us)})
        out["network_points"] = pts
        fits = {}
        for name, s in scen.items():
            ok = [q for q in pts if q["pi_ms"] <= s["network_budget_ms"] and not q["trained"]]
            fits[name] = {"largest_params": max((q["params"] for q in ok), default=0), "largest_macs_m": max((q["macs_m"] for q in ok), default=0.0),
                          "n_fit": len(ok), "n_points": len([q for q in pts if not q["trained"]])}
        out["network_fits"] = fits
        # latency against size: floor + slope, the evidence of how parameters cost
        xs = np.array([q["params"] for q in pts if not q["trained"]], dtype=float)
        ys_pc = np.array([q["pc_ms"] for q in pts if not q["trained"]])
        ys_pi = np.array([q["pi_ms"] for q in pts if not q["trained"]])
        ms = np.array([q["macs_m"] for q in pts if not q["trained"]])
        out["size_fit"] = {"pc_ms_vs_params": _affine(xs, ys_pc), "pi_ms_vs_params": _affine(xs, ys_pi),
                           "pc_ms_vs_macs_m": _affine(ms, ys_pc), "pi_ms_vs_macs_m": _affine(ms, ys_pi)}
    return out


def _affine(x: np.ndarray, y: np.ndarray) -> dict[str, float]:
    """y = floor + slope x, least squares, and its R^2."""
    slope, floor = np.polyfit(x, y, 1)
    pred = floor + slope * x
    r2 = 1.0 - float(np.sum((y - pred) ** 2)) / max(float(np.sum((y - y.mean()) ** 2)), 1e-12)
    return {"floor_ms": float(floor), "slope": float(slope), "r2": r2}


def pc_proxies(budget_ms: float, profile: dict[str, Any], f: Factors, small_op_us: float, c: Constants) -> dict[str, Any]:
    """The bound in quantities the next step measures on the PC (one
    thread, ``latency_profile.py profile``): the PC mean service time that
    projects to the budget for a pipeline of the current class mix, the
    top-level op count if the whole budget were dispatch, and the per-op
    working-set rule of the Pi's L2."""
    pr = project(profile, f, small_op_us)
    k_disp = f.dispatch  # the budget is idle-Pi time
    return {"pc_mean_service_max_ms": budget_ms / pr["ratio"], "pc_to_pi_ratio": pr["ratio"],
            "top_level_ops_max_if_all_dispatch": budget_ms / (small_op_us * 1e-3 * k_disp),
            "op_working_set_max_mb": c.op_ws_max_bytes / (1024.0 * 1024.0),
            "current": {"pc_mean_ms": pr["pc_mean_ms"], "top_level_ops": sum(v["top_ops"] for v in profile["census"]["components"].values()),
                        "max_op_working_set_mb": profile["census"]["max_ws_bytes"] / (1024.0 * 1024.0)}}


# ------------------------------------------------------------ inputs, check
def load(name: str) -> dict[str, Any] | None:
    p = DATA_DIR / name
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else None


@dataclass
class Inputs:
    """The measurements the bound is derived from (``config/pi4_budget``)."""

    profile_pc: dict[str, Any]
    calibration_pc: dict[str, Any]
    simref: dict[str, Any]
    dr: dict[str, Any]
    sweep_pc: dict[str, Any] | None = None
    ros: dict[str, Any] | None = None
    ros_pi: dict[str, Any] | None = None
    profile_pi: dict[str, Any] | None = None
    calibration_pi: dict[str, Any] | None = None
    calibration_pi_bg: dict[str, Any] | None = None
    notes: list[str] = field(default_factory=list)

    @staticmethod
    def from_dir(pc_tag: str = "pc", pi_tag: str = "pi4") -> "Inputs":
        need = {k: load(n) for k, n in (("profile_pc", f"profile_{pc_tag}.json"), ("calibration_pc", f"calibration_{pc_tag}.json"),
                                        ("simref", "simref.json"), ("dr", "dr_curves.json"))}
        missing = [k for k, v in need.items() if v is None]
        if missing:
            raise SystemExit(f"missing measurements in {DATA_DIR}: {missing} (run latency_profile.py first)")
        return Inputs(**need, sweep_pc=load(f"sweep_{pc_tag}.json"), ros=load(f"ros_{pc_tag}.json"), ros_pi=load(f"ros_{pi_tag}.json"),
                      profile_pi=load(f"profile_{pi_tag}.json"),
                      calibration_pi=load(f"calibration_{pi_tag}.json"), calibration_pi_bg=load(f"calibration_{pi_tag}_bg.json"))


ROS_DEFAULT = {"handle_ms": 1.0, "imu_callback_us": 60.0}  # [assumption] PC, when ros_callback_bench.py has not run


def periodic_factor(profile: dict[str, Any], live: dict[str, Any] | None = None) -> tuple[float, str]:
    """Node service / replay service for the ICP backend and network runtime of ``profile``:
    p50 of a ``live --keep-warm`` run at real speed over p50 of the replay
    (the same code, run periodically with the node's two threads). 1.0 when
    no such run exists. Prefers a run of the same machine as the profile.
    ``live``: that run given explicitly (the candidate's own, e.g. another
    checkpoint with the same backends as a run in ``DATA_DIR``)."""
    def key(d: dict[str, Any]) -> tuple[str, str]:  # (ICP search, network runtime); older files have neither
        b = d.get("icp_backend", "checkpoint")
        return ("cdist" if b in ("checkpoint", "", "cdist") else b, d.get("net_backend", "torch") or "torch")

    replay = float(np.percentile(profile["wall_ms_trace"], 50)) if "wall_ms_trace" in profile else profile["wall_ms"]["p50"]
    if live is not None:
        return float(np.median(live["service_ms_trace"][10:])) / replay, "given live run p50 / profile p50"
    backend = key(profile)
    host = profile.get("machine", {}).get("host")
    best = None
    for f in sorted(DATA_DIR.glob("live_*.json")):
        d = json.loads(f.read_text(encoding="utf-8"))
        if not d.get("keep_warm") or d.get("scale", 1.0) != 1.0 or key(d) != backend:
            continue
        same = d.get("machine", {}).get("host") == host
        if best is None or (same and not best[2]):
            best = (float(np.median(d["service_ms_trace"][10:])), f.name, same)
    if best is None:
        return 1.0, "none (no live --keep-warm run)"
    return best[0] / replay, f"{best[1]} p50 / profile p50"


def node_costs(inp: Inputs, f: Factors | None = None) -> tuple[float, dict[str, float], str]:
    """The node's work besides the estimator: _handle per estimate [ms, on
    the Pi] and the IMU callback's add_imu + predict (c0 + c1 n) plus its
    message work [us, PC-equivalent: ``imu_share`` multiplies by the python
    factor]. Pi measurements (``ros_pi4.json``, the Pi profile) replace the
    PC ones scaled by that factor. ``f=None``: the PC values."""
    k = f.python if f is not None else 1.0
    ros = inp.ros or ROS_DEFAULT
    handle, ros_us = float(ros["handle_ms"]) * k, float(ros["imu_callback_us"])
    src = "meas ros_callback_bench (PC)" if inp.ros else "assumption"
    cb = dict(inp.profile_pc["imu_callback_us"])
    if f is not None and inp.ros_pi:
        handle, ros_us = float(inp.ros_pi["handle_ms"]), float(inp.ros_pi["imu_callback_us"]) / k
        src = "meas ros_callback_bench (Pi)"
    if f is not None and inp.profile_pi:
        cb = {key: v / k for key, v in inp.profile_pi["imu_callback_us"].items() if key != "points"}
    cb.pop("points", None)
    return handle, {**cb, "ros_us": ros_us}, src


def factor_sets(inp: Inputs, c: Constants) -> dict[str, Factors]:
    """The spec band, and the measured factors once the Pi has been profiled."""
    conv = inp.calibration_pc["benchmarks"]["conv_stack_pair"]["gflops"]
    sets = spec_band(conv, c)
    if inp.profile_pi is not None:
        cont = contention_from(inp.calibration_pi, inp.calibration_pi_bg) if inp.calibration_pi and inp.calibration_pi_bg else sets["nominal"].contention
        sets["measured"] = measured_factors(inp.profile_pc, inp.profile_pi, cont, c)
    return sets


def small_op(inp: Inputs) -> float:
    return float(inp.calibration_pc["small_op_us"])


def evaluate(inp: Inputs, c: Constants = Constants()) -> dict[str, Any]:
    """The whole derivation (steps 1-6) for every factor set."""
    handle_pc, cb_pc, ros_src = node_costs(inp)
    small = small_op(inp)
    l_ref = inp.simref["latency_ms"]["p95"] * 1e-3
    r2 = latency_max_r2(inp.dr, l_ref, c)
    out: dict[str, Any] = {"constants": {**asdict(c), "xt_budget_m": c.xt_budget_m, "imu_group_delay_s": c.imu_group_delay_s},
                           "reference_latency_ms": inp.simref["latency_ms"], "r2": r2, "guard_latency_p99_max_ms": 1e3 * latency_max_guard(c),
                           "node_costs_pc": {"handle_ms": handle_pc, "imu_callback_us": cb_pc, "source": ros_src}, "sets": {}}
    k_per, per_src = periodic_factor(inp.profile_pi if inp.profile_pi is not None else inp.profile_pc)
    out["periodic_factor"] = {"value": k_per, "source": per_src}
    for name, f in factor_sets(inp, c).items():
        handle, cb, _ = node_costs(inp, f)
        proj = project(inp.profile_pc, f, small) if name != "measured" else _measured_projection(inp.profile_pi)
        node_trace = proj["trace_ms"] * k_per  # service in the node (periodic, threaded), idle Pi
        budget = derive_budget(node_trace, handle, cb, f, r2["latency_p95_max_s"], c)
        now = operating_point(node_trace, handle, cb, f, c)
        entry = {"factors": asdict(f), "projection": {k: v for k, v in proj.items() if k != "trace_ms"}, "budget": budget, "current": {
            "service_ms": _pctl(node_trace), "operating_point": now,
            "meets": meets(now, r2["latency_p95_max_s"], latency_max_guard(c), c)}}
        if budget.get("feasible"):
            b = budget["service_max_ms"]["mean"]
            entry["allocation"] = allocate(b / k_per, inp.profile_pc, inp.sweep_pc, f, small)  # replay units
            entry["pc_proxies"] = pc_proxies(b / k_per, inp.profile_pc, f, small, c)
            entry["reduction_needed"] = proj["mean_ms"] * k_per / b
        out["sets"][name] = entry
    out["verdict_current"] = verdict({k: v for k, v in out["sets"].items()})
    return out


def _measured_projection(pi: dict[str, Any]) -> dict[str, Any]:
    trace = np.asarray(pi["wall_ms_trace"], dtype=float)
    return {"stages_ms": {c: pi["components_ms"][c]["mean"] for c in COMPONENTS if c in pi["components_ms"]}, "mean_ms": float(trace.mean()), "ratio": 1.0, "trace_ms": trace,
            "pc_mean_ms": float("nan")}


def verdict(sets: dict[str, Any]) -> str:
    """PASS: meets the bound even with the pessimistic factors; FAIL: misses
    it even with the optimistic ones; INDETERMINATE: the band straddles it
    (measure on the Pi). A measured set decides alone."""
    def passes(e: dict[str, Any]) -> bool:
        return all(e["current"]["meets"].values())
    if "measured" in sets:
        return "PASS" if passes(sets["measured"]) else "FAIL"
    if passes(sets["high"]):
        return "PASS"
    if not passes(sets["low"]):
        return "FAIL"
    return "INDETERMINATE"


def check(candidate: dict[str, Any], inp: Inputs, c: Constants = Constants(), live: dict[str, Any] | None = None) -> dict[str, Any]:
    """Verdict for a candidate pipeline profiled with ``latency_profile.py
    profile`` (on the PC: projected with every factor set; on the Pi: as is).
    ``live``: the candidate's own ``live --keep-warm`` run for the periodic
    factor (default: the run of ``DATA_DIR`` with the same backends)."""
    small = small_op(inp)
    r2 = latency_max_r2(inp.dr, inp.simref["latency_ms"]["p95"] * 1e-3, c)
    on_pi = "aarch64" in str(candidate.get("machine", {}).get("machine", ""))
    sets = {"measured": Factors(1, 1, 1, 1, 1, source="profiled on the Pi")} if on_pi else factor_sets(inp, c)
    out = {}
    k_per, per_src = periodic_factor(candidate, live)
    for name, f in sets.items():
        proj = _measured_projection(candidate) if on_pi else project(candidate, f, small)
        f_node = f if not on_pi else factor_sets(inp, c).get("measured", factor_sets(inp, c)["high"])
        handle, cb, _ = node_costs(inp, f_node)
        node_trace = proj["trace_ms"] * k_per
        op = operating_point(node_trace, handle, cb, f_node, c)
        budget = derive_budget(node_trace, handle, cb, f_node, r2["latency_p95_max_s"], c)
        out[name] = {"pi_mean_ms": float(np.mean(node_trace)), "pi_replay_mean_ms": proj["mean_ms"],
                     "budget_mean_ms": budget.get("service_max_ms", {}).get("mean", 0.0), "periodic_factor": k_per,
                     "margin": (budget.get("sigma", 0.0) - 1.0), "current": {"operating_point": op, "meets": meets(op, r2["latency_p95_max_s"], latency_max_guard(c), c)}}
    return {"verdict": verdict(out), "periodic_factor": {"value": k_per, "source": per_src}, "sets": out}


def queue_model_check(c: Constants = Constants()) -> list[dict[str, Any]]:
    """The queue model against the real threads: every ``latency_profile.py
    live`` run (IMU callback + scan worker on one pinned core, optionally a
    slower CPU emulated) is re-predicted from its own service trace."""
    rows = []
    for f in sorted(DATA_DIR.glob("live_*.json")):
        d = json.loads(f.read_text(encoding="utf-8"))
        s = np.asarray(d["service_ms_trace"][10:], dtype=float)
        if s.size < 50:
            continue
        op = operating_point(s, 0.0, {"c0": 0.0, "c1_per_sample": 0.0, "ros_us": 0.0}, Factors(1, 1, 1, 1, 1), c)
        rows.append({"run": f.stem, "scale": d.get("scale", 1.0), "service_mean_ms": float(s.mean()), "live_ms": d["latency_ms"], "model_ms": op["latency_ms"],
                     "pending_max_live": d.get("pending_max"), "imu_callback_delay_p99_ms": d["imu_callback_delay_ms"]["p99"]})
    return rows


def platform_check(res: dict[str, Any], inp: Inputs, c: Constants = Constants()) -> list[dict[str, Any]]:
    """Cores of the whole Pi at 10 Hz (nominal band): the estimator at its
    bound and today, the reactive planner of lap 1 (measured on the PC in
    the closed-loop runs), and placeholders to measure on the car."""
    f = res["sets"].get("measured", res["sets"]["nominal"])
    k_py = f["factors"]["python"]
    rows = []
    b = f.get("budget", {})
    if b.get("feasible"):
        rows.append({"process": "learned odometry at its bound", "cores": b["operating_point"]["utilisation"], "source": "this derivation"})
    rows.append({"process": "learned odometry today", "cores": f["current"]["operating_point"]["utilisation"], "source": "projection"})
    react = inp.simref.get("react_ms", {})
    for phase in ("explore", "closing", "race"):
        if phase in react:
            rows.append({"process": f"race driver, reactive planner ({phase})", "cores": react[phase]["mean"] * k_py / (1e3 * c.scan_period_s),
                         "source": f"PC react_ms under Gazebo x python factor ({react[phase]['mean']:.1f} ms mean)"})
    rows += [{"process": "slam_toolbox (5 Hz max)", "cores": 0.5, "source": "assumption: measure with pidstat on the car"},
             {"process": "LiDAR Express driver + IMU serial", "cores": 0.1, "source": "real_vehicle/docs/sensor_timing_requirements.md:69 (~1 % of a PC core) + assumption"},
             {"process": "race driver control loop 50 Hz, DDS, OS", "cores": 0.3, "source": "assumption"}]
    return rows


# ------------------------------------------------------------------ report
def _fmt(x: float, nd: int = 1) -> str:
    return "inf" if not math.isfinite(x) else f"{x:.{nd}f}"


def markdown(res: dict[str, Any], inp: Inputs, plat: list[dict[str, Any]]) -> str:
    c = res["constants"]
    sets = res["sets"]
    main = "measured" if "measured" in sets else "high"
    b = sets[main]["budget"]
    L = ["# Pi 4 compute budget of the learned odometry", "",
         f"Generated by `pi4_budget.py report` from `config/pi4_budget/*.json` (PC profile of {inp.profile_pc['machine']['date']}). "
         f"Factor set of the headline: **{main}** ({sets[main]['factors']['source']}). Verdict for the current model: **{res['verdict_current']}**.", ""]
    if b.get("feasible"):
        s = b["service_max_ms"]
        L += ["## Requirement", "",
              f"On the Raspberry Pi 4 (one core, one thread, fan, `performance` governor), the estimator's work per LiDAR scan "
              f"(`LiveOdometry.process` as it runs in the node, 10 Hz) must stay within "
              f"**{_fmt(s['mean'])} ms on average** (p95 {_fmt(s['p95'])} ms, p99 {_fmt(s['p99'])} ms with today's jitter shape). "
              f"A back-to-back replay (`latency_profile.py profile`) measures {res['periodic_factor']['value']:.2f}× less than the node for the "
              f"same code ({res['periodic_factor']['source']}), so on that measurement the budget is {_fmt(s['mean'] / res['periodic_factor']['value'])} ms. "
              f"Binding requirement: {', '.join(b['binding']) or 'none'}.", "",
              "Budget per factor set (mean ms per scan, service in the node on an idle Pi): "
              + ", ".join(f"{n} {_fmt(e['budget'].get('service_max_ms', {}).get('mean', float('nan')))}" for n, e in sets.items())
              + f". The headline uses the {'measured factors' if main == 'measured' else 'pessimistic end of the spec band until the Pi is measured'}.", ""]
    L += ["## Derivation", "", "| Step | Value |", "| --- | --- |",
          f"| Scan period T | {1e3 * c['scan_period_s']:.0f} ms |",
          f"| IMU coverage wait (group delay {1e3 * c['imu_group_delay_s']:.1f} ms + phase + publish) | ≤ {1e3 * c['imu_wait_s']:.0f} ms |",
          f"| R1 utilisation cap ρ_max | {c['rho_max']:.2f} |",
          f"| Validated latency of the simulation (p50 / p95) | {res['reference_latency_ms']['p50']:.0f} / {res['reference_latency_ms']['p95']:.0f} ms |",
          f"| R2 cross-track budget: {c['xt_share']} × ({c['blind_margin_m']} − {c['belief_error_m']}) m | {100 * c['xt_budget_m']:.1f} cm |",
          f"| R2 e95 at the reference age {res['r2']['age_ref_s']:.3f} s ({c['dr_stratum']}) | {100 * res['r2']['e95_ref_m']:.1f} cm |",
          f"| R2 → max control-pose age / max latency p95 | {res['r2']['age_max_s']:.3f} s / {1e3 * res['r2']['latency_p95_max_s']:.0f} ms |",
          f"| Guard: 2T + L_p99 ≤ {c['max_scan_age_s']} s → max latency p99 | {res['guard_latency_p99_max_ms']:.0f} ms |",
          f"| Periodic regime: node service / replay service ({res['periodic_factor']['source']}) | {res['periodic_factor']['value']:.2f} |",
          f"| Node work on the PC: _handle per estimate / IMU callback c0 + c1·n + ROS | {res['node_costs_pc']['handle_ms']:.2f} ms / "
          f"{res['node_costs_pc']['imu_callback_us']['c0']:.0f} + {res['node_costs_pc']['imu_callback_us']['c1_per_sample']:.2f}·n + "
          f"{res['node_costs_pc']['imu_callback_us']['ros_us']:.0f} µs ({res['node_costs_pc']['source']}) |", ""]
    L += ["## Factor sets and results", "",
          "| Set | k compute | k mem_cache | k mem_dram | k dispatch/python | contention | Current Pi mean (ms) | Budget mean (ms) | Reduction needed | Binding | Current meets R1 / R2 / G |",
          "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for name, e in sets.items():
        f = e["factors"]
        bb = e["budget"]
        m = e["current"]["meets"]
        L.append(f"| {name} | {f['compute']:.1f} | {f['mem_cache']:.1f} | {f['mem_dram']:.1f} | {f['dispatch']:.1f} | {f['contention']:.2f} | "
                 f"{e['projection']['mean_ms']:.0f} | {_fmt(bb.get('service_max_ms', {}).get('mean', float('nan')))} | "
                 f"{_fmt(e.get('reduction_needed', float('nan')), 2)}× | {', '.join(bb.get('binding', [])) or '-'} | "
                 f"{'/'.join('yes' if m[k] else 'no' for k in ('R1_utilisation', 'R2_freshness', 'G_scan_age'))} |")
    e = sets[main]
    L += ["", f"## Where the time goes ({main} factors)", "", "| Stage | PC mean (ms) | Pi projection (ms) |", "| --- | --- | --- |"]
    for comp in COMPONENTS:
        if comp in inp.profile_pc["components_ms"]:
            L.append(f"| {comp} | {inp.profile_pc['components_ms'][comp]['mean']:.2f} | {e['projection']['stages_ms'].get(comp, float('nan')):.1f} |")
    al = e.get("allocation", {})
    if al:
        L += ["", f"Network budget if the ICP is unchanged: **{al['network_budget_if_icp_unchanged_ms']:.1f} ms** "
              f"(the network alone costs {al['current_stages_ms'].get('network', 0):.1f} ms on the Pi).", ""]
        if "icp_scenarios" in al:
            names = [n for n in sets if "allocation" in sets[n]]
            L += ["Network budget left by each ICP setting (Pi ms per scan; any ICP change means retraining the hybrid). "
                  "Fit = networks of the random-weight sweep whose Pi cost fits in that budget.", "",
                  "| ICP setting | PC mean (ms) | " + " | ".join(f"Pi mean {n}" for n in names) + " | " + " | ".join(f"Network budget {n}" for n in names)
                  + " | " + " | ".join(f"Fit {n}" for n in names) + " |", "| --- | --- | " + " | ".join(["---"] * 3 * len(names)) + " |"]
            for name, sc in al["icp_scenarios"].items():
                row = [f"{name} {sc['knobs'] or ''}".strip(), f"{sc['pc_mean_ms']:.1f}"]
                row += [f"{sets[n]['allocation']['icp_scenarios'][name]['pi_mean_ms']:.0f}" for n in names]
                row += [f"{sets[n]['allocation']['icp_scenarios'][name]['network_budget_ms']:.1f}" for n in names]
                row += [f"{sets[n]['allocation'].get('network_fits', {}).get(name, {}).get('n_fit', '-')}/{sets[n]['allocation'].get('network_fits', {}).get(name, {}).get('n_points', '-')}"
                        for n in names]
                L.append("| " + " | ".join(row) + " |")
            L.append("")
        if "size_fit" in al:
            sf = al["size_fit"]
            L += ["", "Latency of the network against its size (random weights, real inputs, no training):", "",
                  f"- PC: {sf['pc_ms_vs_params']['floor_ms']:.2f} ms + {1e6 * sf['pc_ms_vs_params']['slope']:.3f} ms per million parameters (R² {sf['pc_ms_vs_params']['r2']:.2f}); "
                  f"per million MACs: {sf['pc_ms_vs_macs_m']['slope']:.3f} ms (R² {sf['pc_ms_vs_macs_m']['r2']:.2f})",
                  f"- Pi ({main}): {sf['pi_ms_vs_params']['floor_ms']:.1f} ms + {1e6 * sf['pi_ms_vs_params']['slope']:.2f} ms per million parameters; "
                  f"per million MACs: {sf['pi_ms_vs_macs_m']['slope']:.2f} ms",
                  f"- The floor (a network of zero parameters, i.e. its {al['network_points'][0]['top_ops']:.0f} top-level torch ops per scan) is "
                  f"{100 * sf['pi_ms_vs_params']['floor_ms'] / al['budget_mean_ms']:.0f} % of the budget; the v5 network's 0.49 M parameters add "
                  f"{0.488753e6 * sf['pi_ms_vs_params']['slope']:.1f} ms. Parameters are streamed from memory once per scan (batch-1 GEMV), so the "
                  "latency follows the parameter count better than the MACs; but at this size the op count dominates.", "",
                  "| hidden | pair widths | params | MACs (M) | top-level ops | PC (ms) | Pi (ms) |", "| --- | --- | --- | --- | --- | --- | --- |"]
            for q in sorted(al["network_points"], key=lambda q: q["params"]):
                L.append(f"| {q['hidden']}{' (v5 ckpt)' if q['trained'] else ''} | {q['pair_widths']} | {q['params']:,} | {q['macs_m']:.1f} | {q['top_ops']:.0f} | "
                         f"{q['pc_ms']:.2f} | {q['pi_ms']:.1f} |")
    if e.get("pc_proxies"):
        px = e["pc_proxies"]
        L += ["", "## PC proxies for the next step", "",
              "PC mean service time (1 thread, `latency_profile.py profile`) that projects to the budget, for a pipeline of today's class mix: "
              + ", ".join(f"{n} ≤ {sets[n]['pc_proxies']['pc_mean_service_max_ms']:.1f} ms" for n in sets if "pc_proxies" in sets[n])
              + f" (today {px['current']['pc_mean_ms']:.1f} ms).", "",
              "- Top-level torch ops per scan if the whole budget were dispatch: "
              + ", ".join(f"{n} ≤ {sets[n]['pc_proxies']['top_level_ops_max_if_all_dispatch']:.0f}" for n in sets if "pc_proxies" in sets[n])
              + f" (today {px['current']['top_level_ops']:.0f}).",
              f"- Largest working set of one operation: ≤ {px['op_working_set_max_mb']:.2f} MB (today {px['current']['max_op_working_set_mb']:.2f} MB, the 360×1800 submap cdist).",
              "- Authoritative test: `python pi4_budget.py check --profile <candidate profile JSON>` (PASS / FAIL / INDETERMINATE over the factor sets).", ""]
    qc = res.get("queue_check", [])
    if qc:
        L += ["## Queue model against the real threads (`latency_profile.py live`)", "",
              "| Run | Emulated slowdown | Service mean (ms) | Latency p50 / p95 / p99, live (ms) | Same, model (ms) | Pending max | IMU callback delay p99 (ms) |",
              "| --- | --- | --- | --- | --- | --- | --- |"]
        for r in qc:
            lv, md = r["live_ms"], r["model_ms"]
            L.append(f"| {r['run']} | {r['scale']:.1f}× | {r['service_mean_ms']:.1f} | {lv['p50']:.0f} / {lv['p95']:.0f} / {lv['p99']:.0f} | "
                     f"{md['p50']:.0f} / {md['p95']:.0f} / {md['p99']:.0f} | {r['pending_max_live']} | {r['imu_callback_delay_p99_ms']:.1f} |")
        L += ["", "The model matches p50 and p95; the live p99 is higher (GIL switch interval, OS scheduling, few samples), which is why R2 is "
              "stated on p95 and the scan-age guard, on p99, keeps a wide margin.", ""]
    L += ["## Whole Pi at 10 Hz (cores, nominal factors)", "", "| Process | Cores | Source |", "| --- | --- | --- |"]
    for r in plat:
        L.append(f"| {r['process']} | {r['cores']:.2f} | {r['source']} |")
    L += ["", f"Available: {c['pi_cores']} cores × ρ_max {c['rho_max']} = {c['pi_cores'] * c['rho_max']:.1f}.", ""]
    return "\n".join(L)


def cmd_report(args: argparse.Namespace) -> None:
    inp = Inputs.from_dir(args.pc_tag, args.pi_tag)
    c = Constants()
    res = evaluate(inp, c)
    plat = platform_check(res, inp, c)
    res["platform"] = plat
    res["queue_check"] = queue_model_check(c)
    (DATA_DIR / "budget.json").write_text(json.dumps(res, indent=1, default=float), encoding="utf-8")
    (DATA_DIR / "budget_report.md").write_text(markdown(res, inp, plat), encoding="utf-8")
    main = "measured" if "measured" in res["sets"] else "high"
    b = res["sets"][main]["budget"]
    print(f"wrote {DATA_DIR / 'budget.json'} and budget_report.md")
    for name, e in res["sets"].items():
        bb = e["budget"]
        print(f"  {name:8s} current Pi mean {e['projection']['mean_ms']:6.1f} ms | budget mean {bb.get('service_max_ms', {}).get('mean', float('nan')):6.1f} ms "
              f"(binding {', '.join(bb.get('binding', [])) or '-'}) | reduction x{e.get('reduction_needed', float('nan')):.2f}")
    print(f"verdict: {res['verdict_current']}; headline ({main}) budget mean {b.get('service_max_ms', {}).get('mean', float('nan')):.1f} ms")


def cmd_check(args: argparse.Namespace) -> None:
    inp = Inputs.from_dir(args.pc_tag, args.pi_tag)
    cand = json.loads(Path(args.profile).read_text(encoding="utf-8"))
    live = json.loads(Path(args.live).read_text(encoding="utf-8")) if args.live else None
    res = check(cand, inp, live=live)
    print(f"  periodic factor {res['periodic_factor']['value']:.3f} ({res['periodic_factor']['source']})")
    for name, e in res["sets"].items():
        print(f"  {name:8s} Pi mean {e['pi_mean_ms']:6.1f} ms  budget {e['budget_mean_ms']:6.1f} ms  margin {100 * e['margin']:+.0f} %  meets {e['current']['meets']}")
    print(f"verdict: {res['verdict']}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, fn in (("report", cmd_report), ("check", cmd_check)):
        p = sub.add_parser(name)
        p.add_argument("--pc-tag", default="pc")
        p.add_argument("--pi-tag", default="pi4")
        if name == "check":
            p.add_argument("--profile", required=True)
            p.add_argument("--live", default=None, help="the candidate's own live --keep-warm JSON for the periodic factor")
        p.set_defaults(fn=fn)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
