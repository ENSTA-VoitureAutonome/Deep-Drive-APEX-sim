#!/usr/bin/env python3
"""The classical odometry of ``icp_odometry`` with a polar nearest-neighbour
search, compiled (numba): the same estimates at a fraction of the cost.

``IcpStream`` finds every correspondence with ``torch.cdist``: all current
points against all target points (360 x 360 per scan-to-scan iteration,
360 x 1800 against the 5-keyframe submap, 2.6 MB), and runs ~2800 small
torch operations per scan. On the car's Raspberry Pi 4 that is ~80 % of the
estimator's time (``pi4_budget.py``, README section 10).

Here the target points are indexed by their bearing from the LiDAR origin
(360 bins, counting sort). The search for the neighbour of a point y at
range rho visits the bins outwards from y's bearing and stops as soon as the
bearing offset alone rules out anything closer: a point whose bearing
differs by dtheta is at least rho sin(dtheta) away (rho if dtheta >= 90
deg). The result is the exact nearest neighbour within the gate, the same
point ``cdist(...).min()`` returns whenever it is used (a pair is only used
within the gate), usually after a handful of candidates.

Everything else is a line-by-line port of ``IcpStream.step`` for one lane:
de-skew (constant rate or gyro heading profile), line normals (both patch
rules), robust point-to-line Gauss-Newton with the IMU prediction as prior,
scan-to-scan then scan-to-submap with the agreement test, keyframes,
covariance and degeneracy indicators, the gyro-bias Kalman filter and the
standstill update. It runs in float64 (the torch version in float32, whose
cdist expands |a|^2 + |b|^2 - 2ab and loses ~1e-3 of relative precision on
small distances), so the two agree to float precision except where that
precision flips a nearest neighbour or a gate.

Uses: ``IcpConfig(association="polar")`` (``ICP_VARIANTS["submap_v4_polar"]``)
in ``run_icp`` (whole splits, threads over runs) and ``PolarIcpStream`` in
``LiveOdometry`` (``icp_backend="polar"``). No LAPACK: the 2x2 and 3x3
algebra is closed form, so numba needs neither SciPy nor torch on the car.

    python polar_icp.py parity --reference A.pt --candidate B.pt   # two ICP caches of one split
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import math
import os
import time
from typing import Any

import numpy as np
from numba import njit
import torch

SENTINEL = 1.0e4  # coordinates of an invalid point (as icp_odometry)
NBINS = 360  # bearing bins of the search index
EPS_F32 = 1.1920929e-07

# Layout of the packed configuration.
P_FIELDS = ("model_sigma_m", "cauchy", "planarity_max", "prior_trans_sigma_m", "gyro_noise_rad", "bias_walk_rps", "gyro_scale_error",
            "standstill_accel_std", "standstill_rate_rps", "standstill_speed_mps", "standstill_sigma_m", "keyframe_dist_m", "keyframe_yaw_rad",
            "submap_agree_m", "submap_agree_rad", "submap_agree_rel")
I_FIELDS = ("iterations", "deskew_passes", "deskew", "deskew_translation", "deskew_profile", "normal_window", "normal_min_points", "min_pairs",
            "standstill", "submap_keyframes", "map_passes", "map_iterations")
# Scalars per interval: dt, gyro_integral, w_prev, w_cur, dt_next, accel x, accel y, accel_std.
S_DT, S_GYRO, S_WPREV, S_WCUR, S_DTNEXT, S_AX, S_AY, S_ASTD = range(8)
# State: gyro bias, its variance, keyframe count, velocity (2), pose (3), last keyframe pose (3).
ST_BG, ST_PBG, ST_KFN, ST_VX, ST_VY, ST_PX, ST_PY, ST_PYAW, ST_KX, ST_KY, ST_KYAW = range(11)
# Outputs: pred (3), sigma (3), weak_sigma_m, eig_ratio, icp_yaw_sigma_rad, gyro_bias, pairs, standstill, map_used, map_diff (2), yaw_icp, yaw_info.
O_PRED, O_SIGMA, O_WEAK, O_EIGR, O_YAWSIG, O_BG, O_PAIRS, O_STILL, O_MAPUSED, O_MAPDIFF, O_YAWICP, O_YAWINFO = 0, 3, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16
N_OUT = 17


# ------------------------------------------------------------------ kernels
@njit(cache=True, nogil=True)
def _interp(values: np.ndarray, duration: float, tau: float) -> float:
    """``sqlite_streams.sweep_interp`` for one value series and one time."""
    k = values.shape[0] - 1
    u = tau / max(duration, 1e-4) * k
    j0 = math.floor(u)
    if j0 < 0:
        j0 = 0
    if j0 > k - 1:
        j0 = k - 1
    w = u - j0
    return values[j0] + (values[j0 + 1] - values[j0]) * w


@njit(cache=True, nogil=True)
def _deskew(ranges, valid, rate, vx, vy, geo, beam_time, heading, duration, use_sweep, out):
    """``icp_odometry.deskew`` for one scan into ``out`` [N, 2]."""
    n = ranges.shape[0]
    time_inc, lx, ly, amin, ainc = geo[0], geo[1], geo[2], geo[3], geo[4]
    k = heading.shape[0] - 1
    trx = np.zeros(k + 1)
    tr_y = np.zeros(k + 1)
    if use_sweep:
        seg = duration / k
        for m in range(k):
            hm = 0.5 * (heading[m + 1] + heading[m])
            cm, sm = math.cos(hm), math.sin(hm)
            trx[m + 1] = trx[m] + (cm * vx - sm * vy) * seg
            tr_y[m + 1] = tr_y[m] + (sm * vx + cm * vy) * seg
    for i in range(n):
        if not valid[i]:
            out[i, 0] = SENTINEL
            out[i, 1] = SENTINEL
            continue
        tau = beam_time[i] if beam_time.shape[0] > 0 else i * time_inc
        theta = amin + i * ainc
        qx = lx + ranges[i] * math.cos(theta)
        qy = ly + ranges[i] * math.sin(theta)
        if use_sweep:
            psi = _interp(heading, duration, tau)
            px = _interp(trx, duration, tau)
            py = _interp(tr_y, duration, tau)
        else:
            psi = rate * tau
            ch, sh = math.cos(0.5 * psi), math.sin(0.5 * psi)
            px = (ch * vx - sh * vy) * tau
            py = (sh * vx + ch * vy) * tau
        c, s = math.cos(psi), math.sin(psi)
        out[i, 0] = c * qx - s * qy + px
        out[i, 1] = s * qx + c * qy + py


@njit(cache=True, nogil=True)
def _normals(pts, valid, ranges, ainc, planarity_max, window, min_points, normal, nvalid):
    """``icp_odometry.line_normals`` (both patch rules) into ``normal``, ``nvalid``."""
    n = pts.shape[0]
    legacy = window == 2 and min_points == 5
    size = 2 * window + 1
    for i in range(n):
        v = valid[i]
        a = 0.0
        b = 0.0
        c = 0.0
        if legacy:
            mx = 0.0
            my = 0.0
            for o in range(size):
                j = (i + o - window) % n
                if not valid[j]:
                    v = False
                mx += pts[j, 0]
                my += pts[j, 1]
            thr = 0.05 + 3.0 * ranges[i] * ainc
            for o in range(size - 1):
                j0 = (i + o - window) % n
                j1 = (i + o + 1 - window) % n
                if math.hypot(pts[j1, 0] - pts[j0, 0], pts[j1, 1] - pts[j0, 1]) >= thr:
                    v = False
            mx /= size
            my /= size
            for o in range(size):
                j = (i + o - window) % n
                dx, dy = pts[j, 0] - mx, pts[j, 1] - my
                a += dx * dx
                b += dx * dy
                c += dy * dy
            a /= size
            b /= size
            c /= size
        else:
            cnt = 0.0
            mx = 0.0
            my = 0.0
            for o in range(size):
                j = (i + o - window) % n
                off = abs(o - window)
                d = math.hypot(pts[j, 0] - pts[i, 0], pts[j, 1] - pts[i, 1])
                if valid[j] and valid[i] and d < 0.05 + 3.0 * ranges[i] * ainc * off:
                    cnt += 1.0
                    mx += pts[j, 0]
                    my += pts[j, 1]
            v = valid[i] and cnt >= min_points
            den = max(cnt, 1.0)
            mx /= den
            my /= den
            for o in range(size):
                j = (i + o - window) % n
                off = abs(o - window)
                d = math.hypot(pts[j, 0] - pts[i, 0], pts[j, 1] - pts[i, 1])
                if valid[j] and valid[i] and d < 0.05 + 3.0 * ranges[i] * ainc * off:
                    dx, dy = pts[j, 0] - mx, pts[j, 1] - my
                    a += dx * dx
                    b += dx * dy
                    c += dy * dy
            a /= den
            b /= den
            c /= den
        phi = 0.5 * math.atan2(2.0 * b, a - c)
        half = 0.5 * (a + c)
        root = math.sqrt(((a - c) * 0.5) ** 2 + b * b)
        ratio = (half - root) / max(half + root, 1e-12)
        nvalid[i] = v and ratio < planarity_max
        normal[i, 0] = -math.sin(phi)
        normal[i, 1] = math.cos(phi)


@njit(cache=True, nogil=True)
def _bin_of(dx: float, dy: float) -> int:
    b = int(math.floor((math.atan2(dy, dx) + math.pi) / (2.0 * math.pi) * NBINS))
    return min(max(b, 0), NBINS - 1)


@njit(cache=True, nogil=True)
def _build_index(pts, ox, oy, order, start):
    """Counting sort of the valid points by bearing bin (CSR: ``start``
    [NBINS + 1], ``order``)."""
    m = pts.shape[0]
    bins = np.empty(m, np.int64)
    counts = np.zeros(NBINS + 1, np.int64)
    for j in range(m):
        if abs(pts[j, 0]) >= 1.0e3 or abs(pts[j, 1]) >= 1.0e3:
            bins[j] = -1
            continue
        b = _bin_of(pts[j, 0] - ox, pts[j, 1] - oy)
        bins[j] = b
        counts[b + 1] += 1
    start[0] = 0
    for b in range(NBINS):
        start[b + 1] = start[b] + counts[b + 1]
    cursor = start[:NBINS].copy()
    for j in range(m):
        b = bins[j]
        if b >= 0:
            order[cursor[b]] = j
            cursor[b] += 1


@njit(cache=True, nogil=True)
def _nearest(yx, yy, pts, order, start, ox, oy, gate):
    """Index of the nearest point strictly within ``gate`` of (yx, yy), -1
    if none: bins visited outwards from y's bearing until the bearing
    offset proves that no remaining point can be closer."""
    if start[NBINS] == 0:
        return -1
    dx, dy = yx - ox, yy - oy
    rho = math.hypot(dx, dy)
    b0 = _bin_of(dx, dy)
    step = 2.0 * math.pi / NBINS
    best = gate * gate
    bestj = -1
    for k in range(NBINS // 2 + 1):
        if k >= 1:
            ang = (k - 1) * step - 1e-9  # bearing offset to any point of bins b0 +- k
            lb = rho if ang >= 0.5 * math.pi else rho * math.sin(max(ang, 0.0))
            if lb * lb >= best:
                break
        for sgn in (1, -1):
            if sgn == -1 and (k == 0 or 2 * k == NBINS):
                continue
            b = (b0 + sgn * k) % NBINS
            for q in range(start[b], start[b + 1]):
                j = order[q]
                ex, ey = pts[j, 0] - yx, pts[j, 1] - yy
                d2 = ex * ex + ey * ey
                if d2 < best:
                    best = d2
                    bestj = j
    return bestj


@njit(cache=True, nogil=True)
def _inv3(h, out):
    """Inverse of a 3x3 matrix (adjugate), returns the determinant."""
    a, b, c = h[0, 0], h[0, 1], h[0, 2]
    d, e, f = h[1, 0], h[1, 1], h[1, 2]
    g, hh, i = h[2, 0], h[2, 1], h[2, 2]
    c00, c01, c02 = e * i - f * hh, -(d * i - f * g), d * hh - e * g
    det = a * c00 + b * c01 + c * c02
    inv = 1.0 / det
    out[0, 0], out[1, 0], out[2, 0] = c00 * inv, c01 * inv, c02 * inv
    out[0, 1] = -(b * i - c * hh) * inv
    out[1, 1] = (a * i - c * g) * inv
    out[2, 1] = -(a * hh - b * g) * inv
    out[0, 2] = (b * f - c * e) * inv
    out[1, 2] = -(a * f - c * d) * inv
    out[2, 2] = (a * e - b * d) * inv
    return det


@njit(cache=True, nogil=True)
def _sym2_eig(a, b, c):
    """Eigenvalues (ascending) and the eigenvector angle of [[a, b], [b, c]]."""
    t = 0.5 * math.atan2(2.0 * b, a - c)
    ct, st = math.cos(t), math.sin(t)
    l1 = a * ct * ct + 2.0 * b * st * ct + c * st * st  # along (cos t, sin t)
    l2 = a * st * st - 2.0 * b * st * ct + c * ct * ct  # along (-sin t, cos t)
    return l1, l2, t


@njit(cache=True, nogil=True)
def _register(prev_r, prev_v, cur_r, cur_v, x_prior, x0, lam, rate_prev, rate_cur, sig_pt, geo, beam_time, dt,
              map_pts, map_nrm, map_ok, use_map, h_prev, dur_prev, h_cur, dur_cur, use_sweep, P, I, gates, floors,
              x, h, h_icp, w, e, use, tgt, normal, nvalid, src, order, start):
    """``icp_odometry._register`` for one lane: de-skew passes + robust
    Gauss-Newton against the previous scan or the local map. Writes x, h,
    h_icp, w, e, use and the de-skewed target scan; returns ``enough``."""
    n = cur_r.shape[0]
    lx, ly = geo[1], geo[2]
    cauchy, planarity_max = P[1], P[2]
    if use_map:
        passes, iters, first = I[10], I[11], 2
    else:
        passes, iters, first = I[1], I[0], 0
    deskew, deskew_tr, window, min_points, min_pairs = I[2], I[3], I[5], I[6], I[7]
    x[:] = x0
    enough = False
    for _ in range(passes):
        if deskew and deskew_tr:
            vx, vy = x[0] / dt, x[1] / dt
        else:
            vx, vy = 0.0, 0.0
        _deskew(prev_r, prev_v, rate_prev, vx, vy, geo, beam_time, h_prev, dur_prev, use_sweep, tgt)
        cr, sr = math.cos(-x[2]), math.sin(-x[2])
        _deskew(cur_r, cur_v, rate_cur, cr * vx - sr * vy, sr * vx + cr * vy, geo, beam_time, h_cur, dur_cur, use_sweep, src)
        _normals(tgt, prev_v, prev_r, geo[4], planarity_max, window, min_points, normal, nvalid)
        if use_map:
            target, t_nrm, t_ok = map_pts, map_nrm, map_ok
        else:
            target, t_nrm, t_ok = tgt, normal, nvalid
        _build_index(target, lx, ly, order, start)
        for it in range(first, first + iters):
            gate = gates[min(it, gates.shape[0] - 1)]
            floor = floors[min(it, floors.shape[0] - 1)]
            c, s = math.cos(x[2]), math.sin(x[2])
            h_icp[:, :] = 0.0
            g0 = 0.0
            g1 = 0.0
            g2 = 0.0
            cnt = 0
            for i in range(n):
                use[i] = False
                w[i] = 0.0
                e[i] = 0.0
                if not cur_v[i]:
                    continue
                sx, sy = src[i, 0], src[i, 1]
                yx = c * sx - s * sy + x[0]
                yy = s * sx + c * sy + x[1]
                j = _nearest(yx, yy, target, order, start, lx, ly, gate)
                if j < 0 or not t_ok[j]:
                    continue
                nx, ny = t_nrm[j, 0], t_nrm[j, 1]
                ei = (yx - target[j, 0]) * nx + (yy - target[j, 1]) * ny
                dyx = -s * sx - c * sy
                dyy = c * sx - s * sy
                j0, j1, j2 = nx, ny, nx * dyx + ny * dyy
                kern = max(cauchy * sig_pt[i], floor)
                wi = 1.0 / (sig_pt[i] * sig_pt[i]) / (1.0 + (ei / kern) ** 2)
                use[i] = True
                w[i] = wi
                e[i] = ei
                cnt += 1
                h_icp[0, 0] += wi * j0 * j0
                h_icp[0, 1] += wi * j0 * j1
                h_icp[0, 2] += wi * j0 * j2
                h_icp[1, 1] += wi * j1 * j1
                h_icp[1, 2] += wi * j1 * j2
                h_icp[2, 2] += wi * j2 * j2
                g0 += wi * j0 * ei
                g1 += wi * j1 * ei
                g2 += wi * j2 * ei
            h_icp[1, 0], h_icp[2, 0], h_icp[2, 1] = h_icp[0, 1], h_icp[0, 2], h_icp[1, 2]
            h[:, :] = h_icp
            for q in range(3):
                h[q, q] += lam[q]
            g0 += lam[0] * (x[0] - x_prior[0])
            g1 += lam[1] * (x[1] - x_prior[1])
            g2 += lam[2] * (x[2] - x_prior[2])
            enough = cnt >= min_pairs
            if enough:
                hi = np.empty((3, 3))
                _inv3(h, hi)
                for q in range(3):
                    x[q] -= hi[q, 0] * g0 + hi[q, 1] * g1 + hi[q, 2] * g2
            else:
                x[:] = x_prior
    return enough


@njit(cache=True, nogil=True)
def step_kernel(prev_r, prev_v, cur_r, cur_v, sc, prof_prev, prof_cur, geo, beam_time, lidar_sig, P, I, gates, floors,
                st, kf_pts, kf_nrm, kf_ok, out, prev_points, prev_points_valid):
    """One interval of ``IcpStream.step`` for one lane (state ``st``,
    keyframes updated in place; results in ``out`` and the de-skewed
    previous scan)."""
    n = cur_r.shape[0]
    kf = I[9]
    dt = sc[S_DT]
    gyro = sc[S_GYRO]
    bg = st[ST_BG]
    p_bg = st[ST_PBG] + P[5] ** 2
    vx, vy = st[ST_VX], st[ST_VY]
    ax, ay = sc[S_AX], sc[S_AY]
    # prediction: constant velocity + specific force, gyro minus bias
    x_prior = np.array([vx * dt + 0.5 * ax * dt * dt, vy * dt + 0.5 * ay * dt * dt, gyro - bg * dt])
    sig_t = P[3]
    sig_y = math.sqrt(P[4] ** 2 + p_bg * dt * dt)
    still = False
    if I[8]:
        still = sc[S_ASTD] < P[7] and abs(gyro / dt - bg) < P[8] and math.hypot(vx, vy) < P[9]
        if still:
            x_prior[:] = 0.0
            sig_t = P[10]
            sig_y = P[4] * 0.2
    lam = np.array([sig_t ** -2, sig_t ** -2, sig_y ** -2])
    rate_prev, rate_cur = sc[S_WPREV] - bg, sc[S_WCUR] - bg
    if not I[2]:
        rate_prev, rate_cur = 0.0, 0.0
    use_sweep = bool(I[2]) and bool(I[4])
    kk = prof_prev.shape[0]
    h_prev = np.empty(kk)
    h_cur = np.empty(kk)
    for m in range(kk):
        f = m / (kk - 1)
        h_prev[m] = prof_prev[m] - bg * dt * f
        h_cur[m] = prof_cur[m] - bg * sc[S_DTNEXT] * f
    sig_pt = np.empty(n)
    for i in range(n):
        sig_pt[i] = P[0] + lidar_sig[0] + lidar_sig[1] * cur_r[i]
    # work arrays
    x = np.empty(3)
    h = np.empty((3, 3))
    h_icp = np.empty((3, 3))
    w = np.empty(n)
    e = np.empty(n)
    use = np.empty(n, np.bool_)
    tgt = np.empty((n, 2))
    normal = np.empty((n, 2))
    nvalid = np.empty(n, np.bool_)
    src = np.empty((n, 2))
    m_all = max(kf * n, n)
    order = np.empty(m_all, np.int64)
    start = np.empty(NBINS + 1, np.int64)
    empty2 = np.empty((0, 2))
    empty1 = np.empty(0, np.bool_)
    # 1) scan-to-scan
    enough = _register(prev_r, prev_v, cur_r, cur_v, x_prior, x_prior, lam, rate_prev, rate_cur, sig_pt, geo, beam_time, dt,
                       empty2, empty2, empty1, False, h_prev, dt, h_cur, sc[S_DTNEXT], use_sweep, P, I, gates, floors,
                       x, h, h_icp, w, e, use, tgt, normal, nvalid, src, order, start)
    map_used = 0.0
    map_diff0, map_diff1 = np.nan, np.nan
    px, py, pyaw = st[ST_PX], st[ST_PY], st[ST_PYAW]
    if kf > 0:
        # 2) refinement against the local map, from the scan-to-scan solution
        cm, sm = math.cos(-pyaw), math.sin(-pyaw)
        mp = np.empty((kf * n, 2))
        mn = np.empty((kf * n, 2))
        mo = np.empty(kf * n, np.bool_)
        for q in range(kf):
            for i in range(n):
                r = q * n + i
                dx, dy = kf_pts[q, i, 0] - px, kf_pts[q, i, 1] - py
                if kf_ok[q, i]:
                    mp[r, 0] = cm * dx - sm * dy
                    mp[r, 1] = sm * dx + cm * dy
                else:
                    mp[r, 0] = SENTINEL
                    mp[r, 1] = SENTINEL
                mn[r, 0] = cm * kf_nrm[q, i, 0] - sm * kf_nrm[q, i, 1]
                mn[r, 1] = sm * kf_nrm[q, i, 0] + cm * kf_nrm[q, i, 1]
                mo[r] = kf_ok[q, i]
        x2 = np.empty(3)
        h2 = np.empty((3, 3))
        h2_icp = np.empty((3, 3))
        w2 = np.empty(n)
        e2 = np.empty(n)
        use2 = np.empty(n, np.bool_)
        tgt2 = np.empty((n, 2))
        normal2 = np.empty((n, 2))
        nvalid2 = np.empty(n, np.bool_)
        enough2 = _register(prev_r, prev_v, cur_r, cur_v, x_prior, x.copy(), lam, rate_prev, rate_cur, sig_pt, geo, beam_time, dt,
                            mp, mn, mo, True, h_prev, dt, h_cur, sc[S_DTNEXT], use_sweep, P, I, gates, floors,
                            x2, h2, h2_icp, w2, e2, use2, tgt2, normal2, nvalid2, src, order, start)
        dxy = math.hypot(x2[0] - x[0], x2[1] - x[1])
        dyaw = abs(x2[2] - x[2])
        map_diff0, map_diff1 = dxy, dyaw
        agree_m = max(P[15] * math.hypot(x[0], x[1]), P[13])
        kfn = st[ST_KFN]
        agree = kfn > 0 and enough2 and dxy < agree_m and dyaw < P[14]
        if kfn > 0 and not agree:  # disagreement: keep scan-to-scan, restart the local map
            kf_ok[:, :] = False
            st[ST_KFN] = 0.0
        if agree:
            x[:] = x2
            h[:, :] = h2
            h_icp[:, :] = h2_icp
            w[:] = w2
            e[:] = e2
            use[:] = use2
            enough = enough2
            map_used = 1.0
    # uncertainty: MAP covariance scaled by the residual chi^2 per dof
    npairs = 0
    num = 0.0
    for i in range(n):
        if use[i]:
            npairs += 1
        num += w[i] * e[i] * e[i]
    chi2 = max(num / max(npairs, 1), 1.0)
    cov = np.empty((3, 3))
    _inv3(h, cov)
    a2, b2, c2 = h_icp[0, 0], h_icp[0, 1], h_icp[1, 1]
    l1, l2, t = _sym2_eig(a2, b2, c2)
    lo, hi_ = (l1, l2) if l1 <= l2 else (l2, l1)
    lo, hi_ = max(lo, 1e-6), max(hi_, 1e-6)
    # ICP-only heading information: translation marginalised with a pseudo-inverse (float32 tolerance, as torch)
    tol = 2.0 * EPS_F32 * max(abs(l1), abs(l2))
    ct, st_ = math.cos(t), math.sin(t)
    u1x, u1y, u2x, u2y = ct, st_, -st_, ct
    hx, hy = h_icp[0, 2], h_icp[1, 2]
    quad = 0.0
    if abs(l1) > tol and abs(l1) > 0.0:
        pr = u1x * hx + u1y * hy
        quad += pr * pr / l1
    if abs(l2) > tol and abs(l2) > 0.0:
        pr = u2x * hx + u2y * hy
        quad += pr * pr / l2
    yaw_info = max(h_icp[2, 2] - quad, 1e-6) / chi2
    yaw_icp = x[2] + (lam[2] / yaw_info) * (x[2] - x_prior[2])
    # gyro-bias Kalman update
    rate = gyro / dt
    if still:
        meas = rate
        r_meas = P[4] ** 2 / (dt * dt)
    else:
        meas = (gyro - yaw_icp) / dt
        r_meas = (1.0 / yaw_info + P[4] ** 2 + (P[6] * rate * dt) ** 2) / (dt * dt)
    innov = meas - bg
    ok = (enough or still) and innov * innov < 9.0 * (p_bg + r_meas)
    if ok:
        gain = p_bg / (p_bg + r_meas)
        st[ST_BG] = bg + gain * innov
        st[ST_PBG] = (1.0 - gain) * p_bg
    else:
        st[ST_PBG] = p_bg
    # velocity at the end of the interval, in the frame of scan k
    vex = x[0] / dt + 0.5 * ax * dt
    vey = x[1] / dt + 0.5 * ay * dt
    cr, sr = math.cos(-x[2]), math.sin(-x[2])
    st[ST_VX] = cr * vex - sr * vey
    st[ST_VY] = sr * vex + cr * vey
    if kf > 0:
        moved = math.hypot(px - st[ST_KX], py - st[ST_KY])
        dk = pyaw - st[ST_KYAW]
        turned = abs(math.atan2(math.sin(dk), math.cos(dk)))
        if st[ST_KFN] == 0 or moved > P[11] or turned > P[12]:
            slot = int(st[ST_KFN]) % kf
            cp, sp = math.cos(pyaw), math.sin(pyaw)
            for i in range(n):
                kf_pts[slot, i, 0] = cp * tgt[i, 0] - sp * tgt[i, 1] + px
                kf_pts[slot, i, 1] = sp * tgt[i, 0] + cp * tgt[i, 1] + py
                kf_nrm[slot, i, 0] = cp * normal[i, 0] - sp * normal[i, 1]
                kf_nrm[slot, i, 1] = sp * normal[i, 0] + cp * normal[i, 1]
                kf_ok[slot, i] = nvalid[i]
            st[ST_KX], st[ST_KY], st[ST_KYAW] = px, py, pyaw
            st[ST_KFN] += 1.0
    cp, sp = math.cos(pyaw), math.sin(pyaw)
    st[ST_PX] = px + cp * x[0] - sp * x[1]
    st[ST_PY] = py + sp * x[0] + cp * x[1]
    st[ST_PYAW] = pyaw + x[2]
    out[O_PRED:O_PRED + 3] = x
    for q in range(3):
        out[O_SIGMA + q] = math.sqrt(cov[q, q] * chi2)
    out[O_WEAK] = lo ** -0.5
    out[O_EIGR] = lo / hi_
    out[O_YAWSIG] = yaw_info ** -0.5
    out[O_BG] = st[ST_BG]
    out[O_PAIRS] = npairs
    out[O_STILL] = 1.0 if still else 0.0
    out[O_MAPUSED] = map_used
    out[O_MAPDIFF] = map_diff0
    out[O_MAPDIFF + 1] = map_diff1
    out[O_YAWICP] = yaw_icp
    out[O_YAWINFO] = yaw_info
    prev_points[:, :] = tgt
    prev_points_valid[:] = nvalid


# --------------------------------------------------------------- the stream
def pack_config(cfg: Any) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    return (np.array([float(getattr(cfg, k)) for k in P_FIELDS]), np.array([int(getattr(cfg, k)) for k in I_FIELDS], dtype=np.int64),
            np.asarray(cfg.gates_m, dtype=np.float64), np.asarray(cfg.robust_floor_m, dtype=np.float64))


class PolarIcpState:
    """Causal state of one lane (a run or the live car)."""

    def __init__(self, cfg: Any, beams: int) -> None:
        kf = int(cfg.submap_keyframes)
        self.st = np.zeros(11)
        self.st[ST_PBG] = cfg.bias_sigma0_rps ** 2
        self.kf_pts = np.full((kf, beams, 2), SENTINEL)
        self.kf_nrm = np.zeros((kf, beams, 2))
        self.kf_ok = np.zeros((kf, beams), dtype=np.bool_)


def _f64(t: Any) -> np.ndarray:
    return np.ascontiguousarray(t.detach().cpu().numpy() if torch.is_tensor(t) else t, dtype=np.float64)


class PolarIcpStream:
    """Drop-in for ``IcpStream`` with one lane (the live node): same
    constructor and ``step(b, act)`` returning the same fields as tensors."""

    def __init__(self, cfg: Any, geo: tuple, lidar_sigma: torch.Tensor, beams: int, device: torch.device) -> None:
        if lidar_sigma.shape[0] != 1:
            raise ValueError("PolarIcpStream runs one lane")
        self.cfg, self.device, self.beams = cfg, device, beams
        t_inc, lxy, amin, ainc, beam_time = geo
        self.geo = np.array([float(t_inc[0]), float(lxy[0, 0]), float(lxy[0, 1]), float(amin[0]), float(ainc[0])])
        self.beam_time = _f64(beam_time[0]) if beam_time is not None else np.empty(0)
        self.sig = _f64(lidar_sigma[0])
        self.P, self.I, self.gates, self.floors = pack_config(cfg)
        self.state = PolarIcpState(cfg, beams)
        self.out = np.zeros(N_OUT)
        self.prev_points = np.zeros((beams, 2))
        self.prev_valid = np.zeros(beams, dtype=np.bool_)

    def step(self, b: dict[str, torch.Tensor], act: torch.Tensor) -> dict[str, torch.Tensor]:
        if not bool(act[0]):
            raise ValueError("PolarIcpStream: every step is active")
        sc = np.array([float(b["dt"][0]), float(b["gyro_integral"][0]), float(b["w_prev"][0]), float(b["w_cur"][0]), float(b["dt_next"][0]),
                       float(b["accel"][0, 0]), float(b["accel"][0, 1]), float(b["accel_std"][0])])
        run_step(self, _f64(b["prev_ranges"][0]), b["prev_valid"][0].cpu().numpy(), _f64(b["cur_ranges"][0]), b["cur_valid"][0].cpu().numpy(), sc,
                 _f64(b["prof_prev"][0]), _f64(b["prof_cur"][0]))
        return result_tensors(self.out, self.prev_points, self.prev_valid, self.device)


def run_step(s: PolarIcpStream, prev_r, prev_v, cur_r, cur_v, sc, prof_prev, prof_cur) -> None:
    st = s.state
    step_kernel(prev_r, prev_v, cur_r, cur_v, sc, prof_prev, prof_cur, s.geo, s.beam_time, s.sig, s.P, s.I, s.gates, s.floors,
                st.st, st.kf_pts, st.kf_nrm, st.kf_ok, s.out, s.prev_points, s.prev_valid)


def result_tensors(out: np.ndarray, prev_points: np.ndarray, prev_valid: np.ndarray, device: torch.device) -> dict[str, torch.Tensor]:
    f = lambda v: torch.tensor([v], dtype=torch.float32, device=device)  # noqa: E731
    return {"pred": torch.tensor(out[None, O_PRED:O_PRED + 3], dtype=torch.float32, device=device),
            "sigma": torch.tensor(out[None, O_SIGMA:O_SIGMA + 3], dtype=torch.float32, device=device),
            "weak_sigma_m": f(out[O_WEAK]), "eig_ratio": f(out[O_EIGR]), "icp_yaw_sigma_rad": f(out[O_YAWSIG]), "gyro_bias": f(out[O_BG]),
            "pairs": f(out[O_PAIRS]), "standstill": f(out[O_STILL]), "map_used": f(out[O_MAPUSED]),
            "map_diff": torch.tensor(out[None, O_MAPDIFF:O_MAPDIFF + 2], dtype=torch.float32, device=device),
            "yaw_icp": f(out[O_YAWICP]), "yaw_info": f(out[O_YAWINFO]),
            "prev_points": torch.tensor(prev_points[None], dtype=torch.float32, device=device),
            "prev_points_valid": torch.tensor(prev_valid[None], device=device)}


# ------------------------------------------------------------ whole splits
LOG_FIELDS = (("pred", 3), ("sigma", 3), ("weak_sigma_m", 0), ("eig_ratio", 0), ("icp_yaw_sigma_rad", 0), ("gyro_bias", 0), ("pairs", 0),
              ("standstill", 0), ("map_used", 0), ("map_diff", 2), ("yaw_icp", 0), ("yaw_info", 0))
SLOTS = {"pred": O_PRED, "sigma": O_SIGMA, "weak_sigma_m": O_WEAK, "eig_ratio": O_EIGR, "icp_yaw_sigma_rad": O_YAWSIG, "gyro_bias": O_BG,
         "pairs": O_PAIRS, "standstill": O_STILL, "map_used": O_MAPUSED, "map_diff": O_MAPDIFF, "yaw_icp": O_YAWICP, "yaw_info": O_YAWINFO}


def _run_one(cfg: Any, inputs: dict[str, np.ndarray], geo: np.ndarray, beam_time: np.ndarray, sig: np.ndarray, beams: int) -> np.ndarray:
    """Every interval of one run: [intervals, N_OUT]."""
    P, I, gates, floors = pack_config(cfg)
    state = PolarIcpState(cfg, beams)
    n = inputs["sc"].shape[0]
    res = np.zeros((n, N_OUT))
    pp, pv = np.zeros((beams, 2)), np.zeros(beams, dtype=np.bool_)
    for k in range(n):
        step_kernel(inputs["prev_r"][k], inputs["prev_v"][k], inputs["cur_r"][k], inputs["cur_v"][k], inputs["sc"][k], inputs["prof_prev"][k],
                    inputs["prof_cur"][k], geo, beam_time, sig, P, I, gates, floors, state.st, state.kf_pts, state.kf_nrm, state.kf_ok, res[k], pp, pv)
    return res


def run_icp_polar(sd: Any, cfg: Any, log=print, workers: int | None = None) -> dict[str, Any]:
    """``icp_odometry.run_icp`` with the polar backend: the inputs of every
    interval exactly as ``StreamData.gather`` gives them to ``IcpStream``,
    runs processed in parallel threads (the kernel releases the GIL)."""
    n_total = sd.target.shape[0]
    logs = {name: np.full((n_total,) + ((dim,) if dim else ()), np.nan) for name, dim in LOG_FIELDS}
    beams = int(sd.lidar.shape[-1])
    t0 = time.time()
    jobs = []
    for r in range(len(sd.runs)):
        off, length = int(sd.offset[r]), int(sd.length[r])
        if length < 2:
            continue
        ids = torch.arange(off + 1, off + length, device=sd.device)
        b = {k: v[0] for k, v in sd.gather(ids[None]).items()}
        rr = torch.tensor([r], device=sd.device)
        bt = sd.beam_time(rr)
        inputs = {"prev_r": _f64(b["prev_ranges"]), "prev_v": b["prev_valid"].cpu().numpy(), "cur_r": _f64(b["cur_ranges"]), "cur_v": b["cur_valid"].cpu().numpy(),
                  "sc": np.stack([_f64(b["dt"]), _f64(b["gyro_integral"]), _f64(b["w_prev"]), _f64(b["w_cur"]), _f64(b["dt_next"]),
                                  _f64(b["accel"][:, 0]), _f64(b["accel"][:, 1]), _f64(b["accel_std"])], axis=1),
                  "prof_prev": _f64(b["prof_prev"]), "prof_cur": _f64(b["prof_cur"])}
        geo = np.array([float(sd.time_increment[r]), float(sd.lidar_xy[r, 0]), float(sd.lidar_xy[r, 1]), float(sd.angle_min[r]), float(sd.angle_increment[r])])
        jobs.append((ids.cpu().numpy(), inputs, geo, _f64(bt[0]) if bt is not None else np.empty(0), _f64(sd.lidar_sigma[r])))
    with ThreadPoolExecutor(max_workers=workers or min(8, os.cpu_count() or 1)) as pool:
        results = list(pool.map(lambda j: _run_one(cfg, j[1], j[2], j[3], j[4], beams), jobs))
    for (ids, *_), res in zip(jobs, results):
        for name, dim in LOG_FIELDS:
            s = SLOTS[name]
            logs[name][ids] = res[:, s:s + dim] if dim else res[:, s]
    log(f"[icp {sd.split}] polar backend: {len(jobs)} runs in {time.time() - t0:.0f} s")
    return {**{k: torch.from_numpy(v.astype(np.float32)) for k, v in logs.items()}, "config": asdict(cfg)}


# ------------------------------------------------------------------ parity
def parity(reference: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Any]:
    """Per-field differences between two ICP results of the same split."""
    out = {}
    ok = torch.isfinite(reference["pred"]).all(1) & torch.isfinite(candidate["pred"]).all(1)
    for name, _ in LOG_FIELDS:
        a, b = reference[name][ok].double(), candidate[name][ok].double()
        d = (a - b).abs().reshape(len(a), -1).nan_to_num(0.0).max(1).values.numpy()
        out[name] = {"p50": float(np.percentile(d, 50)), "p99": float(np.percentile(d, 99)), "max": float(d.max()),
                     "frac_gt_1e-4": float((d > 1e-4).mean())}
    out["intervals"] = int(ok.sum())
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("parity", help="differences between two ICP caches of one split")
    p.add_argument("--reference", required=True)
    p.add_argument("--candidate", required=True)
    args = ap.parse_args()
    ref = torch.load(args.reference, weights_only=False)
    cand = torch.load(args.candidate, weights_only=False)
    for k, v in parity(ref, cand).items():
        print(k, v)


if __name__ == "__main__":
    main()
