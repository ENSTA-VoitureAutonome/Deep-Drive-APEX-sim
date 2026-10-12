"""A bicycle with the plant the identifier is supposed to find.

Deliberately not the simulation's ``Actuators``: the point of the unit tests is
that the estimator recovers *known* numbers, so the plant here is written from
the same parameters the estimator carries, and the simulation's actuators are
what the harness and Gazebo then check it against.
"""

from __future__ import annotations

import math

import numpy as np


class Plant:
    def __init__(self, gain_l: float = 1.0, gain_r: float = 1.0, trim: float = 0.0, slip: float = 0.0,
                 arm: float = 0.15, v_gain: float = 1.0, v_off: float = 0.0, dead: float = 0.0,
                 tau_k: float = 0.12, tau_v: float = 0.22, sign: float = 1.0) -> None:
        self.__dict__.update(gain_l=gain_l, gain_r=gain_r, trim=trim, slip=slip, arm=arm, v_gain=v_gain,
                             v_off=v_off, dead=dead, tau_k=tau_k, tau_v=tau_v, sign=sign)
        self.x = self.y = self.yaw = 0.0  # rear axle
        self.k = self.v = 0.0

    def step(self, dt: float, kappa_cmd: float, v_cmd: float) -> None:
        kc = self.sign * kappa_cmd
        gain = (self.gain_r if kc < 0.0 else self.gain_l) + self.slip * self.v * self.v
        self.k += (1.0 - math.exp(-dt / self.tau_k)) * (gain * kc + self.trim - self.k)
        over = math.copysign(max(0.0, abs(v_cmd) - self.dead), v_cmd) if self.dead > 0.0 else v_cmd
        target = self.v_gain * over + math.copysign(self.v_off, over) if over else 0.0
        self.v += (1.0 - math.exp(-dt / self.tau_v)) * (target - self.v)
        ds = self.v * dt
        dyaw = self.k * ds
        if abs(dyaw) > 1e-9:
            dx, dy = math.sin(dyaw) / self.k, (1.0 - math.cos(dyaw)) / self.k
        else:
            dx, dy = ds, 0.0
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += c * dx - s * dy
        self.y += s * dx + c * dy
        self.yaw += dyaw

    def report(self) -> tuple[float, float, float]:
        """Pose of the point the odometry reports (``arm`` ahead of the axle)."""
        return self.x + self.arm * math.cos(self.yaw), self.y + self.arm * math.sin(self.yaw), self.yaw


SIGMA = (0.0032, 0.0032, 0.0009)  # per-increment sigma of the trained odometry


def drive(ident, plant: Plant, duration: float, cmd, rng=None, sigma=SIGMA, noise=True,
          control_dt: float = 0.02, scan_dt: float = 0.1, shaper=None, model=None):
    """Run the plant under ``cmd(t) -> (kappa, v)``, feeding the identifier.

    The commands go in at the control rate and the estimates at the revolution
    rate, with the noise and the sigma the trained odometry reports, which is
    the only channel the real car has.
    """
    rng = rng or np.random.default_rng(7)
    t, next_scan, steps = 0.0, scan_dt, int(round(duration / control_dt))
    pose_err = np.zeros(3)
    applied = (0.0, 0.0)
    for _ in range(steps):
        kappa, v = cmd(t)
        if shaper is not None:
            v, kappa = shaper.shape(t, kappa, v, plant.v, model, shaper.envelope(model))
        ident.push_command(t, kappa, v)
        # The actuation acts one control period after the command is published,
        # as the bridge does; the identifier assumes exactly that (cmd_delay_s).
        for _ in range(10):
            plant.step(control_dt / 10.0, *applied)
        applied = (kappa, v)
        t += control_dt
        if t >= next_scan - 1e-9:
            next_scan += scan_dt
            px, py, pyaw = plant.report()
            if noise:
                pose_err += rng.normal(0.0, sigma, 3)
            ident.on_estimate(t, (px + pose_err[0], py + pose_err[1], pyaw + pose_err[2]), sigma)
    return t
