"""Hold the learned odometry still while the geometry says nothing moves.

The network was trained on streams that are almost always moving: the dataset's
stationary prefix is 1.5 s, the training chunks are 3.2 s and the ``age``
feature saturates at 5 s. Standing still for minutes is therefore outside
everything it has seen, and measured offline (``tools/analysis/standstill_drift.py``)
it extrapolates badly: on a parked stream the residual head latches at nearly
its full +-0.01 m per interval authority and the measurement head adds another
2 mm to the LiDAR's zero, for a steady **7 m/min** of invented forward motion.
The same signature was measured on the real car (0.42 m/min fresh, 0.97 after
one 7 cm pulse, 4.78 after a series).

None of that comes from the sensors. The classical ICP inside the same fusion
already detects the standstill (97-99 % of parked revolutions) and already
applies a zero-velocity update, and its increment over three parked minutes
integrates to about a millimetre. So the fix is not to re-estimate anything: it
is to stop *overriding* the part of the fusion that is right.

While a standstill is confirmed:

* the increment is the ICP's own zero-velocity solution, not the network's;
* the network is **not stepped at all**, so its recurrent memory stays exactly
  as it was when the car stopped -- a state the training distribution contains,
  instead of one extrapolated minutes past anything it was trained on. This is
  what bounds the damage of a momentary release: the first scan after a drop
  costs the "just stopped" increment (~2 mm) and not the latched one (~11 mm);
* the velocity state is set to zero, which is the one thing about a parked car
  that is known exactly.

Entering and staying are **not the same test**, and that is the heart of the
detector. Entering needs the ICP's standstill flag, confirmed over
``confirm_scans`` revolutions. Staying must not, because that flag carries a
vibration term (``accel_std < 0.15``) that cannot tell a parked car from a
driving one: measured on the real2sim profile, a parked revolution averages
0.087 and reaches 0.17, while 1.7 % of *driving* revolutions sit below 0.15. On
a parked stream it drops for about one revolution in a hundred, and handing
those single revolutions back to the network costs more than everything else
put together. So the hold stays on geometry alone, which is the direct evidence:
the LiDAR increment, the heading increment, and whether the scan is conditioned
well enough for the first two to mean anything.

The release is immediate -- the first revolution whose geometry does not qualify
ends the hold -- because the cost of holding one revolution too long (14 cm at
race speed) is much larger than the cost of releasing one too early (2 mm). At
0.35 m/s, the slowest speed the driver ever commands, one revolution is 35 mm of
travel against a 6 mm gate, so the first moving revolution releases the hold.

The thresholds are measured, not guessed. On a parked stream the ICP's own
increment is 0.94 mm on average, 2.1 mm at the 99th percentile and 2.8 mm at its
worst, and its heading increment stays under 0.002 deg: the gates sit at roughly
twice the worst case and a hundred times below any real motion. The conditioning
gate is the backstop for the one case where a small LiDAR increment proves
nothing -- a scene that cannot observe the direction the car is moving in. On the
validated real2sim run the ratio of the two information eigenvalues is 0.25 in
the median while parked (0.13 at the 1st percentile) and 0.07 while driving
(0.017 at the 1st percentile), so a gate at 0.08 keeps parked revolutions and
releases degenerate moving ones.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class StandstillConfig:
    enable: bool = True
    confirm_scans: int = 3  # consecutive qualifying revolutions before the hold starts
    max_step_m: float = 0.006  # the ICP's own translation must also be this small
    max_yaw_rad: float = 0.001  # ... and its heading increment this small (0.057 deg)
    min_eig_ratio: float = 0.08  # ... in a scan that can observe translation at all


@dataclass
class StandstillHold:
    """The confirm / release state machine. Pure: floats in, a decision out."""

    cfg: StandstillConfig = field(default_factory=StandstillConfig)
    holding: bool = False
    run: int = 0  # consecutive qualifying revolutions
    entries: int = 0  # times the hold started
    held: int = 0  # revolutions held
    released: int = 0  # times a confirmed hold was released

    def update(self, standstill: bool, step_m: float, yaw_rad: float, eig_ratio: float = 1.0) -> bool:
        """Decide for this revolution.

        ``standstill`` is the ICP's own flag, ``step_m``/``yaw_rad`` its
        increment and ``eig_ratio`` the ratio of the weak to the strong
        information eigenvalue of its solve (1 is isotropic, 0 degenerate).
        """
        cfg = self.cfg
        if not cfg.enable:
            self.holding, self.run = False, 0
            return False
        # Geometry alone: what keeps a confirmed hold, and what releases it.
        geometry = step_m <= cfg.max_step_m and abs(yaw_rad) <= cfg.max_yaw_rad and eig_ratio >= cfg.min_eig_ratio
        if not geometry:
            if self.holding:
                self.released += 1
            self.holding, self.run = False, 0
            return False
        if self.holding:
            self.held += 1
            return True
        self.run = self.run + 1 if standstill else 0
        if self.run >= max(1, cfg.confirm_scans):
            self.entries += 1
            self.holding = True
            self.held += 1
        return self.holding

    def status(self) -> dict[str, int]:
        return {"holding": int(self.holding), "entries": self.entries, "held": self.held, "released": self.released}
