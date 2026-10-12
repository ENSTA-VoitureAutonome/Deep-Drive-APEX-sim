#!/usr/bin/env python3
"""Move the car a few centimetres under its own code, and measure what happened.

This is the smallest possible motion test, and it is the one that answers the
question the static tests cannot: **does the learned odometry follow real
motion?** Standing still, it walks forward at about 0.30 m/min with nothing
moving; that is either a quirk of being stationary for minutes -- which is
outside anything in the training set -- or a bias that would also corrupt a
driving run. Twenty centimetres of commanded motion separates the two.

It also measures, for the first time on the real car:

* whether the affine ESC map actually starts the motor, and at what command;
* the speed the car delivers for a commanded speed, which is the gauge the
  whole envelope rests on (`speed_pct_per_mps`);
* the start threshold, forwards and in reverse.

How it is bounded, in the order the bounds bite:

1. **time**, which is the only bound that does not depend on anything we are
   testing: a leg is cut at `--max-time-s`, so even an ESC three times more
   responsive than believed cannot run away;
2. **distance**, from the odometry, which stops the leg early when it has gone
   far enough -- useful, but it is the thing under test, so it is never the
   only bound;
3. **the bridge's own watchdog** (0.35 s): if this script dies, is killed, or
   the link drops, the car stops by itself;
4. **Ctrl-C**, which publishes zeros before exiting.

    python3 nudge.py --distance-m 0.20 --speed-mps 0.20            # forward only
    python3 nudge.py --distance-m 0.20 --speed-mps 0.20 --back     # forward, pause, back
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# Hard ceilings. They are not configurable on purpose: this script exists to
# move the car a few centimetres, and a typo should not turn it into a lap.
MAX_DISTANCE_M = 0.30
MAX_SPEED_MPS = 0.30
MAX_TIME_S = 2.5


class Nudge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("apex_nudge")
        self.args = args
        self.cmd = self.create_publisher(Twist, args.cmd_topic, 20)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(String, "/apex/vehicle/drive_bridge_status", self._on_bridge, 10)
        self.xy: tuple[float, float] | None = None
        self.yaw: float = 0.0
        self.bridge: dict = {}
        self.applied_pct: list[float] = []

    def _on_odom(self, m: Odometry) -> None:
        self.xy = (m.pose.pose.position.x, m.pose.pose.position.y)
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _on_bridge(self, m: String) -> None:
        import json  # noqa: PLC0415

        try:
            self.bridge = json.loads(m.data)
        except ValueError:
            return
        pct = self.bridge.get("applied_speed_pct")
        if pct is not None:
            self.applied_pct.append(float(pct))

    # --------------------------------------------------------------- helpers
    def pump(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def publish(self, v: float, steer_deg: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        # The bridge inverts exactly this to recover the steering angle, so the
        # wheelbase cancels and what the servo is asked for is what is written
        # here. That identity is what makes the measured gain meaningful.
        msg.angular.z = float(v * math.tan(math.radians(steer_deg)) / 0.30)
        self.cmd.publish(msg)

    def stop(self, seconds: float = 0.6) -> None:  # noqa: D102
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.publish(0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def travelled(self, origin: tuple[float, float]) -> float:
        if self.xy is None:
            return 0.0
        return math.hypot(self.xy[0] - origin[0], self.xy[1] - origin[1])

    # ------------------------------------------------------------------- leg
    def leg(self, name: str, speed: float, distance: float, max_time: float, steer_deg: float = 0.0) -> dict:
        if self.xy is None:
            raise SystemExit("no odometry on " + self.args.odom_topic + ": refusing to move")
        origin, yaw0 = self.xy, self.yaw
        self.applied_pct.clear()
        t0 = time.monotonic()
        reason = "time"
        print(f"[nudge] {name}: commanding {speed:+.2f} m/s, stop at {distance:.2f} m or {max_time:.2f} s")
        while rclpy.ok():
            elapsed = time.monotonic() - t0
            if elapsed >= max_time:
                reason = "time limit"
                break
            if self.travelled(origin) >= distance:
                reason = "distance (odometry)"
                break
            self.publish(speed, steer_deg)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(0.02)
        elapsed = time.monotonic() - t0
        self.stop()
        self.pump(1.0)  # let the odometry catch up with the last motion
        moved = self.travelled(origin)
        pct = max(self.applied_pct, default=float("nan")) if speed > 0 else min(self.applied_pct, default=float("nan"))
        dyaw = math.atan2(math.sin(self.yaw - yaw0), math.cos(self.yaw - yaw0))
        out = {"leg": name, "commanded_mps": speed, "steer_deg": steer_deg, "elapsed_s": round(elapsed, 3),
               "odometry_m": round(moved, 4), "dyaw_deg": round(math.degrees(dyaw), 2),
               "esc_pct": round(pct, 1), "stopped_on": reason}
        print(f"[nudge] {name}: stopped on {reason} after {elapsed:.2f} s; "
              f"odometry says {moved:.3f} m ({moved / max(elapsed, 1e-6):.2f} m/s), "
              f"turned {math.degrees(dyaw):+.2f} deg, ESC reached {pct:.1f} %")
        if abs(steer_deg) > 1e-6 and moved > 0.01:
            # The curvature the car delivered, against the one it was asked for.
            # This is `gain_left` / `gain_right` measured directly, and it is
            # the largest remaining assumption in the car's configuration.
            kappa_cmd = math.tan(math.radians(steer_deg)) / 0.30
            kappa_got = dyaw / moved
            out["kappa_cmd"] = round(kappa_cmd, 4)
            out["kappa_measured"] = round(kappa_got, 4)
            out["steer_gain"] = round(kappa_got / kappa_cmd, 3)
            print(f"[nudge] {name}: curvature asked {kappa_cmd:+.3f} 1/m, delivered {kappa_got:+.3f} 1/m "
                  f"-> steering gain {kappa_got / kappa_cmd:.3f}")
        return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--distance-m", type=float, default=0.20)
    ap.add_argument("--speed-mps", type=float, default=0.20)
    ap.add_argument("--max-time-s", type=float, default=0.0, help="0: distance/speed x 1.5, capped")
    ap.add_argument("--back", action="store_true", help="also drive back after a pause")
    ap.add_argument("--steer-deg", type=float, default=0.0,
                    help="hold this steering angle; the heading change then measures the steering gain")
    ap.add_argument("--pause-s", type=float, default=2.0)
    ap.add_argument("--esc-uncertainty", type=float, default=3.0,
                    help="how much faster than commanded the ESC might be; sets the time limit")
    ap.add_argument("--cmd-topic", default="/apex/cmd_vel_track")
    ap.add_argument("--odom-topic", default="/apex/odometry/learned")
    args = ap.parse_args()

    distance = min(abs(args.distance_m), MAX_DISTANCE_M)
    speed = min(abs(args.speed_mps), MAX_SPEED_MPS)
    max_time = args.max_time_s if args.max_time_s > 0 else 1.5 * distance / max(speed, 1e-3)
    # The time limit is the only bound that holds if the ESC is nothing like
    # what we believe, so it is derived from the worst case rather than from
    # the nominal one: assume the car could deliver `esc_uncertainty` times the
    # speed commanded, and cut the leg before that could cross MAX_DISTANCE_M.
    worst_case = MAX_DISTANCE_M / max(speed * args.esc_uncertainty, 1e-3)
    max_time = min(max_time, MAX_TIME_S, worst_case)
    print(f"[nudge] bounds: <= {distance:.2f} m, <= {speed:.2f} m/s, <= {max_time:.2f} s per leg. "
          f"Even at {args.esc_uncertainty:.0f}x the commanded speed that leg is "
          f"{speed * args.esc_uncertainty * max_time:.2f} m "
          f"(hard ceilings {MAX_DISTANCE_M} m / {MAX_SPEED_MPS} m/s / {MAX_TIME_S} s)")

    rclpy.init()
    node = Nudge(args)
    print("[nudge] waiting for odometry...")
    node.pump(5.0)
    results = []
    try:
        # The odometry walks on its own while the car stands still (measured:
        # ~0.30 m/min). Over the second and a half a leg takes that is about a
        # centimetre -- a tenth of what we are trying to measure -- so measure
        # it here rather than let it masquerade as motion.
        base_origin = node.xy
        if base_origin is None:
            raise SystemExit("no odometry on " + args.odom_topic + ": refusing to move")
        node.stop(3.0)
        drift = node.travelled(base_origin)
        print(f"[nudge] baseline: the odometry moved {1e3 * drift:.0f} mm in 3.0 s with the car still "
              f"({60.0 * drift / 3.0:.2f} m/min)")
        results.append({"leg": "baseline", "odometry_m": round(drift, 4), "elapsed_s": 3.0})
        results.append(node.leg("forward", +speed, distance, max_time, args.steer_deg))
        if args.back:
            print(f"[nudge] pausing {args.pause_s:.1f} s")
            node.stop(args.pause_s)
            results.append(node.leg("back", -speed, distance, max_time, args.steer_deg))
    except KeyboardInterrupt:
        print("\n[nudge] interrupted")
    finally:
        node.stop(1.0)
        print("[nudge] stopped, commanding zero")
        for r in results:
            print("[nudge] result:", r)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
