#!/usr/bin/env python3
"""Publish a recorded run's sensors on the car's real topics, in wall-clock time.

This is how the whole race stack is exercised on the Pi with the car switched
off (ladder step D2). The estimator, slam_toolbox, the driver and the adaptive
layer run exactly as they would on the floor -- same processes, same topics,
same QoS, same CPU contention -- but the LiDAR and IMU nodes are replaced by
this one, so nothing moves and no actuator is powered.

It is what makes the static test reach ``planning`` and ``race``: a stationary
car never leaves ``wait``, because the lap counter needs a closed lap.

The two streams keep their recorded relative timing, and each message is
stamped from the *recording*, not from the moment it is published: the real
drivers stamp at the sampling instant, and the estimator's de-skewing, its
health monitor and the driver's freshness gates all read those stamps. Stamping
at publication would hand them the replay loop's scheduling jitter instead of
the sensors' cadence (measured: a 10 Hz recording reported as 11.1 Hz).

    python3 /work/ros2_ws/scripts/replay_sensors.py /work/data/replay/final_s13_sensors.npz
    python3 /work/ros2_ws/scripts/replay_sensors.py FIXTURE --loop --rate-scale 1.0

Note the deliberate omissions: no ``/clock`` (the stack runs on wall time), no
truth, no map. The fixture carries two sensor streams and nothing else.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan

# The QoS of the real drivers, so the subscribers behave identically.
IMU_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
SCAN_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
RANGE_MIN_M = 0.15
RANGE_MAX_M = 12.0


class SensorReplay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("apex_sensor_replay")
        data = np.load(args.fixture, allow_pickle=False)
        self.imu_t = np.asarray(data["imu_t"], dtype=np.float64)
        self.imu = np.asarray(data["imu"], dtype=np.float64)
        self.scan_t = np.asarray(data["scan_t"], dtype=np.float64)
        self.scan_time = np.asarray(data["scan_time"], dtype=np.float64)
        self.ranges = np.asarray(data["ranges"], dtype=np.float32)
        meta = json.loads(str(data["meta"])) if "meta" in data else {}
        self.args = args
        self.beams = int(self.ranges.shape[1])
        self.imu_pub = self.create_publisher(Imu, args.imu_topic, IMU_QOS)
        self.scan_pub = self.create_publisher(LaserScan, args.scan_topic, SCAN_QOS)
        self.i_imu = self.i_scan = 0
        self.lap = 0
        self.t0 = time.monotonic()
        # The recording's clock, anchored to now: stamps come from here.
        self.stamp0_ns = self.get_clock().now().nanoseconds
        self.dropped = 0
        # How far behind its own schedule the replay runs. Everything
        # downstream reads the recorded stamps, so a replay that falls behind
        # hands the stack messages that are already old and the freshness gates
        # fire -- which is a property of this harness, not of the car.
        self.lag_ms_max = 0.0
        self.lag_ms_sum = 0.0
        self.lag_n = 0
        self.rng = np.random.default_rng(int(args.seed))
        self.create_timer(0.001, self._tick)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"replaying {Path(args.fixture).name}: {len(self.imu_t)} IMU samples, {len(self.scan_t)} scans, "
            f"{meta.get('duration_s', '?')} s at {args.rate_scale:.2f}x onto {args.imu_topic} and {args.scan_topic}"
            + (" (looping)" if args.loop else ""))

    def _stamp(self, t_rec: float):
        """ROS stamp of a recorded sample, on the recording's own cadence."""
        return Time(nanoseconds=self.stamp0_ns + int(round(t_rec * 1e9))).to_msg()

    def _report(self) -> None:
        if self.lag_n:
            mean = self.lag_ms_sum / self.lag_n
            level = self.get_logger().warning if self.lag_ms_max > 50.0 else self.get_logger().info
            level(f"replay lag: mean {mean:.1f} ms, worst {self.lag_ms_max:.0f} ms "
                  f"({self.i_scan}/{len(self.scan_t)} scans, {self.i_imu}/{len(self.imu_t)} IMU)")

    def _tick(self) -> None:
        elapsed = (time.monotonic() - self.t0) * self.args.rate_scale
        if self.i_scan < len(self.scan_t):
            lag = 1e3 * (elapsed - float(self.scan_t[self.i_scan]))
            if lag > 0.0:
                self.lag_ms_max = max(self.lag_ms_max, lag)
                self.lag_ms_sum += lag
                self.lag_n += 1
        while self.i_imu < len(self.imu_t) and self.imu_t[self.i_imu] <= elapsed:
            self._publish_imu(self.i_imu)
            self.i_imu += 1
        while self.i_scan < len(self.scan_t) and self.scan_t[self.i_scan] <= elapsed:
            if self.args.drop_prob <= 0.0 or self.rng.random() >= self.args.drop_prob:
                self._publish_scan(self.i_scan)
            else:
                self.dropped += 1
            self.i_scan += 1
        if self.i_imu >= len(self.imu_t) and self.i_scan >= len(self.scan_t):
            self.lap += 1
            if not self.args.loop:
                self.get_logger().info(f"replay finished ({self.dropped} revolutions dropped on purpose)")
                raise SystemExit(0)
            self.i_imu = self.i_scan = 0
            self.t0 = time.monotonic()
            self.stamp0_ns = self.get_clock().now().nanoseconds

    def _publish_imu(self, i: int) -> None:
        msg = Imu()
        msg.header.stamp = self._stamp(float(self.imu_t[i]))
        msg.header.frame_id = self.args.imu_frame
        a = self.imu[i]
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a[0], a[1], a[2]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = a[3], a[4], a[5]
        msg.orientation_covariance[0] = -1.0  # no orientation, as the real driver reports
        self.imu_pub.publish(msg)

    def _publish_scan(self, i: int) -> None:
        msg = LaserScan()
        msg.header.stamp = self._stamp(float(self.scan_t[i]))
        msg.header.frame_id = self.args.laser_frame
        msg.angle_min = -math.pi
        msg.angle_increment = 2.0 * math.pi / self.beams
        msg.angle_max = msg.angle_min + (self.beams - 1) * msg.angle_increment
        msg.range_min = RANGE_MIN_M
        msg.range_max = RANGE_MAX_M
        msg.scan_time = float(self.scan_time[i])
        msg.time_increment = msg.scan_time / self.beams
        msg.ranges = [float(r) for r in self.ranges[i]]
        self.scan_pub.publish(msg)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fixture", type=Path)
    ap.add_argument("--imu-topic", default="/apex/imu/data_raw")
    ap.add_argument("--scan-topic", default="/lidar/scan_raw")
    ap.add_argument("--imu-frame", default="imu_link")
    ap.add_argument("--laser-frame", default="laser")
    ap.add_argument("--rate-scale", type=float, default=1.0, help=">1 replays faster than real time")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--drop-prob", type=float, default=0.0, help="drop this fraction of revolutions (robustness)")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    rclpy.init()
    node = SensorReplay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
