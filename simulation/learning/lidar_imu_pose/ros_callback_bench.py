#!/usr/bin/env python3
"""ROS 2 message work of the learned odometry node, per call, without torch.

``pi4_budget.py`` needs the Python cost the node adds around the estimator
on the same GIL (``learned_odometry_node.py``):
  handle_ms        ``_handle`` per estimate: the TF, the Odometry, the
                   de-skewed LaserScan (360 ranges to a list) and the CSV row;
  imu_callback_us  ``_on_imu`` besides add_imu + predict (timed by
                   ``latency_profile.py profile``): the Imu message taken from
                   the executor, its arrays, and the predicted Odometry.
Publication is timed as the message serialisation (what rclpy does before
handing the bytes to DDS); the DDS write itself is not included.

Runs wherever rclpy is (the PC with ROS Jazzy sourced, the car's Docker
image); the result goes to ``config/pi4_budget/ros_<tag>.json``.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
from pathlib import Path
import platform
import time

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

HERE = Path(__file__).resolve().parent


def _stamp(t_ns: int) -> TimeMsg:
    return TimeMsg(sec=int(t_ns // 1_000_000_000), nanosec=int(t_ns % 1_000_000_000))


def _odom(t_ns: int, pose: np.ndarray, v: np.ndarray, cov: np.ndarray, sigma: np.ndarray | None) -> Odometry:
    """As ``LearnedOdometryNode._odom``."""
    o = Odometry()
    o.header.stamp = _stamp(t_ns)
    o.header.frame_id = "odom_learned"
    o.child_frame_id = "base_link"
    o.pose.pose.position.x, o.pose.pose.position.y = float(pose[0]), float(pose[1])
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(0.5 * pose[2]), math.cos(0.5 * pose[2])
    o.pose.covariance[0], o.pose.covariance[7], o.pose.covariance[35] = float(cov[0]), float(cov[1]), float(cov[2])
    o.twist.twist.linear.x, o.twist.twist.linear.y = float(v[0]), float(v[1])
    if sigma is not None:
        o.twist.covariance[0], o.twist.covariance[7], o.twist.covariance[35] = (float(x) for x in np.asarray(sigma) ** 2)
    return o


def handle_once(rng: np.random.Generator, writer: csv.writer) -> None:
    """One estimate through ``LearnedOdometryNode._handle``."""
    t = 1_234_567_890_123
    pose, delta, sigma = rng.random(3), rng.random(3) * 0.1, rng.random(3) * 0.01
    v, bias = rng.random(2), rng.random(3) * 0.01
    cov = np.zeros(3)
    c, s = math.cos(pose[2] - delta[2]), math.sin(pose[2] - delta[2])
    var = sigma.astype(float) ** 2
    cov += np.array([c * c * var[0] + s * s * var[1], s * s * var[0] + c * c * var[1], var[2]])
    tf = TransformStamped()
    tf.header.stamp = _stamp(t)
    tf.header.frame_id, tf.child_frame_id = "odom_learned", "base_link"
    tf.transform.translation.x, tf.transform.translation.y = float(pose[0]), float(pose[1])
    tf.transform.rotation.z, tf.transform.rotation.w = math.sin(0.5 * pose[2]), math.cos(0.5 * pose[2])
    serialize_message(TFMessage(transforms=[tf]))
    serialize_message(_odom(t, pose, v, cov, sigma))
    scan = LaserScan()
    scan.header.stamp = _stamp(t)
    scan.header.frame_id = "laser"
    scan.angle_min, scan.angle_increment, scan.angle_max = -math.pi, 2 * math.pi / 360, math.pi
    scan.time_increment, scan.scan_time, scan.range_min, scan.range_max = 0.0, 0.1, 0.15, 12.0
    scan.ranges = (rng.random(360).astype(np.float32) * 8).astype(float).tolist()
    serialize_message(scan)
    writer.writerow([t, *[f"{x:.6f}" for x in pose], *[f"{x:.6f}" for x in delta], *[f"{x:.6g}" for x in sigma], *[f"{x:.4f}" for x in v],
                     *[f"{x:.6f}" for x in bias], "0.10000", 10, "", "", "20.00", "150.00"])


def imu_callback_once(raw: bytes) -> None:
    """``_on_imu`` besides add_imu + predict: take the message, build the
    arrays, publish the predicted Odometry."""
    m = deserialize_message(raw, Imu)
    t = Time.from_msg(m.header.stamp).nanoseconds
    np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    serialize_message(_odom(t, np.array([1.0, 2.0, 0.3]), np.array([2.0, 0.1]), np.zeros(3), None))


def median_us(fn, n: int) -> float:
    for _ in range(50):
        fn()
    out = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        out.append(time.perf_counter() - t0)
    return float(np.median(out) * 1e6)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", default=platform.node() or "host")
    ap.add_argument("--n", type=int, default=3000)
    args = ap.parse_args()
    rng = np.random.default_rng(0)
    writer = csv.writer(io.StringIO())
    imu = Imu()
    imu.header.stamp = _stamp(123_456_789)
    imu.angular_velocity.z, imu.linear_acceleration.z = 0.3, 9.8
    raw = serialize_message(imu)
    out = {"kind": "ros", "host": platform.node(), "machine": platform.machine(), "date": time.strftime("%Y-%m-%d %H:%M:%S"),
           "handle_ms": median_us(lambda: handle_once(rng, writer), args.n) * 1e-3, "imu_callback_us": median_us(lambda: imu_callback_once(raw), args.n)}
    path = HERE / "config" / "pi4_budget" / f"ros_{args.tag}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(f"wrote {path}: _handle {out['handle_ms']:.3f} ms per estimate, IMU callback (ROS part) {out['imu_callback_us']:.1f} us")


if __name__ == "__main__":
    main()
