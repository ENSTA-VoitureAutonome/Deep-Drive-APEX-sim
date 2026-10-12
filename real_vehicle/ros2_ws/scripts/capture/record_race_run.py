#!/usr/bin/env python3
"""Record a race run: the sensors, the commands and what the stack was thinking.

Extends ``record_real2sim_sensors.py`` -- the sensor CSVs are written by the
same code, in the same simulation formats, so every offline tool
(``sensor_timing_audit.py``, ``replay_slam.py``, ``export_replay_fixture.py``)
reads a race run exactly as it reads a recording or a simulated run. On top of
the two sensor streams it keeps the three things an autonomous run has to be
judged on afterwards:

* ``cmd_vel.csv`` -- what the driver asked for, at its own rate, so the gaps in
  the command stream can be measured offline and not only live;
* ``driver_status.jsonl`` / ``bridge_status.jsonl`` -- the phase, the identified
  plant, the counters and what the bridge actually applied;
* ``run_config.json`` -- the checkpoint, the sensor profile and the envelope.

``run_config.json`` carries exactly the keys ``replay_slam.prepare_run`` reads
and no others: in particular there is no truth, no plan and no plant, because a
real run has none and a tool that expected them would be reading a simulation.

    python3 record_race_run.py --out /work/ros2_ws/apex_race/push_01 --duration 180
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from record_real2sim_sensors import Real2SimSensorRecorder  # noqa: E402


class RaceRunRecorder(Real2SimSensorRecorder):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(args.out, args.imu_topic, args.scan_topic, args.duration)
        self._args = args
        self._t_start = time.monotonic()
        self._cmd_file = open(os.path.join(args.out, "cmd_vel.csv"), "w", newline="", encoding="utf-8")
        self._cmd = csv.writer(self._cmd_file)
        self._cmd.writerow(["t_s", "linear_x_mps", "angular_z_rps"])
        self._status_files = {}
        for name, topic in (("driver", args.driver_status_topic), ("bridge", args.bridge_status_topic)):
            handle = open(os.path.join(args.out, f"{name}_status.jsonl"), "w", encoding="utf-8")
            self._status_files[name] = handle
            self.create_subscription(String, topic, self._on_status(name), 10)
        self.create_subscription(Twist, args.cmd_topic, self._on_cmd, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self._counts.update(cmd=0, driver_status=0, bridge_status=0)

    def _on_cmd(self, m: Twist) -> None:
        self._cmd.writerow([f"{time.monotonic() - self._t_start:.4f}", f"{m.linear.x:.4f}", f"{m.angular.z:.5f}"])
        self._counts["cmd"] += 1

    def _on_status(self, name: str):
        def cb(msg: String) -> None:
            self._status_files[name].write(
                json.dumps({"t_s": round(time.monotonic() - self._t_start, 3), "payload": msg.data}) + "\n")
            self._counts[f"{name}_status"] += 1
        return cb

    def close(self) -> None:
        self._cmd_file.close()
        for handle in self._status_files.values():
            handle.close()
        # Only what the offline tools need to replay this recording. Nothing
        # about the track, and nothing the car could not have known.
        (Path(self._args.out) / "run_config.json").write_text(json.dumps({
            "sensor_profile": self._args.sensor_profile,
            "checkpoint": self._args.checkpoint,
            "car": self._args.car,
            "cmd_topic": self._args.cmd_topic,
            "note": self._args.note,
        }, indent=1))
        super().close()
        self.get_logger().info(f"Recorded {self._counts['cmd']} commands, "
                               f"{self._counts['driver_status']} driver and {self._counts['bridge_status']} bridge statuses")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--imu-topic", default="/apex/imu/data_raw")
    ap.add_argument("--scan-topic", default="/lidar/scan_raw")
    ap.add_argument("--cmd-topic", default="/apex/cmd_vel_track")
    ap.add_argument("--driver-status-topic", default="/apex/race_driver/status")
    ap.add_argument("--bridge-status-topic", default="/apex/vehicle/drive_bridge_status")
    ap.add_argument("--sensor-profile", default="APEX_real_wt61")
    ap.add_argument("--checkpoint", default="real2sim_v5_10hz/hybrid_submap_v4/best_model.pt")
    ap.add_argument("--car", default="APEX")
    ap.add_argument("--note", default="")
    args = ap.parse_args()

    rclpy.init()
    node = RaceRunRecorder(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
