#!/usr/bin/env python3
import math
import os
import sys
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


def _add_algorithms_to_path() -> None:
    # Add multiple candidate roots so "algorithm.*" imports resolve even from install/.
    here = os.path.abspath(os.path.dirname(__file__))
    candidates = []

    # Workspace-local paths (when running from source)
    candidates.append(os.path.join(here, "algorithms"))
    candidates.append(os.path.join(here, "..", "algorithms"))
    candidates.append(os.path.join(here, ".."))

    # repo-relative guesses (walk up a few levels)
    current = here
    for _ in range(6):
        parent = os.path.dirname(current)
        candidates.append(os.path.join(parent, "src", "rc_sim_description", "scripts"))
        candidates.append(os.path.join(parent, "rc_sim_description", "scripts"))
        current = parent

    for path in [os.path.abspath(p) for p in candidates]:
        if os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)


_add_algorithms_to_path()

from voiture_algorithm import VoitureAlgorithmCore


class VoitureControlNode(Node):
    def __init__(self) -> None:
        super().__init__("voiture_control_node")
        self.declare_parameter("lidar_topic", "/lidar_processed")
        self.declare_parameter("measured_wheelspeed_topic", "/measured_wheelspeed")
        self.declare_parameter("rear_wheel_speed_topic", "/rear_wheel_speed")
        self.declare_parameter("steering_angle_topic", "/steering_angle")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("lidar_type", "auto")

        self._lidar_topic = self.get_parameter("lidar_topic").value
        self._measured_topic = self.get_parameter("measured_wheelspeed_topic").value
        self._rear_speed_topic = self.get_parameter("rear_wheel_speed_topic").value
        self._steering_topic = self.get_parameter("steering_angle_topic").value
        self._control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._lidar_type = str(self.get_parameter("lidar_type").value).lower()

        self._core = VoitureAlgorithmCore(self.get_logger())
        self._latest_lidar: Optional[np.ndarray] = None
        self._latest_wheelspeed: Optional[float] = None

        self._rear_speed_pub = self.create_publisher(Float64, self._rear_speed_topic, 10)
        self._steering_pub = self.create_publisher(Float64, self._steering_topic, 10)

        self._lidar_sub = self._create_lidar_subscription()
        self._measured_sub = self.create_subscription(
            Float64, self._measured_topic, self._on_measured_speed, 10
        )

        period = 1.0 / self._control_rate_hz if self._control_rate_hz > 0 else 0.033
        self._timer = self.create_timer(period, self._on_timer)

    def _create_lidar_subscription(self):
        lidar_type = self._resolve_lidar_type()
        if lidar_type == "float64multiarray":
            return self.create_subscription(
                Float64MultiArray, self._lidar_topic, self._on_lidar_array, 10
            )
        return self.create_subscription(
            LaserScan, self._lidar_topic, self._on_lidar_scan, 10
        )

    def _resolve_lidar_type(self) -> str:
        if self._lidar_type in ("laserscan", "float64multiarray"):
            return self._lidar_type
        try:
            for name, types in self.get_topic_names_and_types():
                if name == self._lidar_topic:
                    if "sensor_msgs/msg/LaserScan" in types:
                        return "laserscan"
                    if "std_msgs/msg/Float64MultiArray" in types:
                        return "float64multiarray"
        except Exception:
            pass
        return "laserscan"

    def _on_lidar_scan(self, msg: LaserScan) -> None:
        self._latest_lidar = self._laserscan_to_array(msg)

    def _on_lidar_array(self, msg: Float64MultiArray) -> None:
        data = np.asarray(msg.data, dtype=float).reshape(-1)
        if data.shape[0] != 360:
            self.get_logger().warning(
                f"Float64MultiArray length {data.shape[0]} != 360; ignoring."
            )
            return
        self._latest_lidar = self._sanitize_ranges(data)

    def _on_measured_speed(self, msg: Float64) -> None:
        self._latest_wheelspeed = float(msg.data)

    def _on_timer(self) -> None:
        if self._latest_lidar is None:
            return
        steer_cmd, speed_cmd = self._core.compute(
            self._latest_lidar, self._latest_wheelspeed
        )
        steer_msg = Float64()
        steer_msg.data = float(steer_cmd)
        speed_msg = Float64()
        speed_msg.data = float(speed_cmd)
        # Published steering uses the same unit as compute_steer_from_lidar (degrees).
        self._steering_pub.publish(steer_msg)
        self._rear_speed_pub.publish(speed_msg)

    @staticmethod
    def _sanitize_ranges(values: np.ndarray) -> np.ndarray:
        cleaned = values.copy()
        invalid_mask = ~np.isfinite(cleaned) | (cleaned <= 0.0)
        cleaned[invalid_mask] = 0.0
        return cleaned

    def _laserscan_to_array(self, msg: LaserScan) -> np.ndarray:
        data = np.zeros(360, dtype=float)
        angle = msg.angle_min
        for r in msg.ranges:
            ang_deg = math.degrees(angle)
            idx = int(round(ang_deg)) % 360
            if not math.isfinite(r) or r <= 0.0:
                value = 0.0
            else:
                value = float(r)
            data[idx] = value
            angle += msg.angle_increment
        return data


def main() -> None:
    rclpy.init()
    node = VoitureControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
