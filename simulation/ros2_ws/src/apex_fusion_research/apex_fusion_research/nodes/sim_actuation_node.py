"""ESC + servo of the simulated car, identical to the pose-dataset actuators.

Input : ``geometry_msgs/Twist`` on ``/apex/cmd_vel_track`` (the interface of
        the real car: linear.x speed, angular.z yaw rate; the steering angle is
        atan(wheelbase * yaw_rate / speed), as cmd_vel_to_apex_actuation_node).
        That wheelbase is the one *this bridge* assumes, which on the real car
        is a configured constant nobody measured, so a run may draw it: the
        ratio between it and the driver's own value is one more unknown the
        controller has to identify, and it is indistinguishable from a servo
        gain, which is exactly why the identification lumps them together.
Output: Gazebo joint commands (rear wheel velocities, knuckle positions for
        the JointPositionController plugins), stepped on the simulation clock.
The dynamics are ``tools/pose_dataset/vehicle.Actuators`` with the limits of
``vehicle.yaml`` (the single source used to generate the training data):
first-order ESC lag with acceleration / braking limits, servo lag, rate limit
and asymmetric linkage, Ackermann split and electronic differential.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ._pose_tools import tools_path


class SimActuationNode(Node):
    def __init__(self) -> None:
        super().__init__("apex_sim_actuation")
        dp = self.declare_parameter
        dp("cmd_topic", "/apex/cmd_vel_track")
        dp("rate_hz", 500.0)
        dp("cmd_timeout_s", 0.5)
        dp("esc_tau_scale", 1.0)
        dp("servo_tau_scale", 1.0)
        dp("model_name", "rc_car")
        # The plant of this run, drawn by the launch; every default is the
        # identity, so an unset parameter is the car of vehicle.yaml.
        for name in ("wheelbase_m", "steer_gain", "esc_gain", "right_ratio"):
            dp(name, 0.0)
        for name in ("steer_trim_deg", "esc_sag_per_min", "stiction_mps", "reverse_top_speed_mps", "reverse_engage_s"):
            dp(name, 0.0)
        dp("decode_wheelbase_m", 0.0)
        # Below this speed the Twist carries no steering information at all
        # (angular.z = v * kappa goes to zero with v), so the servo holds its
        # angle, as a servo does. The driver guarantees it only manoeuvres
        # above it: its launch ramp leaves standstill at >= 0.1 m/s.
        dp("decode_v_min", 0.02)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        tools_path()
        from pose_dataset.vehicle import Actuators, load_vehicle_config  # noqa: PLC0415
        from gz.msgs10 import double_pb2  # noqa: PLC0415
        from gz.transport13 import Node as GzNode  # noqa: PLC0415

        self._double = double_pb2.Double
        perturb = {"esc_tau_scale": float(gp("esc_tau_scale")), "servo_tau_scale": float(gp("servo_tau_scale"))}
        for name in ("wheelbase_m", "steer_gain", "esc_gain", "right_ratio", "steer_trim_deg",
                     "esc_sag_per_min", "stiction_mps", "reverse_top_speed_mps", "reverse_engage_s"):
            value = float(gp(name))
            if value != 0.0:
                perturb[name] = value
        self.act = Actuators(load_vehicle_config(), perturb)
        self._decode_wheelbase = float(gp("decode_wheelbase_m")) or self.act.wheelbase
        self._decode_v_min = float(gp("decode_v_min"))
        self._frozen_steer = 0
        m = str(gp("model_name"))
        gz = GzNode()
        self._gz = gz
        self._pub = {
            "rl": gz.advertise(f"/model/{m}/joint/rear_left_wheel_joint/cmd_vel", self._double),
            "rr": gz.advertise(f"/model/{m}/joint/rear_right_wheel_joint/cmd_vel", self._double),
            "fl": gz.advertise(f"/model/{m}/joint/front_left_wheel_steer_joint/cmd_pos", self._double),
            "fr": gz.advertise(f"/model/{m}/joint/front_right_wheel_steer_joint/cmd_pos", self._double),
        }
        self._timeout_ns = int(float(gp("cmd_timeout_s")) * 1e9)
        self._last_cmd_ns: int | None = None
        self._last_step_ns: int | None = None
        self._steer_deg = 0.0
        self.create_subscription(Twist, str(gp("cmd_topic")), self._on_cmd, 20)
        self.create_timer(1.0 / float(gp("rate_hz")), self._step)

    def _on_cmd(self, msg: Twist) -> None:
        v = float(msg.linear.x)
        if abs(v) > self._decode_v_min:
            # The sign of the curvature survives a negative speed, so this is
            # also how the car steers while it reverses out of a dead end.
            self._steer_deg = math.degrees(math.atan(self._decode_wheelbase * float(msg.angular.z) / v))
        else:
            self._frozen_steer += 1
        self.act.set_command(v, self._steer_deg)
        self._last_cmd_ns = self.get_clock().now().nanoseconds

    def _send(self, key: str, value: float) -> None:
        msg = self._double()
        msg.data = float(value)
        self._pub[key].publish(msg)

    def _step(self) -> None:
        now = self.get_clock().now().nanoseconds
        if self._last_step_ns is None or now <= self._last_step_ns:
            self._last_step_ns = now
            return
        dt = (now - self._last_step_ns) * 1e-9
        self._last_step_ns = now
        if self._last_cmd_ns is not None and now - self._last_cmd_ns > self._timeout_ns:
            self.act.set_command(0.0, self._steer_deg)  # lost command stream: stop like a failsafe ESC
        # The actuator filters are first order: step them at 1 ms like the dataset.
        n = max(1, int(round(dt / 0.001)))
        for _ in range(n):
            self.act.step(dt / n)
        ol, orr = self.act.wheel_omegas()
        kl, kr = self.act.knuckle_targets()
        self._send("rl", ol)
        self._send("rr", orr)
        self._send("fl", kl)
        self._send("fr", kr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimActuationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
