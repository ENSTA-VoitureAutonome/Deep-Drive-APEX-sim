#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from gz.transport13 import Node as GzNode
from gz.msgs10 import double_pb2


class RcCarControlBridge(Node):
    def __init__(self) -> None:
        super().__init__("rc_car_control_bridge")
        self.declare_parameter("rear_wheel_speed_topic", "/rear_wheel_speed")
        self.declare_parameter("steering_angle_topic", "/steering_angle")
        self.declare_parameter(
            "rear_left_cmd_topic",
            "/model/rc_car/joint/rear_left_wheel_joint/cmd_vel",
        )
        self.declare_parameter(
            "rear_right_cmd_topic",
            "/model/rc_car/joint/rear_right_wheel_joint/cmd_vel",
        )
        self.declare_parameter(
            "front_left_steer_topic",
            "/model/rc_car/joint/front_left_wheel_steer_joint/cmd_pos",
        )
        self.declare_parameter(
            "front_right_steer_topic",
            "/model/rc_car/joint/front_right_wheel_steer_joint/cmd_pos",
        )
        self.declare_parameter("wheel_base", 0.32)
        self.declare_parameter("track_width", 0.29)
        self.declare_parameter("steering_limit", 0.6)
        self.declare_parameter("steering_epsilon", 1e-4)
        self.declare_parameter("publish_rate", 30.0)

        speed_topic = (
            self.get_parameter("rear_wheel_speed_topic")
            .get_parameter_value()
            .string_value
        )
        steering_topic = (
            self.get_parameter("steering_angle_topic")
            .get_parameter_value()
            .string_value
        )
        rear_left_cmd_topic = (
            self.get_parameter("rear_left_cmd_topic")
            .get_parameter_value()
            .string_value
        )
        rear_right_cmd_topic = (
            self.get_parameter("rear_right_cmd_topic")
            .get_parameter_value()
            .string_value
        )
        front_left_steer_topic = (
            self.get_parameter("front_left_steer_topic")
            .get_parameter_value()
            .string_value
        )
        front_right_steer_topic = (
            self.get_parameter("front_right_steer_topic")
            .get_parameter_value()
            .string_value
        )
        self._wheel_base = (
            self.get_parameter("wheel_base").get_parameter_value().double_value
        )
        self._track_width = (
            self.get_parameter("track_width").get_parameter_value().double_value
        )
        self._steering_limit = (
            self.get_parameter("steering_limit").get_parameter_value().double_value
        )
        self._steering_epsilon = (
            self.get_parameter("steering_epsilon").get_parameter_value().double_value
        )
        self._publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        self._gz_node = GzNode()
        self._rear_left_pub = self._gz_node.advertise(
            rear_left_cmd_topic, double_pb2.Double
        )
        self._rear_right_pub = self._gz_node.advertise(
            rear_right_cmd_topic, double_pb2.Double
        )
        self._front_left_pub = self._gz_node.advertise(
            front_left_steer_topic, double_pb2.Double
        )
        self._front_right_pub = self._gz_node.advertise(
            front_right_steer_topic, double_pb2.Double
        )

        self._last_speed = None
        self._last_steering = None
        self.create_subscription(Float64, speed_topic, self._on_speed, 10)
        self.create_subscription(Float64, steering_topic, self._on_steering, 10)

        if self._publish_rate > 0.0:
            self._timer = self.create_timer(
                1.0 / self._publish_rate, self._republish
            )
        else:
            self._timer = None

    def _publish_speed(self, speed: float) -> None:
        msg = double_pb2.Double()
        msg.data = speed
        self._rear_left_pub.publish(msg)
        self._rear_right_pub.publish(msg)

    def _compute_steering_angles(self, steering_angle: float) -> tuple[float, float]:
        angle = max(-self._steering_limit, min(steering_angle, self._steering_limit))
        if abs(angle) < self._steering_epsilon:
            return 0.0, 0.0

        abs_angle = abs(angle)
        radius = self._wheel_base / math.tan(abs_angle)
        half_track = self._track_width / 2.0
        inner_radius = max(radius - half_track, self._steering_epsilon)
        outer_radius = radius + half_track
        inner = math.atan(self._wheel_base / inner_radius)
        outer = math.atan(self._wheel_base / outer_radius)

        if angle > 0.0:
            left, right = inner, outer
        else:
            left, right = outer, inner

        sign = 1.0 if angle >= 0.0 else -1.0
        return left * sign, right * sign

    def _publish_steering(self, steering_angle: float) -> None:
        left_angle, right_angle = self._compute_steering_angles(steering_angle)
        left_msg = double_pb2.Double()
        left_msg.data = left_angle
        right_msg = double_pb2.Double()
        right_msg.data = right_angle
        self._front_left_pub.publish(left_msg)
        self._front_right_pub.publish(right_msg)

    def _on_speed(self, msg: Float64) -> None:
        self._last_speed = msg.data
        self._publish_speed(msg.data)

    def _on_steering(self, msg: Float64) -> None:
        self._last_steering = msg.data
        self._publish_steering(msg.data)

    def _republish(self) -> None:
        if self._last_speed is not None:
            self._publish_speed(self._last_speed)
        if self._last_steering is not None:
            self._publish_steering(self._last_steering)


def main() -> None:
    rclpy.init()
    node = RcCarControlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
