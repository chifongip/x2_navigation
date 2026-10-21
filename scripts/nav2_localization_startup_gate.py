#!/usr/bin/env python3
"""Request initial Nav2 activation once localization and robot TF are usable."""

import math
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class ReadinessHold:
    """Require continuously healthy observations and a fresh readiness heartbeat."""

    def __init__(self, hold_sec, heartbeat_timeout_sec):
        self.hold_sec = hold_sec
        self.heartbeat_timeout_sec = heartbeat_timeout_sec
        self.last_received = None
        self.ready = False
        self.since = None

    def receive(self, ready, now):
        if not ready or (
            self.last_received is not None
            and now - self.last_received > self.heartbeat_timeout_sec
        ):
            self.since = None
        self.ready = ready
        self.last_received = now

    def observe(self, tf_valid, now):
        if (
            not self.ready
            or self.last_received is None
            or now - self.last_received > self.heartbeat_timeout_sec
            or not tf_valid
        ):
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.hold_sec


class Nav2LocalizationStartupGate(Node):
    def __init__(self):
        super().__init__("nav2_localization_startup_gate")
        self.hold = ReadinessHold(
            self.positive_parameter("ready_hold_sec", 0.5),
            self.positive_parameter("readiness_timeout_sec", 0.25),
        )
        self.tf_max_age = self.positive_parameter("tf_max_age_sec", 1.0)
        self.future_tolerance = self.positive_parameter("future_tolerance_sec", 0.05)
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        topic = self.declare_parameter(
            "readiness_topic", "/odin1/localization_ready"
        ).value
        service = self.declare_parameter(
            "manager_service", "/lifecycle_manager_navigation/manage_nodes"
        ).value
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.subscription = self.create_subscription(Bool, topic, self.on_ready, 1)
        self.client = self.create_client(ManageLifecycleNodes, service)
        self.future = None
        self.finished = False
        self.exit_code = 0
        self.last_wait_log = None
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            "Waiting for Odin readiness and fresh map -> odom -> base_link TF"
        )

    def positive_parameter(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def on_ready(self, message):
        self.hold.receive(message.data, time.monotonic())

    def tf_is_fresh(self):
        try:
            # Check each required link as well as the latest common chain time.
            for target, source in (
                (self.map_frame, self.odom_frame),
                (self.odom_frame, self.base_frame),
                (self.map_frame, self.base_frame),
            ):
                transform = self.buffer.lookup_transform(target, source, Time())
                stamp = Time.from_msg(transform.header.stamp)
                age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
                if not -self.future_tolerance <= age <= self.tf_max_age:
                    return False
            return True
        except TransformException:
            return False

    def tick(self):
        if self.finished:
            return
        if self.future is not None:
            if self.future.done():
                try:
                    if not self.future.result().success:
                        raise RuntimeError("lifecycle manager returned success=false")
                    self.get_logger().info(
                        "Nav2 startup completed after Odin localization became ready"
                    )
                except Exception as error:
                    self.exit_code = 1
                    self.get_logger().error(
                        f"Nav2 startup failed: {error}. No automatic retry; inspect lifecycle states."
                    )
                self.finished = True
            return
        now = time.monotonic()
        qualified = self.hold.observe(self.tf_is_fresh(), now)
        if not qualified or not self.client.service_is_ready():
            if self.last_wait_log is None or now - self.last_wait_log >= 5.0:
                self.get_logger().info(
                    "Waiting for sustained Odin readiness, fresh robot TF, and lifecycle manager service"
                )
                self.last_wait_log = now
            return
        request = ManageLifecycleNodes.Request(
            command=ManageLifecycleNodes.Request.STARTUP
        )
        # Never resubmit: a request can partially transition nodes before failing.
        try:
            self.future = self.client.call_async(request)
            self.get_logger().info("Odin localization ready; requesting Nav2 startup")
        except Exception as error:
            self.exit_code = 1
            self.finished = True
            self.get_logger().error(
                f"Cannot request Nav2 startup: {error}. No automatic retry."
            )


def main():
    rclpy.init()
    node = Nav2LocalizationStartupGate()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.listener.unregister()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
