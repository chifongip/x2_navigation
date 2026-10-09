"""Failed ZMQ submissions must never produce fresh final-command telemetry."""
import time
import unittest

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from launch_ros.actions import Node


def generate_test_description():
    bridges = {}
    for index, failure in enumerate(("EAGAIN", "EIO")):
        bridges[failure] = Node(
            package="x2_navigation", executable="nav2_zmq_velocity_bridge",
            name="failed_bridge_" + failure.lower(),
            parameters=[{
                "command_topic": "/test/failed/" + failure + "/command",
                "final_command_topic": "/test/failed/" + failure + "/final",
                "zmq_endpoint": "tcp://*:" + str(18559 + index),
            }],
            additional_env={
                "LD_PRELOAD": LaunchConfiguration("send_failure_library"),
                "X2_TEST_ZMQ_SEND_FAILURE": failure,
            },
            output="screen",
        )
    return launch.LaunchDescription([
        DeclareLaunchArgument("send_failure_library"),
        *bridges.values(), launch_testing.actions.ReadyToTest(),
    ]), {"bridges": bridges}


class TestSendFailure(unittest.TestCase):
    def test_failures_do_not_publish_telemetry(self, proc_output, bridges):
        rclpy.init()
        node = rclpy.create_node("test_send_failure")
        received = []
        publishers, subscriptions = [], []
        try:
            for failure in bridges:
                publishers.append(node.create_publisher(
                    Twist, "/test/failed/" + failure + "/command", 10))
                subscriptions.append(node.create_subscription(
                    TwistStamped, "/test/failed/" + failure + "/final", received.append, 10))
            deadline = time.monotonic() + 5.0
            while not all(p.get_subscription_count() for p in publishers):
                self.assertLess(time.monotonic(), deadline)
                rclpy.spin_once(node, timeout_sec=0.01)
            for process in bridges.values():
                proc_output.assertWaitFor("ZMQ velocity send failed", process=process, timeout=5)
            command = Twist()
            command.linear.x = 0.2
            deadline = time.monotonic() + 0.7
            while time.monotonic() < deadline:
                for publisher in publishers:
                    publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertFalse(received)
        finally:
            node.destroy_node()
            rclpy.shutdown()
