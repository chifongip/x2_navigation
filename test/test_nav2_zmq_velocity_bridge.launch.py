import json
import time
import unittest

import launch
import launch_testing.actions
import rclpy
import zmq
from geometry_msgs.msg import Twist
from launch_ros.actions import Node


BRIDGE_BIND_ENDPOINT = "tcp://*:18558"
BRIDGE_CONNECT_ENDPOINT = "tcp://127.0.0.1:18558"


def generate_test_description():
    bridge = Node(
        package="x2_navigation",
        executable="nav2_zmq_velocity_bridge",
        parameters=[
            {
                "zmq_endpoint": BRIDGE_BIND_ENDPOINT,
                "publish_rate_hz": 20.0,
                "command_timeout_sec": 0.20,
            }
        ],
        output="screen",
    )
    return launch.LaunchDescription([bridge, launch_testing.actions.ReadyToTest()])


class TestNav2ZmqVelocityBridge(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_nav2_zmq_velocity_bridge")
        self.publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.setsockopt(zmq.LINGER, 0)
        self.subscriber.setsockopt(zmq.SUBSCRIBE, b"")
        self.subscriber.connect(BRIDGE_CONNECT_ENDPOINT)
        self.poller = zmq.Poller()
        self.poller.register(self.subscriber, zmq.POLLIN)

    def tearDown(self):
        self.subscriber.close()
        self.context.term()
        self.node.destroy_node()

    def receive_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            events = dict(self.poller.poll(timeout=100))
            if self.subscriber not in events:
                continue
            message = json.loads(self.subscriber.recv().decode("utf-8"))
            if predicate(message):
                return message
        return None

    def wait_for_bridge_subscription(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.publisher.get_subscription_count() > 0:
                return True
        return False

    def test_forwards_clamped_twist_then_zeros_stale_command(self):
        self.assertTrue(self.wait_for_bridge_subscription())
        # Let the ZeroMQ subscription handshake before relying on PUB delivery.
        time.sleep(0.25)
        command = Twist()
        command.linear.x = 2.0
        command.linear.y = 2.0
        command.angular.z = -2.0

        expected = {
            "linear": {"x": 1.0, "y": 1.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": -1.0},
        }
        deadline = time.monotonic() + 2.0
        received = None
        while time.monotonic() < deadline and received is None:
            self.publisher.publish(command)
            received = self.receive_until(lambda message: message == expected, timeout=0.25)
        self.assertEqual(received, expected)

        command.linear.x = -2.0
        command.linear.y = -2.0
        reverse_expected = {
            "linear": {"x": -0.5, "y": -1.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": -1.0},
        }
        received = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and received is None:
            self.publisher.publish(command)
            received = self.receive_until(
                lambda message: message == reverse_expected, timeout=0.25
            )
        self.assertEqual(received, reverse_expected)

        zero = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }
        self.assertEqual(self.receive_until(lambda message: message == zero, timeout=1.0), zero)
