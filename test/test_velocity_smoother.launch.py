import struct
import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist
from launch_ros.actions import Node
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from sensor_msgs.msg import PointCloud2, PointField


NAV_TOPIC = "/test/smoothing/navigation"
SMOOTH_TOPIC = "/test/smoothing/smoothed"
RAW_TOPIC = "/test/smoothing/arbitrated"
OUTPUT_TOPIC = "/test/smoothing/output"
CLOUD_TOPIC = "/test/smoothing/cloud"


def generate_test_description():
    params = str(Path(__file__).parents[1] / "config" / "nav2_params.yaml")
    return launch.LaunchDescription([
        Node(
            package="nav2_velocity_smoother", executable="velocity_smoother",
            name="velocity_smoother", parameters=[params],
            remappings=[("cmd_vel", NAV_TOPIC), ("cmd_vel_smoothed", SMOOTH_TOPIC)],
        ),
        Node(
            package="x2_navigation", executable="fine_align_server",
            parameters=[params, {
                "nav_cmd_topic": SMOOTH_TOPIC, "nav_raw_cmd_topic": NAV_TOPIC,
                "raw_cmd_topic": RAW_TOPIC,
            }],
        ),
        Node(
            package="nav2_collision_monitor", executable="collision_monitor",
            name="collision_monitor", parameters=[params, {
                "cmd_vel_in_topic": RAW_TOPIC, "cmd_vel_out_topic": OUTPUT_TOPIC,
                "base_shift_correction": False, "chest_cloud.topic": CLOUD_TOPIC,
            }],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="test_smoothing_lifecycle_manager", parameters=[{
                "autostart": True,
                "node_names": ["velocity_smoother", "collision_monitor"],
                # This fixture deliberately deactivates just the smoother to
                # exercise the arbiter independently of lifecycle recovery.
                "bond_timeout": 0.0,
            }],
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestVelocitySmoothing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_velocity_smoothing")
        self.nav_pub = self.node.create_publisher(Twist, NAV_TOPIC, 10)
        self.smooth_pub = self.node.create_publisher(Twist, SMOOTH_TOPIC, 10)
        self.cloud_pub = self.node.create_publisher(PointCloud2, CLOUD_TOPIC, 10)
        self.smoothed = []
        self.arbitrated = []
        self.output = []
        self.subscriptions = [
            self.node.create_subscription(Twist, SMOOTH_TOPIC, self.smoothed.append, 10),
            self.node.create_subscription(Twist, RAW_TOPIC, self.arbitrated.append, 10),
            self.node.create_subscription(Twist, OUTPUT_TOPIC, self.output.append, 10),
        ]
        for name in ("velocity_smoother", "collision_monitor"):
            client = self.node.create_client(GetState, f"/{name}/get_state")
            deadline = time.monotonic() + 8.0
            active = False
            while time.monotonic() < deadline:
                if not client.wait_for_service(timeout_sec=0.1):
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.1)
                if future.done() and future.result().current_state.label == "active":
                    active = True
                    break
            self.node.destroy_client(client)
            self.assertTrue(active, name)
        # Reset the actual smoother between scenarios without resetting parameters.
        self.drive(2.2, Twist())
        self.smoothed.clear()
        self.arbitrated.clear()
        self.output.clear()

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def command(x=0.0, y=0.0, yaw=0.0):
        command = Twist()
        command.linear.x, command.linear.y = x, y
        command.angular.z = yaw
        return command

    def drive(self, duration, command=None, obstacle=False, injected_smooth=None):
        cloud = PointCloud2()
        cloud.header.frame_id = "base_link"
        cloud.height = 1
        cloud.width = 5
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = 60
        cloud.is_dense = True
        cloud.data = struct.pack("<fff", 0.0 if obstacle else 2.0, 0.0, 0.0) * 5
        deadline = time.monotonic() + duration
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                if command is not None:
                    self.nav_pub.publish(command)
                if injected_smooth is not None:
                    self.smooth_pub.publish(injected_smooth)
                cloud.header.stamp = self.node.get_clock().now().to_msg()
                self.cloud_pub.publish(cloud)
                next_publish = now + 0.04
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def test_acceleration_limits_targets_reversal_and_bounds(self):
        self.drive(0.5, self.command(yaw=0.1))
        for yaw in (0.7, 0.1, 0.7, 0.1):
            self.drive(0.15, self.command(yaw=yaw))
        self.drive(2.5, self.command(yaw=0.7))
        self.assertAlmostEqual(self.smoothed[-1].angular.z, 0.7, places=6)
        self.drive(4.0, self.command(yaw=-0.7))
        self.assertAlmostEqual(self.smoothed[-1].angular.z, -0.7, places=6)
        self.assertGreater(len(self.smoothed), 100)
        for previous, current in zip(self.smoothed, self.smoothed[1:]):
            delta = abs(current.angular.z - previous.angular.z)
            accelerating = (
                abs(current.angular.z) >= abs(previous.angular.z)
                and current.angular.z * previous.angular.z >= 0.0
            )
            self.assertLessEqual(delta, (0.015 if accelerating else 0.025) + 1e-6)
        self.drive(5.0, self.command(x=2.0, y=1.0, yaw=2.0))
        self.assertAlmostEqual(self.smoothed[-1].linear.x, 0.5, places=6)
        self.assertEqual(self.smoothed[-1].linear.y, 0.0)
        self.assertAlmostEqual(self.smoothed[-1].angular.z, 1.0, places=6)
        self.drive(5.6, self.command(x=-2.0, yaw=-2.0))
        self.assertAlmostEqual(self.smoothed[-1].linear.x, -0.2, places=6)
        self.assertAlmostEqual(self.smoothed[-1].angular.z, -1.0, places=6)

    def test_raw_loss_and_explicit_zero_override_smoother(self):
        self.drive(1.0, self.command(yaw=0.7))
        self.assertGreater(self.arbitrated[-1].angular.z, 0.2)
        self.drive(0.32)
        self.assertGreater(self.smoothed[-1].angular.z, 0.0)
        self.assertEqual(self.arbitrated[-1], Twist())
        self.drive(0.2, injected_smooth=self.command(yaw=0.7))
        self.assertEqual(self.arbitrated[-1], Twist())
        self.drive(0.3, self.command(yaw=0.7))
        self.assertGreater(self.arbitrated[-1].angular.z, 0.0)
        self.drive(0.12, Twist())
        self.assertEqual(self.arbitrated[-1], Twist())
        self.assertGreater(self.smoothed[-1].angular.z, 0.0)
        self.drive(0.12, self.command(yaw=float("nan")))
        self.assertEqual(self.arbitrated[-1], Twist())

    def test_collision_stop_bypasses_smoothing(self):
        self.drive(1.0, self.command(yaw=0.7))
        self.assertTrue(self.output)
        self.assertGreater(self.output[-1].angular.z, 0.2)
        self.drive(0.12, self.command(yaw=0.7), obstacle=True)
        self.assertGreater(self.arbitrated[-1].angular.z, 0.2)
        self.assertEqual(self.output[-1], Twist())

    def test_missing_smoother_output_stops_fresh_navigation(self):
        self.drive(1.0, self.command(yaw=0.7))
        client = self.node.create_client(ChangeState, "/velocity_smoother/change_state")
        self.assertTrue(client.wait_for_service(timeout_sec=2.0))

        def transition(identifier):
            request = ChangeState.Request()
            request.transition.id = identifier
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            self.assertTrue(future.done())
            self.assertTrue(future.result().success)

        transition(Transition.TRANSITION_DEACTIVATE)
        try:
            self.drive(0.32, self.command(yaw=0.7))
            self.assertEqual(self.arbitrated[-1], Twist())
        finally:
            transition(Transition.TRANSITION_ACTIVATE)
            self.node.destroy_client(client)
