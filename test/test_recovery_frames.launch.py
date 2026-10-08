"""Exercise Humble recovery collision checks with a large map/odom offset."""

from pathlib import Path
import time
import unittest

import launch
import launch_testing.actions
from launch_ros.actions import Node
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point32, PolygonStamped, Twist
from nav2_msgs.action import BackUp, Spin
from nav2_msgs.msg import Costmap


def generate_test_description():
    params = str(Path(__file__).parents[1] / "config" / "nav2_params.yaml")
    return launch.LaunchDescription([
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["-38", "2", "0", "0", "0", "0", "1", "map", "odom"],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "base_link"],
        ),
        Node(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", parameters=[params],
            remappings=[("cmd_vel", "/test/recovery/cmd_vel")],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="test_recovery_lifecycle_manager", parameters=[{
                "autostart": True, "node_names": ["behavior_server"],
            }],
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestRecoveryFrames(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_recovery_frames")
        self.commands = []
        self.node.create_subscription(
            Twist, "/test/recovery/cmd_vel", self.commands.append, 10,
        )
        self.costmap_pub = self.node.create_publisher(
            Costmap, "/local_costmap/costmap_raw",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.footprint_pub = self.node.create_publisher(
            PolygonStamped, "/local_costmap/published_footprint", 10,
        )
        self.timer = self.node.create_timer(0.05, self.publish_collision_inputs)

    def tearDown(self):
        self.node.destroy_node()

    def publish_collision_inputs(self):
        grid = Costmap()
        grid.header.frame_id = "odom"
        grid.header.stamp = self.node.get_clock().now().to_msg()
        grid.metadata.resolution = 0.05
        grid.metadata.size_x = grid.metadata.size_y = 120
        grid.metadata.origin.position.x = grid.metadata.origin.position.y = -3.0
        grid.metadata.origin.orientation.w = 1.0
        grid.data = [0] * (120 * 120)
        self.costmap_pub.publish(grid)
        footprint = PolygonStamped()
        footprint.header = grid.header
        footprint.polygon.points = [
            Point32(x=x, y=y) for x, y in
            [(0.15, 0.30), (0.15, -0.30), (-0.15, -0.30), (-0.15, 0.30)]
        ]
        self.footprint_pub.publish(footprint)

    def wait_until(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail("Timed out waiting for recovery action or velocity command")

    def test_backup_and_spin_check_odom_costmap_with_map_offset(self):
        # Let subscriptions and TF become ready before starting either action.
        self.wait_until(lambda: self.costmap_pub.get_subscription_count() > 0)
        for action_type, name in [(BackUp, "backup"), (Spin, "spin")]:
            with self.subTest(behavior=name):
                client = ActionClient(self.node, action_type, name)
                try:
                    self.wait_until(client.server_is_ready)
                    goal = action_type.Goal()
                    goal.time_allowance.sec = 10
                    if name == "backup":
                        goal.target.x = -0.1
                        goal.speed = 0.05
                    else:
                        goal.target_yaw = 0.5
                    self.commands.clear()
                    sent = client.send_goal_async(goal)
                    self.wait_until(sent.done)
                    handle = sent.result()
                    self.assertTrue(handle.accepted)
                    result = handle.get_result_async()
                    self.wait_until(lambda: result.done() or any(
                        abs(c.linear.x) > 0 or abs(c.angular.z) > 0
                        for c in self.commands
                    ))
                    self.assertFalse(
                        result.done(), "Recovery aborted before issuing motion in a free grid",
                    )
                    # Static fake TF cannot complete motion; stop after the
                    # first successful collision check and velocity command.
                    canceled = handle.cancel_goal_async()
                    self.wait_until(canceled.done)
                    self.assertTrue(canceled.result().goals_canceling)
                    self.wait_until(result.done)
                finally:
                    client.destroy()
