import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from tf2_ros import StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage


def generate_test_description():
    package_path = Path(get_package_share_directory("x2_navigation"))
    adapter = Node(
        package="x2_navigation",
        executable="fast_lio_odom_adapter",
        parameters=[
            package_path / "config" / "nav2_params.yaml",
            {"tf_timeout_sec": 0.1},
        ],
        output="screen",
    )
    missing_transform_adapter = Node(
        package="x2_navigation",
        executable="fast_lio_odom_adapter",
        name="missing_transform_adapter",
        parameters=[
            package_path / "config" / "nav2_params.yaml",
            {
                "input_topic": "/Odometry_loc_missing_transform",
                "output_topic": "/odom_missing_transform",
                "tracking_frame": "missing_tracking_frame",
                "tf_timeout_sec": 0.05,
            },
        ],
        output="screen",
    )
    return launch.LaunchDescription(
        [adapter, missing_transform_adapter, launch_testing.actions.ReadyToTest()]
    )


class TestFastLioOdomAdapter(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_fast_lio_odom_adapter")
        self.publisher = self.node.create_publisher(Odometry, "/Odometry_loc", 10)
        self.received = []
        self.subscription = self.node.create_subscription(
            Odometry, "/odom", self.received.append, QoSProfile(depth=10)
        )
        self.missing_transform_received = []
        self.missing_transform_publisher = self.node.create_publisher(
            Odometry, "/Odometry_loc_missing_transform", 10
        )
        self.missing_transform_subscription = self.node.create_subscription(
            Odometry,
            "/odom_missing_transform",
            self.missing_transform_received.append,
            QoSProfile(depth=10),
        )
        self.transforms = []
        self.tf_subscription = self.node.create_subscription(
            TFMessage, "/tf", self.transforms.append, QoSProfile(depth=10)
        )
        self.static_broadcaster = StaticTransformBroadcaster(self.node)
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "lidar_imu_chest_front"
        transform.transform.translation.x = 1.0
        transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(transform)

    def tearDown(self):
        self.node.destroy_node()

    def test_transforms_fast_lio_odometry(self):
        message = Odometry()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "lidar_imu_chest_front"
        message.pose.pose.position.x = 5.0
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.angular.z = 2.0

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not self.received:
            self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(self.received)
        output = self.received[-1]
        self.assertEqual(output.header.frame_id, "odom")
        self.assertEqual(output.child_frame_id, "base_link")
        self.assertAlmostEqual(output.pose.pose.position.x, 4.0)
        self.assertAlmostEqual(output.twist.twist.linear.y, -2.0)

        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        adapter_transforms = [
            transform
            for message in self.transforms
            for transform in message.transforms
            if transform.header.frame_id == "odom"
            and transform.child_frame_id == "base_link"
        ]
        self.assertFalse(adapter_transforms)

    def test_suppresses_odometry_without_required_tf(self):
        message = Odometry()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "missing_tracking_frame"
        message.pose.pose.orientation.w = 1.0

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self.missing_transform_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertFalse(self.missing_transform_received)
