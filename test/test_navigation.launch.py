import struct
import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def generate_test_description():
    package_path = Path(get_package_share_directory("x2_navigation"))
    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
    )
    static_odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "base_link"],
    )
    static_base_to_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "1",
            "base_link",
            "lidar_chest_front",
        ],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_path / "launch" / "navigation.launch.py")),
        launch_arguments={
            "rviz": "false",
            "lidar_pointcloud_topic": "/test/lidar_pointcloud",
            "velocity_zmq_endpoint": "tcp://127.0.0.1:18561",
        }.items(),
    )
    return launch.LaunchDescription(
        [
            static_map_to_odom,
            static_odom_to_base,
            static_base_to_lidar,
            navigation,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestNavigationLifecycle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_navigation_lifecycle")
        self.map_messages = []
        self.map_subscription = self.node.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_messages.append,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.navigation_cloud_messages = []
        self.navigation_cloud_subscription = self.node.create_subscription(
            PointCloud2,
            "/scan_nav/cloud",
            self.navigation_cloud_messages.append,
            sensor_qos,
        )
        self.local_costmap_messages = []
        self.local_costmap_subscription = self.node.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.local_costmap_messages.append,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.cloud_publisher = self.node.create_publisher(
            PointCloud2,
            "/test/lidar_pointcloud",
            sensor_qos,
        )

    def tearDown(self):
        self.node.destroy_publisher(self.cloud_publisher)
        self.node.destroy_subscription(self.local_costmap_subscription)
        self.node.destroy_subscription(self.navigation_cloud_subscription)
        self.node.destroy_subscription(self.map_subscription)
        self.node.destroy_node()

    def wait_for_active_lifecycle_node(self, node_name):
        client = self.node.create_client(GetState, f"/{node_name}/get_state")
        deadline = time.monotonic() + 15.0
        response = None
        while time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=0.5):
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
            if not future.done():
                continue
            response = future.result()
            if response.current_state.label == "active":
                break

        self.assertIsNotNone(response)
        self.assertEqual(response.current_state.label, "active")

    def test_navigation_lifecycle_nodes_become_active(self):
        for node_name in (
            "map_server",
            "planner_server",
            "controller_server",
            "behavior_server",
            "bt_navigator",
        ):
            with self.subTest(node_name=node_name):
                self.wait_for_active_lifecycle_node(node_name)

    def test_map_server_publishes_the_packaged_map(self):
        deadline = time.monotonic() + 10.0
        while not self.map_messages and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.assertTrue(self.map_messages)
        map_message = self.map_messages[-1]
        self.assertEqual(map_message.header.frame_id, "map")
        self.assertEqual(map_message.info.width, 1014)
        self.assertEqual(map_message.info.height, 799)
        self.assertAlmostEqual(map_message.info.resolution, 0.05, places=6)

    def test_raw_lidar_cloud_is_filtered_for_navigation(self):
        cloud = PointCloud2()
        cloud.header.frame_id = "lidar_chest_front"
        cloud.height = 1
        cloud.width = 4
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.width * cloud.point_step
        cloud.data = struct.pack(
            "<ffffffffffff",
            1.01,
            0.00,
            0.00,
            1.04,
            0.02,
            0.00,
            1.08,
            0.00,
            0.00,
            1.00,
            0.00,
            0.40,
        )
        cloud.is_dense = True
        input_stamp = self.node.get_clock().now().to_msg()
        cloud.header.stamp = input_stamp

        deadline = time.monotonic() + 10.0
        while not self.navigation_cloud_messages and time.monotonic() < deadline:
            self.cloud_publisher.publish(cloud)
            rclpy.spin_once(self.node, timeout_sec=0.25)

        self.assertTrue(self.navigation_cloud_messages)
        navigation_cloud = self.navigation_cloud_messages[-1]
        self.assertEqual(navigation_cloud.header.frame_id, "base_link")
        self.assertEqual(navigation_cloud.width, 2)
        self.assertEqual(navigation_cloud.point_step, 12)
        self.assertEqual([field.name for field in navigation_cloud.fields], ["x", "y", "z"])
        input_nanoseconds = input_stamp.sec * 1_000_000_000 + input_stamp.nanosec
        cloud_nanoseconds = (
            navigation_cloud.header.stamp.sec * 1_000_000_000
            + navigation_cloud.header.stamp.nanosec
        )
        self.assertEqual(cloud_nanoseconds, input_nanoseconds)

    def test_navigation_cloud_marks_the_local_costmap(self):
        cloud = PointCloud2()
        cloud.header.frame_id = "lidar_chest_front"
        cloud.height = 1
        cloud.width = 1
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step
        cloud.data = struct.pack("<fff", 1.0, 0.0, 0.0)
        cloud.is_dense = True

        deadline = time.monotonic() + 10.0
        has_lethal_obstacle = False
        while time.monotonic() < deadline:
            cloud.header.stamp = self.node.get_clock().now().to_msg()
            self.cloud_publisher.publish(cloud)
            rclpy.spin_once(self.node, timeout_sec=0.25)
            has_lethal_obstacle = any(
                100 in message.data for message in self.local_costmap_messages
            )
            if has_lethal_obstacle:
                break

        self.assertTrue(self.local_costmap_messages)
        self.assertTrue(has_lethal_obstacle)
