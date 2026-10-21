"""Exercise real Odin adapter root TF with synthetic sensors and the Nav2 chain."""

import importlib.util
from pathlib import Path
import time
import tempfile
import sys

import rclpy
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
from launch.actions import ExecuteProcess
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


SPEC = importlib.util.spec_from_file_location(
    "navigation_fixture", Path(__file__).with_name("test_navigation.launch.py")
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


def generate_test_description():
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as config:
        config.write("odin_localization_adapter:\n  ros__parameters:\n    map_alignment_confirmed: true\n")
        config_file = config.name
    description = BASE.navigation_test_description("odin", root_tf=False, odin_params_file=config_file)
    description.add_action(ExecuteProcess(
        cmd=[sys.executable, str(Path(__file__).with_name("odin_navigation_inputs.py")),
             "--ros-args", "-p", "wait_for_corrections:=true"],
        output="screen",
    ))
    return description


class TestOdinNavigation(BASE.TestNavigationLifecycle):
    def test_00_waits_for_relocalization_before_configuring_nav2(self, proc_output):
        clients = [self.node.create_client(GetState, f"/{name}/get_state")
                   for name in ("map_server", "planner_server", "controller_server")]
        try:
            for client in clients:
                self.assertTrue(client.wait_for_service(timeout_sec=8))
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
                self.assertTrue(future.done())
                self.assertEqual(future.result().current_state.label, "unconfigured")
            # Hold localization unavailable long enough to expose premature activation.
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            for client in clients:
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
                self.assertEqual(future.result().current_state.label, "unconfigured")
            enable = self.node.create_publisher(Bool, "/test/enable_odin_corrections", 10)
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                enable.publish(Bool(data=True))
                rclpy.spin_once(self.node, timeout_sec=0.05)
            self.wait_for_active_lifecycle_node("planner_server")
            self.wait_for_active_lifecycle_node("controller_server")
            output = b"".join(event.text for event in proc_output)
            self.assertNotIn(b'Timed out waiting for transform from base_link to map', output)
            self.node.destroy_publisher(enable)
        finally:
            for client in clients:
                self.node.destroy_client(client)

    def test_adapter_supplies_odometry_and_readiness(self):
        odometry = []
        ready = []
        odom_sub = self.node.create_subscription(Odometry, "/odom", odometry.append, 10)
        ready_sub = self.node.create_subscription(Bool, "/odin1/localization_ready", ready.append, 10)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not (odometry and ready and ready[-1].data):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(odometry)
        self.assertTrue(ready[-1].data)
        self.assertAlmostEqual(odometry[-1].pose.pose.position.x, 0.0, places=5)
        self.assertEqual({p.node_name for p in self.node.get_publishers_info_by_topic("/odom")},
                         {"odin_localization_adapter"})
        self.node.destroy_subscription(odom_sub)
        self.node.destroy_subscription(ready_sub)

    def test_odin_selection_and_bridge_readiness_enforcement(self):
        deadline = time.monotonic() + 5
        names = set()
        node_names = []
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            node_names = self.node.get_node_names()
            names = set(node_names)
            if {"odin_localization_adapter", "robot_state_publisher", "nav2_zmq_velocity_bridge"} <= names:
                break
        self.assertIn("odin_localization_adapter", names)
        self.assertNotIn("fast_lio_odom_adapter", names)
        self.assertEqual(node_names.count("robot_state_publisher"), 1)
        self.assertNotIn("controller_manager", names)
        client = self.node.create_client(GetParameters, "/nav2_zmq_velocity_bridge/get_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        request = GetParameters.Request(names=["require_localization_ready"])
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
        self.assertTrue(future.done())
        self.assertTrue(future.result().values[0].bool_value)
