"""Exercise startup gating with real ROS TF, heartbeats, and lifecycle services."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_testing.markers import keep_alive
import launch_testing.asserts
from launch_ros.actions import Node
from launch.events.process import ProcessExited
from geometry_msgs.msg import TransformStamped
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


@keep_alive
def generate_test_description():
    gates = []
    for name in ("success", "failure"):
        gates.append(
            Node(
                package="x2_navigation",
                executable="nav2_localization_startup_gate.py",
                name=f"startup_gate_{name}",
                output="screen",
                parameters=[
                    {
                        "readiness_topic": "/test/ready",
                        "ready_hold_sec": 0.35,
                        "readiness_timeout_sec": 0.15,
                        "tf_max_age_sec": 0.3,
                        "manager_service": f"/test/{name}/manage_nodes",
                    }
                ],
            )
        )
    return launch.LaunchDescription(gates + [launch_testing.actions.ReadyToTest()]), {
        "success_gate": gates[0],
        "failure_gate": gates[1],
    }


class TestStartupGate(unittest.TestCase):
    def test_waits_for_live_readiness_tf_and_service_then_requests_once(
        self, proc_info, success_gate, failure_gate
    ):
        rclpy.init()
        node = rclpy.create_node("test_startup_gate")
        ready = node.create_publisher(Bool, "/test/ready", 10)
        broadcaster = TransformBroadcaster(node)
        calls = {"success": [], "failure": []}
        services = []

        def run_for(seconds, state=None, tf_age=None):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if state is not None:
                    ready.publish(Bool(data=state))
                if tf_age is not None:
                    stamp = (node.get_clock().now().nanoseconds * 1e-9) - tf_age
                    transforms = []
                    for parent, child in (("map", "odom"), ("odom", "base_link")):
                        transform = TransformStamped()
                        transform.header.frame_id = parent
                        transform.child_frame_id = child
                        transform.header.stamp.sec = int(stamp)
                        transform.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
                        transform.transform.rotation.w = 1.0
                        transforms.append(transform)
                    broadcaster.sendTransform(transforms)
                rclpy.spin_once(node, timeout_sec=0.02)

        def no_requests():
            self.assertEqual(calls, {"success": [], "failure": []})
            proc_info.assertWaitForStartup(process=success_gate, timeout=2)
            self.assertNotIsInstance(proc_info[success_gate], ProcessExited)
            self.assertNotIsInstance(proc_info[failure_gate], ProcessExited)

        try:
            run_for(0.8)  # Neither input exists yet.
            run_for(0.5, state=True)  # Readiness alone is insufficient.
            run_for(0.6, state=True, tf_age=5.0)  # Stale TF is insufficient.
            no_requests()
            # Create services only after testing delayed service availability.
            run_for(0.6, state=True, tf_age=0.02)
            no_requests()
            run_for(0.2, state=False, tf_age=0.02)
            for name in calls:

                def callback(request, response, name=name):
                    calls[name].append(request.command)
                    response.success = name == "success"
                    return response

                services.append(
                    node.create_service(
                        ManageLifecycleNodes, f"/test/{name}/manage_nodes", callback
                    )
                )
            run_for(0.18, state=True, tf_age=0.02)
            run_for(0.1, state=False, tf_age=0.02)  # Reset a partial hold.
            run_for(0.18, state=True, tf_age=0.02)
            no_requests()
            run_for(0.4, tf_age=0.02)  # Heartbeat expires before the hold completes.
            no_requests()
            run_for(0.8, state=True, tf_age=0.02)
            self.assertEqual(calls, {"success": [0], "failure": [0]})
            proc_info.assertWaitForShutdown(process=success_gate, timeout=5)
            proc_info.assertWaitForShutdown(process=failure_gate, timeout=5)
            run_for(0.5, state=True, tf_age=0.02)
            self.assertEqual(calls, {"success": [0], "failure": [0]})
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestExitCodes(unittest.TestCase):
    def test_success_and_failure_are_distinguished(
        self, proc_info, success_gate, failure_gate
    ):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=success_gate, allowable_exit_codes=[0]
        )
        launch_testing.asserts.assertExitCodes(
            proc_info, process=failure_gate, allowable_exit_codes=[1]
        )
