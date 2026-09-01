import time
import unittest
from pathlib import Path

import launch
import launch_testing.asserts
import launch_testing.actions
import rclpy
from agibot_x2_manipulation_msgs.msg import ManipulationState
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, Twist
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from x2_navigation.action import FineAlign


def generate_test_description():
    package_path = Path(__file__).parents[1]
    server = Node(
        package="x2_navigation",
        executable="fine_align_server",
        parameters=[
            package_path / "config" / "nav2_params.yaml",
            {
                "stable_sample_count": 1,
                "settled_sample_count": 1,
                "acquisition_timeout": 2.0,
                "approach_timeout": 3.0,
                "maximum_pose_age": 1.0,
            },
        ],
        output="screen",
    )
    return launch.LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class TestFineAlignServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(
            f"test_fine_align_server_{self._testMethodName}"
        )
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.detections = self.node.create_publisher(
            AprilTagDetectionArray,
            "/front_center_rectify/detections",
            sensor_qos,
        )
        self.states = self.node.create_publisher(
            ManipulationState, "/manipulation_state", state_qos
        )
        self.odometry = self.node.create_publisher(Odometry, "/odom", sensor_qos)
        self.commands = []
        self.command_subscription = self.node.create_subscription(
            Twist, "/cmd_vel_raw", self.commands.append, 10
        )
        self.transforms = TransformBroadcaster(self.node)
        self.client = ActionClient(self.node, FineAlign, "/fine_align")
        self.assertTrue(self.client.wait_for_server(timeout_sec=5.0))

    def tearDown(self):
        self.client.destroy()
        self.node.destroy_node()

    def publish_inputs(self):
        stamp = self.node.get_clock().now().to_msg()

        tag = TransformStamped()
        tag.header.stamp = stamp
        tag.header.frame_id = "odom"
        tag.child_frame_id = "tag9"
        # Tag +Z points out of the table along odom +X. The target is therefore
        # (0.50, 0.0, pi) for the configured 0.50 m standoff.
        tag.transform.rotation.x = 0.5
        tag.transform.rotation.y = 0.5
        tag.transform.rotation.z = 0.5
        tag.transform.rotation.w = 0.5

        base = TransformStamped()
        base.header.stamp = stamp
        base.header.frame_id = "odom"
        base.child_frame_id = "base_link"
        base.transform.translation.x = 1.10
        base.transform.translation.y = 0.20
        base.transform.rotation.z = 1.0
        base.transform.rotation.w = 0.0
        self.transforms.sendTransform([tag, base])

        detection = AprilTagDetection()
        detection.id = 9
        detection.decision_margin = 50.0
        message = AprilTagDetectionArray()
        message.header.stamp = stamp
        message.detections = [detection]
        self.detections.publish(message)

        state = ManipulationState()
        state.state = ManipulationState.EMPTY
        self.states.publish(state)

        odometry = Odometry()
        odometry.header.stamp = stamp
        self.odometry.publish(odometry)

    def spin_with_inputs_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.publish_inputs()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def warm_up_inputs(self, duration=1.0):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish_inputs()
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_measurement_only_validates_without_motion(self):
        self.warm_up_inputs()
        goal = FineAlign.Goal()
        goal.execute = False
        sent = self.client.send_goal_async(goal)
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result_future.done))
        action_result = result_future.result().result
        self.assertTrue(
            action_result.success,
            f"fine alignment failed with {action_result.error_code}: "
            f"{action_result.message}; error={action_result.final_error!r}",
        )
        self.assertEqual(action_result.error_code, FineAlign.Result.SUCCESS)
        self.assertFalse(any(command.linear.x or command.linear.y for command in self.commands))

    def test_missing_tag_aborts_with_diagnostics(self):
        # Let any target retained from an earlier test exceed maximum_pose_age.
        time.sleep(1.1)
        goal = FineAlign.Goal()
        goal.execute = False
        sent = self.client.send_goal_async(goal)
        self.assertTrue(self.spin_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()
        self.assertTrue(self.spin_until(result_future.done, timeout=3.0))
        action_result = result_future.result().result
        self.assertFalse(action_result.success)
        self.assertEqual(action_result.error_code, FineAlign.Result.NO_STABLE_TAG)
        self.assertIn("no stable", action_result.message)

    def test_execution_emits_coupled_planar_command_and_cancels_cleanly(self):
        self.warm_up_inputs()
        goal = FineAlign.Goal()
        goal.execute = True
        sent = self.client.send_goal_async(goal)
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        self.assertTrue(
            self.spin_with_inputs_until(
                lambda: any(
                    command.linear.x > 0.0 and command.linear.y > 0.0
                    for command in self.commands
                )
            ),
            f"received commands: {self.commands!r}",
        )
        cancel = handle.cancel_goal_async()
        self.assertTrue(self.spin_with_inputs_until(cancel.done))
        result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertFalse(result.result().result.success)


@launch_testing.post_shutdown_test()
class TestFineAlignServerShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
