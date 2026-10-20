"""Direct local actions with Nav2 present, without localization or hardware."""

import struct
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import launch_testing.asserts
import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from agibot_x2_manipulation_msgs.msg import ManipulationState
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.msg import CollisionMonitorState
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster
from x2_navigation.action import FineAlign, RotateInPlace, Undock


NAV = "/test/direct/navigation"
SMOOTH = "/test/direct/smoothed"
CHECKED = "/test/direct/checked"
OUTPUT = "/test/direct/output"


def generate_test_description():
    package = Path(__file__).parents[1]
    params = yaml.safe_load((package / "config/nav2_params.yaml").read_text())
    params["fine_align_server"]["ros__parameters"].update({
        "default_docking_profile": "default", "stable_sample_count": 1,
        "standoff": 0.5, "settling_duration": 0.15,
        "tracking_timeout": 0.3, "acquisition_timeout": 0.4,
        "maximum_retries": 0, "approach_timeout": 15.0,
        "collision_stop_timeout": 0.1,
    })
    params["collision_monitor"]["ros__parameters"]["transform_tolerance"] = 0.05
    temporary = tempfile.TemporaryDirectory(prefix="x2-direct-test-")
    params_path = Path(temporary.name) / "params.yaml"
    params_path.write_text(yaml.safe_dump(params))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package / "launch/navigation.launch.py")),
        launch_arguments={
            "params_file": str(params_path), "direct_alignment": "true",
            "autostart": "false", "rviz": "false",
            "nav_cmd_topic": NAV, "smoothed_nav_cmd_topic": SMOOTH,
            "checked_nav_cmd_topic": CHECKED, "velocity_topic": OUTPUT,
            "velocity_zmq_endpoint": "tcp://127.0.0.1:18648",
        }.items(),
    )
    return launch.LaunchDescription([
        navigation, launch_testing.actions.ReadyToTest(),
    ]), {"temporary": temporary}


class TestDirectAlignment(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f"test_direct_{self._testMethodName}")
        self.commands = []
        self.checked = []
        self.subscriptions = [
            self.node.create_subscription(Twist, OUTPUT, self.commands.append, 10),
            self.node.create_subscription(Twist, CHECKED, self.checked.append, 10),
        ]
        self.nav = self.node.create_publisher(Twist, NAV, 10)
        self.status = self.node.create_publisher(
            GoalStatusArray, "/navigate_to_pose/_action/status",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.states = self.node.create_publisher(
            ManipulationState, "/manipulation_state",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.collisions = self.node.create_publisher(
            CollisionMonitorState, "/collision_monitor_state", 10)
        self.tags = self.node.create_publisher(
            AprilTagDetectionArray, "/front_center_rectify/detections", 10)
        self.clouds = self.node.create_publisher(
            PointCloud2, "/scan_nav/payload_filtered_cloud", 10)
        self.transforms = TransformBroadcaster(self.node)
        self.client = ActionClient(self.node, FineAlign, "/fine_align")
        self.rotate = ActionClient(self.node, RotateInPlace, "/rotate_in_place")
        self.undock = ActionClient(self.node, Undock, "/undock")
        self.assertTrue(self.client.wait_for_server(timeout_sec=10.0))
        self.tag_x = 0.8
        self.publish_tags = True
        self.publish_odom_tf = False
        self.nav_active = False
        self.obstacle = False
        self.drive(0.3)

    def tearDown(self):
        for client in (self.client, self.rotate, self.undock):
            client.destroy()
        self.node.destroy_node()

    def inputs(self, navigation=None):
        stamp = self.node.get_clock().now().to_msg()
        self.states.publish(ManipulationState(state=ManipulationState.EMPTY))
        status = GoalStatusArray()
        if self.nav_active:
            status.status_list = [GoalStatus(status=GoalStatus.STATUS_EXECUTING)]
        self.status.publish(status)
        # Deliberate STOP must never interrupt direct local actions.
        self.collisions.publish(CollisionMonitorState(action_type=CollisionMonitorState.STOP))
        if self.publish_tags:
            tag = TransformStamped()
            tag.header.stamp = stamp
            tag.header.frame_id = "base_link"
            tag.child_frame_id = "tag9"
            tag.transform.translation.x = self.tag_x
            tag.transform.rotation.x = 0.5
            tag.transform.rotation.y = -0.5
            tag.transform.rotation.z = -0.5
            tag.transform.rotation.w = 0.5
            self.transforms.sendTransform(tag)
            message = AprilTagDetectionArray()
            message.header.stamp = stamp
            message.detections = [AprilTagDetection(id=9, decision_margin=50.0)]
            self.tags.publish(message)
        if self.publish_odom_tf:
            base = TransformStamped()
            base.header.stamp = stamp
            base.header.frame_id = "odom"
            base.child_frame_id = "base_link"
            base.transform.rotation.w = 1.0
            self.transforms.sendTransform(base)
        if navigation is not None:
            self.nav.publish(navigation)
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "base_link"
        cloud.height, cloud.width = 1, 5
        cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32,
                                   count=1) for name, offset in (("x", 0), ("y", 4), ("z", 8))]
        cloud.point_step, cloud.row_step = 12, 60
        cloud.data = struct.pack("<fff", 0.0 if self.obstacle else 2.0, 0.0, 0.0) * 5
        self.clouds.publish(cloud)

    def drive(self, duration, navigation=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.inputs(navigation)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.inputs()
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(predicate())

    def send(self, client, goal):
        sent = client.send_goal_async(goal)
        self.wait(sent.done)
        self.assertTrue(sent.result().accepted)
        return sent.result()

    def cancel(self, handle):
        canceled = handle.cancel_goal_async()
        self.wait(canceled.done)
        result = handle.get_result_async()
        self.wait(result.done)
        self.drive(0.15)
        self.assertEqual(self.commands[-1], Twist())
        return result.result().result

    def transition(self, node, identifier):
        client = self.node.create_client(ChangeState, f"/{node}/change_state")
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        request = ChangeState.Request()
        request.transition.id = identifier
        future = client.call_async(request)
        self.wait(future.done)
        self.assertTrue(future.result().success)
        self.node.destroy_client(client)

    def test_a_routing_and_navigation_collision_stop(self):
        for node in ("velocity_smoother", "collision_monitor"):
            self.transition(node, Transition.TRANSITION_CONFIGURE)
            self.transition(node, Transition.TRANSITION_ACTIVATE)
        self.drive(0.2)
        self.wait(lambda: bool(self.node.get_publishers_info_by_topic(OUTPUT)))
        self.assertEqual(
            [entry.node_name for entry in self.node.get_publishers_info_by_topic(OUTPUT)],
            ["fine_align_server"])
        self.assertEqual(
            {entry.node_name for entry in self.node.get_subscriptions_info_by_topic(SMOOTH)},
            {"collision_monitor"})
        self.assertEqual(
            {entry.node_name for entry in self.node.get_subscriptions_info_by_topic(CHECKED)},
            {"fine_align_server", self.node.get_name()})
        self.assertNotIn("fine_align_server", {
            entry.node_name for entry in
            self.node.get_subscriptions_info_by_topic("/collision_monitor_state")})
        self.publish_odom_tf = True
        command = Twist()
        command.angular.z = 0.3
        self.drive(1.3, command)
        self.assertGreater(self.commands[-1].angular.z, 0.1)
        self.obstacle = True
        self.drive(0.25, command)
        self.assertEqual(self.checked[-1], Twist())
        self.assertEqual(self.commands[-1], Twist())
        self.obstacle = False
        self.drive(0.6, command)
        self.assertGreater(self.commands[-1].angular.z, 0.1)
        # Loss of original commands must stop despite the smoother's tail.
        self.drive(0.35)
        self.assertEqual(self.commands[-1], Twist())
        self.drive(0.4, command)
        self.assertGreater(self.commands[-1].angular.z, 0.0)
        self.drive(0.15, Twist())
        self.assertEqual(self.commands[-1], Twist())
        for node in ("collision_monitor", "velocity_smoother"):
            self.transition(node, Transition.TRANSITION_DEACTIVATE)
        self.drive(0.3)
        self.assertEqual(self.commands[-1], Twist())

    def test_alignment_continues_after_localization_loss_and_completes(self):
        self.publish_odom_tf = True
        self.drive(0.3)
        handle = self.send(self.client, FineAlign.Goal(execute=True, profile_id="default"))
        self.wait(lambda: self.commands[-1].linear.x > 0.0)
        self.publish_odom_tf = False
        self.commands.clear()
        self.drive(0.8)
        self.assertGreater(len(self.commands), 8)
        self.assertTrue(all(command.linear.x > 0.0 for command in self.commands))
        # Smoothly move the fake tag to the configured standoff.
        while self.tag_x > 0.5:
            self.tag_x = max(0.5, self.tag_x - 0.003)
            self.drive(0.05)
        result = handle.get_result_async()
        self.wait(result.done)
        self.assertTrue(result.result().result.success, result.result().result.message)
        self.drive(0.15)
        self.assertEqual(self.commands[-1], Twist())

    def test_tag_loss_and_cancellation_stop_direct_motion(self):
        handle = self.send(self.client, FineAlign.Goal(execute=True, profile_id="default"))
        self.wait(lambda: self.commands[-1].linear.x > 0.0)
        self.publish_tags = False
        result = handle.get_result_async()
        self.wait(result.done)
        self.assertFalse(result.result().result.success)
        self.drive(0.15)
        self.assertEqual(self.commands[-1], Twist())
        self.publish_tags = True
        handle = self.send(self.client, FineAlign.Goal(execute=True, profile_id="default"))
        self.wait(lambda: self.commands[-1].linear.x > 0.0)
        self.cancel(handle)

    def test_navigation_commands_during_rotation_cannot_replay_on_cancel(self):
        checked = self.node.create_publisher(Twist, CHECKED, 10)
        handle = self.send(self.rotate, RotateInPlace.Goal(angular_speed=0.2, duration=3.0))
        self.wait(lambda: self.commands[-1].angular.z == 0.2)
        navigation = Twist()
        navigation.angular.z = 0.4
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            self.inputs(navigation)
            checked.publish(navigation)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        canceled = handle.cancel_goal_async()
        self.wait(canceled.done)
        result = handle.get_result_async()
        self.wait(result.done)
        self.commands.clear()
        self.drive(0.15)
        self.assertTrue(self.commands)
        self.assertFalse(any(command.angular.z == 0.4 for command in self.commands))
        self.assertEqual(self.commands[-1], Twist())

    def test_retained_navigation_status_and_both_navigation_actions(self, launch_service):
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pose_topic = "/test/direct/retained/pose_status"
        through_topic = "/test/direct/retained/through_status"
        pose = self.node.create_publisher(GoalStatusArray, pose_topic, qos)
        through = self.node.create_publisher(GoalStatusArray, through_topic, qos)

        def status(publisher, code=None):
            message = GoalStatusArray()
            if code is not None:
                message.status_list = [GoalStatus(status=code)]
            publisher.publish(message)

        # Publish exactly once, before the subscriber exists. A volatile
        # subscription would miss this goal and incorrectly allow rotation.
        status(pose, GoalStatus.STATUS_ACCEPTED)
        status(through)
        server = Node(
            package="x2_navigation", executable="fine_align_server",
            name="retained_status_server", parameters=[{
                "bypass_collision_monitor": True,
                "raw_cmd_topic": "/test/direct/retained/output",
            }],
            remappings=[
                (f"/{action}/_action/{endpoint}",
                 f"/test/direct/retained/{action}/_action/{endpoint}")
                for action in ("fine_align", "undock", "rotate_in_place")
                for endpoint in ("send_goal", "get_result", "cancel_goal", "status", "feedback")
            ] + [
                ("/navigate_to_pose/_action/status", pose_topic),
                ("/navigate_through_poses/_action/status", through_topic),
            ],
        )
        launch_service.include_launch_description(launch.LaunchDescription([server]))
        client = ActionClient(self.node, RotateInPlace, "/test/direct/retained/rotate_in_place")
        self.addCleanup(client.destroy)
        self.assertTrue(client.wait_for_server(timeout_sec=5.0))
        self.drive(0.2)

        def attempt(expected):
            handle = self.send(client, RotateInPlace.Goal(angular_speed=0.2, duration=0.15))
            result = handle.get_result_async()
            self.wait(result.done)
            self.assertEqual(result.result().result.error_code, expected)

        attempt(RotateInPlace.Result.NAVIGATION_ACTIVE)
        # Clearing one action must not clear the other action's interlock.
        status(through, GoalStatus.STATUS_EXECUTING)
        status(pose)
        self.drive(0.15)
        attempt(RotateInPlace.Result.NAVIGATION_ACTIVE)
        status(through, GoalStatus.STATUS_CANCELING)
        self.drive(0.15)
        attempt(RotateInPlace.Result.NAVIGATION_ACTIVE)
        status(through, GoalStatus.STATUS_CANCELED)
        self.drive(0.15)
        attempt(RotateInPlace.Result.SUCCESS)
        # Navigation becoming active during local motion must abort it.
        handle = self.send(client, RotateInPlace.Goal(angular_speed=0.2, duration=2.0))
        self.drive(0.15)
        status(through, GoalStatus.STATUS_EXECUTING)
        result = handle.get_result_async()
        self.wait(result.done)
        self.assertEqual(result.result().result.error_code, RotateInPlace.Result.NAVIGATION_ACTIVE)

    def test_navigation_interlock_rotation_and_undock(self):
        self.nav_active = True
        self.drive(0.2)
        handle = self.send(self.client, FineAlign.Goal(execute=True, profile_id="default"))
        result = handle.get_result_async()
        self.wait(result.done)
        self.assertEqual(result.result().result.error_code, FineAlign.Result.NAVIGATION_ACTIVE)
        self.nav_active = False
        self.drive(0.2)
        handle = self.send(self.rotate, RotateInPlace.Goal(angular_speed=0.2, duration=0.4))
        result = handle.get_result_async()
        self.wait(lambda: self.commands[-1].angular.z > 0.0)
        self.wait(result.done)
        self.assertTrue(result.result().result.success)
        self.drive(0.15)
        self.tag_x = 0.5
        handle = self.send(self.undock, Undock.Goal(profile_id="default"))
        self.wait(lambda: self.commands[-1].linear.x < 0.0)
        self.cancel(handle)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info, temporary):
        launch_testing.asserts.assertExitCodes(proc_info)
        temporary.cleanup()
