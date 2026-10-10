"""Exercise open-loop rotation without Nav2, tags, odometry, or hardware."""

import time
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from agibot_x2_manipulation_msgs.msg import ManipulationState
from geometry_msgs.msg import Twist
from launch_ros.actions import Node
from nav2_msgs.msg import CollisionMonitorState
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from x2_navigation.action import FineAlign, RotateInPlace, Undock


def generate_test_description():
    server = Node(package="x2_navigation", executable="fine_align_server",
                  parameters=[{"rotate_max_duration": 2.0}], output="screen")
    return launch.LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class TestRotation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_timed_rotation")
        self.commands = []
        self.subscription = self.node.create_subscription(Twist, "/cmd_vel_raw", self.commands.append, 10)
        self.states = self.node.create_publisher(
            ManipulationState, "/manipulation_state",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.collisions = self.node.create_publisher(CollisionMonitorState, "/collision_monitor_state", 10)
        self.nav = self.node.create_publisher(GoalStatusArray, "/navigate_to_pose/_action/status", 10)
        self.client = ActionClient(self.node, RotateInPlace, "/rotate_in_place")
        self.assertTrue(self.client.wait_for_server(timeout_sec=5.0))
        self.publish_state()
        self.publish_collision(False)
        self.publish_nav(False)
        self.spin_for(0.2)

    def tearDown(self):
        self.client.destroy()
        self.node.destroy_node()

    def spin_until(self, predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(predicate())

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def publish_state(self, state=ManipulationState.EMPTY):
        message = ManipulationState()
        message.state = state
        self.states.publish(message)

    def publish_collision(self, stopped):
        message = CollisionMonitorState()
        message.action_type = CollisionMonitorState.STOP if stopped else CollisionMonitorState.DO_NOTHING
        self.collisions.publish(message)

    def publish_nav(self, active):
        message = GoalStatusArray()
        if active:
            status = GoalStatus()
            status.status = GoalStatus.STATUS_EXECUTING
            message.status_list = [status]
        self.nav.publish(message)

    def send(self, speed=0.2, duration=0.3):
        goal = RotateInPlace.Goal()
        goal.angular_speed = speed
        goal.duration = duration
        feedback = []
        future = self.client.send_goal_async(goal, feedback_callback=lambda message: feedback.append(message.feedback))
        self.spin_until(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        return handle, feedback

    def result(self, handle):
        future = handle.get_result_async()
        self.spin_until(future.done)
        self.spin_until(lambda: self.commands and self.commands[-1] == Twist())
        return future.result()

    def test_both_directions_complete_without_external_pose(self):
        for speed, state in [(0.2, ManipulationState.EMPTY), (-0.3, ManipulationState.HOLDING)]:
            self.publish_state(state)
            self.spin_for(0.1)
            self.commands.clear()
            started = time.monotonic()
            handle, feedback = self.send(speed)
            response = self.result(handle)
            self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
            self.assertTrue(response.result.success)
            self.assertGreaterEqual(response.result.elapsed_time, 0.3)
            self.assertLess(time.monotonic() - started, 1.0)
            self.assertTrue(feedback)
            self.assertTrue(any(command.angular.z == speed for command in self.commands))
            self.assertTrue(all(command.linear.x == command.linear.y == command.linear.z == 0.0
                                and command.angular.x == command.angular.y == 0.0 for command in self.commands))
            self.assertTrue(all(0.0 <= item.progress <= 1.0 for item in feedback))

    def test_invalid_requests_never_move(self):
        for speed, duration in [(0.0, 1.0), (0.6, 1.0), (float("nan"), 1.0),
                                (0.2, 0.0), (0.2, -1.0), (0.2, 2.1), (0.2, float("inf"))]:
            self.commands.clear()
            handle, _ = self.send(speed, duration)
            result = self.result(handle).result
            self.assertEqual(result.error_code, RotateInPlace.Result.INVALID_GOAL)
            self.assertFalse(any(command != Twist() for command in self.commands))

    def test_interruption_stops_and_releases_ownership(self):
        for interrupt, expected in [
                (lambda: self.publish_collision(True), RotateInPlace.Result.COLLISION_STOPPED),
                (lambda: self.publish_state(ManipulationState.UNKNOWN), RotateInPlace.Result.INVALID_STATE),
                (lambda: self.publish_nav(True), RotateInPlace.Result.NAVIGATION_ACTIVE)]:
            self.commands.clear()
            handle, _ = self.send(duration=1.5)
            self.spin_until(lambda: any(command.angular.z != 0.0 for command in self.commands))
            interrupt()
            result = self.result(handle).result
            self.assertEqual(result.error_code, expected)
            self.assertLess(result.elapsed_time, 1.5)
            self.publish_state()
            self.publish_collision(False)
            self.publish_nav(False)
            self.spin_for(0.1)
        handle, _ = self.send(duration=0.1)
        self.assertTrue(self.result(handle).result.success)

    def test_busy_and_cancellation(self):
        handle, _ = self.send(duration=1.5)
        self.spin_until(lambda: any(command.angular.z != 0.0 for command in self.commands))
        for action, name in [(RotateInPlace, "/rotate_in_place"), (FineAlign, "/fine_align"), (Undock, "/undock")]:
            client = ActionClient(self.node, action, name)
            self.assertTrue(client.wait_for_server(timeout_sec=2.0))
            future = client.send_goal_async(action.Goal())
            self.spin_until(future.done)
            self.assertFalse(future.result().accepted)
            client.destroy()
        canceled = handle.cancel_goal_async()
        self.spin_until(canceled.done)
        response = self.result(handle)
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(response.result.error_code, RotateInPlace.Result.CANCELED)
        handle, _ = self.send(duration=0.1)
        self.assertTrue(self.result(handle).result.success)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
