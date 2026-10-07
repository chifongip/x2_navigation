import time
import unittest
from math import cos, sin
from pathlib import Path

import launch
import launch_testing.asserts
import launch_testing.actions
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from agibot_x2_manipulation_msgs.msg import ManipulationState, BoxState, BoxStateArray
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, Twist
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
from nav2_msgs.msg import CollisionMonitorState
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from x2_navigation.action import FineAlign, Undock


def generate_test_description():
    package_path = Path(__file__).parents[1]
    server = Node(
        package="x2_navigation",
        executable="fine_align_server",
        parameters=[
            package_path / "config" / "nav2_params.yaml",
            {
                "docking_profile_names": ["offset", "other_tag", "box", "box_b", "timed_timeout"],
                "docking_profiles.timed_timeout.target_source": "box",
                "docking_profiles.timed_timeout.standoff": 0.5,
                "docking_profiles.timed_timeout.lateral_offset": 0.0,
                "docking_profiles.timed_timeout.yaw_offset": 0.0,
                "docking_profiles.timed_timeout.undock_mode": "timed_reverse",
                # Both deadlines fall in the same controller tick. The shorter
                # global timeout must win, rather than reporting completion.
                "docking_profiles.timed_timeout.timed_reverse_duration": 3.000001,
                "docking_profiles.box_b.target_source": "box",
                "docking_profiles.box_b.standoff": 0.6,
                "docking_profiles.box_b.lateral_offset": 0.0,
                "docking_profiles.box_b.yaw_offset": 0.0,
                "docking_profiles.box_b.detections_topic": "/detections",
                "docking_profiles.box_b.undock_mode": "timed_reverse",
                "docking_profiles.box_b.timed_reverse_duration": 0.6,
                "docking_profiles.box.target_source": "box",
                "docking_profiles.box.standoff": 0.5,
                "docking_profiles.box.lateral_offset": 0.0,
                "docking_profiles.box.yaw_offset": 0.0,
                "docking_profiles.box.detections_topic": "/detections",
                "docking_profiles.box.undock_mode": "timed_reverse",
                "docking_profiles.box.timed_reverse_speed": 0.1,
                "docking_profiles.box.timed_reverse_duration": 0.6,
                "docking_profiles.offset.tag_id": 9,
                "docking_profiles.offset.tag_frame": "tag9",
                "docking_profiles.offset.standoff": 0.7,
                "docking_profiles.offset.lateral_offset": 0.1,
                "docking_profiles.offset.yaw_offset": 0.1,
                "docking_profiles.other_tag.tag_id": 10,
                "docking_profiles.other_tag.tag_frame": "tag10",
                "docking_profiles.other_tag.standoff": 0.6,
                "docking_profiles.other_tag.lateral_offset": 0.0,
                "docking_profiles.other_tag.yaw_offset": 0.0,
                "stable_sample_count": 1,
                "settled_sample_count": 1,
                "settling_duration": 0.15,
                "tracking_timeout": 0.5,
                "translation_gain": 1.0,
                "translation_speed_max": 0.1,
                "angular_speed_max": 0.1,
                # Keep the simulated 0.1 m retreat outside the initial deadband.
                "x_position_tolerance": 0.08,
                "y_position_tolerance": 0.08,
                "acquisition_timeout": 0.5,
                "approach_timeout": 3.0,
                "maximum_pose_age": 1.0,
                "maximum_retries": 1,
                "retry_delay": 0.1,
                "undock_distance": 0.1,
                "undock_timeout": 3.0,
                "undock_translation_speed_min": 0.1,
                "undock_translation_speed_max": 0.1,
                "undock_angular_speed_min": 0.1,
                "undock_angular_speed_max": 0.1,
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
        self.box_states = self.node.create_publisher(BoxStateArray, "/box_states", 10)
        self.states = self.node.create_publisher(
            ManipulationState, "/manipulation_state", state_qos
        )
        self.odometry = self.node.create_publisher(Odometry, "/odom", sensor_qos)
        self.commands = []
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_linear_velocity = 0.0
        self.odom_angular_velocity = 0.0
        self.odom_yaw = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.publish_odometry = False
        self.inject_drift = False
        self.manipulation_state = ManipulationState.EMPTY
        self.nav_active = False
        self.collision_stopped = False
        self.invalid_tag_orientation = False
        self.tag_id = 9
        self.tag_frame = "tag9"
        self.detection_stamp = None
        self.decision_margin = 50.0
        self.tag_x = 1.10
        self.tag_y = 0.20
        self.base_tf_x = 1.10
        self.publish_tags = True
        self.simulate_motion = False
        self.last_input_time = time.monotonic()
        self.command_subscription = self.node.create_subscription(
            Twist, "/cmd_vel_raw", self.commands.append, 10
        )
        self.nav_status = self.node.create_publisher(
            GoalStatusArray, "/navigate_to_pose/_action/status", 10
        )
        self.collision_state = self.node.create_publisher(
            CollisionMonitorState, "/collision_monitor_state", 10
        )
        self.transforms = TransformBroadcaster(self.node)
        self.client = ActionClient(self.node, FineAlign, "/fine_align")
        self.undock_client = ActionClient(self.node, Undock, "/undock")
        self.assertTrue(self.client.wait_for_server(timeout_sec=5.0))
        self.assertTrue(self.undock_client.wait_for_server(timeout_sec=5.0))

    def tearDown(self):
        self.client.destroy()
        self.undock_client.destroy()
        self.node.destroy_node()

    def publish_inputs(self):
        input_time = time.monotonic()
        if self.inject_drift:
            dt = input_time - self.last_input_time
            # Add physically plausible drift rather than teleporting the tag pose.
            self.robot_y = min(0.15, self.robot_y + 0.075 * dt)
            self.robot_yaw = min(0.20, self.robot_yaw + 0.10 * dt)
        if self.simulate_motion and self.commands:
            command = self.commands[-1]
            dt = input_time - self.last_input_time
            self.tag_x -= command.linear.x * dt
            self.tag_y -= command.linear.y * dt
            self.odom_linear_velocity = (
                command.linear.x ** 2 + command.linear.y ** 2
            ) ** 0.5
            # Deliberately drift the odom frame independently of the relative tag.
            self.base_tf_x += dt * 2.0
        self.last_input_time = input_time
        stamp = self.node.get_clock().now().to_msg()

        tag = TransformStamped()
        tag.header.stamp = stamp
        # Publish the tag through the camera/base branch, independent of odom.
        tag.header.frame_id = "base_link"
        tag.child_frame_id = self.tag_frame
        tag.transform.translation.x = (
            cos(self.robot_yaw) * (self.tag_x - self.robot_x)
            + sin(self.robot_yaw) * (self.tag_y - self.robot_y)
        )
        tag.transform.translation.y = (
            -sin(self.robot_yaw) * (self.tag_x - self.robot_x)
            + cos(self.robot_yaw) * (self.tag_y - self.robot_y)
        )
        if self.simulate_motion:
            tag.transform.translation.x += 0.002 * sin(input_time * 20.0)
            tag.transform.translation.y += 0.002 * cos(input_time * 20.0)
        qw = cos(-self.robot_yaw / 2.0)
        qz = sin(-self.robot_yaw / 2.0)
        tag.transform.rotation.x = 0.5 * (qw + qz)
        tag.transform.rotation.y = 0.5 * (-qw + qz)
        tag.transform.rotation.z = 0.5 * (-qw + qz)
        tag.transform.rotation.w = 0.5 * (qw + qz)
        if self.invalid_tag_orientation:
            # Identity rotation puts tag +Z vertically, invalid for table docking.
            tag.transform.rotation.x = 0.0
            tag.transform.rotation.y = 0.0
            tag.transform.rotation.z = 0.0
            tag.transform.rotation.w = 1.0

        base = TransformStamped()
        base.header.stamp = stamp
        base.header.frame_id = "odom"
        base.child_frame_id = "base_link"
        base.transform.translation.x = self.base_tf_x
        base.transform.translation.y = 0.20
        base.transform.rotation.z = 1.0
        base.transform.rotation.w = 0.0
        self.transforms.sendTransform([tag, base] if self.publish_tags else [base])

        detection = AprilTagDetection()
        detection.id = self.tag_id
        detection.decision_margin = self.decision_margin
        message = AprilTagDetectionArray()
        message.header.stamp = self.detection_stamp or stamp
        message.detections = [detection]
        if self.publish_tags:
            self.detections.publish(message)

        state = ManipulationState()
        state.state = self.manipulation_state
        self.states.publish(state)
        box = BoxState()
        box.header.stamp = stamp
        box.instance_id = f"tag:{self.tag_id}"
        box.profile_id = "box_b_type" if self.tag_id == 42 else "box_a_type"
        box.docking_profile_ids = ["box_b"] if self.tag_id == 42 else ["box"]
        box.default_docking_profile = box.docking_profile_ids[0]
        box.tag_frame = self.tag_frame
        self.box_states.publish(BoxStateArray(boxes=[box]))

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = self.odom_x
        odometry.pose.pose.position.y = self.odom_y
        odometry.pose.pose.orientation.z = sin(self.odom_yaw / 2.0)
        odometry.pose.pose.orientation.w = cos(self.odom_yaw / 2.0)
        odometry.twist.twist.linear.x = self.odom_linear_velocity
        odometry.twist.twist.angular.z = self.odom_angular_velocity
        if self.publish_odometry:
            self.odometry.publish(odometry)

        nav_status = GoalStatusArray()
        if self.nav_active:
            active = GoalStatus()
            active.status = GoalStatus.STATUS_EXECUTING
            nav_status.status_list = [active]
        self.nav_status.publish(nav_status)

        collision = CollisionMonitorState()
        collision.action_type = (
            CollisionMonitorState.STOP
            if self.collision_stopped
            else CollisionMonitorState.DO_NOTHING
        )
        self.collision_state.publish(collision)

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

    def submit_profile(self, profile="", execute=False, undock=False, inputs=True, instance_id=""):
        goal = Undock.Goal() if undock else FineAlign.Goal()
        goal.profile_id = profile
        if not undock:
            goal.instance_id = instance_id
            goal.execute = execute
        feedback = []
        client = self.undock_client if undock else self.client
        spin = self.spin_with_inputs_until if inputs else self.spin_until
        if instance_id:
            self.spin_with_inputs_until(lambda: False, timeout=0.2)
        sent = client.send_goal_async(
            goal, feedback_callback=lambda message: feedback.append(message.feedback)
        )
        self.assertTrue(spin(sent.done))
        self.assertTrue(sent.result().accepted)
        result = sent.result().get_result_async()
        self.assertTrue(spin(result.done, timeout=8.0))
        return result.result().result, feedback

    def test_a_undock_uses_default_before_any_successful_dock(self):
        self.simulate_motion = True
        # Action discovery can complete before the first transient state sample
        # reaches the server. Publish state before testing initial Undock.
        self.warm_up_inputs()
        result, feedback = self.submit_profile(undock=True)
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.profile_id, "default")
        self.assertTrue(all(item.profile_id == "default" for item in feedback))

    def test_named_profiles_change_geometry_and_tag_selection(self):
        self.warm_up_inputs(duration=0.2)
        default, _ = self.submit_profile()
        offset, feedback = self.submit_profile("offset")
        self.assertTrue(default.success, default.message)
        self.assertTrue(offset.success, offset.message)
        self.assertEqual(default.profile_id, "default")
        self.assertEqual(offset.profile_id, "offset")
        self.assertAlmostEqual(default.final_error.x - offset.final_error.x, 0.2, places=5)
        self.assertAlmostEqual(offset.final_error.y - default.final_error.y, -0.1, places=5)
        self.assertAlmostEqual(offset.final_error.theta - default.final_error.theta, 0.1, places=5)
        self.assertTrue(feedback)
        self.assertTrue(all(item.profile_id == "offset" for item in feedback))
        wrong_tag, _ = self.submit_profile("other_tag")
        self.assertEqual(wrong_tag.error_code, FineAlign.Result.NO_STABLE_TAG)
        # Matching detection without matching timestamped TF must not use tag9.
        self.tag_id = 10
        missing_tf, _ = self.submit_profile("other_tag")
        self.assertEqual(missing_tf.error_code, FineAlign.Result.NO_STABLE_TAG)
        self.tag_frame = "tag10"
        other, _ = self.submit_profile("other_tag")
        self.assertTrue(other.success, other.message)
        self.assertEqual(other.profile_id, "other_tag")
        self.assertAlmostEqual(other.final_error.x, self.tag_x - 0.6, places=5)
        self.assertFalse(any(command != Twist() for command in self.commands))

    def test_profile_activation_requires_fresh_observations(self):
        self.warm_up_inputs(duration=0.2)
        self.detection_stamp = self.node.get_clock().now().to_msg()
        time.sleep(0.05)
        stale, _ = self.submit_profile("offset")
        self.assertEqual(stale.error_code, FineAlign.Result.NO_STABLE_TAG)
        self.detection_stamp = None
        fresh, _ = self.submit_profile("offset")
        self.assertTrue(fresh.success, fresh.message)
        # Even the same profile cannot reuse the last operation's cached target.
        cached, _ = self.submit_profile("offset", inputs=False)
        self.assertEqual(cached.error_code, FineAlign.Result.NO_STABLE_TAG)
        self.assertFalse(any(command != Twist() for command in self.commands))

    def test_profile_parameters_are_read_only(self):
        client = self.node.create_client(SetParameters, "/fine_align_server/set_parameters")
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=3.0))
            request = SetParameters.Request()
            request.parameters = [
                Parameter("standoff", value=0.9).to_parameter_msg(),
                Parameter("docking_profiles.offset.standoff", value=0.9).to_parameter_msg(),
                Parameter("default_docking_profile", value="offset").to_parameter_msg(),
                Parameter("docking_profile_names", value=["other_tag"]).to_parameter_msg(),
            ]
            future = client.call_async(request)
            self.assertTrue(self.spin_until(future.done))
            self.assertEqual(len(future.result().results), len(request.parameters))
            self.assertTrue(all(not result.successful for result in future.result().results))
            result, _ = self.submit_profile("offset")
            self.assertTrue(result.success, result.message)
            self.assertAlmostEqual(result.final_error.x, self.tag_x - 0.7, places=5)
        finally:
            self.node.destroy_client(client)

    def test_unknown_profiles_abort_without_motion(self):
        for undock in (False, True):
            result, _ = self.submit_profile("missing", undock=undock)
            expected = Undock.Result.INVALID_PROFILE if undock else FineAlign.Result.INVALID_PROFILE
            self.assertEqual(result.error_code, expected)
            self.assertFalse(result.success)
            self.assertEqual(result.profile_id, "")
        self.assertFalse(any(command != Twist() for command in self.commands))

    def test_profile_execution_and_remembered_undocking(self):
        self.tag_x = 0.85
        self.tag_y = 0.1
        self.simulate_motion = True
        dock, _ = self.submit_profile("offset", execute=True)
        self.assertTrue(dock.success, dock.message)
        self.assertLessEqual(abs(self.tag_x - 0.7), 0.085)
        self.assertLessEqual(abs(self.tag_y - 0.1), 0.085)
        measured, _ = self.submit_profile("default")
        self.assertTrue(measured.success, measured.message)
        failed, _ = self.submit_profile("missing", execute=True)
        self.assertEqual(failed.error_code, FineAlign.Result.INVALID_PROFILE)
        # A valid profile failing capture validation must also leave history intact.
        saved_x = self.tag_x
        self.tag_x = 3.0
        failed_capture, _ = self.submit_profile("default", execute=True)
        self.assertEqual(failed_capture.error_code, FineAlign.Result.OUTSIDE_CAPTURE_ENVELOPE)
        self.tag_x = saved_x
        self.simulate_motion = False
        self.commands.clear()
        cancel_goal = FineAlign.Goal()
        cancel_goal.profile_id = "default"
        cancel_goal.execute = True
        sent = self.client.send_goal_async(cancel_goal)
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        self.assertTrue(self.spin_with_inputs_until(
            lambda: any(command.linear.x > 0.0 for command in self.commands)
        ))
        canceled = handle.cancel_goal_async()
        self.assertTrue(self.spin_with_inputs_until(canceled.done))
        canceled_result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(canceled_result.done))
        self.assertEqual(canceled_result.result().status, GoalStatus.STATUS_CANCELED)
        self.simulate_motion = True
        retreat, feedback = self.submit_profile(undock=True)
        self.assertTrue(retreat.success, retreat.message)
        self.assertEqual(retreat.profile_id, "offset")
        self.assertTrue(all(item.profile_id == "offset" for item in feedback))
        explicit, _ = self.submit_profile("default", undock=True)
        self.assertTrue(explicit.success, explicit.message)
        remembered, _ = self.submit_profile(undock=True)
        self.assertEqual(remembered.profile_id, "offset")
        self.assertTrue(remembered.success, remembered.message)
        # Restore successful-dock history for the other shared-server tests.
        self.simulate_motion = False
        self.tag_x = 0.5
        self.tag_y = 0.0
        restored, _ = self.submit_profile("default", execute=True)
        self.assertTrue(restored.success, restored.message)

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

    def test_nonfinite_detection_confidence_cannot_acquire_profile(self):
        for margin in (float("inf"), float("nan")):
            with self.subTest(margin=margin):
                self.decision_margin = margin
                result, _ = self.submit_profile("default")
                self.assertFalse(result.success)
                self.assertEqual(result.error_code, FineAlign.Result.NO_STABLE_TAG)
                self.assertFalse(any(command.linear.x or command.linear.y or command.angular.z
                                     for command in self.commands))

    def test_invalid_tag_aborts_without_crashing_and_recovers(self):
        self.warm_up_inputs()
        self.invalid_tag_orientation = True
        # Publish enough invalid observations to replace the TF and reject the cache.
        self.warm_up_inputs(duration=0.3)
        goal = FineAlign.Goal()
        goal.execute = False
        sent = self.client.send_goal_async(goal)
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertEqual(result.result().status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(result.result().result.error_code, FineAlign.Result.NO_STABLE_TAG)
        self.assertFalse(any(command != Twist() for command in self.commands))

        self.invalid_tag_orientation = False
        self.warm_up_inputs()
        sent = self.client.send_goal_async(goal)
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(result.result().result.success)

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

    def test_execution_retries_after_initial_tag_acquisition_failure(self):
        # Ensure a target retained by another test cannot satisfy the first attempt.
        time.sleep(1.1)
        feedback_stages = []
        goal = FineAlign.Goal()
        goal.execute = True
        sent = self.client.send_goal_async(
            goal,
            feedback_callback=lambda message: feedback_stages.append(
                message.feedback.stage
            ),
        )
        self.assertTrue(self.spin_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        self.assertTrue(
            self.spin_until(
                lambda: FineAlign.Feedback.REACQUIRING in feedback_stages,
                timeout=2.0,
            ),
            f"feedback stages: {feedback_stages!r}",
        )
        self.assertTrue(
            self.spin_with_inputs_until(
                lambda: any(
                    command.linear.x > 0.0 and command.linear.y > 0.0
                    for command in self.commands
                )
            ),
            f"received commands after retry: {self.commands!r}",
        )
        cancel = handle.cancel_goal_async()
        self.assertTrue(self.spin_with_inputs_until(cancel.done))
        result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertFalse(result.result().result.success)

    def test_timed_undock_without_tags_or_odometry(self):
        self.publish_tags = False
        self.publish_odometry = False
        self.assertTrue(self.spin_with_inputs_until(lambda: len(self.commands) > 0))
        started = time.monotonic()
        result, feedback = self.submit_profile("box", undock=True)
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.undock_mode, "timed_reverse")
        self.assertGreaterEqual(result.elapsed_time, 0.6)
        self.assertLess(time.monotonic() - started, 2.0)
        self.assertAlmostEqual(result.distance_traveled, 0.06)
        self.assertTrue(any(f.commanded_speed < 0 for f in feedback))
        self.assertTrue(all(f.commanded_lateral_speed == 0 and f.commanded_yaw_speed == 0
                            for f in feedback))
        self.assertTrue(self.spin_with_inputs_until(lambda: self.commands[-1].linear.x == 0))

    def test_timed_undock_aborts_immediately_on_collision(self):
        self.publish_tags = False
        self.collision_stopped = True
        self.spin_with_inputs_until(lambda: False, timeout=0.2)
        started = time.monotonic()
        result, _ = self.submit_profile("box", undock=True)
        self.assertEqual(result.error_code, Undock.Result.COLLISION_STOPPED)
        self.assertLess(time.monotonic() - started, 0.6)
        self.assertEqual(result.distance_traveled, 0.0)

    def test_timed_undock_cancellation_and_navigation_stop_motion(self):
        self.publish_tags = False
        for cancel in (True, False):
            self.nav_active = False
            self.spin_with_inputs_until(lambda: False, timeout=0.15)
            sent = self.undock_client.send_goal_async(Undock.Goal(profile_id="box"))
            self.assertTrue(self.spin_with_inputs_until(sent.done))
            handle = sent.result()
            self.assertTrue(handle.accepted)
            result = handle.get_result_async()
            self.assertTrue(self.spin_with_inputs_until(
                lambda: bool(self.commands) and self.commands[-1].linear.x < 0))
            if cancel:
                canceled = handle.cancel_goal_async()
                self.assertTrue(self.spin_with_inputs_until(canceled.done))
            else:
                self.nav_active = True
            self.assertTrue(self.spin_with_inputs_until(result.done))
            wrapped = result.result()
            self.assertEqual(wrapped.result.error_code,
                Undock.Result.CANCELED if cancel else Undock.Result.NAVIGATION_ACTIVE)
            self.assertTrue(self.spin_with_inputs_until(lambda: self.commands[-1].linear.x == 0))

    def test_timed_undock_rejects_invalid_state(self):
        self.publish_tags = False
        self.manipulation_state = 0
        self.spin_with_inputs_until(lambda: False, timeout=0.2)
        result, _ = self.submit_profile("box", undock=True)
        self.assertEqual(result.error_code, Undock.Result.INVALID_STATE)

    def test_timed_undock_timeout_precedes_longer_reverse_duration(self):
        self.publish_tags = False
        self.publish_odometry = False
        self.warm_up_inputs(0.2)
        result, _ = self.submit_profile("timed_timeout", undock=True)
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, Undock.Result.UNDOCK_TIMEOUT)
        self.assertTrue(self.spin_with_inputs_until(lambda: self.commands[-1].linear.x == 0))

    def test_box_dock_uses_its_detection_source(self):
        self.tag_id = 180
        self.tag_frame = "tag180"
        result, _ = self.submit_profile("box", instance_id="tag:180")
        self.assertEqual(result.error_code, FineAlign.Result.NO_STABLE_TAG)
        self.detections = self.node.create_publisher(
            AprilTagDetectionArray, "/detections", rclpy.qos.qos_profile_sensor_data)
        result, _ = self.submit_profile("box", instance_id="tag:180")
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.profile_id, "box")

    def test_box_dock_rejects_another_box_types_approach(self):
        self.tag_id = 42
        self.tag_frame = "tag42"
        result, _ = self.submit_profile("box", instance_id="tag:42")
        self.assertEqual(result.error_code, FineAlign.Result.INVALID_PROFILE)

    def test_box_dock_requires_explicit_instance(self):
        result, _ = self.submit_profile("box")
        self.assertEqual(result.error_code, FineAlign.Result.INVALID_PROFILE)

    def test_z_box_profile_binds_two_instances_and_remembers_pick_target(self):
        self.detections = self.node.create_publisher(
            AprilTagDetectionArray, "/detections", rclpy.qos.qos_profile_sensor_data)
        self.tag_x = 0.5
        self.tag_y = 0.0
        for tag_id in (17, 42):
            self.publish_tags = True
            self.tag_id = tag_id
            self.tag_x = 0.6 if tag_id == 42 else 0.5
            self.tag_frame = f"tag{tag_id}"
            result, feedback = self.submit_profile(execute=True, instance_id=f"tag:{tag_id}")
            self.assertTrue(result.success, result.message)
            self.assertEqual(result.instance_id, f"tag:{tag_id}")
            self.assertTrue(all(f.instance_id == result.instance_id for f in feedback))
            self.assertTrue(all(f.profile_id == result.profile_id for f in feedback))
            self.publish_tags = False
            retreat, _ = self.submit_profile(undock=True)
            self.assertTrue(retreat.success, retreat.message)
            self.assertEqual(retreat.profile_id, "box_b" if tag_id == 42 else "box")
            self.assertEqual(retreat.instance_id, result.instance_id)

    def test_undock_commands_reverse_x_and_completes_from_tag(self):
        self.warm_up_inputs()
        feedback = []
        sent = self.undock_client.send_goal_async(
            Undock.Goal(),
            feedback_callback=lambda message: feedback.append(message.feedback),
        )
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        self.assertTrue(
            self.spin_with_inputs_until(
                lambda: any(command.linear.x < 0.0 for command in self.commands)
            ),
            f"received commands: {self.commands!r}",
        )
        reverse_commands = [command for command in self.commands if command.linear.x < 0.0]
        self.assertTrue(reverse_commands)
        self.assertTrue(
            all(
                abs(command.linear.y) < 1e-9 and abs(command.angular.z) < 1e-9
                for command in reverse_commands
            )
        )

        self.robot_x = -0.11
        result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        action_result = result.result().result
        self.assertTrue(action_result.success, action_result.message)
        self.assertEqual(action_result.error_code, Undock.Result.SUCCESS)
        self.assertGreaterEqual(action_result.distance_traveled, 0.1)
        self.assertTrue(any(item.stage == Undock.Feedback.MOVING for item in feedback))
        self.assertTrue(any(item.stage == Undock.Feedback.SETTLING for item in feedback))

    def test_undock_corrects_lateral_and_yaw_drift(self):
        self.warm_up_inputs()
        sent = self.undock_client.send_goal_async(Undock.Goal())
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        self.assertTrue(
            self.spin_with_inputs_until(
                lambda: any(command.linear.x < 0.0 for command in self.commands)
            )
        )

        self.robot_x = -0.02
        self.inject_drift = True
        self.assertTrue(
            self.spin_with_inputs_until(
                lambda: any(
                    command.linear.y < 0.0 and command.angular.z < 0.0
                    for command in self.commands
                )
            ),
            f"received commands: {self.commands!r}",
        )
        cancel = handle.cancel_goal_async()
        self.assertTrue(self.spin_with_inputs_until(cancel.done))
        result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))

    def test_undock_cancellation_stops_motion(self):
        self.warm_up_inputs()
        sent = self.undock_client.send_goal_async(Undock.Goal())
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        self.assertTrue(
            self.spin_with_inputs_until(
                lambda: any(command.linear.x < 0.0 for command in self.commands)
            )
        )
        cancel = handle.cancel_goal_async()
        self.assertTrue(self.spin_with_inputs_until(cancel.done))
        result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertEqual(result.result().result.error_code, Undock.Result.CANCELED)
        self.assertTrue(self.spin_with_inputs_until(lambda: bool(self.commands)))
        self.assertEqual(self.commands[-1], Twist())

    def test_undock_rejects_invalid_manipulation_state(self):
        self.manipulation_state = ManipulationState.RECOVERY_REQUIRED
        self.warm_up_inputs()
        sent = self.undock_client.send_goal_async(Undock.Goal())
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        result = sent.result().get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertEqual(result.result().result.error_code, Undock.Result.INVALID_STATE)

    def test_undock_aborts_when_nav2_is_active(self):
        self.nav_active = True
        self.warm_up_inputs()
        sent = self.undock_client.send_goal_async(Undock.Goal())
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        result = sent.result().get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertEqual(result.result().result.error_code, Undock.Result.NAVIGATION_ACTIVE)

    def test_undock_aborts_for_persistent_collision_stop(self):
        self.collision_stopped = True
        self.warm_up_inputs(duration=1.1)
        sent = self.undock_client.send_goal_async(Undock.Goal())
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        result = sent.result().get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertEqual(result.result().result.error_code, Undock.Result.COLLISION_STOPPED)

    def test_undock_requires_fresh_tag(self):
        self.warm_up_inputs()
        self.publish_tags = False
        time.sleep(0.6)
        sent = self.undock_client.send_goal_async(Undock.Goal())
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        result = sent.result().get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertEqual(result.result().result.error_code, Undock.Result.NO_STABLE_TAG)
        self.assertFalse(any(command != Twist() for command in self.commands))

    def test_docking_and_undocking_are_mutually_exclusive(self):
        self.warm_up_inputs()
        undock_sent = self.undock_client.send_goal_async(Undock.Goal())
        self.assertTrue(self.spin_with_inputs_until(undock_sent.done))
        undock_handle = undock_sent.result()
        self.assertTrue(undock_handle.accepted)
        fine_sent = self.client.send_goal_async(FineAlign.Goal())
        self.assertTrue(self.spin_with_inputs_until(fine_sent.done))
        self.assertFalse(fine_sent.result().accepted)
        cancel = undock_handle.cancel_goal_async()
        self.assertTrue(self.spin_with_inputs_until(cancel.done))
        result = undock_handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))

    def test_docking_ignores_odometry_pose_jumps(self):
        self.base_tf_x = 100.0
        self.odom_x = -100.0
        self.warm_up_inputs()
        goal = FineAlign.Goal()
        goal.execute = False
        sent = self.client.send_goal_async(goal)
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        result = sent.result().get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        outcome = result.result().result
        self.assertTrue(outcome.success, outcome.message)
        self.assertAlmostEqual(outcome.final_error.x, 0.6, delta=0.02)
        self.assertAlmostEqual(outcome.final_error.y, 0.2, delta=0.02)

    def test_docking_settles_without_odometry(self):
        self.tag_x = 0.5
        self.tag_y = 0.0
        self.warm_up_inputs()
        goal = FineAlign.Goal()
        goal.execute = True
        sent = self.client.send_goal_async(goal)
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        result = sent.result().get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertTrue(result.result().result.success, result.result().result.message)
        self.assertFalse(any(command != Twist() for command in self.commands))

    def test_tag_dropout_stops_motion_and_reacquires(self):
        self.warm_up_inputs()
        stages = []
        goal = FineAlign.Goal()
        goal.execute = True
        sent = self.client.send_goal_async(
            goal, feedback_callback=lambda message: stages.append(message.feedback.stage)
        )
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        self.assertTrue(self.spin_with_inputs_until(
            lambda: any(command.linear.x > 0.0 for command in self.commands)
        ))
        self.publish_tags = False
        self.assertTrue(self.spin_with_inputs_until(
            lambda: FineAlign.Feedback.REACQUIRING in stages
        ))
        self.assertTrue(self.spin_with_inputs_until(lambda: self.commands[-1] == Twist()))
        self.commands.clear()
        self.publish_tags = True
        self.assertTrue(self.spin_with_inputs_until(
            lambda: any(command.linear.x > 0.0 for command in self.commands)
        ))
        moving = [command for command in self.commands if command.linear.x > 0.0]
        self.assertTrue(all(
            abs((command.linear.x ** 2 + command.linear.y ** 2) ** 0.5 - 0.1) < 1e-6
            for command in moving
        ))
        cancel = handle.cancel_goal_async()
        self.assertTrue(self.spin_with_inputs_until(cancel.done))
        result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))

    def test_cached_tag_cannot_complete_settling(self):
        self.tag_x = 0.5
        self.tag_y = 0.0
        self.warm_up_inputs()
        self.publish_tags = False
        goal = FineAlign.Goal()
        goal.execute = True
        sent = self.client.send_goal_async(goal)
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        result = handle.get_result_async()
        # Repeated controller cycles must not turn a cached tag observation into success.
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            self.publish_inputs()
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertFalse(result.done())
        self.publish_tags = True
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertTrue(result.result().result.success, result.result().result.message)

    def test_error_between_old_restart_and_completion_thresholds_moves_and_finishes(self):
        # Fixture tolerance is 0.08 m: the previous restart threshold was 0.10 m.
        self.tag_x = 0.59
        self.tag_y = 0.0
        self.warm_up_inputs()
        goal = FineAlign.Goal()
        goal.execute = True
        sent = self.client.send_goal_async(goal)
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(handle.accepted)
        self.assertTrue(self.spin_with_inputs_until(
            lambda: any(command.linear.x > 0.0 for command in self.commands)
        ))
        self.tag_x = 0.55
        result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertTrue(result.result().result.success, result.result().result.message)
        self.assertTrue(self.spin_with_inputs_until(lambda: self.commands[-1] == Twist()))

    def test_simulated_motion_converges_with_tag_jitter_and_odom_drift(self):
        self.tag_x = 0.75
        self.tag_y = 0.12
        self.simulate_motion = True
        self.warm_up_inputs()
        goal = FineAlign.Goal()
        goal.execute = True
        sent = self.client.send_goal_async(goal)
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        result = sent.result().get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done, timeout=8.0))
        self.assertTrue(result.result().result.success, result.result().result.message)
        self.assertLessEqual(abs(self.tag_x - 0.5), 0.085)
        self.assertLessEqual(abs(self.tag_y), 0.085)
        moving = [command for command in self.commands if command.linear.x or command.linear.y]
        self.assertTrue(moving)
        self.assertTrue(all(
            abs((command.linear.x ** 2 + command.linear.y ** 2) ** 0.5 - 0.1) < 1e-6
            for command in moving
        ))

    def test_undock_stops_when_tag_is_lost(self):
        self.warm_up_inputs()
        sent = self.undock_client.send_goal_async(Undock.Goal())
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(self.spin_with_inputs_until(
            lambda: any(command.linear.x < 0.0 for command in self.commands)
        ))
        self.publish_tags = False
        result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertEqual(result.result().result.error_code, Undock.Result.NO_STABLE_TAG)
        self.assertTrue(self.spin_with_inputs_until(lambda: self.commands[-1] == Twist()))

    def test_undock_ignores_inaccurate_odometry(self):
        self.publish_odometry = True
        self.odom_x = 1000.0
        self.odom_linear_velocity = 5.0
        self.odom_angular_velocity = 5.0
        self.warm_up_inputs()
        sent = self.undock_client.send_goal_async(Undock.Goal())
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        handle = sent.result()
        self.assertTrue(self.spin_with_inputs_until(
            lambda: any(command.linear.x < 0.0 for command in self.commands)
        ))
        self.robot_x = -0.11
        result = handle.get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertTrue(result.result().result.success, result.result().result.message)

    def test_docking_completion_ignores_inaccurate_velocity(self):
        self.publish_odometry = True
        self.odom_linear_velocity = 5.0
        self.odom_angular_velocity = 5.0
        self.tag_x = 0.5
        self.tag_y = 0.0
        self.warm_up_inputs()
        goal = FineAlign.Goal()
        goal.execute = True
        sent = self.client.send_goal_async(goal)
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        result = sent.result().get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done))
        self.assertTrue(result.result().result.success, result.result().result.message)

    def test_simulated_tag_undocking_converges_without_odometry(self):
        self.simulate_motion = True
        self.warm_up_inputs()
        sent = self.undock_client.send_goal_async(Undock.Goal())
        self.assertTrue(self.spin_with_inputs_until(sent.done))
        result = sent.result().get_result_async()
        self.assertTrue(self.spin_with_inputs_until(result.done, timeout=8.0))
        self.assertTrue(result.result().result.success, result.result().result.message)
        self.assertGreater(self.tag_x, 1.12)
        self.assertTrue(any(command.linear.x < 0.0 for command in self.commands))


@launch_testing.post_shutdown_test()
class TestFineAlignServerShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
