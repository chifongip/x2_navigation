import time
import unittest

import launch
import launch_testing.actions
import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster


def generate_test_description():
    adapter = Node(
        package="x2_navigation", executable="odin_localization_adapter",
        parameters=[{"map_alignment_confirmed": True, "correction_timeout_sec": 0.3}],
        output="screen",
    )
    offset_adapter = Node(
        package="x2_navigation", executable="odin_localization_adapter", name="offset_adapter",
        parameters=[{"input_topic": "/test/future_odom", "sensor_time_offset_sec": -2.0,
                     "map_alignment_confirmed": True, "native_tf_topic": "/test/future_tf"}],
        remappings=[("/odom", "/test/corrected_odom"), ("/tf", "/test/corrected_tf"),
                    ("/odin1/localization_ready", "/test/corrected_ready"),
                    ("/odin1/localization_status", "/test/corrected_status")],
        output="screen",
    )
    missing_mount_adapter = Node(
        package="x2_navigation", executable="odin_localization_adapter", name="missing_mount_adapter",
        parameters=[{"tracking_frame": "unavailable_odin_mount", "map_alignment_confirmed": True}],
        remappings=[("/odom", "/test/missing_mount_odom"),
                    ("/odin1/localization_ready", "/test/missing_mount_ready"),
                    ("/odin1/localization_status", "/test/missing_mount_status")],
        output="screen",
    )
    bridge = Node(
        package="x2_navigation", executable="nav2_zmq_velocity_bridge",
        parameters=[{"require_localization_ready": True,
                     "zmq_endpoint": "tcp://127.0.0.1:18691",
                     "command_timeout_sec": 5.0}], output="screen",
    )
    heartbeat_bridge = Node(
        package="x2_navigation", executable="nav2_zmq_velocity_bridge", name="heartbeat_bridge",
        parameters=[{"require_localization_ready": True,
                     "localization_ready_topic": "/test/heartbeat",
                     "command_topic": "/test/heartbeat_cmd",
                     "final_command_topic": "/test/heartbeat_final",
                     "zmq_endpoint": "tcp://127.0.0.1:18692",
                     "command_timeout_sec": 5.0}], output="screen",
    )
    return launch.LaunchDescription([
        adapter, offset_adapter, missing_mount_adapter, bridge, heartbeat_bridge,
        launch_testing.actions.ReadyToTest(),
    ])


class TestOdinLocalization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_odin_localization")
        self.odom = []; self.ready = []; self.status = []; self.tf = []; self.final = []
        self.corrected = []; self.heartbeat_final = []
        self.missing_mount = []; self.missing_ready = []
        self.subs = [
            self.node.create_subscription(Odometry, "/odom", self.odom.append, 50),
            self.node.create_subscription(Bool, "/odin1/localization_ready", self.ready.append, 50),
            self.node.create_subscription(String, "/odin1/localization_status", self.status.append, 50),
            self.node.create_subscription(TFMessage, "/tf", self.tf.append, qos_profile_sensor_data),
            self.node.create_subscription(TwistStamped, "/navigation/final_command", self.final.append, 50),
            self.node.create_subscription(Odometry, "/test/corrected_odom", self.corrected.append, 50),
            self.node.create_subscription(TwistStamped, "/test/heartbeat_final", self.heartbeat_final.append, 50),
            self.node.create_subscription(Odometry, "/test/missing_mount_odom", self.missing_mount.append, 10),
            self.node.create_subscription(Bool, "/test/missing_mount_ready", self.missing_ready.append, 10),
        ]
        self.pub = self.node.create_publisher(Odometry, "/odin1/odometry_highfreq", 10)
        self.native = self.node.create_publisher(TFMessage, "/odin1/tf", 10)
        self.future = self.node.create_publisher(Odometry, "/test/future_odom", 10)
        self.command = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.heartbeat = self.node.create_publisher(Bool, "/test/heartbeat", 10)
        self.heartbeat_cmd = self.node.create_publisher(Twist, "/test/heartbeat_cmd", 10)
        self.static = StaticTransformBroadcaster(self.node)
        mount = TransformStamped()
        mount.header.frame_id = "base_link"; mount.child_frame_id = "odin_imu"
        mount.transform.translation.x = 0.5; mount.transform.rotation.w = 1.0
        self.static.sendTransform(mount)
        self.start = time.monotonic()

    def tearDown(self):
        self.node.destroy_node()

    def spin(self, duration, callback=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if callback:
                callback()
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def emit(self, correction=True, correction_x=2.0, future_sec=0.0, child="imu", invalid=False,
             position_offset=0.0):
        msg = Odometry()
        msg.header.stamp = (self.node.get_clock().now() +
                            rclpy.duration.Duration(seconds=future_sec - 0.03)).to_msg()
        msg.header.frame_id = "odom"; msg.child_frame_id = child
        msg.pose.pose.position.x = 2.0 + 0.2 * (time.monotonic() - self.start) + position_offset
        msg.pose.pose.orientation.w = 0.0 if invalid else 1.0
        # Deliberately nonsensical native twist must not affect output velocity.
        msg.twist.twist.linear.x = 999.0
        self.pub.publish(msg)
        if correction:
            tf = TransformStamped()
            tf.header = msg.header; tf.child_frame_id = "map"
            tf.transform.translation.x = correction_x; tf.transform.rotation.w = 1.0
            self.native.publish(TFMessage(transforms=[tf]))
        return msg

    def test_localization_and_fail_closed_command_transport(self):
        self.spin(0.5)
        cmd = Twist(); cmd.linear.x = 0.2
        self.spin(0.3, lambda: self.command.publish(cmd))
        self.assertTrue(self.final)
        self.assertEqual(self.final[-1].twist.linear.x, 0.0)
        self.spin(1.0, lambda: self.emit())
        self.assertTrue(self.ready[-1].data)
        self.assertAlmostEqual(self.odom[-1].twist.twist.linear.x, 0.2, delta=0.015)
        self.assertGreater(self.odom[-1].pose.pose.position.x, 1.5)
        transforms = [t for m in self.tf for t in m.transforms]
        corrections = [t for t in transforms if t.header.frame_id == "map" and t.child_frame_id == "odom"]
        self.assertTrue(corrections)
        self.assertAlmostEqual(corrections[-1].transform.translation.x, -2.0)
        self.assertFalse(any(t.child_frame_id == "imu" for t in transforms))
        self.assertFalse(self.missing_mount)
        self.assertTrue(self.missing_ready)
        self.assertFalse(any(m.data for m in self.missing_ready))
        self.spin(0.3, lambda: (self.emit(), self.command.publish(cmd)))
        self.assertAlmostEqual(self.final[-1].twist.linear.x, 0.2)
        self.spin(0.2, lambda: self.emit(correction_x=20.0))
        self.assertAlmostEqual(self.odom[-1].twist.twist.linear.x, 0.2, delta=0.015)
        self.spin(0.6, lambda: (self.emit(correction=False), self.command.publish(cmd)))
        self.assertFalse(self.ready[-1].data)
        self.assertEqual(self.final[-1].twist.linear.x, 0.0)
        self.spin(0.4, lambda: self.emit())
        self.assertTrue(self.ready[-1].data)
        self.assertEqual(self.final[-1].twist.linear.x, 0.0)  # no buffered command replay
        self.spin(0.2, lambda: (self.emit(), self.command.publish(cmd)))
        self.assertAlmostEqual(self.final[-1].twist.linear.x, 0.2)
        self.spin(0.4, lambda: self.emit(future_sec=2.0))
        self.assertFalse(self.ready[-1].data)
        self.assertEqual(self.final[-1].twist.linear.x, 0.0)
        self.spin(0.2, lambda: self.emit(invalid=True))
        self.assertFalse(self.ready[-1].data)
        self.spin(0.2, lambda: self.emit(child="unexpected"))
        self.assertFalse(self.ready[-1].data)
        self.spin(0.4, lambda: self.emit())
        self.assertTrue(self.ready[-1].data)
        # Repeated stamps must not produce repeated odometry or differentiate dt=0.
        duplicate = self.emit(correction=False)
        self.spin(0.04)
        count = len(self.odom)
        self.spin(0.06, lambda: self.pub.publish(duplicate))
        self.assertEqual(len(self.odom), count)
        # A backward local stamp invalidates correction readiness until a new
        # correction arrives, even while continuous local input resumes.
        duplicate.header.stamp = (self.node.get_clock().now() -
                                  rclpy.duration.Duration(seconds=0.18)).to_msg()
        self.pub.publish(duplicate)
        self.spin(0.04)
        self.spin(0.15, lambda: self.emit(correction=False))
        self.assertFalse(self.ready[-1].data)
        self.spin(0.2, lambda: self.emit())
        self.assertTrue(self.ready[-1].data)
        self.emit(correction=False, position_offset=10.0)
        self.spin(0.04)
        self.spin(0.15, lambda: self.emit(correction=False))
        self.assertFalse(self.ready[-1].data)
        self.spin(0.2, lambda: self.emit())
        self.assertTrue(self.ready[-1].data)
        # Explicit offset corrects future local input without receive-time stamping.
        def future_input():
            msg = self.emit(future_sec=2.0, correction=False)
            self.future.publish(msg)
        self.spin(0.4, future_input)
        self.assertTrue(self.corrected)
        corrected_age = (self.node.get_clock().now() - rclpy.time.Time.from_msg(
            self.corrected[-1].header.stamp)).nanoseconds / 1e9
        self.assertLess(abs(corrected_age), 0.2)
        self.spin(0.4)
        self.assertFalse(self.ready[-1].data)

    def test_readiness_heartbeat_timeout_and_recovery(self):
        cmd = Twist(); cmd.linear.x = 0.3
        ready = Bool(); ready.data = True
        self.spin(0.4, lambda: (self.heartbeat.publish(ready), self.heartbeat_cmd.publish(cmd)))
        self.assertAlmostEqual(self.heartbeat_final[-1].twist.linear.x, 0.3)
        self.spin(0.5, lambda: self.heartbeat_cmd.publish(cmd))
        self.assertEqual(self.heartbeat_final[-1].twist.linear.x, 0.0)
        self.spin(0.3, lambda: self.heartbeat.publish(ready))
        self.assertEqual(self.heartbeat_final[-1].twist.linear.x, 0.0)
        self.spin(0.2, lambda: (self.heartbeat.publish(ready), self.heartbeat_cmd.publish(cmd)))
        self.assertAlmostEqual(self.heartbeat_final[-1].twist.linear.x, 0.3)
