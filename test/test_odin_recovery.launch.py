"""Regression coverage for delayed robot TF and local odometry epoch changes."""

import copy
import time
import unittest

import launch
import launch_testing.actions
from launch_ros.actions import Node
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def generate_test_description():
    nodes = []
    for name in ("delayed", "recovery"):
        nodes.append(Node(
            package="x2_navigation", executable="odin_localization_adapter", name=name,
            parameters=[{
                "map_alignment_confirmed": True, "correction_timeout_sec": 2.0,
                "base_frame": name + "_base", "tracking_frame": name + "_imu",
                "map_frame": name + "_map", "odom_frame": name + "_odom",
                "input_topic": "/test/" + name + "/input",
                "native_tf_topic": "/test/" + name + "/native",
            }],
            remappings=[("/odom", "/test/" + name + "/output"),
                        ("/odin1/localization_ready", "/test/" + name + "/ready"),
                        ("/odin1/localization_status", "/test/" + name + "/status")],
            output="screen",
        ))
    return launch.LaunchDescription(nodes + [launch_testing.actions.ReadyToTest()])


class TestOdinRecovery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_odin_recovery")
        self.outputs = {name: [] for name in ("delayed", "recovery")}
        self.readiness = {name: [] for name in self.outputs}
        self.subscriptions = []
        self.local = {}; self.native = {}
        for name in self.outputs:
            self.local[name] = self.node.create_publisher(Odometry, f"/test/{name}/input", 10)
            self.native[name] = self.node.create_publisher(TFMessage, f"/test/{name}/native", 10)
            self.subscriptions += [
                self.node.create_subscription(Odometry, f"/test/{name}/output", self.outputs[name].append, 100),
                self.node.create_subscription(Bool, f"/test/{name}/ready", self.readiness[name].append, 100),
            ]
        self.static = StaticTransformBroadcaster(self.node)
        self.dynamic = TransformBroadcaster(self.node)
        mount = self.mount("recovery", self.node.get_clock().now())
        self.static.sendTransform(mount)
        self.last_correction = {}
        self.spin_for(0.35)

    def tearDown(self):
        self.node.destroy_node()

    def mount(self, name, stamp):
        mount = TransformStamped()
        mount.header.frame_id = name + "_base"
        mount.child_frame_id = name + "_imu"
        mount.header.stamp = stamp.to_msg()
        mount.transform.translation.x = 0.5
        mount.transform.rotation.w = 1.0
        return mount

    def spin_for(self, seconds, callback=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if callback:
                callback()
            rclpy.spin_once(self.node, timeout_sec=0.004)

    def emit(self, name, x=1.0, correction=True, robot_tf=True, correction_x=0.0):
        now = self.node.get_clock().now()
        if name == "delayed" and robot_tf:
            self.dynamic.sendTransform(self.mount(name, now - Duration(seconds=0.06)))
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "imu"
        odom.header.stamp = (now - Duration(seconds=0.01 if name == "delayed" else 0.03)).to_msg()
        odom.pose.pose.position.x = x
        odom.pose.pose.orientation.w = 1.0
        self.local[name].publish(odom)
        if correction:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = "map"
            tf.transform.translation.x = correction_x
            tf.transform.rotation.w = 1.0
            self.last_correction[name] = copy.deepcopy(tf)
            self.native[name].publish(TFMessage(transforms=[tf]))
        return odom

    def is_ready(self, name):
        return bool(self.readiness[name]) and self.readiness[name][-1].data

    def test_delayed_dynamic_tf_is_buffered_and_tf_loss_invalidates_correction(self):
        self.spin_for(1.0, lambda: self.emit("delayed"))
        self.assertGreater(len(self.outputs["delayed"]), 20)
        self.assertTrue(self.is_ready("delayed"))
        self.assertAlmostEqual(self.outputs["delayed"][-1].pose.pose.position.x, 0.5)
        self.spin_for(0.45, lambda: self.emit("delayed", correction=False, robot_tf=False))
        self.assertFalse(self.is_ready("delayed"))
        old = copy.deepcopy(self.last_correction["delayed"])
        self.spin_for(0.2, lambda: (
            self.emit("delayed", correction=False),
            self.native["delayed"].publish(TFMessage(transforms=[old])),
        ))
        self.assertFalse(self.is_ready("delayed"))
        self.spin_for(0.3, lambda: self.emit("delayed"))
        self.assertTrue(self.is_ready("delayed"))

    def test_outage_and_pose_reset_require_post_reset_correction(self):
        self.spin_for(0.6, lambda: self.emit("recovery"))
        self.assertTrue(self.is_ready("recovery"))
        old = copy.deepcopy(self.last_correction["recovery"])
        self.spin_for(0.35)
        self.assertFalse(self.is_ready("recovery"))
        self.spin_for(0.15, lambda: self.emit("recovery", x=11.0, correction=False))
        self.assertAlmostEqual(self.outputs["recovery"][-1].pose.pose.position.x, 10.5)
        self.assertFalse(self.is_ready("recovery"))
        # Still fresh by the 2-second timeout, but from the old odometry epoch.
        # Test both an exact replay and a previously unseen pre-reset packet.
        newer_old = copy.deepcopy(old)
        newer_old.header.stamp = (rclpy.time.Time.from_msg(old.header.stamp) +
                                  Duration(seconds=0.01)).to_msg()
        self.spin_for(0.15, lambda: (
            self.emit("recovery", x=11.0, correction=False),
            self.native["recovery"].publish(TFMessage(transforms=[old, newer_old])),
        ))
        self.assertFalse(self.is_ready("recovery"))
        self.spin_for(0.3, lambda: self.emit("recovery", x=11.0, correction_x=10.0))
        self.assertTrue(self.is_ready("recovery"))
        self.assertAlmostEqual(self.outputs["recovery"][-1].twist.twist.linear.x, 0.0)

    def test_malformed_sample_cannot_hide_pose_reset(self):
        self.spin_for(0.4, lambda: self.emit("recovery"))
        self.assertTrue(self.is_ready("recovery"))
        bad = self.emit("recovery", correction=False)
        bad.pose.pose.orientation.w = 0.0
        self.local["recovery"].publish(bad)
        self.spin_for(0.04)
        self.spin_for(0.15, lambda: self.emit("recovery", x=21.0, correction=False))
        self.assertFalse(self.is_ready("recovery"))
        self.spin_for(0.2, lambda: self.emit("recovery", x=21.0, correction_x=20.0))
        self.assertTrue(self.is_ready("recovery"))
