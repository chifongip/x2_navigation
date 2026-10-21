"""Synthetic sensor/joint input for the isolated Odin/Nav2 launch test only."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    rclpy.init()
    node = rclpy.create_node("synthetic_odin_navigation_input")
    description = Path(get_package_share_directory("x2_description")) / "urdf/x2_ultra.urdf"
    names = [joint.attrib["name"] for joint in ET.parse(description).getroot().findall("joint")
             if joint.attrib["type"] != "fixed" and joint.find("mimic") is None]
    joints = node.create_publisher(JointState, "/joint_states", 10)
    local = node.create_publisher(Odometry, "/odin1/odometry_highfreq", 10)
    correction = node.create_publisher(TFMessage, "/odin1/tf", 10)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    count = 0
    corrections_enabled = not node.declare_parameter("wait_for_corrections", False).value

    def enable_corrections(message):
        nonlocal corrections_enabled
        corrections_enabled = message.data

    enable_sub = node.create_subscription(
        Bool, "/test/enable_odin_corrections", enable_corrections, 10)


    def publish():
        nonlocal count
        now = node.get_clock().now()
        state = JointState()
        state.header.stamp = now.to_msg()
        state.name = names
        state.position = [0.0] * len(names)
        joints.publish(state)
        stamp = now - Duration(seconds=0.08)
        try:
            mount = buffer.lookup_transform("base_link", "odin_imu", stamp)
        except TransformException:
            return
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "imu"
        odom.pose.pose.position.x = mount.transform.translation.x
        odom.pose.pose.position.y = mount.transform.translation.y
        odom.pose.pose.position.z = mount.transform.translation.z
        odom.pose.pose.orientation = mount.transform.rotation
        local.publish(odom)
        count += 1
        if corrections_enabled and count % 10 == 0:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = "map"
            tf.transform.rotation.w = 1.0
            correction.publish(TFMessage(transforms=[tf]))

    timer = node.create_timer(0.01, publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(timer)
        listener.unregister()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
