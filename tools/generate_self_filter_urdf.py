#!/usr/bin/env python3
"""Generate the kinematic X2 collision-proxy URDF used by robot_self_filter."""

from pathlib import Path
import xml.etree.ElementTree as ET


PROXY_BOXES = {
    "pelvis": (
        ("-0.005442", "0.000000", "0.001304"),
        ("0.140500", "0.171762", "0.133113"),
    ),
    "torso_link": (
        ("-0.036056", "0.000000", "0.178088"),
        ("0.288489", "0.302424", "0.391167"),
    ),
    "left_hip_pitch_link": (
        ("-0.002075", "0.063753", "0.000000"),
        ("0.075850", "0.127505", "0.123642"),
    ),
    "left_hip_roll_link": (
        ("0.000575", "0.000002", "-0.049244"),
        ("0.128001", "0.107996", "0.196500"),
    ),
    "left_hip_yaw_link": (
        ("-0.005660", "0.000000", "-0.124803"),
        ("0.135319", "0.115993", "0.247319"),
    ),
    "left_knee_link": (
        ("0.002789", "-0.000225", "-0.125520"),
        ("0.120322", "0.106916", "0.354810"),
    ),
    "left_ankle_pitch_link": (
        ("-0.001150", "0.000000", "0.000001"),
        ("0.079900", "0.068600", "0.064016"),
    ),
    "left_ankle_roll_link": (
        ("0.038181", "0.000013", "-0.020685"),
        ("0.219556", "0.129043", "0.105370"),
    ),
    "right_hip_pitch_link": (
        ("-0.002470", "-0.063753", "0.000000"),
        ("0.075060", "0.127505", "0.123642"),
    ),
    "right_hip_roll_link": (
        ("0.000000", "-0.000002", "-0.049244"),
        ("0.128001", "0.107996", "0.196500"),
    ),
    "right_hip_yaw_link": (
        ("-0.005660", "0.000000", "-0.124803"),
        ("0.135319", "0.115993", "0.247319"),
    ),
    "right_knee_link": (
        ("0.002789", "0.000225", "-0.125520"),
        ("0.120322", "0.106916", "0.354810"),
    ),
    "right_ankle_pitch_link": (
        ("-0.001150", "0.000000", "0.000001"),
        ("0.079900", "0.068600", "0.064016"),
    ),
    "right_ankle_roll_link": (
        ("0.038181", "0.000013", "-0.020685"),
        ("0.219556", "0.129043", "0.105370"),
    ),
    "left_shoulder_pitch_link": (
        ("-0.001627", "0.047300", "0.000012"),
        ("0.069784", "0.094600", "0.090356"),
    ),
    "left_shoulder_roll_link": (
        ("0.000445", "-0.000001", "-0.043445"),
        ("0.104889", "0.085005", "0.172326"),
    ),
    "left_shoulder_yaw_link": (
        ("0.003108", "-0.000001", "-0.050180"),
        ("0.090311", "0.084000", "0.140361"),
    ),
    "left_elbow_link": (
        ("-0.013029", "0.002132", "-0.049056"),
        ("0.080923", "0.084666", "0.153153"),
    ),
    "left_wrist_yaw_link": (
        ("0.000070", "0.000977", "-0.048355"),
        ("0.080276", "0.087047", "0.102287"),
    ),
    "left_wrist_pitch_link": (
        ("-0.000250", "-0.001500", "-0.001246"),
        ("0.070500", "0.058000", "0.048492"),
    ),
    "left_wrist_roll_link": (
        ("0.006587", "-0.009362", "-0.082601"),
        ("0.097174", "0.071017", "0.198202"),
    ),
    "right_shoulder_pitch_link": (
        ("-0.001737", "-0.047300", "-0.000001"),
        ("0.069489", "0.094600", "0.090002"),
    ),
    "right_shoulder_roll_link": (
        ("0.000443", "0.000000", "-0.043415"),
        ("0.104889", "0.085005", "0.172270"),
    ),
    "right_shoulder_yaw_link": (
        ("0.003441", "0.000001", "-0.049025"),
        ("0.089644", "0.084000", "0.138542"),
    ),
    "right_elbow_link": (
        ("-0.011771", "-0.002132", "-0.048942"),
        ("0.080927", "0.084667", "0.153104"),
    ),
    "right_wrist_yaw_link": (
        ("0.002061", "-0.000976", "-0.048150"),
        ("0.080283", "0.087048", "0.102300"),
    ),
    "right_wrist_pitch_link": (
        ("-0.000250", "0.001500", "-0.001246"),
        ("0.070500", "0.058000", "0.048492"),
    ),
    "right_wrist_roll_link": (
        ("0.006717", "0.009219", "-0.082611"),
        ("0.097433", "0.071031", "0.198222"),
    ),
    "head_yaw_link": (
        ("-0.009625", "0.000048", "0.007950"),
        ("0.108249", "0.112595", "0.036301"),
    ),
    "head_pitch_link": (
        ("-0.001861", "0.000000", "0.000950"),
        ("0.164003", "0.142199", "0.169900"),
    ),
}


def add_box_collision(link, origin, size):
    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", xyz=" ".join(origin), rpy="0 0 0")
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", size=" ".join(size))


def add_hand_pad(root, side, tcp_y):
    pad_link = ET.SubElement(root, "link", name=f"{side}_hand_pad_link")
    visual = ET.SubElement(pad_link, "visual")
    visual_geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(visual_geometry, "box", size="0.08 0.012 0.10")
    material = ET.SubElement(visual, "material", name="x2_contact_pad")
    ET.SubElement(material, "color", rgba="0.15 0.15 0.15 1")
    add_box_collision(pad_link, ("0", "0", "0"), ("0.08", "0.012", "0.10"))

    pad_joint = ET.SubElement(
        root, "joint", name=f"{side}_hand_pad_joint", type="fixed"
    )
    ET.SubElement(pad_joint, "parent", link=f"{side}_wrist_roll_link")
    ET.SubElement(pad_joint, "child", link=f"{side}_hand_pad_link")
    ET.SubElement(pad_joint, "origin", xyz="0 0 -0.13", rpy="0 0 0")

    tcp_link = ET.SubElement(root, "link", name=f"{side}_hand_tcp_link")
    tcp_joint = ET.SubElement(
        root, "joint", name=f"{side}_hand_tcp_joint", type="fixed"
    )
    ET.SubElement(tcp_joint, "parent", link=f"{side}_hand_pad_link")
    ET.SubElement(tcp_joint, "child", link=f"{side}_hand_tcp_link")
    ET.SubElement(tcp_joint, "origin", xyz=f"0 {tcp_y} 0", rpy="0 0 0")


def main():
    package_dir = Path(__file__).resolve().parents[1]
    source = package_dir.parent / "x2_description" / "urdf" / "x2_ultra.urdf"
    output = package_dir / "config" / "x2_self_filter.urdf"

    root = ET.parse(source).getroot()
    root.attrib["name"] = "x2_self_filter_proxy"
    for element in list(root):
        if element.tag == "mujoco":
            root.remove(element)

    source_collision_links = {
        link.attrib["name"]
        for link in root.findall("link")
        if link.find("collision") is not None
    }
    if source_collision_links != set(PROXY_BOXES):
        raise RuntimeError("proxy boxes no longer match the X2 collision links")

    for link in root.findall("link"):
        proxy = PROXY_BOXES.get(link.attrib["name"])
        if proxy is None:
            continue
        for collision in link.findall("collision"):
            link.remove(collision)
        add_box_collision(link, *proxy)

    add_hand_pad(root, "left", "-0.006")
    add_hand_pad(root, "right", "0.006")
    root.insert(
        0,
        ET.Comment(
            " Generated from x2_description/urdf/x2_ultra.urdf by "
            "tools/generate_self_filter_urdf.py. It preserves the X2 "
            "kinematic and visual model while replacing collision meshes "
            "with local bounding boxes for robot_self_filter. "
        ),
    )
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
