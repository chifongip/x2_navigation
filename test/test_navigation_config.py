from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).parents[1]
CONFIG_FILE = PACKAGE_DIR / "config" / "nav2_params.yaml"
SELF_FILTER_CONFIG_FILE = PACKAGE_DIR / "config" / "self_filter.yaml"
GROUND_SEGMENTATION_CONFIG_FILE = PACKAGE_DIR / "config" / "ground_segmentation.yaml"
SELF_FILTER_DESCRIPTION_FILE = PACKAGE_DIR / "config" / "x2_self_filter.urdf"
X2_DESCRIPTION_FILE = PACKAGE_DIR.parent / "x2_description" / "urdf" / "x2_ultra.urdf"
MAP_FILE = PACKAGE_DIR / "map" / "2026-08-18-Lab_voxel_0_05m.yaml"
LAUNCH_FILE = PACKAGE_DIR / "launch" / "navigation.launch.py"
RVIZ_FILE = PACKAGE_DIR / "rviz" / "navigation.rviz"
PACKAGE_XML = PACKAGE_DIR / "package.xml"
CMAKE_FILE = PACKAGE_DIR / "CMakeLists.txt"


def test_navigation_configuration_uses_fast_lio_without_amcl():
    configuration = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))

    assert "amcl" not in configuration
    adapter = configuration["fast_lio_odom_adapter"]["ros__parameters"]
    assert adapter == {
        "input_topic": "/Odometry_loc",
        "output_topic": "/odom",
        "odom_frame": "odom",
        "tracking_frame": "lidar_imu_chest_front",
        "base_frame": "base_link",
        "tf_timeout_sec": 0.05,
    }


def test_navigation_velocity_footprint_and_costmap_settings():
    configuration = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    controller = configuration["controller_server"]["ros__parameters"]["FollowPath"]

    assert controller["min_vel_x"] == 0.0
    assert controller["max_vel_x"] == 0.5
    assert controller["min_vel_theta"] == -1.0
    assert controller["max_vel_theta"] == 1.0
    assert controller["max_speed_theta"] == 1.0

    local_costmap = configuration["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = configuration["global_costmap"]["global_costmap"]["ros__parameters"]
    expected_footprint = "[[0.15, 0.30], [0.15, -0.30], [-0.15, -0.30], [-0.15, 0.30]]"
    assert local_costmap["footprint"] == expected_footprint
    assert global_costmap["footprint"] == expected_footprint
    assert local_costmap["plugins"] == ["obstacle_layer", "inflation_layer"]
    assert global_costmap["plugins"] == [
        "static_layer",
        "obstacle_layer",
        "inflation_layer",
    ]

    obstacle_layer = local_costmap["obstacle_layer"]
    assert obstacle_layer["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert obstacle_layer["min_obstacle_height"] == -1.0
    assert obstacle_layer["max_obstacle_height"] == 2.0
    assert obstacle_layer["footprint_clearing_enabled"] is True
    assert obstacle_layer["observation_sources"] == "chest_cloud"
    assert obstacle_layer["chest_cloud"] == {
        "topic": "/scan_nav/payload_filtered_cloud",
        "sensor_frame": "lidar_chest_front",
        "data_type": "PointCloud2",
        "marking": True,
        "clearing": True,
        "min_obstacle_height": -1.0,
        "max_obstacle_height": 2.0,
        "obstacle_min_range": 0.20,
        "obstacle_max_range": 5.0,
        "raytrace_min_range": 0.20,
        "raytrace_max_range": 5.5,
        "observation_persistence": 1.0,
    }
    assert global_costmap["obstacle_layer"] == obstacle_layer
    assert local_costmap["inflation_layer"] == {
        "plugin": "nav2_costmap_2d::InflationLayer",
        "inflation_radius": 1.0,
        "cost_scaling_factor": 4.0,
    }
    assert global_costmap["inflation_layer"] == {
        "plugin": "nav2_costmap_2d::InflationLayer",
        "inflation_radius": 1.0,
        "cost_scaling_factor": 4.0,
    }

    assert local_costmap["width"] == 6
    assert type(local_costmap["width"]) is int
    assert local_costmap["height"] == 6
    assert type(local_costmap["height"]) is int

    payload_filter = configuration["payload_cloud_filter"]["ros__parameters"]
    assert payload_filter == {
        "min_x": 0.20,
        "max_x": 0.50,
        "min_y": -0.22,
        "max_y": 0.22,
        "min_z": 0.03,
        "max_z": 0.56,
    }


def test_navigation_velocity_zmq_bridge_settings():
    configuration = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    bridge = configuration["nav2_zmq_velocity_bridge"]["ros__parameters"]

    assert bridge == {
        "command_topic": "/cmd_vel",
        "zmq_endpoint": "tcp://*:8558",
        "publish_rate_hz": 20.0,
        "command_timeout_sec": 0.20,
    }


def test_navigation_filters_raw_lidar_for_pointcloud_costmap():
    configuration = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))

    throttle = configuration["lidar_cloud_throttle"]["ros__parameters"]
    assert throttle == {
        "input_topic": "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud",
        "output_topic": "/scan_nav/cloud",
        "max_rate_hz": 10.0,
        "timestamp_offset_sec": 0.0,
        "target_frame": "base_link",
        "tf_timeout_sec": 0.05,
        "voxel_size": 0.05,
        "min_height": -0.45,
        "max_height": 0.30,
        "max_input_points": 40000,
    }
    assert "lidar_to_scan" not in configuration

    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert 'package="x2_navigation"' in launch_source
    assert 'executable="lidar_cloud_throttle"' in launch_source
    assert 'package="ground_segmentation_ros2"' in launch_source
    assert 'executable="ground_segmentation_ros2_node"' in launch_source
    assert '"ground_segmentation.yaml"' in launch_source
    assert '("/ground_segmentation/input_pointcloud", "/scan_nav/cloud")' in launch_source
    assert '"/scan_nav/ground_filtered_cloud"' in launch_source
    assert 'package="robot_self_filter"' in launch_source
    assert 'executable="self_filter"' in launch_source
    assert '"self_filter.yaml"' in launch_source
    assert '"x2_self_filter.urdf"' in launch_source
    assert '"robot_description"' in launch_source
    assert '("cloud_out", "/scan_nav/self_filtered_cloud")' in launch_source
    assert '"input_topic": "/scan_nav/self_filtered_cloud"' in launch_source
    assert '"output_topic": "/scan_nav/payload_filtered_cloud"' in launch_source
    assert '"lidar_timestamp_offset_sec"' in launch_source
    assert 'default_value="0.0"' in launch_source
    assert '"timestamp_offset_sec": ParameterValue(' in launch_source
    assert 'package="pointcloud_to_laserscan"' in launch_source
    assert 'executable="pointcloud_to_laserscan_node"' in launch_source
    assert 'default_value="/scan_nav/laser"' in launch_source
    assert 'derived from /scan_nav/self_filtered_cloud' in launch_source
    assert '("cloud_in", "/scan_nav/payload_filtered_cloud")' in launch_source
    assert 'executable="payload_cloud_filter"' in launch_source
    assert '("scan", laser_scan_topic)' in launch_source
    assert '"laser_scan_range_min"' in launch_source
    assert '"laser_scan_range_max"' in launch_source


def test_navigation_ground_segmentation_removes_ground_before_self_filter():
    configuration = yaml.safe_load(
        GROUND_SEGMENTATION_CONFIG_FILE.read_text(encoding="utf-8")
    )
    segmentation = configuration["ground_segmentation"]["ros__parameters"]

    assert segmentation == {
        "robot_frame": "base_link",
        "maxX": 6.0,
        "minX": -6.0,
        "maxY": 6.0,
        "minY": -6.0,
        "maxZ": 0.30,
        "minZ": -0.45,
        "downsample": False,
        "downsample_resolution": 0.05,
        "lidar_to_ground": -0.45,
        "transform_tolerance": 0.05,
        "cellSizeX": 0.5,
        "cellSizeY": 0.5,
        "cellSizeZ": 1.0,
        "cellSizeZPhase2": 0.25,
        "slopeThresholdDegrees": 20.0,
        "groundInlierThreshold": 0.08,
        "centroidSearchRadius": 5.0,
        "maxGroundHeightDeviation": 0.15,
        "use_imu_orientation": False,
        "show_benchmark": False,
    }


def test_navigation_self_filter_uses_a_kinematic_x2_collision_proxy():
    configuration = yaml.safe_load(SELF_FILTER_CONFIG_FILE.read_text(encoding="utf-8"))
    self_filter = configuration["self_filter"]["ros__parameters"]

    assert self_filter["sensor_frame"] == "lidar_chest_front"
    assert self_filter["lidar_sensor_type"] == 0
    assert self_filter["in_pointcloud_topic"] == "/scan_nav/ground_filtered_cloud"
    assert self_filter["max_queue_size"] == 1
    assert self_filter["keep_organized"] is False
    assert self_filter["zero_for_removed_points"] is False
    assert self_filter["invert"] is False
    assert self_filter["min_sensor_dist"] == 0.05
    assert self_filter["default_box_scale"] == [1.25, 1.25, 1.25]
    assert self_filter["default_box_padding"] == [0.01, 0.01, 0.01]
    assert self_filter["default_sphere_scale"] == 1.25
    assert self_filter["default_sphere_padding"] == 0.0

    links = self_filter["self_see_links"]["names"]
    assert len(links) == len(set(links))
    assert set(links) == {
        "left_shoulder_pitch_link",
        "left_shoulder_roll_link",
        "left_shoulder_yaw_link",
        "left_elbow_link",
        "left_wrist_yaw_link",
        "left_wrist_pitch_link",
        "left_wrist_roll_link",
        "left_hand_pad_link",
        "right_shoulder_pitch_link",
        "right_shoulder_roll_link",
        "right_shoulder_yaw_link",
        "right_elbow_link",
        "right_wrist_yaw_link",
        "right_wrist_pitch_link",
        "right_wrist_roll_link",
        "right_hand_pad_link",
    }

    root = ET.parse(SELF_FILTER_DESCRIPTION_FILE).getroot()
    assert root.attrib["name"] == "x2_self_filter_proxy"

    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    source_root = ET.parse(X2_DESCRIPTION_FILE).getroot()
    source_joints = {
        joint.attrib["name"]: joint for joint in source_root.findall("joint")
    }
    assert source_joints.keys() <= joints.keys()
    for name, source_joint in source_joints.items():
        proxy_joint = joints[name]
        assert proxy_joint.attrib == source_joint.attrib
        for child_name in ("parent", "child", "origin", "axis", "limit"):
            source_child = source_joint.find(child_name)
            proxy_child = proxy_joint.find(child_name)
            assert (proxy_child is None) == (source_child is None)
            if source_child is not None:
                assert proxy_child.attrib == source_child.attrib

    assert joints["base_link_joint"].find("parent").attrib["link"] == "base_link"
    assert joints["base_link_joint"].find("child").attrib["link"] == "pelvis"
    assert joints["waist_yaw_joint"].find("parent").attrib["link"] == "pelvis"
    assert joints["waist_yaw_joint"].find("child").attrib["link"] == "waist_yaw_link"
    assert (
        joints["left_shoulder_pitch_joint"].find("parent").attrib["link"]
        == "torso_link"
    )
    assert (
        joints["left_shoulder_pitch_joint"].find("child").attrib["link"]
        == "left_shoulder_pitch_link"
    )
    assert (
        joints["right_shoulder_pitch_joint"].find("parent").attrib["link"]
        == "torso_link"
    )
    assert (
        joints["right_shoulder_pitch_joint"].find("child").attrib["link"]
        == "right_shoulder_pitch_link"
    )
    assert (
        joints["left_hand_pad_joint"].find("parent").attrib["link"]
        == "left_wrist_roll_link"
    )
    assert (
        joints["right_hand_pad_joint"].find("parent").attrib["link"]
        == "right_wrist_roll_link"
    )

    links_by_name = {link.attrib["name"]: link for link in root.findall("link")}
    assert links_by_name["pelvis"].find("visual/geometry/mesh") is not None
    assert links_by_name["torso_link"].find("visual/geometry/mesh") is not None
    assert (
        links_by_name["left_shoulder_pitch_link"].find("visual/geometry/mesh")
        is not None
    )
    assert (
        links_by_name["right_shoulder_pitch_link"].find("visual/geometry/mesh")
        is not None
    )

    collision_links = {
        name
        for name, link in links_by_name.items()
        if link.find("collision") is not None
    }
    assert set(links) < collision_links
    assert len(collision_links) == 32
    assert all(
        collision.find("geometry/box") is not None
        for link in root.findall("link")
        for collision in link.findall("collision")
    )


def test_navigation_uses_navfn_pluginlib_identifier():
    configuration = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    planner = configuration["planner_server"]["ros__parameters"]

    assert planner["planner_plugins"] == ["GridBased"]
    assert planner["GridBased"]["plugin"] == "nav2_navfn_planner/NavfnPlanner"


def test_navigation_launch_resolves_behavior_tree_xml_paths():
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'get_package_share_directory("nav2_bt_navigator")' in launch_source
    assert '"default_nav_to_pose_bt_xml"' in launch_source
    assert '"default_nav_through_poses_bt_xml"' in launch_source
    assert "navigate_to_pose_w_replanning_and_recovery.xml" in launch_source
    assert "navigate_through_poses_w_replanning_and_recovery.xml" in launch_source


def test_navigation_launch_consumes_existing_robot_state():
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'package="controller_manager"' not in launch_source
    assert 'package="robot_state_publisher"' not in launch_source
    assert 'package="x2_bringup"' not in launch_source


def test_fine_alignment_docking_and_collision_safety_configuration():
    configuration = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    assert "docking_server" not in configuration

    fine_align = configuration["fine_align_server"]["ros__parameters"]
    assert fine_align["standoff"] == 0.5
    assert fine_align["capture_distance"] == 1.5
    assert fine_align["capture_lateral"] == 1.5
    assert fine_align["capture_yaw"] == 0.785398163
    assert fine_align["stable_sample_count"] >= 3
    assert fine_align["acquisition_timeout"] >= 6.0
    assert fine_align["maximum_retries"] == 2
    assert fine_align["retry_delay"] == 1.0
    assert fine_align["controller_frequency"] == 20.0
    assert fine_align["progress_log_interval"] == 1.0
    assert fine_align["translation_gain"] == 0.5
    assert fine_align["yaw_gain"] == 1.0
    assert fine_align["translation_speed_min"] == 0.1
    assert fine_align["translation_speed_max"] == 0.2
    assert fine_align["angular_speed_min"] == 0.1
    assert fine_align["angular_speed_max"] == 0.2
    assert fine_align["translation_yaw_stop"] == 0.3490658504
    assert fine_align["x_position_tolerance"] == 0.08
    assert fine_align["y_position_tolerance"] == 0.08
    assert "position_tolerance" not in fine_align
    assert fine_align["yaw_tolerance"] == 0.0872664626
    assert fine_align["settled_sample_count"] >= 3
    assert fine_align["allow_reverse_x"] is True
    assert fine_align["reverse_capture_distance"] == 0.3
    assert "reacquisition_timeout" not in fine_align

    collision = configuration["collision_monitor"]["ros__parameters"]
    assert collision["cmd_vel_in_topic"] == "/cmd_vel_raw"
    assert collision["cmd_vel_out_topic"] == "/cmd_vel"
    assert collision["polygons"] == ["RobotFootprintStop"]
    footprint_stop = collision["RobotFootprintStop"]
    assert footprint_stop["action_type"] == "stop"
    assert footprint_stop["points"] == [
        0.15, 0.30, 0.15, -0.30, -0.15, -0.30, -0.15, 0.30,
    ]
    assert collision["chest_cloud"]["topic"] == "/scan_nav/payload_filtered_cloud"

    payload_filter = configuration["payload_cloud_filter"]["ros__parameters"]
    assert payload_filter["min_x"] > max(footprint_stop["points"][::2])
    assert payload_filter["max_x"] > max(footprint_stop["points"][::2])

    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert '"nav_cmd_topic"' in launch_source
    assert 'default_value="/cmd_vel_nav"' in launch_source
    assert '"dock_cmd_topic"' not in launch_source
    assert 'package="opennav_docking"' not in launch_source
    assert '"docking_server"' not in launch_source
    assert '"raw_cmd_topic"' in launch_source
    assert 'default_value="/cmd_vel_raw"' in launch_source
    assert launch_source.count('remappings=[("cmd_vel", nav_cmd_topic)]') == 2
    assert '"cmd_vel_in_topic": raw_cmd_topic' in launch_source
    assert '"cmd_vel_out_topic": velocity_topic' in launch_source
    assert 'executable="fine_align_server"' in launch_source
    assert 'executable="collision_monitor"' in launch_source


def test_fine_alignment_uses_latest_tag_transform_with_timestamp_coherence_checks():
    fine_align_source = (CONFIG_FILE.parents[1] / "src" / "fine_align_server.cpp").read_text(
        encoding="utf-8"
    )

    assert "fixed_frame_, tag_frame_, tf2::TimePointZero" in fine_align_source
    assert "tag_frame_, tf2::TimePointZero, tf2::durationFromSec" not in fine_align_source
    assert "stamp - transform_stamp" in fine_align_source
    assert "stable_target_stamp_ = stamp" in fine_align_source
    assert "stable_target_stamp_).seconds() > maximum_pose_age_" in fine_align_source


def test_fine_alignment_owns_holonomic_control_and_settling():
    fine_align_source = (CONFIG_FILE.parents[1] / "src" / "fine_align_server.cpp").read_text(
        encoding="utf-8"
    )

    assert "holonomicFineAlignCommand(error, controller_config_)" in fine_align_source
    assert "message->twist.twist.linear.y" in fine_align_source
    assert "settled_samples >= settled_sample_count_" in fine_align_source
    assert "DockRobot" not in fine_align_source


def test_fine_alignment_logs_error_and_velocity_command_during_execution():
    fine_align_source = (CONFIG_FILE.parents[1] / "src" / "fine_align_server.cpp").read_text(
        encoding="utf-8"
    )

    assert "progress_log_interval" in fine_align_source
    assert "Fine-align progress:" in fine_align_source
    assert "attempt=%zu/%zu" in fine_align_source
    assert "error_base=(x=%.3f m, y=%.3f m, yaw=%.3f rad)" in fine_align_source
    assert "command=(linear.x=%.3f m/s, linear.y=%.3f m/s, angular.z=%.3f rad/s)" in fine_align_source


def test_fine_alignment_retries_only_recoverable_failures_with_a_new_target():
    fine_align_source = (CONFIG_FILE.parents[1] / "src" / "fine_align_server.cpp").read_text(
        encoding="utf-8"
    )

    assert "retryableFailure" in fine_align_source
    assert "Fine-align retry:" in fine_align_source
    assert "FineAlign::Feedback::REACQUIRING" in fine_align_source
    assert "minimum_sequence = latestStableTargetSequence();" in fine_align_source


def test_fine_alignment_logs_abort_context_before_terminating():
    fine_align_source = (CONFIG_FILE.parents[1] / "src" / "fine_align_server.cpp").read_text(
        encoding="utf-8"
    )

    assert "Fine-align action abort:" in fine_align_source
    assert "stable_target_%s" in fine_align_source
    assert "odometry_%s" in fine_align_source
    assert fine_align_source.index("logAbortDiagnostic(handle, *result, code, message);") < (
        fine_align_source.index("handle->abort(result);")
    )


def test_navigation_runtime_dependencies_and_resources_are_packaged():
    package = ET.parse(PACKAGE_XML).getroot()
    exec_dependencies = {
        dependency.text for dependency in package.findall("exec_depend")
    }

    assert {
        "ament_index_python",
        "ground_segmentation_ros2",
        "nav2_costmap_2d",
        "pointcloud_to_laserscan",
        "robot_self_filter",
    } <= exec_dependencies

    cmake_source = CMAKE_FILE.read_text(encoding="utf-8")
    assert "install(DIRECTORY config launch map rviz DESTINATION share/${PROJECT_NAME})" in cmake_source


def test_installed_map_references_packaged_image():
    map_config = yaml.safe_load(MAP_FILE.read_text(encoding="utf-8"))

    assert map_config["resolution"] == 0.05
    assert (MAP_FILE.parent / map_config["image"]).is_file()


def test_rviz_uses_transient_local_qos_for_the_packaged_map():
    configuration = yaml.safe_load(RVIZ_FILE.read_text(encoding="utf-8"))
    displays = configuration["Visualization Manager"]["Displays"]
    displays_by_name = {display["Name"]: display for display in displays}

    map_display = displays_by_name["Map"]
    assert map_display["Topic"]["Value"] == "/map"
    assert map_display["Topic"]["Durability Policy"] == "Transient Local"
    assert map_display["Topic"]["Reliability Policy"] == "Reliable"

    robot_display = displays_by_name["Robot"]
    assert robot_display["Description Source"] == "Topic"
    assert robot_display["Description Topic"]["Value"] == "/robot_description"
    assert robot_display["Description Topic"]["Durability Policy"] == "Transient Local"

    cloud_display = displays_by_name["Navigation Cloud"]
    assert cloud_display["Class"] == "rviz_default_plugins/PointCloud2"
    assert cloud_display["Topic"]["Value"] == "/scan_nav/self_filtered_cloud"
    assert cloud_display["Topic"]["Reliability Policy"] == "Best Effort"
