from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).parents[1]
CONFIG_FILE = PACKAGE_DIR / "config" / "nav2_params.yaml"
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
    assert controller["min_vel_theta"] == -0.5
    assert controller["max_vel_theta"] == 0.5

    local_costmap = configuration["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = configuration["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local_costmap["robot_radius"] == 0.30
    assert global_costmap["robot_radius"] == 0.30
    assert local_costmap["plugins"] == ["obstacle_layer", "inflation_layer"]
    assert global_costmap["plugins"] == ["static_layer", "inflation_layer"]
    assert "obstacle_layer" not in global_costmap

    obstacle_layer = local_costmap["obstacle_layer"]
    assert obstacle_layer["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert obstacle_layer["observation_sources"] == "chest_scan"
    assert obstacle_layer["chest_scan"] == {
        "topic": "/scan_nav",
        "data_type": "LaserScan",
        "marking": True,
        "clearing": True,
        "inf_is_valid": True,
        "min_obstacle_height": -1.0,
        "max_obstacle_height": 2.0,
        "obstacle_min_range": 0.20,
        "obstacle_max_range": 5.0,
        "raytrace_min_range": 0.20,
        "raytrace_max_range": 5.5,
    }

    assert local_costmap["width"] == 6
    assert type(local_costmap["width"]) is int
    assert local_costmap["height"] == 6
    assert type(local_costmap["height"]) is int


def test_navigation_velocity_zmq_bridge_settings():
    configuration = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    bridge = configuration["nav2_zmq_velocity_bridge"]["ros__parameters"]

    assert bridge == {
        "command_topic": "/cmd_vel",
        "zmq_endpoint": "tcp://*:8558",
        "publish_rate_hz": 20.0,
        "command_timeout_sec": 0.20,
    }


def test_navigation_converts_throttled_raw_lidar_to_laserscan():
    configuration = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))

    throttle = configuration["lidar_cloud_throttle"]["ros__parameters"]
    assert throttle == {
        "input_topic": "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud",
        "output_topic": "/scan_nav/cloud",
        "max_rate_hz": 5.0,
        "timestamp_offset_sec": 0.0,
    }

    converter = configuration["lidar_to_scan"]["ros__parameters"]
    assert converter == {
        "target_frame": "base_link",
        "transform_tolerance": 5.0,
        "min_height": -0.45,
        "max_height": 0.30,
        "angle_min": -3.141592653589793,
        "angle_max": 3.141592653589793,
        "angle_increment": 0.008726646259971648,
        "scan_time": 0.20,
        "range_min": 0.20,
        "range_max": 5.0,
        "use_inf": True,
        "inf_epsilon": 0.0,
        "queue_size": 1,
    }

    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert 'package="x2_navigation"' in launch_source
    assert 'executable="lidar_cloud_throttle"' in launch_source
    assert '"lidar_timestamp_offset_sec"' in launch_source
    assert 'default_value="0.0"' in launch_source
    assert '"timestamp_offset_sec": ParameterValue(' in launch_source
    assert 'package="pointcloud_to_laserscan"' in launch_source
    assert 'executable="pointcloud_to_laserscan_node"' in launch_source
    assert '"cloud_in", "/scan_nav/cloud"' in launch_source
    assert '"scan", "/scan_nav"' in launch_source


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


def test_navigation_runtime_dependencies_and_resources_are_packaged():
    package = ET.parse(PACKAGE_XML).getroot()
    exec_dependencies = {
        dependency.text for dependency in package.findall("exec_depend")
    }

    assert {"ament_index_python", "nav2_costmap_2d"} <= exec_dependencies

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

    scan_display = displays_by_name["Navigation Scan"]
    assert scan_display["Topic"]["Value"] == "/scan_nav"
    assert scan_display["Topic"]["Reliability Policy"] == "Best Effort"
