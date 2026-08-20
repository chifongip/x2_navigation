from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_path = Path(get_package_share_directory("x2_navigation"))
    nav2_bt_navigator_path = Path(get_package_share_directory("nav2_bt_navigator"))
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    velocity_topic = LaunchConfiguration("velocity_topic")
    velocity_zmq_endpoint = LaunchConfiguration("velocity_zmq_endpoint")
    velocity_publish_rate_hz = LaunchConfiguration("velocity_publish_rate_hz")
    velocity_command_timeout_sec = LaunchConfiguration("velocity_command_timeout_sec")
    lidar_pointcloud_topic = LaunchConfiguration("lidar_pointcloud_topic")
    lidar_timestamp_offset_sec = LaunchConfiguration("lidar_timestamp_offset_sec")
    default_nav_to_pose_bt_xml = LaunchConfiguration("default_nav_to_pose_bt_xml")
    default_nav_through_poses_bt_xml = LaunchConfiguration(
        "default_nav_through_poses_bt_xml"
    )

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,
    )

    lifecycle_nodes = [
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=str(
                    package_path / "map" / "2026-08-18-Lab_voxel_0_05m.yaml"
                ),
                description="Absolute path to the Nav2 occupancy-map YAML file.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_path / "config" / "nav2_params.yaml"),
                description="Absolute path to the Nav2 parameters YAML file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "velocity_topic",
                default_value="/cmd_vel",
                description="Nav2 Twist topic forwarded to RoboJuDo over ZMQ.",
            ),
            DeclareLaunchArgument(
                "velocity_zmq_endpoint",
                default_value="tcp://*:8558",
                description="ZMQ PUB endpoint for RoboJuDo VelocityZmqCtrl.",
            ),
            DeclareLaunchArgument(
                "velocity_publish_rate_hz",
                default_value="20.0",
                description="Velocity ZMQ publication rate in Hz.",
            ),
            DeclareLaunchArgument(
                "velocity_command_timeout_sec",
                default_value="0.20",
                description="Age at which a Nav2 command is replaced with zero velocity.",
            ),
            DeclareLaunchArgument(
                "lidar_pointcloud_topic",
                default_value="/aima/hal/sensor/lidar_chest_front/lidar_pointcloud",
                description="Raw chest LiDAR point cloud converted into /scan_nav.",
            ),
            DeclareLaunchArgument(
                "lidar_timestamp_offset_sec",
                default_value="0.0",
                description=(
                    "Seconds added to raw-cloud stamps before conversion; use only "
                    "for a measured LiDAR-to-TF timing offset."
                ),
            ),
            DeclareLaunchArgument(
                "default_nav_to_pose_bt_xml",
                default_value=str(
                    nav2_bt_navigator_path
                    / "behavior_trees"
                    / "navigate_to_pose_w_replanning_and_recovery.xml"
                ),
                description="Behavior-tree XML used by NavigateToPose.",
            ),
            DeclareLaunchArgument(
                "default_nav_through_poses_bt_xml",
                default_value=str(
                    nav2_bt_navigator_path
                    / "behavior_trees"
                    / "navigate_through_poses_w_replanning_and_recovery.xml"
                ),
                description="Behavior-tree XML used by NavigateThroughPoses.",
            ),
            Node(
                package="x2_navigation",
                executable="fast_lio_odom_adapter",
                name="fast_lio_odom_adapter",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="x2_navigation",
                executable="nav2_zmq_velocity_bridge",
                name="nav2_zmq_velocity_bridge",
                output="screen",
                parameters=[
                    configured_params,
                    {
                        "command_topic": velocity_topic,
                        "zmq_endpoint": velocity_zmq_endpoint,
                        "publish_rate_hz": ParameterValue(
                            velocity_publish_rate_hz, value_type=float
                        ),
                        "command_timeout_sec": ParameterValue(
                            velocity_command_timeout_sec, value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="x2_navigation",
                executable="lidar_cloud_throttle",
                name="lidar_cloud_throttle",
                output="screen",
                parameters=[
                    configured_params,
                    {
                        "input_topic": lidar_pointcloud_topic,
                        "timestamp_offset_sec": ParameterValue(
                            lidar_timestamp_offset_sec, value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="lidar_to_scan",
                output="screen",
                parameters=[configured_params],
                remappings=[
                    ("cloud_in", "/scan_nav/cloud"),
                    ("scan", "/scan_nav"),
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[configured_params, {"yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    configured_params,
                    {
                        "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml,
                        "default_nav_through_poses_bt_xml": default_nav_through_poses_bt_xml,
                    },
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "autostart": ParameterValue(autostart, value_type=bool),
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(package_path / "rviz" / "navigation.rviz")],
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
