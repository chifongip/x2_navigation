from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = Path(get_package_share_directory("x2_navigation"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=str(package / "config" / "odin_localization.yaml")
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        Node(
            package="x2_navigation", executable="odin_localization_adapter",
            name="odin_localization_adapter", output="screen",
            parameters=[LaunchConfiguration("params_file"), {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            }],
        ),
    ])
