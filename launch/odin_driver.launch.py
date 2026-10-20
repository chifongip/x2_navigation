"""X2 localization-only wrapper around the official Odin driver executable."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def driver_actions(context):
    config_file = Path(LaunchConfiguration("config_file").perform(context)).expanduser().resolve()
    with config_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get("register_keys"), dict):
        raise RuntimeError("Odin config must contain a register_keys mapping")
    keys = config["register_keys"]
    # Official auxiliary executables read a hard-coded YAML internally. Do not
    # launch them with a selected configuration they would silently ignore.
    for name in ("senddepth", "sendreprojection", "sendoverlay"):
        if name in keys and (type(keys[name]) not in (int, bool) or keys[name] not in (0, 1)):
            raise RuntimeError(f"{name} must be 0 or 1")
        if keys.get(name, 0):
            raise RuntimeError(
                f"{name} is unsupported by the X2 localization-only launcher; "
                "disable it in a separate startup config. Official auxiliary "
                "executables do not honor custom config_file settings."
            )
    return [Node(
        package="odin_ros_driver", executable="host_sdk_sample", name="host_sdk_sample",
        output="screen", parameters=[{"config_file": str(config_file)}],
        remappings=[("/tf", "/odin1/tf")],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", description="Odin startup YAML; reviewed copy of effective device settings."
        ),
        OpaqueFunction(function=driver_actions),
    ])
