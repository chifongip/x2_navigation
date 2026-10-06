"""Check startup parameter loading without sending motion goals."""

import os
from pathlib import Path
import signal
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest
import yaml


def server_command(tmp_path, parameters):
    config = tmp_path / "profiles.yaml"
    config.write_text(
        yaml.safe_dump({"fine_align_server": {"ros__parameters": parameters}}),
        encoding="utf-8",
    )
    executable = (
        Path(get_package_prefix("x2_navigation"))
        / "lib/x2_navigation/fine_align_server"
    )
    environment = dict(os.environ, ROS_LOG_DIR=str(tmp_path / "ros_logs"))
    return [str(executable), "--ros-args", "--params-file", str(config)], environment


def named_parameters():
    return {
        "docking_profile_names": ["station"],
        "docking_profiles.station.tag_id": 10,
        "docking_profiles.station.tag_frame": "tag10",
        "docking_profiles.station.standoff": 0.6,
        "docking_profiles.station.lateral_offset": -0.1,
        "docking_profiles.station.yaw_offset": 0.2,
    }


@pytest.mark.parametrize(
    "change, error",
    [
        ({"default_docking_profile": "missing"}, "unknown docking profile"),
        ({"docking_profile_names": ["station", "station"]}, "duplicate docking profile"),
        ({"docking_profile_names": ["default"]}, "duplicate docking profile"),
        ({"docking_profile_names": ["bad.name"]}, "invalid docking profile name"),
        ({"docking_profiles.station.tag_id": -1}, "invalid docking profile"),
        ({"docking_profiles.station.tag_frame": ""}, "invalid docking profile"),
        ({"docking_profiles.station.standoff": 0.0}, "invalid docking profile"),
        ({"docking_profiles.station.yaw_offset": "wrong_type"}, "invalid type"),
        ({"docking_profiles.station.standoff": None}, "missing docking profile parameter"),
    ],
)
def test_invalid_profiles_fail_at_startup(tmp_path, change, error):
    parameters = named_parameters()
    parameters.update(change)
    # A missing field must be omitted rather than serialized as YAML null.
    parameters = {key: value for key, value in parameters.items() if value is not None}
    command, environment = server_command(tmp_path, parameters)
    result = subprocess.run(
        command, env=environment, capture_output=True, text=True, timeout=5,
    )
    assert result.returncode != 0
    assert error in result.stdout + result.stderr


@pytest.mark.parametrize("named_default", [False, True])
def test_valid_default_configuration_starts(tmp_path, named_default):
    parameters = named_parameters() if named_default else {"standoff": 0.5}
    if named_default:
        parameters["default_docking_profile"] = "station"
    command, environment = server_command(tmp_path, parameters)
    process = subprocess.Popen(
        command, env=environment, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True,
    )
    try:
        with pytest.raises(subprocess.TimeoutExpired):
            process.communicate(timeout=1.0)
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=5)
        assert process.returncode == 0, output
        expected_frame = "tag10" if named_default else "tag9"
        assert f"base_link <- {expected_frame}" in output
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
