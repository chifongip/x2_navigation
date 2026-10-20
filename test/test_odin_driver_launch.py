import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("odin_driver_launch", ROOT / "launch/odin_driver.launch.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def context_for(tmp_path, keys):
    path = tmp_path / "driver.yaml"
    path.write_text(yaml.safe_dump({"register_keys": keys}))
    context = LaunchContext()
    context.launch_configurations["config_file"] = str(path)
    return context, path


def test_only_official_sdk_node_receives_selected_file(tmp_path):
    context, path = context_for(tmp_path, {
        "custom_map_mode": 2, "senddepth": 0, "sendreprojection": 0, "sendoverlay": 0,
    })
    nodes = MODULE.driver_actions(context)
    assert len(nodes) == 1
    assert nodes[0].node_executable == "host_sdk_sample"
    parameters = evaluate_parameters(context, nodes[0]._Node__parameters)
    assert list(parameters) == [{"config_file": str(path)}]
    remappings = nodes[0]._Node__remappings
    assert len(remappings) == 1
    assert remappings[0][0][0].text == "/tf"
    assert remappings[0][1][0].text == "/odin1/tf"


@pytest.mark.parametrize("name", ["senddepth", "sendreprojection", "sendoverlay"])
def test_auxiliary_requests_fail_before_launch(tmp_path, name):
    context, _ = context_for(tmp_path, {name: 1})
    with pytest.raises(RuntimeError, match="unsupported.*localization-only"):
        MODULE.driver_actions(context)


@pytest.mark.parametrize("value", [2, "0", None])
def test_auxiliary_flags_are_validated(tmp_path, value):
    context, _ = context_for(tmp_path, {"senddepth": value})
    with pytest.raises(RuntimeError, match="must be 0 or 1"):
        MODULE.driver_actions(context)


def test_missing_or_malformed_config_fails_before_launch(tmp_path):
    context = LaunchContext()
    path = tmp_path / "driver.yaml"
    context.launch_configurations["config_file"] = str(path)
    with pytest.raises(FileNotFoundError):
        MODULE.driver_actions(context)
    path.write_text("register_keys: []")
    with pytest.raises(RuntimeError, match="register_keys mapping"):
        MODULE.driver_actions(context)
