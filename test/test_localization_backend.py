import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("navigation_launch", ROOT / "launch/navigation.launch.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize("backend,executable", [
    ("fast_lio", "fast_lio_odom_adapter"), ("odin", "odin_localization_adapter")
])
def test_backend_selects_exactly_one_adapter(backend, executable):
    context = LaunchContext()
    context.launch_configurations.update({
        "localization_backend": backend, "params_file": "nav.yaml",
        "odin_params_file": "odin.yaml", "use_sim_time": "false",
    })
    actions = MODULE.backend_actions(context)
    assert len(actions) == 1
    assert actions[0].node_executable == executable


def test_invalid_backend_fails():
    context = LaunchContext()
    context.launch_configurations["localization_backend"] = "unknown"
    with pytest.raises(RuntimeError):
        MODULE.backend_actions(context)


def test_launch_preserves_external_ownership_and_chest_obstacles():
    source = (ROOT / "launch/navigation.launch.py").read_text()
    assert 'default_value="fast_lio"' in source
    assert 'LaunchConfiguration("enable_velocity_bridge")' in source
    assert '"require_localization_ready"' in source
    for package in ("robot_state_publisher", "controller_manager", "odin_ros_driver", "open3d_loc"):
        assert f'package="{package}"' not in source
    assert "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud" in source
    assert "/odin1/cloud" not in source


@pytest.mark.parametrize("backend,autostart,gated,manager_autostart", [
    ("odin", "true", True, False),
    ("odin", "false", False, False),
    ("fast_lio", "true", False, True),
    ("fast_lio", "false", False, False),
])
def test_lifecycle_startup_selection(backend, autostart, gated, manager_autostart):
    context = LaunchContext()
    context.launch_configurations.update({
        "localization_backend": backend, "autostart": autostart, "use_sim_time": "false",
    })
    actions = MODULE.navigation_lifecycle_actions(context, ["map_server", "planner_server"])
    assert len(actions) == 1 + int(gated)
    assert actions[0].node_executable == "lifecycle_manager"
    from launch_ros.utilities import evaluate_parameters
    parameters = evaluate_parameters(context, actions[0]._Node__parameters)
    assert parameters[0]["autostart"] is manager_autostart
    if gated:
        assert actions[1].node_executable == "nav2_localization_startup_gate.py"
