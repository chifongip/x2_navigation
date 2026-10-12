"""Reject command routing that could loop or bypass arbitration."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


SPEC = importlib.util.spec_from_file_location(
    "navigation_launch", Path(__file__).parents[1] / "launch/navigation.launch.py")
NAVIGATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NAVIGATION)


def context(direct, **overrides):
    result = LaunchContext()
    result.launch_configurations.update({
        "direct_alignment": "true" if direct else "false",
        "nav_cmd_topic": "/cmd_vel_nav",
        "smoothed_nav_cmd_topic": "/cmd_vel_nav_smoothed",
        "checked_nav_cmd_topic": "/cmd_vel_nav_checked",
        "raw_cmd_topic": "/cmd_vel_raw",
        "velocity_topic": "/cmd_vel",
        **overrides,
    })
    return result


@pytest.mark.parametrize("direct", [False, True])
def test_distinct_default_topics_are_valid(direct):
    assert NAVIGATION.validate_command_topics(context(direct)) == []


@pytest.mark.parametrize("direct", [False, True])
@pytest.mark.parametrize("topic", ["nav_cmd_topic", "smoothed_nav_cmd_topic"])
def test_final_output_cannot_alias_navigation_inputs(direct, topic):
    with pytest.raises(RuntimeError, match="must be distinct"):
        NAVIGATION.validate_command_topics(context(direct, **{topic: "cmd_vel"}))


@pytest.mark.parametrize("direct,topic", [(True, "checked_nav_cmd_topic"), (False, "raw_cmd_topic")])
def test_intermediate_topic_cannot_alias_final_output(direct, topic):
    with pytest.raises(RuntimeError, match="must be distinct"):
        NAVIGATION.validate_command_topics(context(direct, **{topic: "/cmd_vel"}))


def test_relative_topics_are_resolved_with_launch_namespace():
    with pytest.raises(RuntimeError, match="must be distinct"):
        NAVIGATION.validate_command_topics(context(
            True, ros_namespace="/robot", checked_nav_cmd_topic="cmd_vel",
            velocity_topic="/robot/cmd_vel"))


def test_private_topics_are_rejected_and_unused_raw_topic_is_ignored():
    with pytest.raises(RuntimeError, match="not private"):
        NAVIGATION.validate_command_topics(context(True, checked_nav_cmd_topic="~/checked"))
    assert NAVIGATION.validate_command_topics(context(True, raw_cmd_topic="/cmd_vel")) == []
