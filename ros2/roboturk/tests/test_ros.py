"""Runs in a built/sourced ROS workspace. Skipped on the standalone macOS core environment."""

# ruff: noqa: E402 -- generated ROS imports must follow importorskip on non-ROS hosts.
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("roboturk_interfaces.msg")

from roboturk.backend import ReferenceBackend
from roboturk.model import Command, Config, Referee, RobotKey
from roboturk_interfaces.msg import CommandArray, RefereeState
from roboturk_interfaces.srv import ResetEpisode
from roboturk_ros.bridge import Bridge, formation
from roboturk_ros.conversions import command_message, from_world_message, set_stamp, world_message


def test_ros_world_roundtrip():
    w = formation(12)
    assert from_world_message(world_message(w)) == w


def test_ros_watchdogs_and_reset():
    class Supervisor:
        def getTime(self):
            return 1.0

    rclpy.init()
    backend = ReferenceBackend()
    node = Bridge(Supervisor(), backend, Config())
    try:
        referee = RefereeState(command="running", attack_sign=1)
        set_stamp(referee.header.stamp, 1)
        node.on_referee(referee)
        message = CommandArray()
        set_stamp(message.header.stamp, 1)
        message.header.frame_id = "robot_body"
        key = RobotKey("blue", 1)
        message.commands.append(command_message(key, Command(1, 0)))
        node.on_commands(message, "blue")
        node.step()
        assert backend.world.robot(key).vx == 1
        node.command_received["blue"] -= 1
        node.step()
        assert backend.world.robot(key).vx == 0
        node.referee_received -= 1
        node.step()
        assert backend.world.referee == Referee.HALT
        response = node.reset_episode(ResetEpisode.Request(seed=5), ResetEpisode.Response())
        assert response.success and not node.commands
        assert response.world.referee == "halt"
    finally:
        node.destroy_node()
        rclpy.shutdown()
