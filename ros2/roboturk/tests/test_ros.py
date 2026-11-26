"""Runs in a built/sourced ROS workspace. Skipped on the standalone macOS core environment."""

# ruff: noqa: E402 -- generated ROS imports must follow importorskip on non-ROS hosts.
import pytest
from dataclasses import replace

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("roboturk_interfaces.msg")

from roboturk.backend import ReferenceBackend
from roboturk.model import Command, Config, Referee, RobotKey
from roboturk_interfaces.msg import CommandArray, RefereeState
from roboturk_interfaces.srv import ResetEpisode
from roboturk_ros.bridge import Bridge, formation
from roboturk_ros.conversions import (
    command_message,
    from_command_message,
    from_world_message,
    set_stamp,
    world_message,
)
from roboturk_ros.strategy_node import StrategyNode


def test_ros_world_roundtrip():
    w = formation(12)
    w = replace(w, ball=replace(w.ball, z=0.35, vz=1.2))
    assert from_world_message(world_message(w)) == w
    key, chip = RobotKey("blue", 1), Command(kick=4.1, kick_elevation=0.6)
    assert from_command_message(command_message(key, chip)) == (key, chip)


def test_ros_practice_limits_reach_the_actuator_and_strategy(tmp_path):
    import json
    from types import SimpleNamespace

    from roboturk.match_limits import GAME_LIMITS, limit_values

    class Supervisor:
        def getTime(self):
            return 1.0

    limits_file = tmp_path / "motion-limits.json"
    config = dict(GAME_LIMITS, max_speed=4, max_acceleration=10, max_kick=10)
    limits_file.write_text(json.dumps(config))
    rclpy.init()
    backend = ReferenceBackend()
    bridge = Bridge(Supervisor(), backend, Config(), limits_file)
    strategy = StrategyNode()
    received = []
    bridge.world_pub = SimpleNamespace(publish=received.append)
    try:
        referee = RefereeState(command="running", attack_sign=1)
        set_stamp(referee.header.stamp, 1)
        bridge.on_referee(referee)
        bridge.step()
        strategy.on_world(received[-1])
        assert limit_values(backend.cfg) == limit_values(strategy.cfg) == config
        assert strategy.library.fallback.full_power_shots

        updated = dict(config, max_speed=1.4, max_omega=4, max_acceleration=2.0, max_kick=2.5)
        temporary = limits_file.with_suffix(".tmp")
        temporary.write_text(json.dumps(updated))
        temporary.replace(limits_file)
        bridge.step()
        strategy.on_world(received[-1])
        assert limit_values(backend.cfg) == limit_values(strategy.strategy.cfg) == updated

        referee.ssl_command = "FORCE_START"
        bridge.on_referee(referee)
        bridge.step()
        strategy.on_world(received[-1])
        assert limit_values(backend.cfg) == limit_values(strategy.cfg) == GAME_LIMITS
        assert not strategy.library.fallback.full_power_shots
    finally:
        strategy.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


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
        assert backend.world.robot(key).vx == pytest.approx(
            backend.cfg.dt * backend.cfg.max_acceleration
        )
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


def test_official_udp_reaches_ros_actuator_gate():
    import socket
    import time

    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from roboturk.ssl_referee import Packet
    from roboturk_ros.ssl_referee import OfficialReferee

    class Supervisor:
        def getTime(self):
            return time.time()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
        reserve.bind(("127.0.0.1", 0))
        port = reserve.getsockname()[1]

    rclpy.init()
    source = OfficialReferee(
        parameter_overrides=[Parameter("address", value="127.0.0.1"), Parameter("port", value=port)]
    )
    backend = ReferenceBackend()
    bridge = Bridge(Supervisor(), backend, Config())
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(source)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    p = Packet(stage=Packet.NORMAL_FIRST_HALF, source_identifier="ros-test")
    for info in (p.blue, p.yellow):
        info.name = "test"
        info.score = info.red_cards = info.yellow_cards = 0
        info.timeouts = info.timeout_time = info.goalkeeper = 0

    def wait_for(mode, transmit):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if transmit:
                p.packet_timestamp = time.time_ns() // 1000
                sender.sendto(p.SerializeToString(), ("127.0.0.1", port))
            executor.spin_once(timeout_sec=0.02)
            if bridge.referee == mode:
                return
        pytest.fail(f"ROS bridge did not receive {mode}")

    def drive():
        for team in ("blue", "yellow"):
            message = CommandArray()
            set_stamp(message.header.stamp, time.time())
            message.header.frame_id = "robot_body"
            key = RobotKey(team, 1)
            message.commands.append(command_message(key, Command(1, 0)))
            bridge.on_commands(message, team)
        bridge.step()
        return [abs(backend.world.robot(RobotKey(t, 1)).vx) for t in ("blue", "yellow")]

    try:
        p.command, p.command_counter, p.command_timestamp = Packet.DIRECT_FREE_BLUE, 1, 1
        wait_for(Referee.RUNNING, True)
        increment = backend.cfg.dt * backend.cfg.max_acceleration
        assert drive() == pytest.approx([increment, 0])
        p.command, p.command_counter = Packet.STOP, 2
        wait_for(Referee.STOP, True)
        assert drive() == [0, 0]
        p.command, p.command_counter = Packet.FORCE_START, 3
        wait_for(Referee.RUNNING, True)
        assert drive() == pytest.approx([increment, increment])
        wait_for(Referee.HALT, False)
        assert drive() == [0, 0]
    finally:
        sender.close()
        executor.shutdown()
        source.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(("ssl_command", "limit"), [("", 10.0), ("FORCE_START", 6.5)])
def test_ros_kick_ceiling(ssl_command, limit):
    from dataclasses import replace
    from roboturk.model import Ball

    class Supervisor:
        def getTime(self):
            return 1.0

    rclpy.init()
    backend = ReferenceBackend()
    bridge = Bridge(Supervisor(), backend, Config())
    try:
        key = RobotKey("blue", 1)
        r = backend.world.robot(key)
        backend.reset(replace(backend.world, ball=Ball(r.x + backend.cfg.possession_x, r.y)))
        state = RefereeState(command="running", attack_sign=1, ssl_command=ssl_command)
        set_stamp(state.header.stamp, 1)
        bridge.on_referee(state)
        commands = CommandArray()
        commands.header.frame_id = "robot_body"
        set_stamp(commands.header.stamp, 1)
        commands.commands.append(command_message(key, Command(kick=20)))
        bridge.on_commands(commands, "blue")
        bridge.step()
        assert [speed for _, _, speed in backend.last_kick_events] == [limit]
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
