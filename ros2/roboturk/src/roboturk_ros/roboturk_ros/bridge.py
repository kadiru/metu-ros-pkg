"""Single simulation owner: command watchdog, clock, world state and reset service."""

from dataclasses import replace
import random
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from roboturk.model import Ball, Command, Referee, Robot, RobotKey, World, safe_command
from roboturk.ssl_referee import RestartGate
from roboturk_interfaces.msg import CommandArray, RefereeState, WorldState
from roboturk_interfaces.srv import ResetEpisode
from .conversions import from_command_message, set_stamp, stamp_seconds, world_message


def formation(seed=0):
    rng = random.Random(seed)
    robots = tuple(
        Robot(
            RobotKey(team, i),
            sign * (3.0 - i * 0.7),
            (i - 1) * 0.9,
            0.0 if team == "blue" else 3.141592653589793,
        )
        for team, sign in (("blue", -1), ("yellow", 1))
        for i in range(3)
    )
    return World(0.0, robots, Ball(0.0, rng.uniform(-0.2, 0.2)), Referee.HALT)


class Bridge(Node):
    def __init__(self, supervisor, backend, cfg):
        super().__init__("webots_bridge")
        self.supervisor, self.backend, self.cfg = supervisor, backend, cfg
        self.commands = {}
        self.command_received = {}
        self.referee = Referee.HALT
        self.referee_received = 0.0
        self.restart_gate = RestartGate()
        self.restart = ("", 0, "")
        self.attack_sign = 1
        backend.reset(formation())
        self.world_pub = self.create_publisher(WorldState, "/roboturk/world", 1)
        self.clock_pub = self.create_publisher(Clock, "/clock", 1)
        self.create_subscription(RefereeState, "/roboturk/referee", self.on_referee, 1)
        for team in ("blue", "yellow"):
            self.create_subscription(
                CommandArray,
                f"/roboturk/{team}/commands",
                lambda msg, team=team: self.on_commands(msg, team),
                1,
            )
        self.create_service(ResetEpisode, "/roboturk/reset", self.reset_episode)

    def on_referee(self, message):
        try:
            referee = Referee(message.command)
            if message.attack_sign not in (-1, 1):
                raise ValueError("attack_sign must be -1 or +1")
            age = self.supervisor.getTime() - stamp_seconds(message.header.stamp)
            if not -0.02 <= age <= 0.5:
                raise ValueError("Stale referee stamp")
            if message.restart_team not in ("", "blue", "yellow"):
                raise ValueError("Invalid restart team")
            self.restart = (message.source, message.command_counter, message.restart_team)
            self.referee = referee
            self.attack_sign = message.attack_sign
            self.referee_received = time.monotonic()
        except ValueError as error:
            self.referee = Referee.HALT
            self.get_logger().error(str(error))

    def on_commands(self, message, team):
        age = self.supervisor.getTime() - stamp_seconds(message.header.stamp)
        if message.header.frame_id != "robot_body" or not -0.02 <= age <= 0.25:
            return
        try:
            commands = dict(from_command_message(c) for c in message.commands)
            if any(k.team != team for k in commands):
                raise ValueError("Command sent to wrong team topic")
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        # Full team snapshot: omitted commands stop immediately.
        self.commands = {k: v for k, v in self.commands.items() if k.team != team}
        self.commands.update(commands)
        self.command_received[team] = time.monotonic()

    def reset_episode(self, request, response):
        self.commands.clear()
        self.command_received.clear()
        self.referee = Referee.HALT
        self.referee_received = 0.0
        self.restart_gate = RestartGate()
        self.restart = ("", 0, "")
        self.backend.reset(formation(request.seed))
        response.world = world_message(replace(self.backend.world, time=self.supervisor.getTime()))
        response.success = True
        return response

    def step(self):
        now = time.monotonic()
        referee_age = now - self.referee_received
        referee = self.referee if referee_age <= 0.5 else Referee.HALT
        self.backend.world = replace(
            self.backend.world, referee=referee, attack_sign=self.attack_sign
        )
        restricted = self.restart_gate.restricted_to(*self.restart, self.backend.world.ball)
        commands = {}
        for robot in self.backend.world.robots:
            if restricted and robot.key.team != restricted:
                continue
            commands[robot.key] = safe_command(
                self.backend.world,
                robot.key,
                self.commands.get(robot.key, Command()),
                self.cfg,
                state_age=now - self.command_received.get(robot.key.team, 0.0),
                referee_age=referee_age,
            )
        world = self.backend.step(commands)
        # ROS clock remains monotonic across episode resets.
        world = replace(world, time=self.supervisor.getTime())
        clock = Clock()
        set_stamp(clock.clock, world.time)
        self.clock_pub.publish(clock)
        self.world_pub.publish(world_message(world))


def run(supervisor, backend, cfg):
    rclpy.init()
    node = Bridge(supervisor, backend, cfg)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.step()
    finally:
        node.destroy_node()
        rclpy.shutdown()
