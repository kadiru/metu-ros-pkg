import json
import time

import rclpy
from rclpy.node import Node
from roboturk.behavior import BehaviorLibrary, CheckpointPolicy
from roboturk.model import Command, Config, Skill, constrain_skill, safe_command
from roboturk.strategy import TeamStrategy
from roboturk_interfaces.msg import CommandArray, WorldState
from .conversions import command_message, from_world_message


class StrategyNode(Node):
    def __init__(self):
        super().__init__("team_strategy")
        team = self.declare_parameter("team", "blue").value
        if team not in ("blue", "yellow"):
            raise ValueError("team must be blue or yellow")
        self.cfg = Config()
        self.strategy = TeamStrategy(team, self.cfg)
        self.library = BehaviorLibrary()
        # JSON mapping makes individual policies replaceable without changing team play.
        paths = json.loads(self.declare_parameter("policies", "{}").value)
        for skill, path in paths.items():
            self.library.register(Skill(skill), CheckpointPolicy(path, Skill(skill), self.cfg))
        self.world = None
        self.received = 0.0
        self.publisher = self.create_publisher(CommandArray, f"/roboturk/{team}/commands", 1)
        self.create_subscription(WorldState, "/roboturk/world", self.on_world, 1)
        self.create_timer(self.cfg.dt, self.tick)

    def on_world(self, message):
        try:
            world = from_world_message(message)
            if self.world is not None and world.time < self.world.time:
                self.strategy.reset()
            self.world, self.received = world, time.monotonic()
        except (ValueError, KeyError) as error:
            self.get_logger().error(f"Rejected world state: {error}")

    def tick(self):
        if self.world is None:
            return
        play = self.strategy.plan(self.world)
        message = CommandArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "robot_body"
        for key, intent in play.assignments.items():
            try:
                command = constrain_skill(
                    self.library.command(self.world, key, intent, self.cfg), intent.skill
                )
                command = safe_command(
                    self.world, key, command, self.cfg, state_age=time.monotonic() - self.received
                )
            except (ValueError, RuntimeError) as error:
                self.get_logger().error(f"Policy failed for {key}: {error}")
                command = Command()
            message.commands.append(command_message(key, command))
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = StrategyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
