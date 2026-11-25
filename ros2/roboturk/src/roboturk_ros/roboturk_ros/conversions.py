from dataclasses import replace

from roboturk.match_limits import PRACTICE_RANGES, limit_values, validate_limits
from roboturk.model import Ball, Command, Config, Referee, Robot, RobotKey, World
from roboturk_interfaces.msg import BallState, MotionLimits, RobotCommand, RobotState, WorldState


def set_stamp(stamp, seconds):
    ns = round(seconds * 1_000_000_000)
    stamp.sec, stamp.nanosec = divmod(ns, 1_000_000_000)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def world_message(world, cfg=Config(), *, practice_tuning=False):
    message = WorldState()
    message.header.frame_id = "field"
    set_stamp(message.header.stamp, world.time)
    message.attack_sign = world.attack_sign
    message.referee = world.referee.value
    values = {key: float(value) for key, value in limit_values(cfg).items()}
    message.motion_limits = MotionLimits(**values, practice_tuning=practice_tuning)
    for robot in world.robots:
        item = RobotState()
        item.team, item.robot_id = robot.key.team, robot.key.id
        item.x, item.y, item.yaw = float(robot.x), float(robot.y), float(robot.yaw)
        item.vx, item.vy, item.omega = float(robot.vx), float(robot.vy), float(robot.omega)
        item.visible = robot.visible
        message.robots.append(item)
    message.ball = BallState(
        x=float(world.ball.x),
        y=float(world.ball.y),
        vx=float(world.ball.vx),
        vy=float(world.ball.vy),
        visible=world.ball.visible,
    )
    return message


def config_from_world_message(message, cfg):
    values = message.motion_limits
    overrides = validate_limits({key: getattr(values, key) for key in PRACTICE_RANGES})
    return replace(cfg, **overrides)


def from_world_message(message):
    return World(
        stamp_seconds(message.header.stamp),
        tuple(
            Robot(RobotKey(r.team, r.robot_id), r.x, r.y, r.yaw, r.vx, r.vy, r.omega, r.visible)
            for r in message.robots
        ),
        Ball(
            message.ball.x, message.ball.y, message.ball.vx, message.ball.vy, message.ball.visible
        ),
        Referee(message.referee),
        message.attack_sign,
    )


def command_message(key, command):
    return RobotCommand(
        team=key.team,
        robot_id=key.id,
        vx=float(command.vx),
        vy=float(command.vy),
        omega=float(command.omega),
        dribbler=float(command.dribbler),
        kick=float(command.kick),
    )


def from_command_message(message):
    return RobotKey(message.team, message.robot_id), Command(
        message.vx, message.vy, message.omega, message.dribbler, message.kick
    )
