"""Measure free rolls, driven acceleration and high-speed goal impacts in native Webots."""

import math
from dataclasses import replace

from roboturk.carpet import ROLL_DECEL, SLIDE_DECEL
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.model import Ball, Command, Referee, Robot, RobotKey, SSL_KICK_LIMIT, World
from roboturk.ssl_vision import geometry


def speed(entity):
    return math.hypot(entity.vx, entity.vy)


def audit(sim, cfg):
    cfg = replace(cfg, dt=sim.getBasicTimeStep() / 1000)
    key = RobotKey("blue", 0)
    player = Robot(key, -3.6, 0)
    start = World(0, (player,), Ball(player.x + cfg.possession_x, 0), Referee.RUNNING)
    backend = ContinuousWebotsBackend(sim, cfg)
    field = sim.getFromDef("FIELD")

    def solid(name):
        node = field.getFromProtoDef(name)
        return (
            node.getPosition(),
            node.getField("boundingObject").getSFNode().getField("size").getSFVec3f(),
        )

    carpet = solid("CARPET")[1]
    geometry_mm = geometry().geometry.field
    assert math.dist(carpet, (10.4, 7.4, 0.05)) < 1e-6
    # Check rendered line endpoints against the independently serialized wire packet.
    for line in geometry_mm.field_lines:
        node = field.getFromProtoDef(line.name)
        center = node.getPosition()
        shape = node.getField("children").getMFNode(0)
        size = shape.getField("geometry").getSFNode().getField("size").getSFVec3f()
        assert (
            math.dist(center[:2], ((line.p1.x + line.p2.x) / 2000, (line.p1.y + line.p2.y) / 2000))
            < 1e-6
        )
        assert (
            math.dist(
                size[:2],
                (
                    (abs(line.p1.x - line.p2.x) or line.thickness) / 1000,
                    (abs(line.p1.y - line.p2.y) or line.thickness) / 1000,
                ),
            )
            < 1e-6
        )
    ends = {}
    for side, sign in (("LEFT", -1), ("RIGHT", 1)):
        top, side_size = solid(side + "_GOAL_TOP")
        bottom, _ = solid(side + "_GOAL_BOTTOM")
        rear, rear_size = solid(side + "_GOAL_BACK")
        wall, wall_size = solid(side + "_END_BOARD")
        touch, touch_size = solid(side + "_TOUCH_BOARD")
        entry = sign * top[0] - side_size[0] / 2
        mouth = top[1] - bottom[1] - side_size[1]
        depth = sign * rear[0] - rear_size[0] / 2 - entry
        end_margin = sign * wall[0] - wall_size[0] / 2 - geometry_mm.field_length / 2000
        touch_margin = sign * touch[1] - touch_size[1] / 2 - geometry_mm.field_width / 2000
        measured = [mouth, depth, rear_size[2], rear_size[0], entry, end_margin, touch_margin]
        assert math.dist(measured, [1, 0.18, 0.16, 0.02, 4.505, 0.30, 0.30]) < 1e-6
        ends[side] = dict(
            zip(
                (
                    "mouth",
                    "clear_depth",
                    "height",
                    "wall_thickness",
                    "entry_from_center",
                    "end_wall_margin",
                    "touch_wall_margin",
                ),
                measured,
                strict=True,
            )
        )

    rolls = []
    for launch in (0.5, 1.0, 2.0):
        w = backend.reset(start)
        speeds = [launch]
        for tick in range(round(10 / cfg.dt)):
            w = backend.step({key: Command(kick=launch)} if tick == 0 else {})
            speeds.append(speed(w.ball))
            if speeds[-1] < 0.0001:
                break
        assert speeds[-1] < 0.0001
        stationary = w.ball
        for _ in range(round(1 / cfg.dt)):
            w = backend.step({})
        drift = math.dist((stationary.x, stationary.y), (w.ball.x, w.ball.y))
        assert drift < 0.0002
        slide = (speeds[1] - speeds[4]) / (3 * cfg.dt)
        mid = len(speeds) // 2
        roll = (speeds[mid] - speeds[mid + 10]) / (10 * cfg.dt)
        rolls.append(
            dict(
                kick=launch,
                seconds=(len(speeds) - 1) * cfg.dt,
                distance=stationary.x - start.ball.x,
                slide_decel=slide,
                roll_decel=roll,
                rest_drift=drift,
            )
        )
        assert abs(roll - ROLL_DECEL) < 0.03 and abs(slide - SLIDE_DECEL) < 0.08, rolls
    # Sample opposite travel and oblique headings as well as the field axes.
    directions = []
    for heading in (math.pi / 4, math.pi / 2, math.pi):
        dx, dy = math.cos(heading), math.sin(heading)
        robot = replace(player, x=0, y=0, yaw=heading)
        w = backend.reset(
            replace(start, robots=(robot,), ball=Ball(cfg.possession_x * dx, cfg.possession_x * dy))
        )
        origin = w.ball
        for tick in range(round(5 / cfg.dt)):
            w = backend.step({key: Command(kick=1)} if tick == 0 else {})
            if speed(w.ball) < 0.0001:
                break
        directions.append(
            dict(
                heading=heading,
                distance=math.dist((w.ball.x, w.ball.y), (origin.x, origin.y)),
                seconds=(tick + 1) * cfg.dt,
                final_speed=speed(w.ball),
            )
        )
    for direction in directions:
        assert abs(direction["distance"] - rolls[1]["distance"]) < 0.005, directions
        assert direction["final_speed"] < 0.0001
    kicks = []
    for sign, cap in ((1, cfg.max_kick), (-1, cfg.max_kick), (1, SSL_KICK_LIMIT)):
        robot = replace(player, x=sign * 3.5, yaw=0 if sign == 1 else math.pi)
        w = backend.reset(
            replace(start, robots=(robot,), ball=Ball(robot.x + sign * cfg.possession_x, 0))
        )
        backend.cfg = replace(cfg, max_kick=cap)
        w = backend.step({key: Command(kick=99)})
        first = speed(w.ball)
        assert abs(first - cap) < 0.05
        furthest = sign * w.ball.x
        for _ in range(round(0.55 / cfg.dt)):
            w = backend.step({})
            furthest = max(furthest, sign * w.ball.x)
        assert 4.52 < furthest < 4.70
        kicks.append(dict(direction=sign, limit=cap, first_sample=first, furthest_x=furthest))
    # Run far from ball and walls. Include braking, reversal and diagonal input.
    backend.cfg = cfg
    w = backend.reset(replace(start, ball=Ball(0, -2.8), robots=(replace(player, x=-2, y=-1),)))
    peak_speed = peak_accel = 0
    for command, seconds in ((Command(8, 8), 0.8), (Command(-8, -8), 0.9), (Command(), 0.6)):
        for _ in range(round(seconds / cfg.dt)):
            before = w.robot(key)
            w = backend.step({key: command})
            r = w.robot(key)
            peak_accel = max(peak_accel, math.dist((before.vx, before.vy), (r.vx, r.vy)) / cfg.dt)
            peak_speed = max(peak_speed, speed(r))
    assert abs(peak_speed - cfg.max_speed) < 0.015 and abs(peak_accel - cfg.max_acceleration) < 0.03
    assert speed(w.robot(key)) < 0.001
    return dict(
        step_ms=backend.basic_ms,
        free_kicks=rolls,
        high_speed_kicks=kicks,
        direction_checks=directions,
        drive=dict(
            max_speed=peak_speed, max_acceleration=peak_accel, stopped_speed=speed(w.robot(key))
        ),
        carpet=carpet,
        goal_and_wall_dimensions=ends,
    )


def film_roll(sim, cfg, path):
    """Record the 2 m/s release at real time using the native overhead view."""
    from tempfile import TemporaryDirectory
    from pathlib import Path
    from PIL import Image, ImageDraw, ImageFont

    camera = sim.getFromDef("CAMERA")
    camera.getField("orientation").setSFRotation(
        [-0.577350269, 0.577350269, 0.577350269, 2.094395102]
    )
    camera.getField("position").setSFVec3f([-1.55, 0, 6.4])
    fov = camera.getField("fieldOfView").getSFFloat()
    cfg = replace(cfg, dt=0.02)
    backend = ContinuousWebotsBackend(sim, cfg)
    key = RobotKey("blue", 0)
    initial = Ball(-3.6 + cfg.possession_x, 0)
    w = backend.reset(World(0, (Robot(key, -3.6, 0),), initial, Referee.RUNNING))
    frames = []
    font = ImageFont.load_default(size=20)
    with TemporaryDirectory() as temp:
        capture = Path(temp) / "frame.jpg"
        for tick in range(321):
            if tick % 4 == 0:
                sim.exportImage(str(capture), 95)
                with Image.open(capture) as original:
                    width, height = original.size
                    strip = original.crop((0, height * 0.28, width, height * 0.72))
                    strip = strip.resize(
                        (680, round(680 * strip.height / width)), Image.Resampling.LANCZOS
                    )
                frame = Image.new("RGB", (strip.width, strip.height + 86), "#10212b")
                frame.paste(strip, (0, 40))
                draw = ImageDraw.Draw(frame)
                px = strip.width / (2 * (6.4 - cfg.ball_radius) * math.tan(fov / 2))
                bx = strip.width / 2 + (w.ball.x + 1.55) * px
                by = 40 + strip.height / 2 - w.ball.y * px
                draw.ellipse((bx - 8, by - 8, bx + 8, by + 8), outline="#ffbe52", width=2)
                draw.text((14, 7), "A 2 m/s kick on the Division B carpet", font=font, fill="white")
                draw.text(
                    (14, strip.height + 51),
                    f"{tick * cfg.dt:.2f} s     {speed(w.ball):.2f} m/s     {w.ball.x - initial.x:.2f} m traveled",
                    font=font,
                    fill="white",
                )
                frames.append(frame)
            if tick < 320:
                w = backend.step({key: Command(kick=2)} if tick == 0 else {})
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=80, loop=0)
