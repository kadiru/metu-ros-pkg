"""Execute the 5v5 role assignment using the existing roller, drive and pass scorer."""

from dataclasses import replace
import math
from .carrier_motor import track_roller
from .counterpass import MOTOR, shot_window
from . import field as pitch
from .five_roles import CFG, Flight, limit
from .model import wrap
from .open_motion import steer_team
from .three_match import filter_commands
from .timed_motion import PRIORITY, plan_team
from .touchline import recovery_command
from .three_recipient import features, move


def commands(world, positions, weights, sole, tactics, targets=None):
    ball = world.ball
    out = {}
    passes = {}
    for r in world.robots:
        team = r.key.team
        sign = world.direction(team)
        role = positions[r.key].role
        tx, ty = positions[r.key].target
        angle = math.atan2(ball.y - r.y, ball.x - r.x)
        if r.key == sole and (
            role in ("Keeper", "Presser", "Carrier", "Chaser", "Receiver")
            or tactics.control == r.key
        ):
            mates = [
                q for q in world.robots if q.key.team == team and q.key != r.key and q.key.id != 0
            ]
            rivals = [q for q in world.robots if q.key.team != team]
            gx, gy, gap = shot_window(
                ball.x, ball.y, sign, [q for q in world.robots if q.key != r.key]
            )
            pressure = min(math.hypot(q.x - ball.x, q.y - ball.y) for q in rivals)
            choices = []
            for m in mates:
                obs = features(world, r, m)
                progress = sign * (m.x - r.x)
                if progress < -0.60 or not 0.72 < math.hypot(m.x - r.x, m.y - r.y) < 4.1:
                    continue
                choices.append((float(obs @ weights[team]) + 0.8 * progress, m))
            mate = max(choices, key=lambda v: v[0])[1] if choices else None
            shoot = sign * (gx - ball.x) < 2.45 and gap > 0.13
            pass_now = (
                mate is not None
                and not shoot
                and (role == "Keeper" or pressure < 1.32 or sign * (mate.x - r.x) > 0.75)
            )
            if pass_now:
                gx = limit(mate.x + 0.16 * mate.vx, 4.22)
                gy = limit(mate.y + 0.16 * mate.vy, 2.68)
            aim = math.atan2(gy - ball.y, gx - ball.x)
            advance = 0.22 if pass_now else 1.0
            cmd = track_roller(
                world,
                r,
                (advance * math.cos(aim), advance * math.sin(aim)),
                9 * wrap(aim - r.yaw),
                MOTOR,
            )
            if tactics.control == r.key and abs(wrap(aim - r.yaw)) < 0.07 and (shoot or pass_now):
                cmd = replace(cmd, kick=2.6 if pass_now else 4.0)
                if pass_now:
                    passes[r.key] = Flight(team, mate.key, (gx, gy), world.time + 1.65)
            out[r.key] = cmd
            continue
        if role in ("Chaser", "Presser", "Carrier"):
            # Point the roller into the ball. Intercept a moving ball ahead of its current position.
            lead = min(0.44, math.hypot(r.x - ball.x, r.y - ball.y) / 2.0)
            x, y = ball.x + lead * ball.vx, ball.y + lead * ball.vy
            angle = math.atan2(y - r.y, x - r.x)
            tx, ty = x - 0.103 * math.cos(angle), y - 0.103 * math.sin(angle)
        elif role == "Receiver":
            speed = math.hypot(ball.vx, ball.vy)
            if speed > 0.20:
                dx, dy = ball.vx / speed, ball.vy / speed
                dist = max(0, (tx - ball.x) * dx + (ty - ball.y) * dy)
                tx, ty = (
                    ball.x + (dist + CFG.possession_x) * dx,
                    ball.y + (dist + CFG.possession_x) * dy,
                )
                angle = math.atan2(-dy, -dx)
        front = CFG.length / 2 - pitch.DEFENSE_DEPTH - CFG.robot_radius
        if (
            r.key.id
            and abs(ty) < pitch.DEFENSE_WIDTH / 2 + CFG.robot_radius
            and sign * tx > front
            and role != "Receiver"
        ):
            tx = front * sign
        target = limit(tx, 4.32), limit(ty, 2.76)
        if targets is not None and role in PRIORITY:
            targets[r.key] = target
        out[r.key] = move(r, *target, angle, speed=1.75)
        if role == "Contain":
            out[r.key] = replace(out[r.key], dribbler=0)
    return out, passes


def open_play(
    world, tactics, weights, measured, policy, touchline=True, trajectory=False, counts=None
):
    """One open-play control tick shared by ordinary matches and paired replays."""
    sole = tactics.observe(world)
    roles, phases = {}, {}
    for team in ("blue", "yellow"):
        phases[team], side = tactics.assign(world, team)
        roles.update(side)
    targets = {} if trajectory else None
    out, planned = commands(world, roles, weights, sole, tactics, targets)
    recovering = set()
    if touchline:
        for r in world.robots:
            command = recovery_command(world, r, roles[r.key].role, sole)
            if command is not None:
                if targets is not None:
                    targets.pop(r.key, None)
                recovering.add(r.key)
                out[r.key] = command
                planned.pop(r.key, None)
    if policy:
        raw, out = out, {}
        for team in phases:
            own = {key: value for key, value in raw.items() if key.team == team}
            steered = steer_team(world.robots, team, own, measured, policy, CFG.dt, recovering)
            if trajectory:
                steered = plan_team(
                    world.robots, team, steered, targets, roles, measured, policy.clearance, counts
                )
            out.update(steered)
    else:
        out, _ = filter_commands(world, out, recovering)
    return out, planned, roles, phases
