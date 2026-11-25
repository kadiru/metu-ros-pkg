"""Execute the 5v5 role assignment using the existing roller, drive and pass scorer."""

from dataclasses import replace
import math
from .carrier_motor import track_roller
from .counterpass import MOTOR, shot_window
from .five_receive import carrying
from .five_roles import CFG, PENALTY_FRONT, PENALTY_HALF_WIDTH, TIMED_ROLES, Flight, limit
from .goal_cover import opposing_holder
from .model import possession, wrap
from .open_motion import steer_team
from .pivot_fetch import PivotFetch
from .tackle_selector import CONTAIN, TackleSelector
from .timed_keeper import TimedGoalCover
from .three_match import filter_commands
from .timed_motion import inside, plan_team
from .touchline import recovery_command
from .three_recipient import features, move


def commands(
    world,
    positions,
    weights,
    sole,
    tactics,
    targets=None,
    receive=None,
    smooth_carry=False,
    tasks=None,
):
    ball = world.ball
    out = {}
    passes = {}
    for r in world.robots:
        team = r.key.team
        sign = world.direction(team)
        role = positions[r.key].role
        tx, ty = positions[r.key].target
        angle = math.atan2(ball.y - r.y, ball.x - r.x)
        if receive and role == "Receiver" and tactics.flight and tactics.flight.receiver == r.key:
            out[r.key] = receive.command(world, r, tactics.flight.target)
            if tasks is not None:
                tasks[r.key] = "Cushion the pass"
            continue
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
            if smooth_carry and shoot:
                advance = 0.18
            desired = advance * math.cos(aim), advance * math.sin(aim)
            cmd = (
                carrying(world, r, desired, aim)
                if smooth_carry
                else track_roller(world, r, desired, 9 * wrap(aim - r.yaw), MOTOR)
            )
            if tasks is not None:
                tasks[r.key] = (
                    "Line up a pass"
                    if pass_now
                    else "Line up a shot"
                    if shoot
                    else "Carry the ball"
                )
            if tactics.control == r.key and abs(wrap(aim - r.yaw)) < 0.07 and (shoot or pass_now):
                kick = 2.6 if pass_now else 4.0
                if receive and pass_now:
                    # Preserve range on long links and deliver a slower ball on short passes.
                    kick = min(2.6, 1.70 + 0.23 * math.hypot(gx - ball.x, gy - ball.y))
                cmd = replace(cmd, kick=kick)
                if pass_now:
                    passes[r.key] = Flight(
                        team, mate.key, (gx, gy), world.time + (2.15 if receive else 1.65)
                    )
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
        if (
            r.key.id
            and abs(ty) < PENALTY_HALF_WIDTH
            and sign * tx > PENALTY_FRONT
            and role != "Receiver"
        ):
            tx = PENALTY_FRONT * sign
        target = limit(tx, 4.32), limit(ty, 2.76)
        if targets is not None and role in TIMED_ROLES:
            targets[r.key] = target
        out[r.key] = move(r, *target, angle, speed=1.75)
        if role == "Contain":
            out[r.key] = replace(out[r.key], dribbler=0)
    return out, passes


def can_tackle(world, presser, carrier):
    if presser.key.team == carrier.key.team or not inside((presser.x, presser.y)):
        return False
    if not inside((carrier.x, carrier.y)):
        return False
    if math.hypot(presser.x - carrier.x, presser.y - carrier.y) > 0.95:
        return False
    return all(
        other.key in (presser.key, carrier.key)
        or math.hypot(presser.x - other.x, presser.y - other.y) >= 0.38
        for other in world.robots
    )


def crowded_chaser(world, tactics, roles, measured, policy, recovering, sole):
    ball = world.ball
    can_fetch = (
        not tactics.flight
        and tactics.control is None
        and ball.visible
        and inside((ball.x, ball.y))
        and math.hypot(ball.vx, ball.vy) <= 0.65
    )
    nearby = {
        r.key: r
        for r in world.robots
        if can_fetch
        and roles[r.key].role == "Chaser"
        and r.visible
        and r.key not in recovering
        and (sole is None or sole == r.key)
        and inside((r.x, r.y))
        and math.hypot(r.x - ball.x, r.y - ball.y) <= 0.55
        and math.hypot(*measured[r.key]) <= 0.55
        and any(
            q.key != r.key
            and q.visible
            and math.hypot(*measured[q.key]) <= 0.45
            and math.hypot(r.x - q.x, r.y - q.y) < policy.clearance + 0.14
            for q in world.robots
        )
    }
    stalls = {}
    eligible = []
    for key, r in nearby.items():
        ticks = tactics.fetch_stalls.get(key, 0)
        if key == tactics.fetcher or math.hypot(*measured[key]) < 0.14:
            ticks += 1
        else:
            ticks = 0
        stalls[key] = ticks
        if ticks >= 5:
            eligible.append(key)
    tactics.fetch_stalls = stalls
    if tactics.fetcher not in eligible:
        tactics.fetcher = min(
            eligible,
            key=lambda key: math.hypot(nearby[key].x - ball.x, nearby[key].y - ball.y),
            default=None,
        )
    return {tactics.fetcher} if tactics.fetcher is not None else set()


def open_play(
    world,
    tactics,
    weights,
    measured,
    policy,
    touchline=True,
    trajectory=False,
    counts=None,
    fallback=None,
    trace=None,
    legal_off_ball=True,
    tackle=None,
    tackle_audit=None,
    crowded_fetch=False,
    receive=None,
    smooth_carry=False,
    allocation_audit=None,
    task_audit=None,
):
    """One open-play control tick shared by ordinary matches and paired replays."""
    carrier = None
    tackling = None
    if (
        tackle
        and policy
        and tactics.control
        and not tactics.flight
        and possession(world, tactics.control, CFG)
    ):
        carrier = world.robot(tactics.control)
        tackling = next(
            (
                r.key
                for r in world.robots
                if tactics.previous.get(r.key) == "Presser" and can_tackle(world, r, carrier)
            ),
            None,
        )
    sole = tactics.observe(world, tackling=tackling)
    roles, phases = {}, {}
    for team in ("blue", "yellow"):
        phases[team], side = tactics.assign(
            world, team, legal_off_ball=legal_off_ball, audit=allocation_audit
        )
        roles.update(side)
    targets = {} if trajectory else None
    out, planned = commands(
        world, roles, weights, sole, tactics, targets, receive, smooth_carry, task_audit
    )
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
                if task_audit is not None:
                    task_audit[r.key] = "Recover along the touchline"
    if policy:
        fetching = (
            crowded_chaser(world, tactics, roles, measured, policy, recovering, sole)
            if crowded_fetch
            else set()
        )
        if trace is not None:
            trace.update({key: {"mode": "crowded_fetch"} for key in fetching})
        if task_audit is not None:
            task_audit.update({key: "Approach a crowded loose ball" for key in fetching})
        challenges = {}
        vetoed = set()
        goal_vetoes = set()
        cover_keepers = {}
        keeper_handoffs = {}
        if carrier and tactics.control == carrier.key:
            for r in world.robots:
                if (
                    roles[r.key].role == "Presser"
                    and r.key not in recovering
                    and can_tackle(world, r, carrier)
                ):
                    out[r.key] = tackle.command(
                        world, r, carrier, measured, possession(world, r.key, CFG)
                    )
                    if task_audit is not None:
                        task_audit[r.key] = (
                            {
                                "direct": "Draw the ball away",
                                "pivot_left": "Pivot left to free the ball",
                                "pivot_right": "Pivot right to free the ball",
                                "contain": "Contain an unsafe release",
                            }[tackle.action(r.key)]
                            if isinstance(tackle, TackleSelector)
                            else "Challenge from the ball side"
                        )
                    if isinstance(tackle, TackleSelector) and tackle.goal_veto(r.key):
                        goal_vetoes.add(r.key)
                    if isinstance(tackle, TackleSelector) and tackle.action(r.key) == CONTAIN:
                        vetoed.add(r.key)
                    else:
                        challenges[r.key] = carrier
        if isinstance(tackle, (PivotFetch, TackleSelector)) and sole is None and not tactics.flight:
            for r in world.robots:
                if (
                    roles[r.key].role in ("Chaser", "Presser")
                    and r.key not in recovering
                    and r.key not in challenges
                    and inside((r.x, r.y))
                ):
                    fetch = tackle.collect_loose(world, r)
                    if fetch is not None:
                        out[r.key] = fetch
                        if task_audit is not None:
                            task_audit[r.key] = "Collect after the pivot"
        if isinstance(tackle, TackleSelector) and tackle.cover is not None:
            for team, phase in phases.items():
                field_player = next(
                    (
                        r
                        for r in world.robots
                        if r.key.team == team
                        and roles[r.key].role in ("Presser", "Guard box", "Contain", "Chaser")
                    ),
                    None,
                )
                if field_player is None:
                    continue
                key = field_player.key
                guarded = roles[key].role in ("Guard box", "Contain")
                blocked = (
                    phase == "Defend"
                    and key not in challenges
                    and (guarded or (carrier is not None and carrier.key.team != team))
                )
                if key in recovering:
                    continue
                cover = tackle.cover.commands(
                    world, field_player, key in vetoed or blocked, preparing=key in goal_vetoes
                )
                for actor, request in cover.items():
                    if targets is not None:
                        targets.pop(actor, None)
                    out[actor] = request
                    planned.pop(actor, None)
                    challenges.pop(actor, None)
                    if task_audit is not None:
                        task_audit[actor] = (
                            "Meet the ball at goal" if actor.id == 0 else "Clear the keeper's route"
                        )
                    if actor.id == 0:
                        cover_keepers[actor] = -world.direction(team)
                        if isinstance(tackle.cover, TimedGoalCover):
                            keeper_handoffs[actor] = key
        if tackle_audit is not None:
            tackle_audit.update(challenges)
        for keeper in cover_keepers:
            rival = opposing_holder(world, keeper.team)
            if rival is not None:
                challenges[keeper] = rival
        raw, out = out, {}
        for team in phases:
            own = {key: value for key, value in raw.items() if key.team == team}
            steered = steer_team(
                world.robots,
                team,
                own,
                measured,
                policy,
                CFG.dt,
                recovering,
                challenges=challenges,
                goal_cover=cover_keepers,
                keeper_handoffs=keeper_handoffs,
                fetching=fetching,
            )
            if trajectory:
                steered = plan_team(
                    world.robots,
                    team,
                    steered,
                    targets,
                    roles,
                    measured,
                    policy.clearance,
                    counts,
                    fallback,
                    trace=trace,
                )
            out.update(steered)
    else:
        out, _ = filter_commands(world, out, recovering)
    return out, planned, roles, phases
