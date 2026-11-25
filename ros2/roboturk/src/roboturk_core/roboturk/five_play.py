"""Execute the 5v5 role assignment using the existing roller, drive and pass scorer."""

from dataclasses import replace
import math
from .carpet import SWITCH_RATIO, ground_step
from .carrier_motor import track_roller
from .chip import chip_pass
from .counterpass import MOTOR, clearance
from .five_decision import CONTROLLERS
from .five_receive import carrying
from .five_roles import CFG, PENALTY_FRONT, PENALTY_HALF_WIDTH, TIMED_ROLES, Flight, limit
from .five_strategy import BALANCED
from .goal_cover import opposing_holder
from .model import Command, Skill, possession, unit, wrap
from .open_motion import steer_team
from .pivot_fetch import PivotFetch
from .tackle_selector import CONTAIN, PIVOT_SIDES, TackleSelector
from .timed_keeper import TimedGoalCover
from .three_match import filter_commands
from .timed_motion import QUALIFIED_PACE, inside, plan_team
from .touchline import recovery_command
from .three_recipient import move


def pass_speed(ball, target, receive):
    distance = math.hypot(target[0] - ball.x, target[1] - ball.y)
    return min(2.6, 1.70 + 0.23 * distance) if receive else 2.6


def turn_release(
    world, r, spin, target, receiver, cfg, observed, margin, receive, shot_speed, can_kick
):
    ball = world.ball
    turn = spin.turn
    if turn:
        receiver = turn.receiver
        target = turn.target
    if target is None:
        return None
    kick = pass_speed(ball, target, receive) if receiver else shot_speed
    kick = min(kick, cfg.max_kick)
    distance = math.dist((ball.x, ball.y), target)
    others = [
        replace(q, vx=observed.get(q.key, (0, 0))[0], vy=observed.get(q.key, (0, 0))[1])
        for q in world.robots
        if q.visible and q.key not in (r.key, receiver)
    ]
    clear_lane = (
        clearance(ball.x, ball.y, *target, others, kick)
        >= cfg.robot_radius + cfg.ball_radius + 0.028
        and ground_step(kick, SWITCH_RATIO * kick, 60.0)[0] >= distance + 0.04
    )
    if receiver:
        clear_lane = (
            clear_lane and world.robot(receiver).visible and inside(target) and 0.7 < distance < 4.1
        )
    if turn is None:
        if not clear_lane:
            return None
        turn = spin.start(world, r, target, receiver, cfg, observed, margin)
        if turn is None:
            return None
    control = spin.step(world, r, target, cfg, observed, margin) if clear_lane else None
    if control is None:
        spin.turn = None
        return Command(dribbler=1), None, "Hold the ball after a blocked turn"
    cmd, ready = control
    if receiver and math.dist((world.robot(receiver).x, world.robot(receiver).y), target) > 0.34:
        ready = False
    direction = "left" if turn.sweep > 0 else "right"
    if not ready or not can_kick:
        description = f"Turn {direction} for a {'pass' if receiver else 'shot'}"
        if turn.reverse:
            description = f"Reverse turn {direction} for a {'pass' if receiver else 'shot'}"
        return cmd, None, description
    flight = (
        Flight(r.key.team, receiver, target, world.time + (2.15 if receive else 1.65))
        if receiver
        else None
    )
    return replace(cmd, kick=kick), flight, "Spin pass" if receiver else "Spin shot"


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
    strategies=None,
    practice_motion=None,
    chip_motion=None,
    spin_motion=None,
    observed=None,
    spin_clearance=0.31,
    controllers=None,
    decisions=None,
):
    ball = world.ball
    out = {}
    passes = {}
    for r in world.robots:
        team = r.key.team
        motion = practice_motion[team] if practice_motion else None
        sign = world.direction(team)
        role = positions[r.key].role
        tx, ty = positions[r.key].target
        angle = math.atan2(ball.y - r.y, ball.x - r.x)
        if role == "Spin outlet":
            out[r.key] = move(r, tx, ty, angle, speed=0.85)
            if tasks is not None:
                tasks[r.key] = "Hold the outlet for the spin pass"
            continue
        if receive and role == "Receiver" and tactics.flight and tactics.flight.receiver == r.key:
            if ball.z > CFG.ball_radius + 0.035:
                x, y = tactics.flight.target
                ux, uy = unit(ball.vx, ball.vy)
                along = max(0, (x - ball.x) * ux + (y - ball.y) * uy) + CFG.possession_x
                heading = math.atan2(-uy, -ux)
                out[r.key] = move(
                    r, ball.x + along * ux, ball.y + along * uy, heading, speed=1.55, turn_limit=8
                )
            else:
                out[r.key] = receive.command(world, r, tactics.flight.target)
            if tasks is not None:
                tasks[r.key] = "Cushion the pass"
            continue
        if r.key == sole and (
            role in ("Keeper", "Presser", "Carrier", "Chaser", "Receiver")
            or tactics.control == r.key
        ):
            strategy = strategies[team] if strategies else BALANCED
            controller = controllers[team] if controllers else CONTROLLERS[strategy.controller]
            decision = controller.attack(world, r, strategy, weights[team], role == "Keeper")
            pass_now = decision.intent.skill == Skill.PASS
            shoot = decision.intent.skill == Skill.SHOOT
            receiver = decision.intent.receiver
            gx, gy = decision.intent.target
            chip = (
                chip_pass(world, r.key, receiver, (gx, gy), chip_motion[team])
                if chip_motion and pass_now and inside((gx, gy))
                else None
            )
            if spin_motion and not tactics.flight:
                target = (gx, gy) if (shoot or pass_now) and not chip else None
                outcome = turn_release(
                    world,
                    r,
                    tactics.spin,
                    target,
                    receiver,
                    spin_motion[team],
                    observed or {},
                    spin_clearance,
                    receive,
                    motion.max_kick if motion else 4.0,
                    tactics.control == r.key,
                )
                if outcome:
                    cmd, flight, task = outcome
                    out[r.key] = cmd
                    if flight:
                        passes[r.key] = flight
                    if tasks is not None:
                        tasks[r.key] = task
                    continue
            if decisions is not None and decision.reason:
                decisions[r.key] = decision.reason
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
                    "Line up a chip pass"
                    if chip
                    else "Line up a pass"
                    if pass_now
                    else "Line up a shot"
                    if shoot
                    else "Carry the ball"
                )
            tolerance = 0.035 if chip else 0.07
            if (
                tactics.control == r.key
                and abs(wrap(aim - r.yaw)) < tolerance
                and (shoot or pass_now)
            ):
                kick = pass_speed(ball, (gx, gy), receive) if pass_now else 4.0
                if shoot and motion:
                    kick = motion.max_kick
                cmd = (
                    replace(cmd, kick=chip.speed, kick_elevation=chip.elevation)
                    if chip
                    else replace(cmd, kick=kick)
                )
                if pass_now:
                    passes[r.key] = Flight(
                        team, receiver, (gx, gy), world.time + (2.15 if receive else 1.65)
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
        speed = 1.75
        turn_limit = 5
        if motion and r.key.id and role not in ("Receiver", "Contain"):
            speed = motion.max_speed
            turn_limit = motion.max_omega
            if role in ("Chaser", "Presser", "Carrier"):
                room = max(0, math.hypot(r.x - ball.x, r.y - ball.y) - 0.65)
                speed = min(speed, math.sqrt(1.0 + motion.max_acceleration * room))
        out[r.key] = move(r, *target, angle, speed=speed, turn_limit=turn_limit)
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
    strategies=None,
    motion_configs=None,
    practice_pace=False,
    chip_passes=False,
    spin_releases=False,
    controllers=None,
    decision_audit=None,
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
    secure = (
        tackle.secure_after_tackle(world, world.robot(sole))
        if policy and isinstance(tackle, TackleSelector) and sole and not tactics.flight
        else None
    )
    if not spin_releases or secure is not None:
        tactics.spin.turn = None
    roles, phases = {}, {}
    for team in ("blue", "yellow"):
        phases[team], side = tactics.assign(
            world,
            team,
            legal_off_ball=legal_off_ball,
            audit=allocation_audit,
            strategy=strategies[team] if strategies else BALANCED,
            controller=controllers[team] if controllers else None,
        )
        roles.update(side)
    targets = {} if trajectory else None
    spin_margin = max(0.24, policy.clearance + 0.009) if policy else 0.31
    out, planned = commands(
        world,
        roles,
        weights,
        sole,
        tactics,
        targets,
        receive,
        smooth_carry,
        task_audit,
        strategies,
        motion_configs if practice_pace else None,
        motion_configs if chip_passes else None,
        (motion_configs or dict.fromkeys(phases, CFG))
        if spin_releases and secure is None
        else None,
        measured,
        spin_margin,
        controllers=controllers,
        decisions=decision_audit,
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
                if tactics.spin.turn and tactics.spin.turn.actor == r.key:
                    tactics.spin.turn = None
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
        turning = set()
        if tactics.spin.turn:
            turning.add(tactics.spin.turn.actor)
        rival_in_recovery = None
        if secure is not None and sole not in recovering:
            # Keep the other robot on ordinary clearance while this duel separates.
            out[sole], rival_in_recovery = secure
            planned.pop(sole, None)
            turning.add(sole)
            if task_audit is not None:
                task_audit[sole] = "Pull clear with the recovered ball"
        vetoed = set()
        goal_vetoes = set()
        cover_keepers = {}
        keeper_handoffs = {}
        if carrier and tactics.control == carrier.key:
            for r in world.robots:
                if (
                    roles[r.key].role == "Presser"
                    and r.key not in recovering
                    and r.key != rival_in_recovery
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
                    if isinstance(tackle, TackleSelector) and tackle.action(r.key) in PIVOT_SIDES:
                        turning.add(r.key)
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
            motion = motion_configs[team] if motion_configs else CFG
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
                cfg=motion,
                expand_pace=practice_pace,
                turning=turning,
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
                    cfg=motion
                    if practice_pace
                    else replace(motion, max_speed=min(QUALIFIED_PACE, motion.max_speed)),
                )
            out.update(steered)
    else:
        raw = out
        out, _ = filter_commands(world, out, recovering)
    if tactics.spin.turn:
        key = tactics.spin.turn.actor
        requested, sent = raw[key], out[key]
        if math.hypot(requested.vx - sent.vx, requested.vy - sent.vy) > 0.13:
            tactics.spin.turn = None
            out[key] = replace(sent, kick=0, omega=0)
            planned.pop(key, None)
            if task_audit is not None:
                task_audit[key] = "Brake for clearance"
    return out, planned, roles, phases
