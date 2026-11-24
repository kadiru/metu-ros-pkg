from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np

from roboturk.five_play import open_play
from roboturk.five_roles import CFG, FiveRoles, kickoff
from roboturk.model import Ball, Command, RobotKey
from roboturk.open_motion import Steering
from roboturk.pivot_fetch import PivotFetch
from roboturk.tackle import BallSideTackle
from roboturk.tackle_selector import TackleSelector, release_risks

CARRIER = RobotKey("blue", 1)
PRESSER = RobotKey("yellow", 1)


def contest(distance):
    w = kickoff(12, "blue", 1)
    robots = []
    for r in w.robots:
        if r.key == CARRIER:
            robots.append(replace(r, x=0, y=0, yaw=0))
        elif r.key == PRESSER:
            robots.append(replace(r, x=distance, y=0, yaw=math.pi))
        else:
            robots.append(replace(r, x=-2 if r.key.team == "blue" else 2, y=r.key.id * 0.6 - 1.3))
    return replace(w, robots=tuple(robots), ball=Ball(CFG.possession_x, 0))


def test_presser_tackle_only_acts_on_a_live_opponent_carrier():
    w = contest(0.6)
    roles = FiveRoles()
    for _ in range(3):
        roles.observe(w)
    seen = []

    def act(world, presser, carrier, observed, has_ball):
        seen.append((presser.key, carrier.key, has_ball))
        return Command(dribbler=1)

    measured = {r.key: (0, 0) for r in w.robots}
    tackle = SimpleNamespace(command=act)
    policy = Steering(0.48, 0.305, 0.66)
    # The carrier's actual pass selector remains part of the normal entry point.
    weights = {team: np.zeros(6) for team in ("blue", "yellow")}
    open_play(w, roles, weights, measured, policy, touchline=False, tackle=tackle)
    assert seen == [(PRESSER, CARRIER, False)]
    seen.clear()
    loose = replace(w, ball=Ball(-1, -1))
    open_play(loose, roles, weights, measured, policy, touchline=False, tackle=tackle)
    assert not seen


def test_double_grip_grants_one_bounded_window_for_a_tackle():
    w = contest(0.23)
    standard = FiveRoles()
    fitted = FiveRoles()
    for tactics in (standard, fitted):
        tactics.control = CARRIER
        tactics.last_touch = 0
    for _ in range(4):
        standard.observe(w)
        fitted.observe(w, tackling=PRESSER)
    assert standard.contain is not None and fitted.contain is None
    for _ in range(16):
        fitted.observe(w, tackling=PRESSER)
    assert fitted.contain is not None


def test_pivot_waits_for_shared_control_and_can_collect_a_nearby_loose_ball():
    w = contest(0.23)
    actor, carrier = w.robot(PRESSER), w.robot(CARRIER)
    base = Command(vx=0.2, dribbler=1)
    pivot = PivotFetch(SimpleNamespace(command=lambda *args: base))
    for tick in range(7):
        assert pivot.command(replace(w, time=tick * CFG.dt), actor, carrier, {}, True) == base
    turn = pivot.command(replace(w, time=7 * CFG.dt), actor, carrier, {}, True)
    assert turn.omega and turn.dribbler == 1 and not turn.kick
    assert len(pivot.sweeps) == 1
    assert pivot.collect_loose(replace(w, time=8 * CFG.dt), actor) is None
    loose = replace(w, time=8 * CFG.dt, ball=Ball(0.115, 0.06))
    fetch = pivot.collect_loose(loose, actor)
    assert fetch.dribbler == 1 and not fetch.kick
    assert pivot.collect_loose(replace(loose, time=24 * CFG.dt), actor) is None


def test_interrupted_shared_grip_cannot_activate_the_pivot():
    w = contest(0.23)
    actor, carrier = w.robot(PRESSER), w.robot(CARRIER)
    base = Command(vx=0.2, dribbler=1)
    pivot = PivotFetch(SimpleNamespace(command=lambda *args: base))
    for tick in range(20):
        result = pivot.command(replace(w, time=tick * CFG.dt), actor, carrier, {}, tick % 4 != 3)
        assert result == base
    assert not pivot.sweeps


def test_brief_role_interruptions_do_not_reset_attempt_budget():
    w = contest(0.23)
    actor, carrier = w.robot(PRESSER), w.robot(CARRIER)
    base = Command(vx=0.2, dribbler=1)
    pivot = PivotFetch(SimpleNamespace(command=lambda *args: base))
    for start in (0, 12, 24):
        for tick in range(start, start + 9):
            pivot.command(replace(w, time=tick * CFG.dt), actor, carrier, {}, True)
    assert len(pivot.sweeps) == 2
    assert pivot.duels[PRESSER].attempts == 2


def test_tackle_release_veto_mirrors_both_touchlines_and_both_goals():
    for side in (-1, 1):
        ball = Ball(0, 2.90 * side)
        assert release_risks(ball, 1, (0, side)) == ("touchline",)
        assert release_risks(ball, 1, (0, -side)) == ()
        ball = Ball(-4.38 * side, 0.1)
        assert "own_goal" in release_risks(ball, side, (-side, 0))
        assert release_risks(ball, side, (side, 0)) == ()


def test_selector_tries_direct_then_uses_the_inward_pivot_in_a_stalled_duel():
    w = contest(0.23)
    w = replace(
        w,
        ball=replace(w.ball, y=2.9),
        robots=tuple(replace(r, y=2.9) if r.key in (PRESSER, CARRIER) else r for r in w.robots),
    )
    selector = TackleSelector(BallSideTackle(37, 0.108, 0.22, 0))
    observed = {r.key: (0, 0) for r in w.robots}
    for tick in range(8):
        command = selector.command(
            replace(w, time=tick * CFG.dt), w.robot(PRESSER), w.robot(CARRIER), observed, True
        )
        assert selector.action(PRESSER) == ("direct" if tick < 7 else "pivot_right")
        assert command.kick == 0
    assert selector.events[-1]["vetoes"] == {"pivot_left": ("touchline",)}
    drifting = replace(w, time=8 * CFG.dt, ball=replace(w.ball, vy=0.6))
    command = selector.command(drifting, w.robot(PRESSER), w.robot(CARRIER), observed, True)
    assert selector.action(PRESSER) == "contain"
    assert command.dribbler == 0
