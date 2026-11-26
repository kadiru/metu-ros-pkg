from dataclasses import replace
import math

import pytest

from roboturk.five_decision import CONTROLLERS
from roboturk.five_roles import CFG, kickoff
from roboturk.model import Ball, Command, Referee
from roboturk.placement import KickoffKick, PlacementRestart
from roboturk.ssl_referee import Decision


@pytest.mark.parametrize(
    ("team", "sign"), [("blue", 1), ("yellow", 1), ("blue", -1), ("yellow", -1)]
)
def test_kickoff_plans_a_team_pass_with_one_legal_kicker(team, sign):
    world = kickoff(12, team, sign, referee_placement=True)
    kicker = min((r for r in world.robots if r.key.team == team), key=lambda r: abs(r.x))
    play = KickoffKick(kicker.key, CONTROLLERS["balanced"])
    assert abs(kicker.x) == pytest.approx(0.20)
    assert all(math.hypot(r.x, r.y) > 0.59 for r in world.robots if r.key != kicker.key)
    tasks = {}
    commands, roles, planned = play.commands(world, CFG, tasks)
    assert set(commands) == set(roles) == set(tasks) == {r.key for r in world.robots}
    receiver = play.intent.receiver
    assert receiver.team == team and receiver != kicker.key
    assert roles[receiver].role == "Kickoff receiver"
    assert roles[kicker.key].role == "Kickoff"
    assert not planned and all(
        cmd == Command() for key, cmd in commands.items() if key != kicker.key
    )
    assert commands[kicker.key].kick == 0
    assert commands[kicker.key].dribbler == 0

    aim = math.atan2(play.intent.target[1], play.intent.target[0])
    ready = replace(kicker, x=-0.13 * math.cos(aim), y=-0.13 * math.sin(aim), yaw=aim)
    world = replace(world, robots=tuple(ready if r.key == kicker.key else r for r in world.robots))
    commands, _, planned = play.commands(world, CFG, tasks)
    assert commands[kicker.key].kick > 0
    assert planned[kicker.key].receiver == receiver
    play.origin = (0, 0)
    play.observe([world])
    assert not play.in_play and not play.second_touch
    play.observe([replace(world, ball=Ball(0.06, 0))])
    assert play.in_play and not play.second_touch
    other = next(r for r in world.robots if r.key.team != team)
    play.observe([replace(world, ball=Ball(other.x, other.y))])
    assert play.second_touch


def test_kickoff_waits_when_the_sole_outlet_is_blocked():
    world = kickoff(12, "blue", 1, referee_placement=True)
    kicker = min((r for r in world.robots if r.key.team == "blue"), key=lambda r: abs(r.x))
    mate = next(
        r for r in world.robots if r.key.team == "blue" and r.key not in (kicker.key,) and r.key.id
    )
    rival = next(r for r in world.robots if r.key.team == "yellow")
    world = replace(world, robots=(kicker, mate, replace(rival, x=mate.x / 2, y=mate.y / 2)))
    play = KickoffKick(kicker.key, CONTROLLERS["balanced"])
    commands, _, planned = play.commands(world, CFG, {})
    assert play.intent.receiver is None
    assert not planned and all(cmd == Command() for cmd in commands.values())
    assert not play.timed_out(world.time + 9.96)
    assert play.timed_out(world.time + 10.0)


@pytest.mark.parametrize("spot", [(1.4, 2.8), (-3.5, -2.8), (4.3, 2.8)])
def test_virtual_referee_holds_rollers_and_prepares_free_kick_at_awarded_spot(spot):
    world = replace(kickoff(15, "blue", 1), ball=Ball(-4.55, -2.95))
    restart = PlacementRestart(None, referee_places=True)
    award = Decision(
        mode=Referee.STOP,
        command="BALL_PLACEMENT_BLUE",
        source="test-referee",
        counter=1,
        placement_team="blue",
        placement_target=spot,
    )
    commands, roles = restart.commands(world, award)
    placer = min(
        (r for r in world.robots if r.key.team == "blue" and r.key.id),
        key=lambda r: math.dist((r.x, r.y), spot),
    )
    assert restart.play.target == spot and restart.play.placer == placer.key
    assert all(c.dribbler == c.kick == 0 for c in commands.values())
    assert {r.role for r in roles.values()} == {"Stand clear"}

    world = replace(world, ball=Ball(*spot))
    direct = replace(award, mode=Referee.RUNNING, command="DIRECT_FREE_BLUE", placement_target=None)
    _, roles = restart.commands(world, direct)
    assert roles[placer.key].role == "Pass" and roles[placer.key].target == spot
