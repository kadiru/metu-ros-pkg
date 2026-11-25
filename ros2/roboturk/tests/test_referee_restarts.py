from dataclasses import replace
import math

import pytest

from roboturk.five_roles import CFG, kickoff
from roboturk.model import Ball, Referee
from roboturk.placement import KickoffKick, PlacementRestart
from roboturk.ssl_referee import Decision


@pytest.mark.parametrize(
    ("team", "sign"), [("blue", 1), ("yellow", 1), ("blue", -1), ("yellow", -1)]
)
def test_kickoff_uses_one_kicker_with_no_roller(team, sign):
    world = kickoff(12, team, sign, referee_placement=True)
    kicker = min((r for r in world.robots if r.key.team == team), key=lambda r: abs(r.x))
    play = KickoffKick(kicker.key)
    assert abs(kicker.x) == pytest.approx(0.20)
    assert all(math.hypot(r.x, r.y) > 0.59 for r in world.robots if r.key != kicker.key)
    commands = play.commands(world)
    assert list(commands) == [kicker.key] and commands[kicker.key].kick == 0
    assert commands[kicker.key].dribbler == 0

    world = replace(world, ball=Ball(kicker.x + world.direction(team) * CFG.possession_x, 0))
    assert play.commands(world)[kicker.key].kick > 0
    play.observe([world])
    assert not play.second_touch
    other = next(r for r in world.robots if r.key.team != team)
    play.observe([replace(world, ball=Ball(other.x, other.y))])
    assert play.second_touch


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
