import math

import pytest

from roboturk.five_roles import CFG
from roboturk.match_limits import GAME_LIMITS, limit_values, team_configs
from roboturk.model import Command, Robot, RobotKey, bounded, drive_velocity
from roboturk.timed_motion import Leg


def test_practice_limits_apply_to_each_team_and_game_uses_the_locked_defaults():
    game = team_configs(CFG, officiated=True)
    assert limit_values(game["blue"]) == limit_values(game["yellow"]) == GAME_LIMITS
    assert list(GAME_LIMITS.values()) == [3.0, 8.0, 5.0, 6.5]
    practice = {"blue": dict(GAME_LIMITS, max_kick=10), "yellow": dict(GAME_LIMITS, max_kick=1.5)}
    practice["yellow"].update(max_speed=0.7, max_omega=4.0, max_acceleration=2.5)
    blue, yellow = team_configs(CFG, officiated=True, practice=practice).values()
    assert bounded(Command(kick=99), blue).kick == 10
    robot = Robot(RobotKey("yellow", 1), 0, 0)
    adjusted = bounded(Command(vx=4, vy=4, omega=-20, kick=10), yellow)
    assert math.hypot(adjusted.vx, adjusted.vy) == pytest.approx(0.7)
    assert (adjusted.omega, adjusted.kick) == (-4, 1.5)
    velocity = drive_velocity(robot, adjusted, CFG.dt, yellow)
    assert math.hypot(*velocity) == pytest.approx(CFG.dt * 2.5)
    leg = Leg.to((0, 0), (0, 0), (2, 2), cfg=yellow)
    assert math.hypot(*leg.velocity(CFG.dt)) == pytest.approx(math.hypot(*velocity))
    assert math.hypot(*leg.velocity(1.0)) == pytest.approx(0.7)


@pytest.mark.parametrize("too_fast", [10.1, float("inf"), float("nan"), True, "10"])
def test_practice_rejects_invalid_kick_limits(too_fast):
    practice = {"blue": dict(GAME_LIMITS, max_kick=too_fast), "yellow": GAME_LIMITS}
    with pytest.raises(ValueError, match="Blue max kick"):
        team_configs(CFG, officiated=True, practice=practice)
