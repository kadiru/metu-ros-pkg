from dataclasses import replace
import math

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from roboturk.backend import ReferenceBackend
from roboturk.behavior import AnalyticPolicy, BehaviorLibrary
from roboturk.cli import evaluate
from roboturk.contract import decode_action, encode_action, observe, observation_size
from roboturk.env import SkillEnv, goal_crossed
from roboturk.model import (
    Ball,
    Command,
    Config,
    Intent,
    Referee,
    Robot,
    RobotKey,
    Skill,
    World,
    body_vector,
    safe_command,
    world_vector,
)
from roboturk.strategy import TeamStrategy

CFG = Config()
KEY = RobotKey("blue", 0)


def world(referee=Referee.RUNNING):
    return World(
        0.0,
        (Robot(KEY, 0, 0), Robot(RobotKey("blue", 1), 1, 1)),
        Ball(CFG.possession_x, 0),
        referee,
    )


def test_frames_roundtrip():
    for yaw in (0, 0.7, math.pi, -2.3):
        assert world_vector(*body_vector(0.3, -0.5, yaw), yaw) == pytest.approx((0.3, -0.5))


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_action_rejected(value):
    with pytest.raises(ValueError):
        decode_action([value, 0, 0, 0, 0], CFG)


def test_speed_and_actuator_limits():
    command = decode_action([10, 10, 10, -10, 10], CFG)
    assert math.hypot(command.vx, command.vy) == pytest.approx(CFG.max_speed)
    assert command.omega == CFG.max_omega
    assert command.dribbler == 0
    assert command.kick == CFG.max_kick
    restored = decode_action(encode_action(command, CFG), CFG)
    assert restored.vx == pytest.approx(command.vx)
    assert restored.vy == pytest.approx(command.vy)
    assert restored.omega == command.omega


@pytest.mark.parametrize("referee", [Referee.HALT, Referee.STOP])
def test_referee_overrides_policy(referee):
    assert safe_command(world(referee), KEY, Command(2, 0, 5, 1, 5), CFG) == Command()


@pytest.mark.parametrize("age,refage", [(0.3, 0), (0, 0.6), (-1, 0), (0, float("nan"))])
def test_stale_or_invalid_inputs_stop(age, refage):
    assert safe_command(world(), KEY, Command(1, 0, 0, 1, 3), CFG, age, refage) == Command()


def test_policy_mask_applies_to_learned_plugins():
    class BadActuators:
        def command(self, *args):
            return Command(1, 0, 0, 1, 5)

    library = BehaviorLibrary()
    library.register(Skill.MOVE, BadActuators())
    cmd = library.command(world(), KEY, Intent(Skill.MOVE, (1, 0)), CFG)
    assert cmd.kick == cmd.dribbler == 0


def test_observation_order_padding_and_receiver_mask():
    w = world()
    intent = Intent(Skill.PASS, (1, 1), RobotKey("blue", 1))
    a = observe(w, KEY, intent, CFG)
    b = observe(replace(w, robots=tuple(reversed(w.robots))), KEY, intent, CFG)
    np.testing.assert_array_equal(a, b)
    assert a.shape == (observation_size(CFG),)
    start = 17 + len(Skill)
    assert a[start + 4] == a[start + 5] == 1
    assert not a[start + 6 :].any()


def test_yellow_identity_and_attack_direction():
    w = World(
        0,
        (Robot(KEY, 0, 0), Robot(RobotKey("yellow", 0), 2, 0, math.pi)),
        Ball(1, 0),
        Referee.RUNNING,
    )
    assert w.direction("blue") == 1 and w.direction("yellow") == -1
    assert w.robot(RobotKey("yellow", 0)).x == 2
    assert TeamStrategy("yellow").plan(w).assignments[RobotKey("yellow", 0)].target[0] < 0


def test_goal_requires_full_ball_and_segment_crossing():
    assert not goal_crossed(Ball(4.4, 0), Ball(4.51, 0), CFG)
    assert goal_crossed(Ball(4.4, 0), Ball(4.6, 0), CFG)
    assert not goal_crossed(Ball(4.4, 0.6), Ball(4.6, 0.6), CFG)
    assert not goal_crossed(Ball(4.6, 0), Ball(4.7, 0), CFG)
    assert goal_crossed(Ball(-4.4, 0), Ball(-4.6, 0), CFG, sign=-1)
    assert not goal_crossed(Ball(4.4, 0.8), Ball(4.8, 0), CFG)


@pytest.mark.parametrize("skill", [s for s in Skill if s != Skill.HOLD])
def test_gym_contract_and_seeded_replay(skill):
    env = SkillEnv(skill)
    check_env(env, skip_render_check=True)
    first, _ = env.reset(seed=19)
    result = env.step(np.array([0.1, 0.2, 0.3, 0, 0], np.float32))
    second, _ = env.reset(seed=19)
    again = env.step(np.array([0.1, 0.2, 0.3, 0, 0], np.float32))
    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(result[0], again[0])
    assert result[1:] == again[1:]


@pytest.mark.parametrize("skill", [s for s in Skill if s != Skill.HOLD])
def test_analytic_baselines_complete_synthetic_scenarios(skill):
    result = evaluate(SkillEnv(skill), AnalyticPolicy(), episodes=5, seed=300)
    assert result["success_rate"] == 1.0


def test_timeout_is_truncation_and_step_requires_reset():
    env = SkillEnv(Skill.MOVE, replace(CFG, max_steps=1))
    with pytest.raises(RuntimeError):
        env.step(np.zeros(5))
    env.reset(seed=0)
    _, _, terminated, truncated, info = env.step(np.zeros(5))
    assert truncated and not terminated and info["reason"] == "timeout"
    with pytest.raises(RuntimeError):
        env.step(np.zeros(5))


def test_reset_clears_kick_latch_and_velocity():
    backend = ReferenceBackend()
    backend.reset(world())
    backend.step({KEY: Command(kick=3)})
    assert backend.kick_held == {KEY}
    backend.reset(world())
    assert not backend.kick_held
    assert backend.world.ball.vx == 0


def test_pass_is_latched_and_halt_clears_roles():
    w = world()
    strategy = TeamStrategy()
    plan = strategy.plan(w)
    assert plan.name == "pass_and_receive"
    next_world = replace(w, time=0.2, ball=Ball(0.5, 0.5, 1, 1))
    plan = strategy.plan(next_world)
    assert plan.assignments[KEY].skill == Skill.HOLD
    assert plan.assignments[RobotKey("blue", 1)].skill == Skill.RECEIVE
    assert strategy.plan(replace(w, referee=Referee.HALT)).name == "halt"
    assert strategy.pass_pair is None


def test_collision_is_failure():
    env = SkillEnv(Skill.MOVE)
    env.reset(seed=0)
    active = env.world.robot(KEY)
    env.world = env.backend.reset(
        replace(env.world, robots=(active, Robot(env.receiver, active.x, active.y)))
    )
    _, _, terminated, truncated, info = env.step(np.zeros(5))
    assert terminated and not truncated and info["reason"] == "collision"


def test_passer_aligns_before_kicking():
    w = world()
    target = (1.5, 0.12)
    command = AnalyticPolicy().command(w, KEY, Intent(Skill.PASS, target), CFG)
    assert command.kick == 0
    assert command.omega > 0
