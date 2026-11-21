from dataclasses import replace
import math
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env
from roboturk.model import Ball, Config, Intent, Robot, RobotKey, Skill, World
from roboturk.shoot import ShootEnv
from roboturk.shoot_contract import shoot_action, shoot_command, shoot_observation


def test_shoot_contract_seed_replay():
    env = ShootEnv()
    check_env(env, skip_render_check=True)
    first, _ = env.reset(seed=33)
    a = env.step([0, 0, 0, -1])
    second, _ = env.reset(seed=33)
    b = env.step([0, 0, 0, -1])
    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(a[0], b[0])
    assert a[1:] == b[1:]
    assert env.intent.target == (env.world.attack_sign * 4.5, 0)


def test_shoot_actuator_semantics_and_nonfinite():
    cfg = Config()
    retain = shoot_command([0.1, 0.2, 0.3, -0.001], cfg)
    fire = shoot_command([0, 0, 0, 0.001], cfg)
    assert retain.dribbler == 1 and retain.kick == 0
    assert fire.dribbler == 0 and fire.kick == 3.5
    assert shoot_command(shoot_action(fire, cfg), cfg) == fire
    with pytest.raises(ValueError):
        shoot_command([0, 0, 0, float("nan")], cfg)


def test_goal_conditioned_shoot_observation_rotation():
    cfg, key = Config(), RobotKey("blue", 0)
    a = World(0, (Robot(key, 1, 1, 0, 0.2, 0.1),), Ball(1.7, 1.3, 0.3, 0.1))
    b = World(0, (Robot(key, -1, 1, math.pi / 2, -0.1, 0.2),), Ball(-1.3, 1.7, -0.1, 0.3))
    np.testing.assert_allclose(
        shoot_observation(a, key, Intent(Skill.SHOOT, (4.5, 0)), cfg),
        shoot_observation(b, key, Intent(Skill.SHOOT, (0, 4.5)), cfg),
        atol=1e-6,
    )


def test_immediate_shot_cannot_skip_capture_and_carry():
    env = ShootEnv()
    env.reset(seed=0)
    env.intent = Intent(Skill.SHOOT, (4.5, 0))
    env.world = env.backend.reset(
        replace(env.world, attack_sign=1, robots=(Robot(env.key, 3.2, 0),), ball=Ball(3.315, 0))
    )
    _, _, done, _, info = env.step([0, 0, 0, 1])
    assert done and not info["is_success"] and info["reason"] == "invalid_shot"
    assert info["applied_kicks"] == 1


@pytest.mark.parametrize("sign", [-1, 1])
def test_success_requires_full_ball_crossing_inside_goal(sign):
    env = ShootEnv()
    env.reset(seed=0)
    env.intent = Intent(Skill.SHOOT, (sign * 4.5, 0))
    env.world = env.backend.reset(
        replace(
            env.world,
            attack_sign=sign,
            robots=(Robot(env.key, sign * 3.2, 0, 0 if sign == 1 else math.pi),),
            ball=Ball(sign * 3.315, 0),
        )
    )
    env.acquired, env.capture_step = True, 20
    env.capture_distance = 3.0
    _, _, done, _, info = env.step([0, 0, 0, 1])
    assert not done and info["shot"] and info["applied_kicks"] == 1
    while not done:
        _, _, done, _, info = env.step([0, 0, 0, 1])
    assert info["is_success"] and info["reason"] == "goal"
    assert env.world.ball.x * sign >= 4.5 + env.cfg.ball_radius
    assert info["lost_possession_checks"] == 0


def test_pushing_across_line_without_valid_shot_is_not_a_goal(monkeypatch):
    env = ShootEnv()
    env.reset(seed=0)
    env.intent = Intent(Skill.SHOOT, (4.5, 0))
    previous = replace(env.world, attack_sign=1, ball=Ball(4.51, 0))
    current = replace(previous, ball=Ball(4.54, 0))
    env.world = previous
    monkeypatch.setattr(env.backend, "step", lambda _: current)
    env.backend.last_substep_worlds = [current]
    env.backend.last_kick_events = []
    _, _, done, _, info = env.step([0, 0, 0, -1])
    assert done and not info["is_success"]


def test_possession_loss_before_shot_is_not_allowed(monkeypatch):
    env = ShootEnv()
    env.reset(seed=0)
    held = replace(env.world, robots=(Robot(env.key, 0, 0),), ball=Ball(0.115, 0))
    env.world = held
    env.acquired, env.capture_step = True, 20
    env.backend.last_substep_worlds = [held, replace(held, ball=Ball(0.3, 0)), held]
    env.backend.last_kick_events = []
    monkeypatch.setattr(env.backend, "step", lambda _: held)
    _, _, done, _, info = env.step([0, 0, 0, -1])
    assert done and info["reason"] == "lost_possession" and info["lost_possession_checks"] == 1
