"""Regression for Webots' deferred resetPhysics semantics, without requiring its binary."""

from dataclasses import replace

from roboturk.model import Ball, Config, World
from roboturk.webots_backend import WebotsBackend


class Field:
    def __init__(self, node):
        self.node = node

    def setSFVec3f(self, position):
        self.node.position = position


class DeferredPhysicsBall:
    def __init__(self):
        self.position = [0.0, 0.0, 0.0215]
        self.velocity = [0.0] * 6
        self.pending_reset = False

    def getField(self, name):
        assert name == "translation"
        return Field(self)

    def resetPhysics(self):
        self.pending_reset = True

    def setVelocity(self, values):
        self.velocity = list(values)

    def getPosition(self):
        return self.position

    def getVelocity(self):
        return self.velocity


class DeferredPhysicsSupervisor:
    def __init__(self):
        self.ball = DeferredPhysicsBall()
        self.time = 0.0

    def getBasicTimeStep(self):
        return 5

    def getFromDef(self, name):
        assert name == "BALL"
        return self.ball

    def getTime(self):
        return self.time

    def step(self, milliseconds):
        if milliseconds > 0:
            if self.ball.pending_reset:
                self.ball.velocity = [0.0] * 6
                self.ball.pending_reset = False
            self.time += milliseconds / 1000
            self.ball.position[0] += self.ball.velocity[0] * milliseconds / 1000
        return 0


def test_reset_does_not_erase_incoming_ball_velocity():
    supervisor = DeferredPhysicsSupervisor()
    backend = WebotsBackend(supervisor, Config())
    initial = World(0, (), Ball(0, 0, 1.2, 0))
    result = backend.reset(initial)
    assert result.time == 0 and result.ball.vx == 1.2
    result = backend.step({})
    assert result.ball.x > 0
    result = backend.reset(replace(initial, ball=Ball(0, 0)))
    assert result.time == 0 and result.ball.vx == 0
    assert backend.step({}).ball.x == 0
