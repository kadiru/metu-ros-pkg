"""Continuous-only contact arbitration; preserved legacy backend remains unchanged.

At most one kicker contacts the ball per physics sample. Of simultaneous eligible
requests, the mouth closest to the ball acts. Exact distance ties defer the kick.
This removes global robot/color iteration order from the contact decision.
Motion and dribbler forces retain the same native ODE/ideal-velocity model.
"""

import math
from .model import Command, bounded, possession, world_vector
from .webots_backend import WebotsBackend


def nearest_kicker(world, commands, cfg, blocked):
    candidates = []
    for r in world.robots:
        c = commands.get(r.key, Command())
        if c.kick > 0 and r.key not in blocked and possession(world, r.key, cfg):
            dx = world.ball.x - r.x - cfg.possession_x * math.cos(r.yaw)
            dy = world.ball.y - r.y - cfg.possession_x * math.sin(r.yaw)
            candidates.append((r.key, dx * dx + dy * dy))
    if not candidates:
        return None, candidates
    distances = sorted(d for _, d in candidates)
    if len(distances) > 1 and distances[1] - distances[0] <= 1e-12:
        return None, candidates
    return min(candidates, key=lambda item: item[1])[0], candidates


class ContinuousWebotsBackend(WebotsBackend):
    contract = "roboturk.webots.closest_kicker.v1"

    def step(self, commands):
        if self.world is None:
            raise RuntimeError("reset must precede step")
        cfg = self.cfg
        commands = {k: bounded(c, cfg) for k, c in commands.items()}
        kicked = set()
        self.last_substep_worlds = []
        self.last_kick_events = []
        self.last_kick_arbitrations = []
        for _ in range(self.substeps):
            current = self.read()
            winner, candidates = nearest_kicker(current, commands, cfg, self.kick_held | kicked)
            if len(candidates) > 1:
                self.last_kick_arbitrations.append(
                    {
                        "time": current.time,
                        "winner": winner,
                        "candidates": candidates,
                        "ball": [current.ball.x, current.ball.y],
                    }
                )
            for r in current.robots:
                node = self.nodes[r.key]
                c = commands.get(r.key, Command())
                vx, vy = world_vector(c.vx, c.vy, r.yaw)
                vertical = node.getVelocity()[2]
                node.setVelocity([vx, vy, vertical, 0.0, 0.0, c.omega])
                if possession(current, r.key, cfg):
                    if r.key == winner:
                        self.ball.setVelocity(
                            [c.kick * math.cos(r.yaw), c.kick * math.sin(r.yaw), 0.0, 0.0, 0.0, 0.0]
                        )
                        kicked.add(r.key)
                        self.last_kick_events.append((r.key, current, c.kick))
                    elif c.dribbler > 0.5 and r.key not in kicked:
                        # Bounded spring/damper attraction at the mouth. Calibrate before transfer.
                        x = r.x + cfg.possession_x * math.cos(r.yaw)
                        y = r.y + cfg.possession_x * math.sin(r.yaw)
                        fx = 8.0 * (x - current.ball.x) + 0.15 * (vx - current.ball.vx)
                        fy = 8.0 * (y - current.ball.y) + 0.15 * (vy - current.ball.vy)
                        norm = max(1.0, math.hypot(fx, fy) / 0.4)
                        self.ball.addForce([fx / norm, fy / norm, 0.0], False)
            if self.supervisor.step(self.basic_ms) == -1:
                self.closed = True
                raise RuntimeError("Webots simulation ended")
            self.world = self.read()
            self.last_substep_worlds.append(self.world)
        self.kick_held = {k for k, c in commands.items() if c.kick > 0}
        return self.world
