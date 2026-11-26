"""A live ball-winning and counterattack policy for ordinary three-a-side matches."""

from .finish_touch import CleanRecoveryGame
from .model import possession
from .three_match import CFG, face
from .three_recipient import move


class OpenPlayGame(CleanRecoveryGame):
    def reset(self, team, reason):
        # The ancestor's "start" layout seeds a dual-mouth contest. Use its ordinary
        # restart formation on the very first kickoff as well as subsequent restarts.
        super().reset(team, "kickoff" if reason == "start" else reason)
        self.events[-1]["event"] = reason
        self.terminal_restart = reason != "start"
        self.last_control_team = None
        self.control_key = None
        self.control_age = 0

    def observe_recovery(self):
        skip = self.skip_frames
        super().observe_recovery()
        if skip:
            return
        for j, world in enumerate(self.backend.last_substep_worlds):
            held = [r.key for r in world.robots if possession(world, r.key, CFG)]
            key = held[0] if len(held) == 1 else None
            self.control_age = self.control_age + 1 if key == self.control_key else 1
            self.control_key = key
            if key is None or self.control_age != 20:
                continue
            previous = self.last_control_team
            self.last_control_team = key.team
            if previous and previous != key.team and "counter_press" in self.teams[key.team]:
                self.active = (key.team, (self.tick - 1) * 4 + j)
                self.option = None

    def formation(self, out, held):
        super().formation(out, held)
        world = self.world
        ball = world.ball
        for team, policy in self.teams.items():
            if "counter_press" not in policy or any(k.team == team for k in held):
                continue
            if self.flight and self.flight["team"] == team:
                continue
            # Leave the learned shared-ball maneuver in charge once the jaws meet.
            px = ball.x + policy["anticipate"] * ball.vx
            py = ball.y + policy["anticipate"] * ball.vy
            players = [r for r in world.robots if r.key.team == team]
            chaser = min(players, key=lambda r: (r.x - px) ** 2 + (r.y - py) ** 2)
            if chaser.key in self.contestants:
                continue
            heading = face(chaser, ball)
            out[chaser.key] = move(chaser, px, py, heading, speed=policy["counter_press"])
