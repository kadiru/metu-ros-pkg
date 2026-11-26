"""Evaluate learned options through observable play contracts and commitments."""

import math
from .tdp_playbook import TdpPlaybook, ReceptionContract


class OrchestratedActor:
    def __init__(self, actor):
        self.actor = actor
        self.model = actor.model
        self.book = None
        self.neural_calls = 0
        self.binding_error = None

    def action(self, env):
        if env.steps == 0:
            self.book = TdpPlaybook()
            self.neural_calls = 0
            self.binding_error = None
            plan = None
            # An announced teammate delivery is an explicit tactical input.
            if any(r.key == env.server for r in env.world.robots):
                source = (env.world.ball.x, env.world.ball.y)
                distance = math.dist(source, env.intent.point)
                plan = ReceptionContract(
                    f"pass:{env.seed_value}",
                    source,
                    env.intent.point,
                    env.world.time + distance / env.pass_speed,
                    env.world.time + 8,
                )
            assignment = self.book.choose(env.world, env.actor.team, [env.actor], plan)
            if assignment is None or assignment.intent.kind != env.kind:
                self.binding_error = "Coordinator could not bind the expected skill contract"
            else:
                env.intent = assignment.intent
        if self.binding_error:
            return [0.0, 0.0, 0.0, -0.8]
        assignment = self.book.choose(env.world, env.actor.team, [env.actor])
        if assignment is None:
            return [0.0, 0.0, 0.0, -0.8]
        self.neural_calls += 1
        return self.actor.action(env)


def evaluate(env, actor, count, seed, recorder=None, orchestrated=False):
    runner = OrchestratedActor(actor) if orchestrated else actor
    episodes = []
    for i in range(count):
        env.reset(seed=seed + i)
        if recorder:
            recorder.begin(i, env)
        calls = 0
        while True:
            action = runner.action(env)
            calls += 1
            _, _, done, truncated, info = env.step(action)
            if recorder:
                recorder.step(env, info)
            if done or truncated:
                info["neural_calls"] = runner.neural_calls if orchestrated else calls
                info["decision_calls"] = calls
                if orchestrated:
                    runner.book.finish(
                        env.world,
                        "success" if info["is_success"] else "timeout" if truncated else "failure",
                    )
                    info["orchestration_events"] = runner.book.events
                    info["orchestration_error"] = runner.binding_error
                episodes.append(info)
                if recorder:
                    recorder.end(env, info)
                break
        if (i + 1) % 20 == 0:
            print(env.kind, i + 1, count, sum(e["is_success"] for e in episodes), flush=True)
    return {
        "kind": env.kind,
        "control": "neural",
        "backend": env.backend.name,
        "orchestration": "scripted_tdp_playbook.v1"
        if orchestrated
        else "direct_goal_conditioned_option",
        "episodes": episodes,
        "successes": sum(e["is_success"] for e in episodes),
        "count": count,
    }
