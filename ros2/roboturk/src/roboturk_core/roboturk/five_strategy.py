"""Team playbooks for the 5v5 assignment and possession controller."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Playbook:
    name: str
    attack: str
    description: str
    wing_width: float = 0.0
    wing_advance: float = 0.0
    shoot_range: float = 2.45
    pass_pressure: float = 1.32
    receiver_progress: float = 0.75


PLAYBOOKS = {
    "balanced": Playbook("Balanced", "Build the attack", "Keep two outlets and an anchor."),
    "wide": Playbook(
        "Wide build-up",
        "Work the wings",
        "Spread both outlets and look for a pass earlier.",
        wing_width=0.42,
        pass_pressure=1.64,
        receiver_progress=0.48,
    ),
    "direct": Playbook(
        "Direct attack",
        "Drive to goal",
        "Push support forward and attack open shots from farther out.",
        wing_width=-0.16,
        wing_advance=0.40,
        shoot_range=3.0,
        pass_pressure=0.95,
        receiver_progress=1.02,
    ),
}
PASS_PREFERENCES = {-1: "Carry more", 0: "Preset default", 1: "Pass more"}


@dataclass(frozen=True)
class TeamStrategy:
    book: Playbook
    pass_preference: int = 0

    def pass_is_available(self, keeper, pressure, progress):
        return (
            keeper
            or pressure < self.book.pass_pressure + 0.18 * self.pass_preference
            or progress > self.book.receiver_progress - 0.18 * self.pass_preference
        )


BALANCED = TeamStrategy(PLAYBOOKS["balanced"])


def decode_strategies(config=None):
    if config is None:
        return dict.fromkeys(("blue", "yellow"), BALANCED)
    if not isinstance(config, dict) or set(config) != {"blue", "yellow"}:
        raise ValueError("Choose a strategy for both teams.")
    strategies = {}
    for team, entry in config.items():
        if not isinstance(entry, dict) or set(entry) != {"strategy", "pass_preference"}:
            raise ValueError("Choose a strategy and pass preference for each team.")
        name, preference = entry["strategy"], entry["pass_preference"]
        if (
            type(name) is not str
            or name not in PLAYBOOKS
            or type(preference) is not int
            or preference not in PASS_PREFERENCES
        ):
            raise ValueError("Unknown strategy or pass preference.")
        strategies[team] = TeamStrategy(PLAYBOOKS[name], preference)
    return strategies


def describe_play(strategy, phase, *, command="", task=None, in_flight=False):
    if phase == "Attack":
        play = strategy.book.attack
        step = "Receive the pass" if in_flight else task or "Find an outlet"
        reason = strategy.book.description
    elif phase == "Defend":
        play = "Press and cover"
        step = "Cover the pass" if in_flight else "Press the carrier"
        reason = "The opponent controls the ball or has a pass in flight."
    elif phase == "Recover":
        play, step = "Recover possession", "Close down the loose ball"
        reason = "The ball is loose. A chaser approaches while teammates offer outlets."
    elif phase == "Kickoff":
        play = "Kickoff"
        step = (
            "Build the formation" if command.startswith("PREPARE_KICKOFF") else "Release the ball"
        )
        reason = "The referee has awarded a kickoff."
    elif phase == "Restart":
        play = "Free kick"
        step = "Place the ball" if command.startswith("BALL_PLACEMENT") else "Restart play"
        reason = "The referee has awarded a free kick."
    else:
        play, step = "Referee hold", "Wait for the referee"
        reason = "The referee is holding play."
    return dict(
        strategy=strategy.book.name,
        passing=PASS_PREFERENCES[strategy.pass_preference],
        play=play,
        step=step,
        reason=reason,
    )
