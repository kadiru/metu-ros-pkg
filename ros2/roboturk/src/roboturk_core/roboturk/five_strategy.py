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
    "balanced": Playbook(
        "Balanced", "Build the attack", "Keep two passing options and one player behind the ball."
    ),
    "wide": Playbook(
        "Wide build-up",
        "Work the wings",
        "Spread teammates wider and look for a pass earlier.",
        wing_width=0.42,
        pass_pressure=1.64,
        receiver_progress=0.48,
    ),
    "direct": Playbook(
        "Direct attack",
        "Drive to goal",
        "Move support forward and take open shots from farther out.",
        wing_width=-0.16,
        wing_advance=0.40,
        shoot_range=3.0,
        pass_pressure=0.95,
        receiver_progress=1.02,
    ),
}
PASS_PREFERENCES = {-1: "Carry more", 0: "Preset default", 1: "Pass more"}
DECISION_CONTROLLERS = {
    "balanced": (
        "Team-wide assignment",
        "Assign roles together to reduce total travel and role changes.",
    ),
    "priority": ("Local Priority", "Fill each role in order with the best available robot."),
    "coordinated": (
        "Coordinated",
        "Position two passing options together and check where defenders could intercept.",
    ),
    "relay-outlet": ("Relay Outlet", "Recycle a crowded ball and reopen two wide passing options."),
}


@dataclass(frozen=True)
class TeamStrategy:
    book: Playbook
    pass_preference: int = 0
    controller: str = "balanced"

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
        required = {"strategy", "pass_preference"}
        if not isinstance(entry, dict) or not required <= entry.keys() <= required | {"controller"}:
            raise ValueError("Choose a strategy and pass preference for each team.")
        name, preference = entry["strategy"], entry["pass_preference"]
        controller = entry.get("controller", "balanced")
        if (
            type(name) is not str
            or name not in PLAYBOOKS
            or type(preference) is not int
            or preference not in PASS_PREFERENCES
        ):
            raise ValueError("Unknown strategy or pass preference.")
        if type(controller) is not str or controller not in DECISION_CONTROLLERS:
            raise ValueError("Unknown decision controller.")
        strategies[team] = TeamStrategy(PLAYBOOKS[name], preference, controller)
    return strategies


def describe_play(
    strategy, phase, *, command="", task=None, in_flight=False, decision=None, interception=None
):
    if phase == "Attack":
        play = strategy.book.attack
        step = "Receive the pass" if in_flight else task or "Find an outlet"
        reason = decision or strategy.book.description
    elif phase == "Defend" and interception:
        play, step = "Intercept the pass", "Meet the ball before the receiver"
        reason = interception.reason()
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
            "Build the formation"
            if command.startswith("PREPARE_KICKOFF")
            else task or "Wait for the kickoff"
        )
        reason = decision or "The referee has awarded a kickoff."
    elif phase == "Restart":
        play = "Free kick"
        step = "Place the ball" if command.startswith("BALL_PLACEMENT") else "Restart play"
        reason = "The referee has awarded a free kick."
    else:
        play, step = "Referee hold", "Wait for the referee"
        reason = "The referee is holding play."
    default = strategy.controller == "balanced"
    return dict(
        strategy=strategy.book.name if default else DECISION_CONTROLLERS[strategy.controller][0],
        passing=PASS_PREFERENCES[strategy.pass_preference] if default else strategy.book.name,
        play=play,
        step=step,
        reason=reason,
    )
