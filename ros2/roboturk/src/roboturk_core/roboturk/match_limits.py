"""Drive and kicker limits for an officiated or standalone simulated match."""

from dataclasses import replace
import math

from .model import Config, KICKER_LIMIT, SSL_KICK_LIMIT


PRACTICE_RANGES = {
    "max_speed": (0.5, 5.0),
    "max_omega": (1.0, 16.0),
    "max_acceleration": (0.5, 10.0),
    "max_kick": (0.5, KICKER_LIMIT),
}
TEAMS = ("blue", "yellow")


def team_configs(cfg, *, officiated=False, practice=None):
    base = replace(cfg, max_kick=SSL_KICK_LIMIT) if officiated else cfg
    if practice is None:
        return dict.fromkeys(TEAMS, base)
    if not isinstance(practice, dict) or set(practice) != set(TEAMS):
        raise ValueError("Set motion limits for both teams.")
    configs = {}
    for team, limits in practice.items():
        if not isinstance(limits, dict) or limits.keys() != PRACTICE_RANGES.keys():
            raise ValueError(f"Set all four motion limits for {team.title()}.")
        for name, (low, high) in PRACTICE_RANGES.items():
            value = limits[name]
            if (
                type(value) not in (float, int)
                or not low <= value <= high
                or not math.isfinite(value)
            ):
                raise ValueError(
                    f"{team.title()} {name.replace('_', ' ')} must be {low:g}–{high:g}."
                )
        configs[team] = replace(base, **limits)
    return configs


def limit_values(cfg):
    return {name: getattr(cfg, name) for name in PRACTICE_RANGES}


GAME_LIMITS = limit_values(team_configs(Config(), officiated=True)["blue"])
