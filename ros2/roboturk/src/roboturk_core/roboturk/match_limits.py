"""Drive and kicker limits for an officiated or standalone simulated match."""

from dataclasses import replace
import json
from pathlib import Path

from .model import Config, KICKER_LIMIT, SSL_KICK_LIMIT


PRACTICE_RANGES = {
    "max_speed": (0.5, 5.0),
    "max_omega": (1.0, 16.0),
    "max_acceleration": (0.5, 10.0),
    "max_kick": (0.5, KICKER_LIMIT),
}
TEAMS = ("blue", "yellow")


def validate_limits(limits, team=None):
    title = f"{team.title()} " if team else ""
    if not isinstance(limits, dict) or limits.keys() != PRACTICE_RANGES.keys():
        raise ValueError(f"Set all four {title}motion limits.")
    for name, (low, high) in PRACTICE_RANGES.items():
        value = limits[name]
        if type(value) not in (float, int) or not low <= value <= high:
            label = f"{title}{name.replace('_', ' ')}".capitalize()
            raise ValueError(f"{label} must be {low:g}–{high:g}.")
    return limits


def team_configs(cfg, *, officiated=False, practice=None):
    base = replace(cfg, max_kick=SSL_KICK_LIMIT) if officiated else cfg
    if practice is None:
        return dict.fromkeys(TEAMS, base)
    if not isinstance(practice, dict) or set(practice) != set(TEAMS):
        raise ValueError("Set motion limits for both teams.")
    return {
        team: replace(base, **validate_limits(limits, team)) for team, limits in practice.items()
    }


def shared_configs(cfg, limits):
    return dict.fromkeys(TEAMS, replace(cfg, **validate_limits(limits)))


def limit_values(cfg):
    return {name: getattr(cfg, name) for name in PRACTICE_RANGES}


GAME_LIMITS = limit_values(team_configs(Config(), officiated=True)["blue"])


class PracticeLimitFeed:
    """Read complete snapshots written atomically by the local match launcher."""

    def __init__(self, path):
        self.path = Path(path)
        self.stamp = None
        self.values = None

    def poll(self):
        stat = self.path.stat()
        stamp = (stat.st_ino, stat.st_mtime_ns)
        if stamp == self.stamp:
            return None
        values = validate_limits(json.loads(self.path.read_text()))
        self.stamp = stamp
        if values == self.values:
            return None
        self.values = values
        return values
