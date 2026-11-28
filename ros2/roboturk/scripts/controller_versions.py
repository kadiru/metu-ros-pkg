"""Identify selectable controller releases and the source build used by a match."""

import hashlib
import json
from pathlib import Path
import subprocess

from roboturk.five_strategy import DECISION_CONTROLLERS


ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "docs/controller-versions.json"
MATCH_VERSIONS_FILE = "controller-versions.json"


def load_catalog():
    catalog = json.loads(CATALOG.read_text())
    releases = catalog["releases"]
    for release_id in catalog["installed"]:
        if releases[release_id]["implementation"] not in DECISION_CONTROLLERS:
            raise ValueError(f"Controller release {release_id} has no installed implementation.")
    return catalog


def match_releases(catalog, requested, strategies):
    """Resolve picker IDs or preserve the controller IDs used by older clients."""
    releases = catalog["releases"]
    available = catalog["installed"]
    if requested is None:
        by_implementation = {releases[key]["implementation"]: key for key in available}
        requested = {
            team: by_implementation.get((strategies or {}).get(team, {}).get("controller", "balanced"))
            for team in ("blue", "yellow")
        }
    if not isinstance(requested, dict) or set(requested) != {"blue", "yellow"}:
        raise ValueError("Choose a controller version for both teams.")
    if any(type(key) is not str or key not in available for key in requested.values()):
        raise ValueError("Choose an installed controller version for each team.")
    return {team: {"id": key, **releases[key]} for team, key in requested.items()}


def source_build():
    """Use Git ancestry for the build number and distinguish local working copies."""
    def git(*args):
        return subprocess.check_output(["git", *args], cwd=ROOT, stderr=subprocess.DEVNULL)

    try:
        commit = git("rev-parse", "HEAD").strip().decode("ascii")
        number = int(git("rev-list", "--first-parent", "--count", "HEAD"))
        tracked = git("diff", "--binary", "HEAD", "--", ".")
        untracked = git("ls-files", "--others", "--exclude-standard", "-z", "--", ".")
        delta = None
        if tracked or untracked:
            digest = hashlib.sha256(tracked)
            for path in sorted(untracked.split(b"\0")[:-1]):
                file = ROOT / path.decode(errors="surrogateescape")
                digest.update(path + b"\0")
                contents = str(file.readlink()).encode() if file.is_symlink() else file.read_bytes()
                digest.update(hashlib.sha256(contents).digest())
            delta = digest.hexdigest()
    except (OSError, subprocess.CalledProcessError):
        return None
    return {"number": number, "commit": commit, "working_copy_sha256": delta}


def match_artifacts(spec):
    """Fingerprint the shared world and the policy inputs selected by the match spec."""
    paths = {
        "world": "src/roboturk_ros/webots/worlds/ssl_five_roles.wbt",
        "pass_ranker": "policies/recipient_3v3_v1/selectors.json",
        **{key: spec.get(key) for key in ("open_play_policy", "receive_profile", "tackle_policy")},
    }
    return {
        key: {"path": name, "sha256": hashlib.sha256((ROOT / name).read_bytes()).hexdigest()}
        for key, name in paths.items()
        if name and (ROOT / name).is_file()
    }
