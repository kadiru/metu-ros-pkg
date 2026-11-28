"""Turn a Hoca coaching request into an isolated controller candidate worktree."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


STACK = Path(__file__).resolve().parents[1]
REQUEST_FORMAT = "roboturk.decision_change_request.v1"
LAYERS = ("strategy", "interaction", "behavior")


def git(folder, *args, data=None):
    result = subprocess.run(
        ["git", *args], cwd=folder, input=data, text=True, capture_output=True, check=False
    )
    if result.returncode:
        raise ValueError(result.stderr.strip() or f"Git {args[0]} failed.")
    return result.stdout.strip()


def validate_request(value):
    if not isinstance(value, dict) or value.get("format") != REQUEST_FORMAT:
        raise ValueError("Use a Hoca decision change request.")
    for field, limit in (("title", 100), ("intent", 4000)):
        text = value.get(field)
        if not isinstance(text, str) or not 0 < len(text.strip()) <= limit:
            raise ValueError(f"Enter a {field} for the decision change.")
    if value.get("team") not in ("blue", "yellow"):
        raise ValueError("Choose the team whose decision should change.")
    name = value.get("controller_name", "")
    parent = value.get("controller_base")
    if not isinstance(name, str) or len(name) > 100:
        raise ValueError("Keep the controller name within 100 characters.")
    if parent is not None and (not isinstance(parent, str) or not 0 < len(parent) <= 160):
        raise ValueError("Choose a controller version to build on.")
    checks = value.get("acceptance", "")
    if not isinstance(checks, str) or len(checks) > 4000:
        raise ValueError("Keep the acceptance criteria within 4,000 characters.")
    clip = value.get("evidence")
    if not isinstance(clip, dict) or clip.get("format") != "roboturk.telemetry_clip.v1":
        raise ValueError("Include a telemetry clip as evidence.")
    review, interval, samples, source = (
        clip.get("review"), clip.get("range"), clip.get("samples"), clip.get("source")
    )
    if not all(isinstance(item, dict) for item in (review, interval, source)):
        raise ValueError("Include the recorded decision frame and match source.")
    versions = source.get("controller_versions") or {}
    teams = versions.get("teams", {}) if isinstance(versions, dict) else None
    if not isinstance(teams, dict) or any(not isinstance(release, dict) for release in teams.values()):
        raise ValueError("The clip's recorded controller versions are invalid.")
    tick, start, end = review.get("tick"), interval.get("start_tick"), interval.get("end_tick")
    if (
        any(type(n) is not int for n in (tick, start, end))
        or not 0 <= start <= tick <= end
        or start >= end
        or not isinstance(samples, list)
        or not any(isinstance(sample, dict) and sample.get("tick") == tick for sample in samples)
    ):
        raise ValueError("The exported clip must contain the decision frame.")


def next_version(previous, changed_layers, *, breaks_contract=False):
    """Bump the highest changed layer, with a major bump for contract breaks."""
    if not re.fullmatch(r"(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)", previous):
        raise ValueError("Use a release version such as 1.0.0.")
    if not changed_layers or any(layer not in LAYERS for layer in changed_layers):
        raise ValueError("Choose the layers whose behavior changes.")
    parts = [int(part) for part in previous.split(".")]
    level = 0 if breaks_contract else min(LAYERS.index(layer) for layer in changed_layers)
    parts[level] += 1
    parts[level + 1:] = [0] * (2 - level)
    return ".".join(map(str, parts))


def checkout(repo, directory, commit, branch=None):
    selection = ["-b", branch] if branch else ["--detach"]
    git(repo, "worktree", "add", "--no-checkout", *selection, str(directory), commit)
    sparse = subprocess.run(
        ["git", "config", "--bool", "--get", "core.sparseCheckout"],
        cwd=repo, capture_output=True, text=True, check=False,
    )
    if sparse.stdout.strip() == "true":
        cone = git(repo, "config", "--default", "true", "--bool", "--get", "core.sparseCheckoutCone")
        patterns = git(repo, "sparse-checkout", "list")
        git(
            directory, "sparse-checkout", "set", "--cone" if cone == "true" else "--no-cone",
            "--stdin", data=patterns + "\n",
        )
    git(directory, "read-tree", "-mu", "HEAD")


def create_candidate(request, *, stack=STACK, base="HEAD", workspace=None):
    validate_request(request)
    repo = Path(git(stack, "rev-parse", "--show-toplevel"))
    relative_stack = stack.resolve().relative_to(repo.resolve())
    commit = git(repo, "rev-parse", "--verify", "--end-of-options", f"{base}^{{commit}}")
    encoded = json.dumps(request, sort_keys=True, ensure_ascii=False, allow_nan=False).encode()
    fingerprint = hashlib.sha256(commit.encode() + encoded).hexdigest()
    slug = re.sub(r"[^a-z0-9]+", "-", request["title"].lower()).strip("-")[:42].rstrip("-")
    name = f"{slug or 'decision'}-{fingerprint[:12]}"
    branch = f"candidate/{name}"
    directory = (workspace or (repo.parent / "controller-candidates")).resolve() / name
    record_dir = directory / relative_stack / "docs/controller-changes" / name
    manifest = record_dir / "candidate.json"
    response = {"branch": branch, "worktree": str(directory), "record": str(manifest)}
    if manifest.is_file():
        stored = json.loads(manifest.read_text())
        if stored.get("request_sha256") == fingerprint and git(directory, "branch", "--show-current") == branch:
            return response | {"created": False}
    if directory.exists():
        raise ValueError(f"Candidate workspace already exists without a matching request record: {directory}")
    directory.parent.mkdir(parents=True, exist_ok=True)
    try:
        checkout(repo, directory, commit, branch)
        record_dir.mkdir(parents=True, exist_ok=False)
        (record_dir / "request.json").write_text(json.dumps(request, indent=2, ensure_ascii=False) + "\n")
        source = request["evidence"]["source"]
        teams = (source.get("controller_versions") or {}).get("teams") or {}
        release = teams.get(request["team"], {}).get("id")
        record = {
            "format": "roboturk.controller_candidate.v1",
            "request_sha256": fingerprint,
            "branch": branch,
            "base_commit": commit,
            "recorded_release": release,
            "status": "requested",
            "change_plan": {"layers": [], "changes": [], "dependencies": [], "breaks_contract": None},
            "candidate_version": None,
            "comparisons": [],
        }
        with manifest.open("x") as file:
            file.write(json.dumps(record, indent=2) + "\n")
    except (OSError, ValueError) as error:
        raise ValueError(f"Candidate worktree created at {directory}, but setup is incomplete: {error}") from error
    return response | {"created": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    create = commands.add_parser("create", help="Create the candidate worktree")
    create.add_argument("request", type=Path)
    create.add_argument("--base", default="HEAD", help="Git commit or ref for the candidate")
    create.add_argument("--workspace", type=Path)
    version = commands.add_parser("version", help="Suggest the release number from the finished scope")
    version.add_argument("previous")
    version.add_argument("layers", choices=LAYERS, nargs="+")
    version.add_argument("--breaks-contract", action="store_true")
    args = parser.parse_args()
    try:
        if args.command == "create":
            request = json.loads(args.request.read_text())
            print(json.dumps(create_candidate(request, base=args.base, workspace=args.workspace), indent=2))
        else:
            print(next_version(args.previous, args.layers, breaks_contract=args.breaks_contract))
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
