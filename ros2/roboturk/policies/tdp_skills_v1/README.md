# TDP-inspired ball skill pack — frozen for qualification

Three separately trained neural options: timed receive-and-stop, one-touch
redirect to goal, and recover-and-clear to a flank outlet. Each passed60/60
native development cases and30/30fresh coordinator integration cases. Reserved
qualification has not run yet; development is not a reliability claim.

Each128×128MLP uses34observations/four motor-release outputs. Native200episode
imitation,100mixedDAggerepisodes and32768anchoredPPOsteps per skill completed;
all32PPOupdates accepted and actor weights changed for each. BC,DAgger andPPO
all passed60/60development; no isolatedPPO improvement is claimed. One training
initialization per skill, not a cross-seed robustness result.

The explicit scripted play→role→committed option coordinator chooses receive,
redirect or clearance from observable conditions and pass contracts. Motor and
release decisions are neural, without teacher fallback or motion projection.
Tactical intents, eligibility, dribbler enabling and fixed kick power remain
scripted. The previous qualified learned continuous manager is unchanged.

See [source study and orchestration design](../../docs/TDP_SKILL_PACK.md),
[predeclared protocol](protocol.json), [training audit](training-audit.json), and
[immutable runtime/model freeze](freeze.json). Native analytic baselines20/20
per task are teachers, not learned results. All stages, demonstration datasets,
reset/rollout receipts and full development/integration5ms traces remain under
preparation. Large JSON traces use deterministic lossless gzip; transport
manifests record original and packaged SHA256s.

Scope: approximate native Webots ground-truth physics, stationary scripted pass
server for receive/redirect, unguarded redirect goal, static clearance obstacle.
No moving-adversary, complete match, hardware or live ROS-graph qualification.
