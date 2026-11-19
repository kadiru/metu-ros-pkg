# Defender + goalkeeper experiment v1

Predeclared November19,2026 (America/Los_Angeles), before new training.

The qualified defended-transfer v3 actor and the existing moving midfield defender
remain unchanged. Add a separate analytic goalkeeper on the goal line, with lateral
speed0.65–0.95m/s, acceleration2–3m/s² and reaction delay0.08–0.14s. Each reset selects
an orientation-tracking or ball-line style. Both predict incoming rolling shots.
The keeper is active throughout passing and scoring. These are bounded opponents,
not free-ranging opponents, self-play or a full official SSL match.

Baseline: frozen previous attackers on development seeds910000–910039. Teacher
baseline uses the same full-play scenarios. New finisher demonstrations900000–900299,
DAgger920000–920149; finisher development910000–910059 and full-play development
910000–910119. Reserve950000–950199 for frozen complete-play qualification and
960000–960005 for six consecutive native Webots demo cases. Gate: ≥95% of200 goals,
zero pre-release possession-loss or collision episodes. Retain every outcome.
Reserved results may not be used to tune a checkpoint claimed qualified on them.

The finisher begins holding the received ball. Standalone training randomizes
attack direction, robot ID, heading, x−0.4..1.1m in the attack frame and either flank
at |y|1.5..2.5m. Carry progress is always measured toward the fixed goal center,
independently of the selected aim corner. The complete play retains the real
transfer-to-finish physics state without a reset.

Draw/finish phase and aim selection are scripted; body motion and release timing
are learned from Webots demonstrations, DAgger and constrained PPO. No analytic
motion fallback is allowed at learned inference. PPO improvement must be measured,
not inferred merely because PPO ran. All training uses one CPU thread.

Additional micro task seed families: block1000000, cover1100000; demonstrations
base..base+199, DAggerbase+20000..base+20099, developmentbase+10000..base+10059,
qualificationbase+50000..base+50199. Their model cards must distinguish physical
saves from geometric lane coverage. These separate learned defenders do not change
the opponent during the attacking experiment.

Development history (all outcomes retained): original opponent implementation had
a sign error in ball-line positioning for negative attacks. Initial baseline1/40
and first teacher11/40 used that implementation. Corrected before training or
qualification; rerun the previous-policy baseline on the corrected implementation.
Fast pivot (3rad/s, proportional gain16) improved standalone teacher to17/20 with
zero possession losses. Testing a closer1.0m parking point and0.43m corner aim.
These are attacker-side changes; corrected opponent parameters remain fixed.

Initial source push54dc0c14 is parented directly to the current GitHub branch, with
authenticated kadiru and verified Kadir Uyanik/noreply author and committer. Legacy
local snapshot history is not uploaded or merged into the personal remote branch.

## Reproduce the bounded experiment

Use the project's isolated Python interpreter and the pinned Webots nightly recorded
in the repository/runtime notes. From the modern source directory:

```sh
../.venv/bin/python scripts/run_webots.py --webots "$WEBOTS_BINARY" \
  --mode keeper-train --steps 65536 --seed 97 --output runs/keeper-v1 --timeout 5400
../.venv/bin/python scripts/run_webots.py --webots "$WEBOTS_BINARY" \
  --mode keeper-evaluate --checkpoint policies/defended_pass_receive_v3/policy.zip \
  --shoot-checkpoint policies/keeper_finish_v1/policy.zip \
  --episodes 120 --seed 910000 --output runs/keeper-v1/full-development --timeout 1800
```

After freezing the chosen checkpoint and source hashes, run complete-play
qualification with200 episodes, seed950000; then native recording with
`--mode keeper-demo --render --episodes 6 --seed 960000`. Preserve the JSON next
to the video, including failures. No best-episode selection.

For separate defensive primitives, use `ROBOTURK_MICRO=block` or `cover` with
`--mode micro-train --steps 32768`, respectively training seed107 or117. Evaluation
uses `--mode micro-evaluate --checkpoint <policy.zip>` and the declared seed family.
Run native Webots jobs sequentially. The launcher owns and bounds its process group.

The cover actor's inference adapter accepts `(target_x,target_y,target_vx,target_vy)`
and measured stable dwell. The block actor uses the same interface with a predicted
crossing point. These tactical contexts are not neural outputs. Fixed defensive
heading and, for block, a fixed longitudinal line are part of the action contract.
`micro_catalog.py` exposes entry/outcome/action/provenance records for later strategy
composition. Catalog membership alone is not a qualification claim.
