# Keeper-aware finishing v1

The frozen two-attacker play scored **199/200 reserved native Webots cases (99.5%)**
against the moving midfield defender and reactive goalkeeper. There were **zero
pre-release possession losses or robot collisions**. One pass was intercepted
(seed 950134); all 199 successful receptions ended in goals. Every outcome remains
in [qualification.json](qualification.json). Wilson 95% interval: 97.22–99.91% for
this distribution. The predeclared ≥95% / zero-loss / zero-collision gate passed.

| Measurement | Result |
|---|---:|
| Complete-play development | 120/120 |
| Reserved complete-play qualification | 199/200 |
| Old scorer against corrected keeper, same first 40 development cases | 0/40; 40 keeper saves |
| Mean successful complete sequence | 14.943s |
| Mean ball displacement to reception | 2.826m |
| Mean retained receiver carry | 3.490m |
| Possession checks at 5 ms physics substeps | 472,364 |
| Opponent checks at 5 ms physics substeps | 594,988 |
| Keeper saves after a completed reception | 0 |
| Minimum measured ball-to-keeper center distance during finishing | 0.215m |

Qualification directions:91/91 toward −X,108/109 toward +X. Passer IDs:100/101 for ID0,
99/99 for ID1. Keeper styles:107/107 aim-tracking,92/93 ball-line. The one failure was
a transfer-phase interception, before the finisher ran. It remains a limitation of
the composed play; the new qualification is not a perfect-success claim.

[Freeze manifest](freeze.json) written 04:19:46 Pacific, November 19, 2026, then
published at `be02c6fe` before qualification. No subsequent policy or core runtime
changes. Reserved seeds 950000–950199; six consecutive demo seeds 960000–960005.
The [native Webots demo](../../media/keeper-pass-receive-score-demo.mp4) scored **6/6** fresh cases, with every outcome retained. Duration99.16s,1280×720 at25fps; all2,479 native frames preserved through H.264 encoding.

## Learned controls and scripted tactics

The frozen [defended-transfer v3](../defended_pass_receive_v3/README.md) controls both
attackers' movement, pass timing and reception. The passer starts holding the ball.
A scripted handoff occurs after 0.4 s measured stable reception, without resetting
physics. The new 32-input/four-output 128×128 MLP then controls body vx/vy, rotation
and shot release. No teacher fallback or action shield runs during learned inference.

The finishing tactic selects a parking point 1.0 m from the goal line and 0.18 m to the
ball's initial side, initially aiming toward that side's 0.43 m goal corner. Once
settled, it latches the opposite corner when the keeper moves toward the lure, or
after 1.2 s settled dwell for a ball-line keeper. These target/phase choices are
scripted. Kick speed 3.5 m/s and the dribbler mask are fixed interface rules. Carry
progress is always measured to the fixed goal center, independently of aim changes.

The keeper is an independent analytic opponent: lateral speed 0.65–0.95m/s,
acceleration 2–3m/s², reaction delay 0.08–0.14s, randomized aim-tracking or ball-line
style. Both styles predict incoming rolling shots. The existing midfield lane
defender remains unchanged and active throughout scoring. Neither opponent is
claimed to be learned, free-ranging or adversarial self-play.

## Training and retained evidence

300/300 native demonstrations and 150/150 mixed DAgger episodes scored. Both cloned
and refitted actors scored 60/60 development episodes. The faster clone was selected
for 65,536 anchored PPO steps; all 64 actor updates passed the training-observation
trust bounds. Final standalone development: 60/60, zero losses, mean 8.823 s. No
success-rate improvement over imitation was observed; this is one training seed,
not independent training-seed replication.

A None-seed automatic-reset bug interrupted PPO before its first update. The fix
has deterministic seeded-then-unseeded replay coverage. Refinement resumed from
saved BC and the retained dataset; [original traceback](training-interruption.txt)
is preserved. Initial failed baselines and the original keeper coordinate-sign bug
are documented in the [protocol](../../docs/KEEPER_TRAINING.md). Source upload
truncation was separately caught by CI and repaired with hash-verified Git blobs;
local training files were unaffected.

Teacher seeds 900000–900299; DAgger 920000–920149; standalone development 910000–910059;
complete-play development 910000–910119. No reserved observations entered training.
Large datasets/logs remain in ignored `runs/keeper-v1`; all stage checkpoints and
per-episode results are retained here. CPU training seed 97, one Torch thread.

- Finisher SHA256:`7252f42b80f71f1578b238d28f288544ab38a5344660c7ac7106233a278e0be7`
- Frozen transfer SHA256:`7e3cf3125d10a532716ca5b621f436ef22278541504bc28f99063d7c580f7d5c`
- Shared inference: `FinishCheckpoint` and `PassReceiveScorePlay`.
- [Team-paper provenance](../../docs/BEHAVIOR_SOURCES.md): ER-Force's keeper/aim
  analysis inspires the tactic; our neural control implementation is separate.

## Scope

Ground-truth observations, a 9×6 m field and 1 m goal mouth, approximate Webots ideal
body velocity, ODE contacts, spring dribbler and edge-triggered velocity kicks.
Success requires a real pass, ≥1 m travel, stable reception, ≥0.5 m retained carry and
an actual shot from ≤1.8 m followed by a whole-ball crossing inside the posts.
Any opponent touch fails that episode. No calibrated wheels/rollers/kicker,
vision/radio latency, hardware or official-match qualification. Native testing uses
the core play through the Supervisor; live ROS/Webots applied-kick feedback and
full graph integration still require separate validation.
