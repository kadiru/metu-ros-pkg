# Continuous 2v2 with explicit velocity safety — qualified

**Both independent training seeds passed every reserved gate across 120 native
Webots matches. The fresh uncut three-match demo is complete.**

Two neural actors, trained with seeds 611 and 622, control both colors through
continuous 45-second matches. Seven learned choices select neural recover,
dribble, pass, shoot, press, cover and support heads. Interceptions, misses and
possession losses continue play; goals and ball-outs cause logged referee restarts.
There is no permanent goalkeeper, runtime teacher, scripted tactical target
selection or reception-triggered handoff. Dribbler enable is coupled to release
(enabled except while firing); pass/shot speeds are fixed at2.2/3.5m/s. These
actuator rules are not independently learned.

This is a **hybrid learned controller with analytic translation safety**. A joint
geometric projection adjusts requested translations for robot separation, field
margin and speed. Rotation, dribbler and kick outputs remain untouched. Every
intervention is logged and independently audited. The result does not establish
entirely neural collision avoidance. The [unfiltered v1 attempts](../continuous_play_v1/README.md)
and their failures remain available; no reserved v1 cases were used.

The [v2 protocol](protocol.json) retained every numerical v1 gate with new reserved
seeds. Each training seed passed 60 games against self, historical and other-seed
opponents. Models and runtime were published and remotely verified before evaluation.

## Reserved qualification — passed

The frozen candidate passed every predeclared gate independently for both training
seeds. All 120 consecutive reserved matches completed; no cases were selected,
dropped or rerun. Goals and pass/reception counts include both teams. This is a
functional continuous-play benchmark, not a guaranteed scoring or winning rate.

| Panel | Completed | Goals blue/yellow | Blue wins / draws / yellow wins | Passes | Receptions |
| --- | ---: | --- | --- | ---: | ---: |
|611-self|20/20|6/8|3 / 11 / 6|49|22|
|622-self|20/20|8/5|7 / 9 / 4|50|29|
|611-history|20/20|10/9|8 / 6 / 6|62|22|
|622-history|20/20|2/5|2 / 13 / 5|60|27|
|611-cross|20/20|6/5|6 / 9 / 5|45|23|
|622-cross|20/20|1/8|0 / 13 / 7|39|23|

Totals: **120/120 complete, 73 goals, 305 passes, 146 stable receptions, 140 shots,
74 counterattack matches, zero detected robot contacts and zero field exits**.
Every loose-ball start was acquired. There were 61 draws. Counterattacks mean
at least 0.75 m of controlled forward progress within 10 s after a turnover;
they do not necessarily end in a goal. Contact geometry is sampled every 5 ms.

| Training seed | Goals | Receptions | Counterattack matches | Ticks with safety changes | Mean correction per robot/tick |
| --- | ---: | ---: | ---: | ---: | ---: |
|611|44|67|37|20.26%|0.008250 m/s|
|622|29|79|37|16.44%|0.004409 m/s|

In the two cross-seed panels combined, seed611 won 13, drew 22 and lost 5,
with 14 goals for and 6 against. This describes these held-out pairings only;
it does not establish an isolated benefit from PPO.

The [freeze](freeze.json) was published at
[196ac809](https://github.com/kadiru/metu-ros-pkg/commit/196ac8094ebd1aa3f863c4c3921840aa4e3b7554)
before the reserved cases. All 247 manifest Git blobs were independently verified
against the remote tree. See the [publication receipt](freeze-publication.json),
[complete audit](qualification-audit.json) and [lossless packaging receipt](qualification-packaging.json).
Original native JSON files remain locally under `runs/continuous-v2/qualification-raw`;
the compact published JSON reconstructs those original bytes with Python
`json.dumps(json.loads(published), indent=2) + "\n"`, as verified in the receipt.

## Fresh uncut native Webots demo

[Watch the complete video](../../media/continuous-self-play-demo.mp4).
Three consecutive preselected seeds4610000–4610002 finished **0–0, 1–0, 0–0**:
one goal, seven passes, three stable receptions, five shots and one counterattack
match, with zero detected contacts or field exits. Every outcome is retained.
All seven behavior heads were selected during each match. Safety changed
translation on17.59%of control ticks.

Passing/reception appears around00:18–00:20 and00:30–00:32; the goal is around01:15;
the third match includes another reception around01:38. Both attacking directions
appear. Score, clock, selected behaviors and safety interventions are visible.

The native H.264 file is134.96s,1280×720,25fps,3374frames and12479486bytes. It
contains135simulation seconds at normal speed with native recorder frame rounding.
The delivered video is byte-identical: no reencoding, cuts, outcome selection or
post-match holds. Full decode passed; first/final frames, both match boundaries,
passing, reception, shooting and the goal restart were inspected. Webots logged
a bitrate-convergence warning while encoding; every completed native frame decoded
without errors. [Match records](demo.json), [event audit](demo-audit.json) and
[video provenance](demo-video.json) include hashes and all outcomes.

Video SHA256: `1765d0373c27838cbec1880c45f4a94ed8201e185af667e897c846a264f2a114`.

## Training and provenance

The two seeds share135224native teacher states but have independent initializations
and corrective rollouts. Preparation includes imitation, mixed and pure-neural
DAgger, canonical geometry corrections and precision fitting. Training-only
counterfactual motor labels teach every head on recorded states; they preserve
the original selected teacher command and create no new physics transitions.
All-head fitting uses320epochs followed by160release-balanced epochs per seed.

Categorical historical PPO then completed65536native steps/64accepted updates per
seed, in two generations:131072selected steps/128updates total. All four learner
roles rotate; teammates are frozen latest checkpoints and opponents are sampled
from each seed's historical pool. The other training seed is excluded from gradient
training and development pairings. Only seven behavior-score readouts train;
neural trunks and motor functions stay fixed.

The [independent saved-weight audit](training/decision-audit.json) found109/132
changed deterministic choices for seed611 and158/269 for seed622 on45056recorded
probes per generation. Maximum motor-output delta was zero. Counts compare each
generation separately, not unique states across generations. No isolated PPO
performance benefit is established. Earlier full-motion and Gaussian decision-only
attempts consumed262144native steps and failed development gates; those attempts
remain in v1. Total native PPO work for this continuous milestone was393216steps.

The last generation was selected for each seed; primary611 was specified before
reserved evaluation. Exact launch sources, checkpoint and dataset hashes, updates,
resets, historical pools and development records are published. Large training
arrays remain in ignored local run directories; they are not included in GitHub.

## Development evidence

Paired six-game checks of the initial all-head skill banks:

| Training seed | Unfiltered: goals / contacts / field exits | With safety: goals / contacts / field exits |
| --- | --- | --- |
|611|3 /1.91% /0|1 /0% /0|
|622|3 /5.82% /0|3 /0% /0|

All24matches completed. Filtered panels produced10and12stable receptions and
4and3counterattack matches. Translation changed on2515/13500control ticks for611
and2699/13500for622. Every intervention and event record passed the independent
audits. These small development panels do not establish reliability or a safety
layer improvement in scoring.

Alternative shot releases had only about12%recall on recorded training labels,
although originally selected shot releases had100%recall. An additional160epochs
at3e-5 balance counterfactual pass/shot releases and mouth-contact states. Original
320epoch all-head fits, label receipts and all subsequent stages are retained.

Release-balanced development completed6/6games per seed, with zero contacts or
field exits:611 scored5goals with9receptions;622 scored1goal with9receptions.
These small panels preceded the two completed categorical historical leagues.
All earlier preparations and failed unfiltered attempts remain available.

The selected final generations then passed all fresh development gates across
60 unselected native matches, without using the other seed as an opponent:

| Seed | Completed | Goals | Receptions | Counterattack matches | Contacts / field exits | Ticks with safety changes |
| --- | ---: | ---: | ---: | ---: | --- | ---: |
|611|30/30|35|29|20|0 /0|18.84%|
|622|30/30|13|35|16|0 /0|16.45%|

Every loose-ball start was acquired. All four event/intervention audits passed.
These are development results, not reserved qualification. The last generation
was selected for each seed; primary seed611 was specified before qualification.
Training lineages, exact launch source snapshots, independent decision audit and
all 60 development records are included in the freeze. Large JSON copies remove
formatting whitespace only; original hashes and byte-level verification are in
[json-packaging.json](json-packaging.json).


## Scope and use

Native Webots ODE contacts, ground-truth state, ideal body velocity, spring
dribbler and velocity kicks remain approximate. This qualifies the declared
45-second continuous-play distribution, not official SSL matches, calibrated
hardware, noisy vision/radio or a complete live ROS graph. The core policies run
directly through the Supervisor. All89previously qualified runtime files remain
unchanged. See [architecture, metrics and limits](../../docs/CONTINUOUS_PLAY.md).

To run new cases, use a fresh output path and seeds distinct from the published
qualification/demo sets. For example, from the modern workspace:

```sh
PYTHONPATH=src/roboturk_core ../.venv/bin/python scripts/run_continuous.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode demo --render --velocity-safety \
  --freeze policies/continuous_play_v2/freeze.json \
  --checkpoint policies/continuous_play_v2/seed611/generation2/policy.zip \
  --opponent policies/continuous_play_v2/seed622/generation2/policy.zip \
  --episodes 3 --duration 45 --seed 4700000 --output runs/my-continuous-demo
```

Pinned runtime: Webots R2025a nightly17/11/2026,
commit`e5d1122a3cd59f007ee4b37c8f651b50e7ee83d2`. Qualification used one isolated CPU
environment; no hardware or GPU training was run.
