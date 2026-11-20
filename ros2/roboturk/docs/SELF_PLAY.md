# Mobile defense and alternating self-play v1

November19,2026, America/Los_Angeles. This extends the previous fixed-opponent
release. Results are pending; previous200/200 outcomes do not qualify this task.

## Freedom and observation contracts

Both defensive neural policies can translate in either field direction and rotate;
no anchor-line, goal-line or goal-area position clamp supplies movement commands.
All nearby robots are visible to the actors. Speed limits are0.55–0.85m/s for the
defender and0.8–1.15m/s for the keeper, acceleration2–3.5m/s²; keeper command latency
0.04–0.10s. Position/anchor/geometry features are context, not actuator overrides.
Excursion and field-exit measurements report the space actually used.

The new contracts are mobile_opponent.v1 (45 inputs/three outputs, role-specific
weights), mobile_transfer.v1 (70/seven), mobile_finish.v1 (36/four). Existing weights
initialize the shared inputs; new input columns begin at zero. The transfer actor
controls both attackers. The finisher now also observes the defender. Tactical
receiving points, a farther1.35m finishing setup, open-corner aim selection, measured
stable-reception handoff, fixed kick speeds and dribbler masks remain scripted.
Opponents remain active through handoff, without resets or teacher fallback.

## Training protocol

1. Check analytic mobile-lane interception/pursuit and keeper advancement with
  40 native development plays. Preserve all outcomes, including collisions/timeouts.
2. Bootstrap the four expanded neural actors with200 full-play teacher episodes
  and100 mixed DAgger episodes (80% neural/20% teacher per command), seed217.
  Train60 cloning epochs and40 DAgger epochs; retain all stages and datasets.
3. Alternate defender, keeper, transfer and finisher PPO updates, two generations,
  32,768 steps per actor/generation, training seed227 plus declared role/generation
  offsets. Each episode samples an immutable historical opponent bundle. Attackers
  also face the newly updated defense. Snapshot every generation and keep every
  candidate. No analytic labels or motion fallback enter self-play rollouts.
4. Evaluate the complete cross-play matrix on development seeds. Freeze selected
  models, all runtime inputs and the historical test panel before reserved runs.
  Selection uses only development evidence; opponent quality is never selected
  to make attackers win. Retain a failed generation rather than hide it.

PPO uses native full plays. Defensive terminal reward:+25 interception/save,-25
goal,-40 collision,-5 other outcome, plus a small ball-distance shaping term.
Attacking terminal reward:+35 goal,-25 defensive stop/other failure,-40 collision;
intermediate task rewards are clipped/scaled. A physical contact-distance or modeled
mouth-possession predicate terminates a defensive stop; collisions are not wins.
PPO actor updates are projected toward bootstrap training observations (MSE≤3e-5,
maximum action-mean change≤0.06 per generation), with every accepted/rejected update
and before/after actor-parameter hash retained. This is constrained self-play;
no benefit over imitation is presumed before comparing outcomes.

Transfer learning fast-forwards the frozen finishing policy after reception to
assign a complete-play terminal reward. Finisher training starts from a naturally
completed frozen-policy transfer, retaining failed training prefixes separately.
Defenders learn through both phases. Evaluation always runs the full deployed
PassReceiveScorePlay and independent physical event monitor, including failed passes.

Past-opponent sampling is informed by [Bansal et al., Emergent Complexity via
Multi-Agent Competition](https://arxiv.org/abs/1710.03748); this small alternating
checkpoint pool is not a population-scale replication or an equilibrium claim.

## Seeds and qualification

| Purpose | Seeds |
|---|---|
| Teacher feasibility |2020000–2020039|
| Full-play teacher / mixed DAgger |2000000–2000199 /2010000–2010099|
| PPO episode resets |2100000–2199999 only, sampled by training RNG|
| Bootstrap development |2200000–2200039|
| Generation development |2200100–2200159|
| Cross-play development |2200200–2200219 shared across all nine pairs (bootstrap, generation1, generation2)|
| Reserved final vs final |2300000–2300199|
| Reserved final attackers vs bootstrap defense |2310000–2310099|
| Reserved bootstrap attackers vs final defense |2320000–2320099|
| Fresh consecutive demo |2400000–2400007|

Self-play reliability does not mean attackers must always win. Predeclared full-play
gate:≥95% clean terminal soccer outcomes (whole-ball goal, interception or save),
≤2% robot-collision episodes,≤3% unforced control failures, and no runtime exceptions
or nonfinite actions. Final attackers must score at least20% against final defense;
final defense must stop at least20% of bootstrap attacks. Report the whole cross-play
panel, confidence intervals and all failures. Verify actual pass, stable reception,
retained carry and shot events for every goal; verify physical contact predicates
for stops, and continuous opponent inference/physics monitoring for all episodes.
Report both x/y excursion distributions to demonstrate exercised freedom.

Freeze and publish before the400 reserved panel cases. Never tune on reserved
observations or reuse them to qualify a modified candidate. Record all eight
predeclared demo cases, including defensive wins and failures; no outcome selection.

Ground-truth approximate Webots physics, initial passer possession, ideal body
velocity, ODE contacts, spring dribbler and velocity kicks remain explicit limits.
This is a repeated attack/defend drill, not unrestricted full-match self-play,
perception learning, official SSL rule compliance or hardware qualification.
Earlier releases are reproducible at their original Git commits; their broad
runtime freezes intentionally do not match this new experiment's added source.

## Retained development

The first40-case analytic feasibility test timed out40/40 before any pass. The mobile
defender followed the original fixed receiving point and continuously blocked its
lane. No collisions or possession losses occurred. [All outcomes](../policies/self_play_v1/development/fixed-target-teacher.json).
A target-switching attacking feint will be tested next, without changing defensive
freedom or actuator limits. This is scripted tactic development, not self-play learning.

The first flank-switch revision (2020100–2020119) produced four interceptions and
16timeouts, no receptions. The receiving robot braked too early, allowing the
mobile guard to re-cover the lane. Next development probe2020200–2020209 increases
only attacking off-ball travel to the existing2m/s actuator limit, uses a faster
arrival controller, and raises the fixed pass speed from1.8 to2.5m/s in the new
mobile transfer contract. Both training and deployed command decoders share that
change. Receiving target switches remain scripted; defenders are unchanged.
Core CI exposed an optional-dependency import in the new tests (torch absent in
minimal core CI); the pure Gymnasium match environment is now separated from the
training module so its contract tests do not require PyTorch.

The faster-feint ten-case teacher test reached9/10 receptions: three goals, six
interceptions, one defender collision, zero possession losses. [All outcomes](../policies/self_play_v1/development/quick-feint-teacher.json).
Defender longitudinal excursions exceeded3m and keeper excursions reached0.65m.
Before bootstrap, demonstration labels gained a relative closing-speed limit near
other robots to discourage ramming; no such motion override is used at inference.
Both defensive field exits and teammate collisions are now explicit invalid outcomes.
The98 local tests pass. Native200-demonstration/100-DAgger bootstrap has started;
no trained-policy qualification claim is made yet.

Bootstrap initially stopped before collecting any episode: the finisher source uses128-unit hidden layers while the other actors use192. The expansion loader now preserves each source architecture; four role-specific optional training tests verify identical actor outputs when appended inputs are zero. The original traceback is retained.

The native development matrix runs with `--mode selfplay-panel --episodes 20
--seed 2200200 --output runs/selfplay-v1`, reusing the same20 development scenarios
for every attack/defense snapshot pair. `scripts/stage_self_play.py` retains all
stage models, update logs and unfiltered outcomes, and refuses a frozen release.
`scripts/audit_self_play.py` freezes both final and bootstrap models and all runtime
inputs, then checks each reserved panel without deleting failures. Seven synthetic
unit tampering tests reject missing seeds, false goal geometry, absent contact,
inactive opponents, collisions and changed model bytes; those tests are not native
performance evidence. Core and ROS CI passed on65b1fd61 after the dependency fix.

The completed DAgger development run produced9goals,29defender interceptions,
one invalid pass and one defensive-teammate collision:38/40 clean outcomes,
zero timeouts/field exits/pre-release losses. Evaluation initially stopped on a
premature pass because play memory records applied kicks while the task monitor
rejects invalid kicks. A regression test now verifies that such terminal failures
are retained as outcomes; valid/running state agreement is still asserted. The
unchanged saved models were evaluated on the same development set after that fix.

Before PPO, collision penalties were made dominant over simultaneous contact events,
and per-role checkpoint/progress recovery was added. Historical opponents are checked
by file hashes; resumed phases verify actor hashes and do not repeat completed updates.
The bootstrap initial checkpoint is the full DAgger snapshot, not a weaker defender
selected to increase attacker scoring. Its preserved development remains below the
collision-rate gate on this small40-case set. Reserved evaluation remains untouched.

Generation one completed131072 PPOsteps across four actors. All128updates were
accepted and every actor hash changed. Complete development:20goals,38interceptions,
two defender collisions/60games. Both collisions occurred under pressure after
successful reception. The96.7% clean-outcome rate passes that gate, but3.3%
collisions misses the≤2% gate. Preserve this generation without qualification.

## Reproduction

Use the isolated environment and pinned Webots build in `docs/runtime-manifest.json`.
Initialization takes the previous released four actors; original `.npz` arrays are
large ignored training products, regenerated by the bootstrap command.

```bash
python scripts/run_webots.py --webots /path/to/webots \
  --mode selfplay-bootstrap --seed 217 --output runs/selfplay-v1 --timeout 7200 \
  --checkpoint policies/learned_opponents_play_v1/transfer/policy.zip \
  --shoot-checkpoint policies/learned_opponents_play_v1/finish/policy.zip \
  --defender-checkpoint policies/learned_opponents_v1/defender/policy.zip \
  --keeper-checkpoint policies/learned_opponents_v1/keeper/policy.zip
python scripts/run_webots.py --webots /path/to/webots \
  --mode selfplay-train --steps 32768 --seed 227 \
  --output runs/selfplay-v1 --timeout 7200
python scripts/run_webots.py --webots /path/to/webots \
  --mode selfplay-panel --episodes 20 --seed 2200200 \
  --output runs/selfplay-v1 --timeout 3600
```

Run each command to completion before starting another native simulator. Historical
JSON keeps the original local paths as provenance; freshly regenerated runs write
their own paths. All actual model bytes and manifests are retained in the release.
Training is one seeded run, not independent-seed replication. A changed actor hash
proves parameter updates; it does not by itself prove improved playing strength.

## Development amendment before any reserved evaluation

Generation two again produced20goals,38interceptions and2collisions/60games, at the
same seeds2200143 and2200159. The original update scale did not resolve these
close-range failures. Retain both generations, and add a third four-role generation
with32768steps per role. Only defensive training changes: a robot-spacing reward
cost0.3×clip((0.38−nearest_robot_distance)/0.19,0,1)² per control step; learning
rate3e-5; actor trust limits MSE3e-4 and maximum action-mean change0.15. Original
terminal collision penalty−40 remains dominant. Attacking update settings stay
unchanged. No inference action shield, motion clamp or additional analytic teacher
is introduced. Training remains restricted to2100000–2199999.

The third generation faces all historical snapshots; attackers also face its
updated defenders. Extend the cross-play matrix to4×4 with the same20development
scenarios. Existing raw pair results are hash/seed matched and retained, not
replaced. Last completed generation remains the selection rule. All400reserved
cases and the eight demo cases remain untouched; qualification gates are unchanged.
To continue the existing run, use `ROBOTURK_ROUNDS=3` with the train command above.

## Close-spacing preparation after generation three

The revised third generation completed all128actor updates, but its60development
games still had the same two defender collisions:19goals/39interceptions/2collisions.
The full4×4cross-play matrix is retained. No reserved cases were used.

Before another self-play generation, refit only the two defensive actors on the
retained182419native teacher/DAgger training observations per role. Use current
actor outputs as reference outside0.65m nearest-robot spacing. Inside that region,
relabel only translation using the original training teacher and a conservative
relative closing-speed target,2×max(0,distance−0.28)m/s; three projection passes
and normal actuator speed bounds. Nearby blue velocities are observed; the other
defender uses a stationary forecast because the45-input contract lacks its velocity.
Yaw targets remain the current actor output. This is training-only supervised
preparation, with no deployed action projection or new development observations.

Seed337,80epochs/actor, Adam1e-4,8192uniform+32768near samples/epoch. Defender24763
near examples andkeeper4819. Near label MSE fell0.02127→0.001842 and0.004605→0.001009;
far reference MSE0.001705/0.000447. These are training fit errors, not success rates.
The prepared bundle is evaluated on the existing60development scenarios before
resuming four-role PPO. Its exact parent actors, dataset hashes and losses are
retained in spacing-repair/training.json. Attacking actor hashes are unchanged.

Reproduce preparation with `python -m roboturk.self_play_repair --source
runs/selfplay-v1 --output runs/selfplay-v1/spacing-repair --epochs 80 --seed 337`.
The continuation accepts `ROBOTURK_PREPARED_BUNDLE` pointing to that bundle.json;
it verifies the parent generation and records the preparation separately from PPO.
All original qualification gates and reserved/demo seeds remain unchanged.

The prepared defensive actors produced23goals,32interceptions,4keeper saves and
one defender collision/60development games:59/60clean. Both previously recurring
collision cases were resolved; new failure2200137 is retained. This is development
evidence. Resume generation four with32768PPOsteps for each role and all prior
opponents retained. The prepared actors are explicit initializations; actor hashes
trace both the supervised preparation and subsequent PPO. Last completed generation
remains the selection rule. Extend the matrix to5×5 on the same20development seeds.

Run `ROBOTURK_ROUNDS=4 ROBOTURK_PREPARED_BUNDLE=/absolute/path/to/spacing-repair/bundle.json`
with the selfplay-train command. After training, run broader development panels on
2201000–2201099(final/final),2201100–2201199(final attack/bootstrap defense), and
2201200–2201299(bootstrap attack/final defense) before freezing. These are development
sets; all prior reserved qualification/demo seeds and gates remain unchanged.

Generation four completed128accepted updates, bringing total PPOsteps to524288.
All60full development games ended cleanly:26goals,32interceptions,2keeper saves,
zero collisions/invalid outcomes. Preparation and PPO are separate recorded stages;
the change from59/60 to60/60 is one development comparison, not independent-seed
replication or an isolated causal estimate of PPO benefit. Broader panels and
reserved qualification are still pending.


## Broader development failures and release-timing preparation

Generation four's broader panels failed the unchanged development gates:

| Pair | Goals | Defensive stops | Clean | Collisions | Other failures |
| --- | ---: | ---: | ---: | ---: | ---: |
| Current/current, 2201000–2201099 | 35 | 57 | 92/100 | 4 | 4 |
| Current attack/bootstrap defense, 2201100–2201199 | 27 | 61 | 88/100 | 12 | 0 |
| Bootstrap attack/current defense, 2201200–2201299 | 35 | 54 | 89/100 | 3 | 8 |

The complete5×5×20cross-play matrix is retained. The current/current20-case
cell had7goals,11stops,one timeout andone premature pass. Four broader current-pair
timeouts stalled around1.26m from goal before the scripted parking-phase transition.
Several collisions occurred during late finishing convergence. This disproves a
reliability claim based only on the earlier60development games.

A whole-actor timing refit,80epochs/seed347, is retained as a failed candidate:
2goals/6stops/one field exit/one possession loss over2200200–2200209. Its movement
regressions motivate restricting the next preparation to the neural kick readout.

The next preparation starts again from generation four. Freeze every actor feature
and movement-output weight; fit only action-head row3 and bias3 for transfer and
finish. Transfer labels require stable possession and legal range. Finisher labels
allow a forward ray through the goal mouth from≤1.70m, held ball and bounded yaw
speed, including phase-zero moving releases. These are training-only labels on the
original native datasets, not a runtime kick gate or analytic motion fallback.
80epochs,seed357,Adam3e-4. The finisher dataset contains16557positive held-ball
phase-zero examples. Movement outputs are verified bit-identical over all83193
transfer and99226finisher training observations; unchanged network parameters
also preserve the motion mapping outside that sample. Training classification
errors are recorded and do not establish Webots reliability. Native development
is required before another four-role self-play continuation. Reserved and demo
cases remain untouched; qualification gates remain unchanged.


The conservative kick-readout candidate (seed367,120epochs, negative weight8,
1.60m training range,4130synthetic incomplete-dwell observations) also failed:
18/20clean, zero goals,11interceptions,7keeper saves, one loss andone field exit.
All movement outputs remained identical, but changed release timing altered the
subsequent trajectories. Retain this failure; do not select either kick refit.

Inspection isolated a tactical phase-transition issue: native stopping positions
around1.26m from goal are about9cm beyond the1.35m parking target, while phase zero
required a7cm parking radius. The mobile tactic now uses12cm, retaining possession,
low-speed and aim-angle conditions. It supplies context only; original neural
motion and kick outputs remain responsible for every command. Other finishing
tactics retain7cm. This is an explicit scripted-tactic correction, not an RL gain.

A second defensive preparation starts from generation four's original four actors,
not any kick refit. Train both defenders for100epochs/seed377 on the same native
training data, with0.38m desired clearance and0.80m near-state radius. Attackers
remain byte-equivalent in actor parameters. No runtime projection is added.
Validate on the existing100current/current development cases before generation5.
Since tactical runtime changed, archive the old5×5matrix and rerun cross-play
under a source fingerprint; never reuse an old-runtime result as new evidence.
Reserved/demo cases remain untouched and all original gates remain unchanged.


The first wider-clearance panel returned95/100clean:45goals,47interceptions,
3keeper saves,3defender–keeper collisions and2timeouts. It fails the≤2%collision
gate. A native diagnostic repeat at2201033 reproduced a phase-zero stall with
parking error0.10846m, speed0.01213m/s and aim error−0.05749rad: the remaining
0.05rad transition threshold prevented the precise-finishing phase. The mobile
setup transition now uses0.14m/0.10rad, still requiring possession and speed<0.15m/s;
other tactics retain0.07m/0.05rad. Neural precise aim and kick decisions are unchanged.
This is a scripted-context correction, not a learned policy improvement. Terminal
parking error, aim error and speed are now included in native evidence. Rerun the
same100development cases with identical spacing-v2 actor parameters before PPO.


With the corrected mobile setup transition, the unchanged spacing-v2 bundle
completed100/100clean development games:47goals,44interceptions and9keeper saves,
zero collisions or unforced errors. Both preceding100-case failure panels remain
retained. Continue generation five with32768PPOsteps per role, the spacing-v2
prepared bundle and all five historical snapshots. The preparation and tactical
correction are separate from the forthcoming PPO result; do not attribute their
gain to PPO. Complete cross-play under the new runtime and broader development
before freezing the last completed generation. Reserved splits/gates unchanged.


Generation five completed all four32768-step role jobs. Accepted updates: defender32,
keeper29,transfer32,finisher32; three keeper updates were rejected by the trust-region
check. All four actor hashes changed. Total655360PPOsteps and637accepted/640attempted
updates across20role jobs. Saved-snapshot development60/60clean:21goals,34interceptions,
5keeper saves, no collisions or unforced errors. The training audit verifies retained
checkpoint bytes, initialization/update links and all recorded training seeds.
Broader300-case development and the complete6×6×20matrix run under the revised
runtime. A two-case development movie checks the recorder before the final freeze;
it is not the reserved eight-case demo. All reserved cases remain untouched.


## Generation-five broader development

| Pair | Goals | Defensive stops | Clean | Collisions | Unforced errors | Gate |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Current/current | 38 | 61 | 99/100 | 1 | 0 | Pass |
| Current attack/bootstrap defense | 34 | 60 | 94/100 | 6 | 0 | Fail |
| Bootstrap attack/current defense | 32 | 64 | 96/100 | 1 | 3 | Pass |

The historical-defense panel remains a failed robustness challenge. Five of its
six collisions occur after an applied shot; two involve the original defensive
teammates colliding with one another. These outcomes remain failures under the
unchanged monitor and gates. The current-pair result supports only that pairing's
development reliability, not a claim that all historical matchups are qualified.
Do not infer an isolated PPO benefit from these sequential, differently prepared
candidates. All raw outcomes are retained in artifacts/broader-generation5.

The two-case native recorder smoke test used only existing development seeds:
one interception and one goal,14.36s,359frames,1280×720 at25fps. Full decode and
actual interception/dribble/goal-crossing/display frames were inspected. The native
encoder reported a target-bitrate convergence warning, but completed successfully;
full decoding found no errors. This clip is recorder validation, not the fresh demo.
Full6×6×20cross-play is underway; all reserved tests/demo remain untouched.

## Replaying the frozen policies

After the release is staged, activate the pinned Python environment and run from
`ros2/roboturk`. The four `final` checkpoint paths are portable; absolute paths in
historical training records preserve the original run's provenance.

```sh
python scripts/run_webots.py --webots /path/to/webots \
  --mode selfplay-demo --episodes 8 --seed 2400000 --render \
  --checkpoint policies/self_play_v1/final/transfer/policy.zip \
  --shoot-checkpoint policies/self_play_v1/final/finish/policy.zip \
  --defender-checkpoint policies/self_play_v1/final/defender/policy.zip \
  --keeper-checkpoint policies/self_play_v1/final/keeper/policy.zip \
  --output runs/selfplay-replay --timeout 1800
```

This replays the declared demo scenarios; it is not a new independent evaluation.
For the frozen current-pair panel, use `selfplay-evaluate`,200episodes and seed2300000.
The historical-attack panel uses100episodes/2310000 and the two `bootstrap` defensive
paths; the historical-defense panel uses100episodes/2320000 and the two `bootstrap`
attacking paths. Retain every result. Audit each result with:

```sh
python scripts/audit_self_play.py audit --release policies/self_play_v1 \
  --result /path/to/result.json --split final --output /path/to/audit.json
```

Use the corresponding `attack`, `defense` or `demo` split. A failed gate returns1
while retaining its audit report. Changed models or runtime require a new freeze
and fresh reserved scenarios; do not reuse these seeds to qualify a tuned candidate.
The installed executable, ODE library and controller library hashes are recorded in
`docs/runtime-manifest.json` before the release freeze. Earlier training-runtime
receipts retain their original source hashes rather than claiming this later binary
hash observation occurred before those runs.


## Complete matched-runtime cross-play

All36matchups ran20shared development scenarios (720games), runtime SHA256
`29b2d6bf0e96b068a502b57cb052089859e4f3164c4051d6c0f9a33a5f0eecca`. Rows are attacking snapshots; columns are defensive
snapshots. B is the initial neural bootstrap, G1–G5 are completed self-play generations.
Each cell is **goals / clean outcomes**, both out of20.

| Attack \ Defense | B | G1 | G2 | G3 | G4 | G5 |
| --- | --- | --- | --- | --- | --- | --- |
| B | 2 / 19 | 2 / 19 | 2 / 19 | 4 / 20 | 6 / 19 | 6 / 18 |
| G1 | 3 / 19 | 3 / 18 | 3 / 19 | 4 / 20 | 7 / 20 | 7 / 19 |
| G2 | 4 / 19 | 4 / 19 | 4 / 19 | 4 / 20 | 6 / 20 | 9 / 20 |
| G3 | 3 / 19 | 3 / 19 | 2 / 19 | 3 / 19 | 7 / 20 | 8 / 19 |
| G4 | 2 / 18 | 2 / 18 | 2 / 18 | 2 / 18 | 6 / 19 | 7 / 18 |
| G5 | 3 / 19 | 1 / 18 | 2 / 19 | 3 / 19 | 5 / 19 | 7 / 18 |

The final/final cell has7goals,11defensive stops,one premature pass andone possession
loss:18/20clean,zero collisions. Together with its separate60/60and99/100development
panels, this is mixed development evidence; the small shared set exposes failures
that the larger panel missed. Keep the last completed generation as predeclared;
freeze it before untouched qualification, retain all failures and do not tune on
reserved cases. The original-defense100-case challenge also remains failed.
No claim of monotonically improving playing strength or isolated PPO benefit is made.
