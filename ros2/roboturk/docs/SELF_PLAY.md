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
