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
| Cross-play development |2200200–2200999, declared pair mapping before run|
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
