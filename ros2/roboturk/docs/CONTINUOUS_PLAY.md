# Continuous 2v2: learned behavior and explicit velocity safety

Qualified November20,2026, America/Los_Angeles: both independent seeds passed all
120reserved matches and all predeclared gates. The fresh uncut demo retains three
matches, scores0–0,1–0,0–0. The earlier qualified initial-possession drills and their
frozen runtimes remain intact. See [v1 retained attempts](../policies/continuous_play_v1/README.md)
and the [current v2 model card](../policies/continuous_play_v2/README.md).

## What runs during a match

Both teams have two robots and opposite goals. Every robot receives 122 measured,
canonical observations: robot and ball poses/velocities, local relative geometry,
mouth contact and stable-control history, its previous behavior, score and clock.
Seed, scenario label, teacher target and future outcome are excluded.

A 256 by 256 MLP emits 35 values: seven behavior scores and seven four-value motor
heads. The selected head supplies body vx/vy, angular velocity and release.
The behaviors are recover, dribble, pass, shoot, press, cover and support. The
same interface serves either color and robot ID. There is no permanent goalkeeper,
runtime teacher, scripted tactical target selection or reception-triggered handoff.
Pass and shot power remain fixed at 2.2 and 3.5 m/s; translation is capped at 1.2 m/s
and rotation at 4 rad/s. The dribbler stays enabled except while firing; its
enable is coupled to the learned release, not an independent learned output.
These actuator rules and limits are not learned.

Interception, loss of possession, missed shots and robot contacts continue play.
Whole-ball goals and ball-outs cause explicit logged referee restarts. These are
simplified rules, not official SSL set pieces. Each match lasts 45 simulation seconds.

## Analytic safety is reported separately

V2 projects all four requested world translations jointly onto separation, field
margin and speed constraints. Rotation, dribbler and kick outputs are untouched.
The constraints use a 0.205 m robot-center separation, field-center bounds 4.35 m by
2.85 m, barrier gain 4 per second and maximum 1.2 m/s speed. NumPy Dykstra projection
finds the closest feasible joint velocity; failure to converge raises an error.
This is a geometric safety controller, not a learned collision-avoidance skill.

Every modified control tick records the measured poses, requested/applied world
velocities, correction magnitudes and iteration count. The independent auditor
recomputes pair and field constraints from those records. Intervention rates must
be reported alongside actual native contacts, goals and possession. V1's original
unfiltered protocol stays unchanged; v2 has a separate published protocol and new
reserved seeds. A filtered result cannot establish entirely neural collision avoidance.

Projection runs at 50 Hz. Native contacts and field excursions are measured every
5 ms. The model still uses ground-truth state, ideal body velocity, ODE contacts,
a spring dribbler and velocity kicks. It is not a calibrated wheel/roller/kicker
model or a hardware safety guarantee.

The continuous-only backend also fixes simultaneous kick ordering. At a physics
sample, at most one eligible kicker acts: the mouth closest to the ball wins;
exact distance ties defer the kick. Later ODE motion can break ties numerically.
Eight native mirrored/color-swapped preflights passed. Actual kick timestamps,
contender geometry and arbitration decisions are recorded and audited. Earlier
frozen backends were not changed.

## Training and provenance

Independent seeds 611 and 622 share the original native teacher demonstration set
but have separate initializations and corrective rollouts. Primary seed 611 was
chosen before evaluation. Preparation includes imitation, mixed DAgger, pure-neural
DAgger, canonical-observation corrections and precision fitting. Native states,
original labels, failed checkpoints and launch-source snapshots are retained.

Training-only counterfactual labels now teach every motor head at recorded states,
including alternatives the original teacher did not select. They preserve the
original selected command exactly and create no new physics transitions. Fitting
balances behavior classes, close robots, field margins, similar ball distances,
pass/shot release examples and mouth-contact states. This makes exploratory
behavior choices executable without a runtime fallback.

The two earlier Gaussian PPO attempts completed 262,144 native steps in total and
failed development gates. Full-motion updates disturbed motor accuracy; freezing
motion preserved it but changed almost no deterministic choices. They remain
published as failed development, with no isolated PPO improvement claim.

V2 categorical PPO optimizes the probability of the chosen behavior directly.
The first seven score readouts train; the actor trunk and every motor-head weight
stay fixed and are verified by hashes. Sampling uses categorical logits equal to
three times the learned scores. Deterministic inference selects their argmax and
emits a bounded mode code, preventing Box clipping from creating score ties.
The raw network means remain available for the measured anchor trust region.

A training match selects one learner robot from shuffled cycles of all four
color/ID assignments. Its teammate is the latest frozen snapshot; opponents are
sampled from retained historical snapshots. Only the learner receives PPO actions.
Goal difference is rewarded; contacts, field excursions and the learner's safety
corrections are penalized. Acquisition, reception, possession and progress provide
smaller shaping terms. Every reset, opponent hash, update decision and changed
actor/decision hash is retained. Each selected league completed two32,768-step
generations per seed (131,072native steps/128accepted updates total). The final cross-seed opponent is excluded from gradient training.

## Evidence and reserved evaluation

Stable acquisition requires 0.3 s of exclusive mouth contact at every5 ms sample,
with relative ball/robot speed below 0.25 m/s at every 20 ms control endpoint. Raw
exclusive mouth occupancy is also reported separately as possession seconds.
A reception needs a real teammate pass, a different stable receiver within 5 s,
at least 0.5 m ball displacement and no intervening opponent acquisition. A loose
interval of at least 0.2 s can produce a recovery, including the same robot's recovery.

A counterattack requires stable team control after a turnover to advance the ball
at least 0.75 m toward goal within 10 s, before another loss, goal, ball-out or restart.
Support and cover distances are separate. Only completed matches receive W/D/L;
unfinished games are explicit. Goals, passes and receptions include both sides;
reliability gates test functional play, not a guaranteed win rate against a peer.

The [v2 protocol](../policies/continuous_play_v2/protocol.json) fixes six 20-match
panels, 60 games per independent seed: self, historical and cross-seed opponents.
Reserved starts are 4510000, 4510100, 4510200, 4510300, 4510400 and 4510500. The gates
are unchanged from v1: at least 95% completion and 90% loose-start acquisition;
at most 2% mean contact time and 5% field-exit matches; at least 20 goals, 5 per color,
20 passes, 15 receptions, 20 shots, 10 counterattack matches and 1 m mean support distance
per color. Every event, intervention and hash audit must pass.

Freeze and publish the runtime, all selected checkpoints and the protocol before
using reserved seeds. Run every panel once, retain every outcome, and do not select
models on reserved results. The fresh demo is three consecutive uncut 45 s matches,
seeds 4610000–4610002, 611 final against 622 final. Native video timing, every frame,
model hashes and match records must be verified. Previous camera preflights and
failed development clips are not qualified demos.

The reserved result contains73goals,305passes,146receptions,140shots and74counterattack
matches, with zero detected contacts or field exits. Every loose-ball start was
acquired. Safety adjusted translation on20.26%of ticks for seed611 and16.44%for622.
The [complete audit](../policies/continuous_play_v2/qualification-audit.json) and
[uncut native video](../media/continuous-self-play-demo.mp4) retain every outcome.
There were61reserved draws; these results do not establish a guaranteed scoring
rate or isolated PPO improvement.

## Scope still outstanding

The native Supervisor runs core policies directly. A complete live ROS/Webots
graph, official SSL vision/referee integration, perception and radio latency,
calibrated drive/dribbler/kicker dynamics and hardware transfer remain separate
work. This experiment does not establish full SSL match compliance.
