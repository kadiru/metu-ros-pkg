# Continuous learned opponents — protocol v1

November 19, 2026, America/Los_Angeles. The previous keeper milestone's analytic
opponents remain reproducible. This new experiment replaces both motion controllers
with separate neural actors and retrains both attacking actors against frozen opponents.
Results are pending; no reliability claim is made by this protocol.

## Contracts and boundaries

`roboturk.continuous_opponent.v1`: 33 observations, three continuous outputs, 192×192
MLP. Outputs are two translations in the attack-relative field frame and yaw rate.
The same decoder converts to body commands for training and inference. No teacher
fallback is available in the deployed controller. Both opponents remain active through
the reception/scoring handoff, with no physics reset.

Observations include measured robot/ball state, role anchors, physical parameters,
patrol clock, ball crossing projection, holder ray projection and previous applied
action. Geometric forecasts are engineered inputs; this is not end-to-end perception.
Role assignment, the defender's initial line, keeper's goal line, physical limits,
reaction latency and dribbler enable are fixed context. Neural actors choose all
translations and rotation. The attackers retain scripted receiving/aim targets and
handoff; their neural actors control motion, pass/reception and shot timing.

Defender speeds remain 0.25–0.55 m/s. Keeper speeds 0.65–0.95 m/s, acceleration
2–3 m/s², command latency 0.08–0.14 s. These are bounded opponents, not unconstrained
SSL rivals. Simulation remains ground-truth, ideal body velocity, spring dribbler,
and applied velocity kicks in pinned Webots R2025a nightly17/11/2026.

## Sequence and untouched splits

1. Train opponents with native full-play demonstrations, direct incoming-ball
   challenges, mixed DAgger and 32,768 anchored PPO steps per actor (CPU seed127).
   Select by defensive development performance, never by making attackers win.
2. Freeze opponent files/hashes. Evaluate direct challenges on reserved cases.
3. Baseline old attackers, then train transfer and finishing against those fixed
   opponents with 300 demonstrations,150 DAgger episodes and65,536 anchored PPO
   steps per actor (CPU seed137). Preserve every stage and failure.
4. Freeze all four actors and runtime. Qualify the complete play on200 new cases;
   require≥95% goals, zero pre-release possession losses and robot collisions.
   Reserve six consecutive new cases for the native Webots movie. Retain all outcomes.

| Purpose | Seeds |
|---|---|
| Opponent full-play demonstrations |1200000–1200159|
| Defender incoming-pass demonstrations / development / DAgger |1210000–1210299 /1211000–1211059 /1212000–1212149|
| Keeper incoming-shot demonstrations / development / DAgger |1220000–1220299 /1221000–1221059 /1222000–1222149|
| Opponent full-play DAgger |1230000–1230079|
| Opponent full-play development |1240000–1240059|
| Reserved direct defender / keeper challenges |1250000–1250099 /1260000–1260099|
| Attacking transfer demonstrations / development / DAgger |1400000–1400299 /1410000–1410059 /1420000–1420149|
| Finishing demonstrations / development / DAgger |1500000–1500299 /1510000–1510059 /1520000–1520149|
| Complete trained-play development |1640000–1640119|
| Reserved full-play qualification |1650000–1650199|
| Native demo |1660000–1660005|

PPO automatic resets draw from their independently seeded training RNG. Reserved
explicit resets/observations never enter datasets, anchors or candidate selection.

Direct defensive challenges use1.8m/s longitudinal passes and3.5m/s longitudinal
shots (total speed slightly higher with lateral velocity),1.8–2.8m from the line;
initial guard offset≤0.12m. Keeper target y±0.28m, pass target within±0.22m of the
initial lane. A pass interception requires measured contact. A keeper save requires
contact and no goal through at least2s. These are makeable direct challenges, not
attacker feints or a claim that a keeper should save every shot. Compare the neural
actor with its analytic teacher on the same challenge distribution. Planned release
gate:≥90/100 direct challenges for each opponent, separately from attacking success.

No incremental PPO benefit, self-play equilibrium, hardware transfer or broad
competition strength may be inferred merely from a high final success rate.

## Reproduction

Use `scripts/run_webots.py` with the project Python and pinned Webots binary.
Modes: `opponents-train`, `opponents-challenge`, `opponents-attack-train`,
`opponents-evaluate`, `opponents-demo`. Full-play modes require `--checkpoint`
(transfer), `--shoot-checkpoint`, `--defender-checkpoint`, `--keeper-checkpoint`.
The direct challenge role is `ROBOTURK_OPPONENT_ROLE=defender|keeper`.
Training and qualification run sequentially, one native Webots process at a time.

## Development amendment: closer centered finishing

The80 mixed-DAgger full plays completed80/80 transfers but only66/80 goals;
all14 failures were keeper saves, with no pre-release possession losses. This
is development evidence, not frozen-opponent qualification. The old finisher
and all of these failures remain available.

Before attacker training, add the distinct `roboturk.pressure_finish.v1` contract:
the32-input/four-output actor receives a parking target0.70m from goal on the
centerline, retaining the latched lure/opposite-corner plan. This reduces shot
flight distance and changes the open lane geometry. Target choice is scripted;
the neural actor learns carrying, pivoting and release from native demonstrations,
DAgger and PPO against the frozen learned keeper. It is not an opponent nerf.
Qualification/demonstration seed reservations and gates are unchanged.

The pre-training pressure-finisher teacher feasibility check reserves development
seeds1530000–1530039. Stationary/teacher defensive comparisons reuse only the
declared defensive development seeds. They never alter reserved opponent outcomes.


The close-only teacher scored32/40 on1530000–1530039: eight keeper saves and zero
possession losses. This failed development tactic is retained. The revised
`roboturk.pressure_finish.v2` keeps the0.70m centered parking point and selects
between the two goal corners using observed keeper position/velocity and the
predicted shot-line clearance. An already open corner can be shot directly;
a covered corner triggers another aim change. This remains scripted aim selection,
with neural motion and release to be trained. Teacher feasibility development
seeds1530100–1530159 are reserved before that run. Opponent hashes and all final
qualification/demo splits remain unchanged.

The revised open-lane teacher passed60/60 native development cases with zero
pre-release losses (mean8.8777s). These are teacher feasibility results, not the
learned attacker release. Both attacking actors are now being trained against
defender06fd042a and keeper9dea9369; all final full-play seeds remain untouched.
