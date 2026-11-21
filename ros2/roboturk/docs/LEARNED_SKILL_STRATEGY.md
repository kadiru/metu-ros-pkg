# Learned strategy over ten soccer skills

Work in progress, November 20, 2026. This integrates the qualified receive-stop,
one-touch redirect and recover-clear actors into continuous 2v2. Earlier frozen
releases remain unchanged. No new learned-strategy qualification is claimed yet.

The shared selector observes 162 measurements/state features and produces 17
deployed values: ten behavior scores, six target parameters and a target-refresh
score. The revised actor internally predicts 31 values: ten choice logits and
three separate seven-value intent heads. The neural choice selects its matching
intent head, so receiving, redirecting and clearing have independent target
functions. All three target functions stay fixed during decision-head PPO.
The behavior choices are recover, dribble, pass, shoot, press, cover, support,
receive, redirect and clear. Four neural selectors run every 20 ms. Each robot may
change responsibilities; there is no permanent scripted keeper or attacker.

Seven prior neural motor heads and the three TDP actors are reused as a frozen
motor bank. The new selector learns when to enter/leave a skill and where to aim
its intent. Intent parameters latch at entry and change only when the neural
refresh output requests it. An actual teammate kick can change the observed
trajectory; refreshing from learned outputs avoids relying on a perfect pass.
Targets are prepared with explicit native demonstration/DAgger supervision, then
kept fixed during categorical PPO updates of the behavior readouts. The scripted
TDP playbook is not called at inference. Fixed actuator limits/kick power and the
previous joint translation safety projection remain explicit, with interventions
recorded. This is a hybrid learned controller, not learned collision avoidance.

Native preparation includes open play, forward-reception, backpass/redirect and
defensive-clearance starting formations. Team colors, IDs, directions and geometry
vary. All four robots use neural motor policies and can move from the first tick;
there is no stationary pass server or frozen opponent fixture. Referee restarts
after goals/ball-outs use ordinary loose-ball formations. Skill outcomes do not
terminate a match. Training-only teachers provide initialization labels and are
not evidence of a learned strategic result.

[Protocol](../policies/strategy_self_play_v1/protocol.json) declares two independent
training seeds, retained historical opponents, 120 reserved matches and four
fresh uncut demo matches. Models/runtime must be published and verified before
reserved/demo seeds run. Metrics distinguish merely selecting a new skill from
actual stopped reception, a contact-timed redirect and whole-ball goal, or a
controlled defensive clearance through an outlet gate. Failed preparation and
all reserved outcomes remain available.

The task is approximate ground-truth Webots simulation with ideal body velocity,
spring dribbling, velocity kicks and simplified restarts. Full SSL rules, noisy
perception/radio, hardware and a complete live ROS graph remain separate work.
Paper provenance for the new skills is in [the TDP study](TDP_SKILL_PACK.md).

### Retained first preparation and loss correction

The first preparation used 48 native teacher matches, two independent cloned
selectors, 24 mixed DAgger matches per seed (80% student decisions), and refits.
The initial clones each scored twice in eight development matches. The DAgger
refits regressed to zero goals in 16 matches each, despite completing every match
without detected robot contact or field exit. These are failed development
candidates, retained under `preparation/prepare-v1`; neither is a release candidate.
The independent 5 ms trace auditor reproduced the initial clone and refit metrics.

The next preparation adds 96 fresh native teacher matches. Target regression now
uses option entry and positive target-refresh decisions, with those states
oversampled; a retained target is not treated as a newly issued target command.
Choice and refresh decisions remain learned. No reserved cases have been used.

### First self-play attempt and conditional intent correction

The first two leagues completed 131,072 native PPO steps and accepted 112 of 128
updates. Both seeds changed strategic decisions while preserving their target
functions and all motor actors. A broader 64-match development check completed
without robot contacts or field exits, but produced only 12 goals. Paired panels
scored four and three goals; fresh cross-play panels scored two and three. These
candidates were not frozen or run on reserved qualification cases.

The next preparation separates the previously shared intent vector into three
learned heads, retaining the 162-observation/17-action deployment interface. It
collects 32 additional mixed DAgger matches per seed from the first self-play
policies, then refits each seed with supervision applied only to the selected
skill head. A new categorical choice therefore selects its own learned intent.
This is a new preparation experiment, not an isolated causal attribution of the
previous results to either architecture or PPO. All old candidates are retained.

### Resumed diagnosis and pass-motor preparation

The separate-head adaptation scored only one goal across 32 development matches;
those failed candidates and all native records remain in `preparation/adapt-v3`.
The initial coach also supplied very few shooting labels and copied ordinary
choices from the previous neural selector. The revised training-only coach uses
explicit continuous-play labels while deployment remains fully neural.

Recorded backpass states exposed unwanted lateral motion and delayed release in
the old pass head. A diagnostic using analytic pass commands scored three goals
in eight native matches, including both redirect opportunities, with zero robot
contacts or field exits. This is a diagnostic, not a learned-controller result
or a controlled estimate of improvement over the other seed panels.

A small learned pass motor now uses 16 robot-relative geometry/contact features
and predicts four motor outputs. Its v5 preparation uses only recorded native
states from training-range seeds, with explicit coaching labels. The exploratory
v4 fit included previously inspected development states and is excluded from the
candidate pipeline; it is retained with that limitation. All other motor actors
remain unchanged. The pass replacement is opt-in through `--pass-motor` and must
be frozen alongside the eventual strategy before reserved evaluation. No new
reliable integrated release or final demo is claimed at this stage.


The learned-pass diagnostic completed eight fresh training-range cases with three
goals, two stopped receptions, two redirect goals and four clearances, without
robot contacts or field exits. All 32,000 native 5 ms states passed the independent
geometry audit. Strategic choices were supplied by the explicit coach, so this
is motor-integration evidence only. The next selector dataset contains complete
20-second matches rather than the previous 12-second prefixes. Both strategy
seeds use the same frozen motor bank and independently initialized selectors.

### Receiving-target correction

Both v5 selectors scored one goal in eight fresh native development matches;
neither completed a stopped reception. Each completed two clearances; the first
seed made two redirect kicks/one goal and the second one kick/one goal. Geometry
audits passed for both, and exact checkpoint replay reproduced every decision and
intent for the first seed. These are retained failed development candidates.

The traces show initial receive-point errors up to roughly 0.10 m and successive
updates that move the receiver away from the incoming ball. The corrective coach
also previously projected the student's own latched target, allowing that error
to contaminate correction labels. It now projects the current mouth position
onto the measured trajectory. Historical refresh labels are corrected from their
recorded native observations during the new fit; raw datasets remain unchanged.

The next actor adds direct observations and four velocity/projection features to
its learned features. All choice and target coefficients remain learned; there
is no runtime target planner. Entry and refresh examples receive separate
sampling weight. A final regularized target regression balances those phases
without changing behavior-choice or refresh coefficients. Coaching allows a
0.8-second stopped hold above the unchanged 0.5-second outcome requirement;
termination remains a learned decision. Future collections retain every option
entry instead of allowing alternate-tick sampling to drop rare entries.

Two additional native correction collections each contain 16 full matches with
80% student decisions. Seed811's mixed controller scored seven goals with three
stopped receptions, three redirect goals and five clearances; seed822's scored
two goals with three stopped receptions, one redirect goal and four clearances.
All matches completed with zero detected contacts or field exits. These mixed
training results do not qualify either autonomous policy.
