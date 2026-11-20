# Learned strategy over ten soccer skills

Work in progress, November 20, 2026. This integrates the qualified receive-stop,
one-touch redirect and recover-clear actors into continuous 2v2. Earlier frozen
releases remain unchanged. No new learned-strategy qualification is claimed yet.

The shared selector observes 162 measurements/state features and produces 17
outputs: ten behavior scores, six target parameters and a target-refresh score.
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
