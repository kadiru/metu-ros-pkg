# Continuous 2v2 with explicit velocity safety — development

**Not qualified. No reliable continuous-match demo has been released yet.**

Two independently trained neural actors control both colors through continuous
45-second matches. Seven learned choices select neural recover, dribble, pass,
shoot, press, cover and support heads. Tactical targets, ownership handoffs and
robot roles are not assigned by a runtime teacher.

This version adds an **analytic translation safety layer**. It projects the four
requested velocities jointly onto robot-separation, field-margin and speed
constraints. Rotation, dribbler and release commands remain untouched. Every
intervention is recorded, including requested/applied velocities and measured
poses. This is a hybrid learned controller, not entirely neural collision avoidance.
The [unfiltered v1 experiment](../continuous_play_v1/README.md) and its failures
remain available. No reserved v1 cases were used.

The [separate protocol](protocol.json) retains every numerical reliability gate
from v1, uses new reserved seeds, and requires publication of the runtime and
model freeze before those cases. Each independent seed must pass 60 games against
self, historical and other-seed opponents. The fresh demo will retain all three
consecutive 45-second matches. All outcomes and interventions will be reported.

Training preparation expands every recorded state with training-only motor labels
for all seven skills. These are counterfactual commands on real native states,
not new simulator transitions. The original selected teacher command is preserved.
Categorical PPO will optimize behavior-choice probabilities while keeping the
neural motor functions fixed. Native testing and self-play are still in progress.

Scope: approximate native Webots ODE contacts, ground-truth state, ideal body
velocity, spring dribbler and velocity kicks. No calibrated hardware, official
SSL rules, noisy perception or complete live ROS graph qualification is claimed.

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
Both still need broad validation. Categorical historical self-play is now running
in new isolated league directories, with frozen neural motor weights and logged
safety interventions. No reserved cases have been run.
