# Learned tactical decisions and self-play v1 — in development

A centralized neural manager chooses pass/receive, dribble or shoot, the acting
robot, receiving and dribble points, and goal-mouth aim. Frozen trained motion
skills execute its decisions; the old scripted target planners and handoff are
absent from deployed inference. The manager and both opponents will be trained
through alternating native self-play. Teacher logic is restricted to initialization
and labeled baselines. No new reliability result or demo is claimed yet.

[Protocol, fresh seeds and gates](../../docs/LEARNED_DECISIONS.md).
Local contract/regression checks:124passed,oneROS-onlyskip; Ruff clean. Native
teacher baseline completed57/60clean:28goals,29defensive stops,one possession loss,
one field exit and one collision. The first56/60panel is retained; a premature
acquisition measurement was corrected before training.300native demonstration
episodes and150mixedDAgger episodes are now being collected/trained. Existing mobile-self-play policies are the motion
skill/opponent starting point, not evidence for learned tactical decisions.
