# Defender + goalkeeper experiment v1

Predeclared November19,2026 (America/Los_Angeles), before new training.

The qualified defended-transfer v3 actor and the existing moving midfield defender
remain unchanged. Add a separate analytic goalkeeper on the goal line, with lateral
speed0.65–0.95m/s, acceleration2–3m/s² and reaction delay0.08–0.14s. Each reset selects
an orientation-tracking or ball-line style. Both predict incoming rolling shots.
The keeper is active throughout passing and scoring. These are bounded opponents,
not free-ranging opponents, self-play or a full official SSL match.

Baseline: frozen previous attackers on development seeds910000–910039. Teacher
baseline uses the same full-play scenarios. New finisher demonstrations900000–900299,
DAgger920000–920149; finisher development910000–910059 and full-play development
910000–910119. Reserve950000–950199 for frozen complete-play qualification and
960000–960005 for six consecutive native Webots demo cases. Gate: ≥95% of200 goals,
zero pre-release possession-loss or collision episodes. Retain every outcome.
Reserved results may not be used to tune a checkpoint claimed qualified on them.

The finisher begins holding the received ball. Standalone training randomizes
attack direction, robot ID, heading, x−0.4..1.1m in the attack frame and either flank
at |y|1.5..2.5m. Carry progress is always measured toward the fixed goal center,
independently of the selected aim corner. The complete play retains the real
transfer-to-finish physics state without a reset.

Draw/finish phase and aim selection are scripted; body motion and release timing
are learned from Webots demonstrations, DAgger and constrained PPO. No analytic
motion fallback is allowed at learned inference. PPO improvement must be measured,
not inferred merely because PPO ran. All training uses one CPU thread.

Additional micro task seed families: block1000000, cover1100000; demonstrations
base..base+199, DAggerbase+20000..base+20099, developmentbase+10000..base+10059,
qualificationbase+50000..base+50199. Their model cards must distinguish physical
saves from geometric lane coverage. These separate learned defenders do not change
the opponent during the attacking experiment.
