# Learned tactical decisions and self-play v2 — development

The v1 three-generation experiment failed development: G1 55/60clean, G2 53/60,
G3 54/60. All 294,912native PPO steps, 288accepted updates, checkpoints and failures
remain in decision_self_play_v1. No reserved cases were used.

V2 keeps the same75-input/11-output neural inference contract. It starts from the
v1 final defenders and restores the pre-PPO v3 manager after target-coordinate drift.
The keeper receives supervised boundary-recovery labels derived from native
observations; this coaching is explicit, with no runtime action projection.
A60-game and120-game native development evaluation precede further self-play.

Planned next training: two fresh alternating PPO generations, all three roles,
32,768steps per role. Manager PPO freezes its learned coordinate function while
updating option, robot, corner and target-update head rows. All runtime choices
and targets remain neural. No reliability or demo claim yet. The original unused
reserved seed ranges and release gates remain in docs/LEARNED_DECISIONS.md.
