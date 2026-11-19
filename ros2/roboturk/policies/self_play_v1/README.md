# Mobile defense and alternating self-play v1 — in development

The new curriculum is being trained; no reserved qualification or demo has run.
Both opponents can choose translation in two dimensions across the field. New
45-input defensive actors observe nearby robots;70-input transfer and36-input
finishing actors observe both opponents. Defensive limits are0.55–0.85m/s and
0.8–1.15m/s, with acceleration and keeper latency. Tactical receiving-point feints,
finishing target/corner selection and role handoff remain scripted.

[Protocol, seed splits and reliability gates](../../docs/SELF_PLAY.md).

Retained native teacher development:

| Tactic | Goals | Defensive stops | Other outcomes |
|---|---:|---:|---|
| Fixed receiving target |0/40|0|40 timeouts|
| Slow flank-switch feint |0/20|4|16 timeouts|
| Fast feint and2.5m/s pass |3/10|6|1 defender collision|

These are analytic demonstrations, not learned-policy results. The last test had
9/10 stable receptions and no pre-release losses. A closing-speed limit was then
added to training-only defensive labels; inference contains no such motion override.
[Raw development records](development). The initial source-network expansion error
occurred before any bootstrap episode; its traceback is preserved.

Initialization uses200 teacher and100 mixed DAgger full plays, then two generations
of alternating four-role PPO against immutable historical checkpoints. Both sides'
updates, checkpoints, outcomes and failures will be retained. The current simulation
uses initial passer possession, ground-truth state, ideal body velocities, ODE
contacts, a spring dribbler and applied velocity kicks. It is a repeated attack/defend
drill with approximate physics, not full-match or hardware qualification.
