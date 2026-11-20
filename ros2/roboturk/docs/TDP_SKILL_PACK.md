# TDP study: reusable ball skills and orchestration

Study and implementation started November20,2026. The new release is
[tdp_skills_v1](../policies/tdp_skills_v1/README.md). A task definition or analytic
baseline is not a trained-policy result. Team papers inspire the decomposition;
we do not claim those teams used our reinforcement-learning methods.

## What the papers contribute

| Primary paper | Coordination idea | Our adaptation |
| --- | --- | --- |
| [CMDragons 2016 ETDP](https://ssl.robocup.org/wp-content/uploads/2019/01/2016_ETDP_CMDragons.pdf), §2,§3.3,§4–4.2, pp2–9 | Plays allocate offense/defense roles; support positioning is timed to an intended pass. Defense distinguishes immediate shooting threats from subsequent receiving threats. A keeper can reuse a passing tactic. | Explicit play applicability, role assignment, commitment/cancellation and reusable skill intents. Emergency clearance has priority; teammate readiness is a contract. |
| [Skuba 2013 ETDP](https://ssl.robocup.org/wp-content/uploads/2019/01/2013_ETDP_Skuba.pdf), §3.2–3.2.1, pp7–8 | The Thai team describes playbook→role→skill-set execution. Aggressor, Blocker, Creator and Defender have different duties; defensive roles clear the ball, while Creator supports attacks and rebounds. | A distinct recover-and-clear skill, usable by a keeper or defender through the same interface. Role names select responsibilities, not permanent robot IDs. |
| [TIGERs Mannheim 2025 ETDP](https://ssl.robocup.org/wp-content/uploads/2025/04/2025_ETDP_TIGERs-Mannheim.pdf), §1.1,§2.1–2.4,§3.2, pp1–8,12 | A pass includes arrival timing, receiving speed and stop/redirect mode. Candidate generation, feasibility, ranking and execution are separate; observed kicks/receptions are matched to plans. | Separate timed reception and one-touch redirect policies, eligibility checks before committing, and physical event audits rather than counting a requested kick as success. |

These are paraphrased architectural ideas. Our geometry, thresholds, curriculum,
reward, neural networks and evaluation gates are new prototype choices. We have
not reproduced the teams' complete algorithms, calibrated models or match results.

## New skill contracts

All three actors use34observations and four outputs: body vx/vy, angular velocity
and release. Inputs include measured robot/ball state and the explicit tactical
intent. Requested receive point, bounded receiving region and clearance outlet
are scripted context, not hidden teacher outputs. Each skill has its own weights.

- **Receive-and-stop:** move to an announced pass rendezvous, face the incoming
  ball and settle it for0.5s. A stationary teammate makes a real native kick at
  1.6–2.5m/s. No receiver kick is allowed.
- **One-touch redirect:** reach the pass rendezvous oriented toward goal, apply
  exactly one kick within0.15s of first mouth contact, then score a whole-ball
  goal. Incoming side and heading, receiver offset, IDs, color and field direction
  vary. This is a bounded, unguarded redirect curriculum.
- **Recover-and-clear:** acquire a loose ball in our third, establish0.4s stable
  control, then kick through a flank outlet beyond the immediate danger region.
  A stationary opponent occupies the central alternative. Opponent contact,
  own goals, premature release, collisions and field exits are failures.

Native physics checks run every5ms; policy decisions every20ms. Kicking and
dribbling use the existing approximate velocity-kick/spring model. Release speed
is fixed at3.5m/s; dribbler enable is coupled to release. These actuator rules are
not learned. No translation safety projection or analytic motion fallback runs
inside the new learned skills.

## How orchestration fits

The extension follows play conditions → role assignment → committed skill option
→ measured outcome. A pass contract announces a receive point and source. The
coordinator checks reachability and goal-lane feasibility, selects a robot,
commits to receive or redirect, and explicitly cancels on referee stop, lost
visibility or an expired contract. A loose ball in our defensive third takes
clearance priority. Each option has a success/failure/timeout exit.

This first coordinator is scripted and auditable. Motor/release policies are
learned. The already-qualified continuous self-play selector and its frozen
runtime are preserved; adding these options to a new learned match-level selector
requires a separate training and qualification cycle. Isolated skill success
rates do not establish composed full-match strategy strength.

## Training and evaluation

One independent initialization per new skill; no cross-seed robustness claim.
Native demonstrations, mixed DAgger and32768anchoredPPOsteps per skill retain
all checkpoints and failed development attempts. Reserve200fresh cases per skill
and four fresh demo cases, with runtime/model publication before use. Qualification
requires at least95%success and zero collision/field-exit/own-goal/premature-clearance
episodes, plus independent event/hash audits. See [exact splits](../policies/tdp_skills_v1/protocol.json).

Ground-truth Webots prototype dynamics, scripted delivery fixtures, unguarded
redirect goal and simplified field geometry remain explicit limits. Hardware,
official SSL rules, noisy vision/radio and complete live ROS graph validation
are separate work.
