# Behavior ideas and their provenance

November 20 update: the [new TDP study and trained skill pack](TDP_SKILL_PACK.md)
adds CMDragons 2016 coordination, Thai Skuba 2013 play/role/skill decomposition and
TIGERs 2025 stop/redirect contracts. Receive-stop 197/200, redirect 200/200, and defensive
clearance 200/200 passed reserved native tests; all 12 fresh demo cases passed. The
historical candidate descriptions below record the earlier iteration, not current
unimplemented status. Support/recovery also exists in the separately qualified
[continuous 2v2 release](../policies/continuous_play_v2/README.md).

These papers inspire our task decomposition and tactics. They do not establish that
our learned policies reproduce the teams' implementations or match performance.
The teams' methods below are not being described as reinforcement learning.

| Team and primary source | Idea adapted here | Current implementation / evidence contract |
|---|---|---|
| [CMDragons 2015 ETDP](https://ssl.robocup.org/wp-content/uploads/2019/01/2015_ETDP_CMDragons.pdf), §2–3, pp2–7 | Skills/tactics/plays; support near the ball; prioritize immediate threats and cover shot or pass lanes | Existing behavior/play split; new `cover_pass_lane` tracks an assigned threat lane; `block_shot` tests physical interception. Support/recovery remains a future task. |
| [TIGERs Mannheim 2025 ETDP](https://ssl.robocup.org/wp-content/uploads/2025/04/2025_ETDP_TIGERs-Mannheim.pdf), §1–2 pp1–8, §3.2 p12 | Separate pass construction, feasibility/risk assessment, selection and execution; distinguish planned and observed pass outcomes | Scripted receiving-point selection is separate from the learned transfer actor. Actual kick events, reception dwell and complete-play outcomes are recorded. Moving receptions and redirects remain future tasks. |
| [ER-Force 2024 ETDP](https://ssl.robocup.org/wp-content/uploads/2024/04/2024_ETDP_ER-Force.pdf), §4.3 pp9–10, §4.5–4.6 pp12–14 | Analyze keeper prediction from shooter orientation and exploit reversal/braking; use hysteresis to avoid rapid switching | `draw_keeper` selects a lure aim; a latched transition selects the opposite corner for `shoot_open_window`. The neural finisher produces motion and kick timing. Our bounded delayed keeper is an experimental opponent; ER-Force's noise/trend estimator is not implemented. |

## Contracts and evaluation boundaries

- `capture`, `controlled_dribble`, `shoot`, `pass_receive`, `create_pass_lane`:
  existing trained checkpoints and qualification records remain unchanged.
- `draw_keeper` / `shoot_open_window`: one shared goal-conditioned neural finisher;
  scripted target/phase selection, four learned outputs (body vx/vy/omega/fire),
  fixed kick power. Entry: stable possession after a real teammate pass.
  Exit: actual receiver kick and whole-ball crossing inside the goal posts.
  Any keeper/defender touch, collision or pre-release possession loss fails.
- `block_shot`: bounded lateral interception micro task; learned translation to intercept and recover near an
  assigned defensive line. Success requires physically stopping and retaining a
  rolling shot. This is a reachable-shot curriculum, not unrestricted goalkeeping.
- `cover_pass_lane`: learned tracking of a geometrically assigned ball-to-opponent
  lane. Success requires continuous coverage dwell. This measures positioning,
  not a completed interception or match-level defensive strength.

Exact checkpoint status, training splits and outcomes belong in the corresponding
model cards. A task definition is not evidence that a policy has been trained or
qualified. Future strategy work can select these contracts based on observable
threats, teammate readiness and possession, then assess complete plays.

## Candidate behaviors for the next strategy layer

These are proposals, not trained releases:

- **Support and recover:** assign a nearby teammate to collect a loose ball while
  preserving a useful attacking angle; switch to covering a direct threat when
  possession changes. Inspired by CMDragons' support-attacker discussion (§2.2).
  Proposed metric: recovery time and possession outcome after randomized loose-ball events.
- **Receive and redirect:** choose between stopping the incoming ball and a timed
  direct shot. Inspired by TIGERs' explicit receive/redirect pass descriptions.
  Proposed metric: physically measured reception or redirect goal under randomized
  ball speed, receiver orientation and defender interception windows.
- **Keeper feint counter:** retain central coverage while the holder rotates, then
  commit when shot evidence is stronger. Inspired by ER-Force's keeper countermeasure
  analysis. This should become a distinct adversarial benchmark rather than changing
  the opponent during an existing qualification run.

Our current fixed-target and fixed-opponent curricula are starting points for these
contracts. General strategy quality requires evaluating role selection and behavior
composition against varied opponents, not adding isolated success rates together.

## Qualified releases from this iteration

| Behavior / composition | Reserved native Webots result | Fresh native demo | Evidence |
|---|---:|---:|---|
| Pass around defender, receive, draw keeper and score | 199/200; one intercepted pass, zero losses/collisions | 6/6 | [Keeper model card](../policies/keeper_finish_v1/README.md) |
| Block and recover possession | 200/200; at least0.5s stable retention | 3/3 | [Block model card](../policies/block_shot_v1/README.md) |
| Cover a moving passing lane | 200/200; within0.08m continuously for3s | 3/3 | [Coverage model card](../policies/cover_pass_lane_v1/README.md) |

The finisher uses one shared actor across scripted draw/finish phases. All three
new actors used demonstration learning, DAgger and anchored PPO; no incremental
PPO success-rate gain was observed over their selected imitation actors. These
are separate bounded curricula using approximate ground-truth simulation.
