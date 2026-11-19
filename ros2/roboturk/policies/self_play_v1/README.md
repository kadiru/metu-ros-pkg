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

## First neural initialization

The200 native teacher plays produced91goals,98defender interceptions,7keeper saves
and4defender collisions;181stable receptions and zero pre-release losses. After60
cloning epochs per actor, fully neural development scored17/40, with15defensive stops,
5field exits and3timeouts; zero collisions/losses.32/40 clean outcomes is below the
reliability gate. The100-episode mixed DAgger collection produced73goals and23stops,
plus four invalid outcomes. Its80% neural/20% teacher command mixture is training
data, not a fully learned result. After40refit epochs, the fully neural DAgger
development set produced9goals,29interceptions, one invalid pass and one defensive
teammate collision:38/40 clean outcomes. All outcomes and four checkpoints are retained.
Two generations of four-role self-play PPO have now started from this full DAgger
snapshot, with32768steps per role/generation. Reserved evaluation remains untouched.

[Teacher outcomes](artifacts/teacher-collection.json) ·
[Neural development](artifacts/teacher-development.json) ·
[Four initial checkpoints](artifacts/teacher) ·
[Bootstrap numerical source hashes](artifacts/bootstrap-runtime.json).

[DAgger collection](artifacts/dagger-collection.json) ·
[DAgger neural development](artifacts/dagger-development.json) ·
[DAgger checkpoints](artifacts/dagger).

## First self-play generation

All four roles completed32768PPOsteps each, with32/32accepted updates per role and
changed actor-parameter hashes. Full neural development produced20goals,38defender
interceptions and2defender collisions in60games:96.7% clean outcomes, but3.3%
collisions misses the≤2% gate. Generation two is running; no reserved evaluation
or demo has run. [Every first-generation checkpoint and rollout](artifacts/round1).

## Second generation and development amendment

Generation two completed another131072PPOsteps,128/128accepted updates and four
changed actors. It repeated20goals/38interceptions/2collisions on60development
cases, including the same collision seeds. No success-count improvement is claimed.
[All second-generation records](artifacts/round2). The complete initial3×3cross-play
matrix is being retained. A third four-role generation will use a dense close-robot
spacing cost and a larger defensive update allowance, with unchanged inference,
attacking update settings and qualification gates. See the protocol amendment;
reserved qualification and fresh demo cases are still untouched.

## Third generation and close-spacing preparation

Generation three completed128accepted PPOupdates but still had19goals,39interceptions
and the same2collisions/60development games. [All third-generation records](artifacts/round3)
and [full4×4cross-play matrix](artifacts/cross-play-generation3-matrix.json) remain
unqualified. Training-only weighted spacing distillation then refit the two defensive
actors using the retained teacher/DAgger observations; the attacking weights were
unchanged. Native development improved to23goals/36stops/1collision,59/60clean.
[Preparation, dataset hashes and fit losses](artifacts/spacing-repair/training.json) ·
[All60prepared-policy outcomes](artifacts/spacing-repair-development.json).
A fourth four-role PPOgeneration and broader development panels follow before any
reserved qualification. This preparation is supervised learning, not a PPO-only gain.

## Fourth generation

After the documented defensive preparation, all four actors completed another
32768PPOsteps. Full development:26goals,32interceptions and2keeper saves/60games,
**60/60clean with zero collisions or other control failures**. This remains
development evidence; historical and broader development panels are running and
reserved qualification/demo cases are untouched. [All fourth-generation artifacts](artifacts/round4)
retain the prepared initialization separately from each PPOupdate.
[Training audit](artifacts/training-audit.json) verifies524288totalPPOsteps,16role
jobs, historical checkpoint hashes, actor transitions and recorded training resets.
