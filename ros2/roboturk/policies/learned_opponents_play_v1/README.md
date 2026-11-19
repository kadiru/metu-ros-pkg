# Passing and scoring against learned defense v1

Both attacking actors have completed training against the frozen learned
midfielder and goalkeeper. Complete-play development and final qualification
are in progress; no reserved full-play result is claimed yet.

## Four deployed policies

| Role | Contract | SHA256 |
|---|---|---|
| Centralized passer/receiver |`roboturk.defended_transfer.v3`|`457a645ecad0ab35cbcf320710a8d3e20af648c1dc80131c2211e0f539803361`|
| Finisher |`roboturk.pressure_finish.v2`|`c6575fa1e183ef58e0a4af75e1d5170801b8ccba0836919aa2fb0d7abace2e08`|
| Defender |`roboturk.continuous_opponent.v1`, defender role|`06fd042a37390dd0e51ab73f668763730a2e52faade721fbd421fa847612a726`|
| Goalkeeper |`roboturk.continuous_opponent.v1`, keeper role|`9dea9369b2b6bfe3811ca338c4d89f9b7854f7146ae0f33f1fd755967ecc52f4`|

Opponent movement comes from neural checkpoints, with no analytic fallback.
Their speed/acceleration/latency limits, role anchors and geometric input features
remain explicit context. [Opponent model card and100/100,99/100 challenge evidence](../learned_opponents_v1/README.md).

The62-input/seven-output transfer actor controls both attackers until stable
reception. The32-input/four-output finisher controls carrying, turning and shot
release. Receiving-point selection, the0.70m centered finishing point, open-corner
aim selection, stable-reception handoff, dribbler masks and fixed kick speeds remain
scripted. The finisher observes keeper position and velocity through the same
training/deployment contract. Neither opponent is reset or disabled at handoff.

## Training and retained development

One CPU seed137 per attacking actor;300 native demonstrations,150 mixed DAgger
episodes and65,536 anchored PPO steps per actor. Both actors accepted64 PPO updates
with zero rejected updates. Opponent checkpoints stayed frozen throughout.
Transfer training includes both opponents; standalone finishing training includes
the same frozen keeper. The complete play is then tested with both opponents active.

| Native component development | Source weights | Cloned | DAgger | Final PPO |
|---|---:|---:|---:|---:|
| Passing/reception |60/60|60/60|60/60|60/60|
| New finishing tactic |0/60|57/60|60/60|60/60|

Passing collection completed300/300 teacher and150/150 mixed receptions. Finishing
collection completed300/300 teacher and150/150 mixed goals. The unchanged finishing
weights, evaluated with the new tactic, had58 timeouts and two possession losses;
this is a zero-shot compatibility baseline, not the previous full-play benchmark.
The cloned finisher had three arrival timeouts and zero losses. DAgger removed
those timeouts on the component development set; the selected PPO checkpoint
preserved60/60 with zero losses. No incremental PPO success-count benefit over
selected imitation actors is established. No independent training-seed replication.

Every candidate checkpoint and stage outcome is retained in [transfer/stages](transfer/stages)
and [finish/stages](finish/stages). [Selection and hashes](selection.json).
Large demonstration arrays and process logs remain in ignored local `runs/learned-attackers-v1`.
PPO update/projection logs are included; collection/development/qualification
outcomes are retained without filtering.

The previous complete play scored45/60 against these exact learned opponents,
with60 receptions and15 keeper saves, zero pre-release losses. [Baseline](../learned_opponents_v1/old-attackers-baseline.json).
The first closer-only teacher scored32/40, with eight saves. Revised open-lane
teacher feasibility was60/60. Both are retained under [development](development)
and are explicitly teacher results, not learned-policy qualification.

## Qualification protocol and limits

Complete development:1640000–1640119. Reserved qualification:1650000–1650199.
Fresh consecutive native demo:1660000–1660005. Require at least95% complete goals,
zero pre-release possession losses and zero robot collisions. Every success needs
one actual pass,≥1m ball travel,≥0.4s stable reception,≥0.5m retained carry, one
actual shot and whole-ball crossing inside the posts. Any opponent contact/possession
predicate fails the play, even if a deflected ball subsequently crosses the goal.
Opponent and possession checks run at5ms; neural control runs at20ms.

The passer begins holding the ball. Roles, headings, positions, attack direction
and bounded opponent parameters vary. This is ground-truth, approximate native
Webots physics with ideal body velocity, a spring dribbler and applied velocity
kicks. It does not establish unrestricted competition play, perception learning,
or calibrated hardware performance. Opponent training uses imitation/DAgger plus
PPO on direct defensive challenges; attackers train against fixed opponents,
rather than a continually changing self-play league.

[Full protocol](../../docs/LEARNED_OPPONENTS.md). Candidate staging uses
`scripts/stage_learned_play.py`; the four-actor/runtime freeze and unfiltered
whole-play audit use `scripts/audit_opponent_release.py`.
