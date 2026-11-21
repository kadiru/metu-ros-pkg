# Learned skill strategy — self-play training in progress

This experiment integrates receive-stop, one-touch redirect and recover-clear
into continuous learned 2v2 strategy. The corrected preparation has passed its
native development checks; historical self-play is underway. **No reserved
qualification or final reliable demonstration is claimed yet.** Prior isolated
skill success rates do not establish composed strategy performance.

The selector takes 162 observations and selects among ten skills. It produces
31 internal values: ten choice logits and three separate seven-value intent
heads, packed into a 17-value deployed action. Entry, exit, target parameters and
refresh timing are learned. Direct geometric features improve target precision.
Four robots continuously use neural strategies and motor policies; responsibilities
can change. No scripted playbook or tactical handoff runs at deployment.

The original continuous and TDP motor files remain unchanged. A separate learned
pass motor repairs inaccurate backpasses and is explicitly selected with
`--pass-motor`. All motors and aiming functions remain fixed during strategic
PPO. The existing geometric translation safety filter remains explicit; this is
not a claim of learned collision avoidance.

## Corrected preparation: heads-v6

Two independently initialized selectors were fitted using 32 full-length native
coach matches and 16 mixed correction matches per seed. Native state labels
correct inherited intercept errors, balance option entry against later refreshes,
and provide a hold margin above the reception metric. Raw datasets and all
previous candidates remain available.

Each seed then ran eight fresh 20-second development matches, seeds6304000–6304007:

| Training seed | Goals | Receive-stop | Redirect goals | Clearances | Contacts / exits |
|---|---:|---:|---:|---:|---:|
| 811 | 3 | 2 | 2 | 3 | 0 / 0 |
| 822 | 3 | 2 | 2 | 3 | 0 / 0 |

Both independent 32,000-sample geometry audits and all eight exact checkpoint
inference replays passed. These are development results, not reserved reliability
estimates or an isolated PPO improvement. The new historical leagues train
65,536 native PPO steps per seed, with all four learner roles and retained opponents.

See [architecture and retained failures](../../docs/LEARNED_SKILL_STRATEGY.md),
[predeclared protocol](protocol.json), and [preparation artifacts](preparation).
The protocol requires six reserved panels totaling120 matches after publication
and independent verification of the selected runtime freeze, followed by four
fresh consecutive uncut demo matches. No reserved seeds have run.

## Reproduce a training league

From the modern project root, using the installed training environment and the
pinned Webots build:

```bash
python scripts/run_strategy.py --webots /path/to/webots \
  --mode train --training-seed 811 --rounds 2 --steps 32768 --duration 20 \
  --checkpoint policies/strategy_self_play_v1/preparation/heads-v6/seed811/initial.zip \
  --pass-motor policies/strategy_self_play_v1/preparation/pass-motor-v5/policy.pt \
  --output runs/reproduce-strategy811 --timeout 7200
```

Run seeds serially. Dataset hashes are checked, including packaged copies when
original training paths are unavailable. The launcher refuses to overwrite an
existing attempt and records source snapshots and simulator hashes.

## Scope

Approximate native Webots physics, ground-truth state, ideal body velocities,
spring dribbling, velocity kicks and simplified referee restarts. The tactical
starting opportunities vary team, robot identity, geometry and attack direction;
subsequent restarts use loose-ball formations. No official SSL match, hardware,
full live ROS graph or PPO-only improvement claim is made. Earlier frozen releases
and videos remain unchanged.

Current first corrected league (seed811) completed65,536 native PPO steps,
accepting58/64updates. Both generations changed strategic decisions while
preserving the complete aiming functions and motor bank. Final development:
8/8completed,3goals,2stopped receptions,2redirectgoals,2clearances,zero contacts/
exits. The second seed is training; broader cross-play and reserved qualification
are still pending.
