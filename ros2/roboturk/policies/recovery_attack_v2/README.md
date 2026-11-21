# Scoring after recovery under pressure

The first scoring policy improved on the previous outlet team in two independent studies. It converted 28 of 60 opening contests, below the target of 36. Training records showed that recovering players sometimes lost the ball while turning and sometimes shot out or into an intercepted lane. [First policy and all 174 games](../recovery_attack_v1/README.md)

This experiment gives the ball carrier four additional combinations of turn speed and motion away from a nearby opponent. During a pass or shot the carrier can pivot around the ball while translating. The new shot planner compares seven points inside the goal and estimates where intervening robots will be when the ball reaches them. Every configuration uses the same kick speeds, field, 3v3 defenses and underlying Webots contacts. The original controller is also available in training.

Both colors play each of 120 training layouts under five motor configurations. The learner selects the control with the highest count of actual opening-recovery goals. On a further 300 layouts it tries each of pass, dribble and shoot and fits opening returns from the resulting play. Five-fold validation keeps all trials of a layout in the same fold. The published protocol specifies the development comparison and held-out thresholds. [Protocol and seed ranges](protocol.json)

## Motor results

| Maximum turn rate (rad/s) | Escape speed coefficient (m/s) | Opening goals / 120 | Pass-assisted | Dribble-assisted |
| --- | ---: | ---: | ---: | ---: |
| Original motor | 0 | **80/120** | 11 | 36 |
| 4.4 | 0.28 | 71/120 | 9 | 27 |
| 5.5 | 0.55 | 66/120 | 6 | 31 |
| 6.8 | 0.85 | 74/120 | 5 | 29 |
| 5.5 | 1.15 | 64/120 | 6 | 18 |

The original motor won the selection. The additional opening-policy study therefore keeps the original pivot and shot planner. Each goal above starts with the opening contest and satisfies the full physical conversion audit.

## Learned decisions

The opening study ran 900 trials. The selected estimate used 18 neighbors for a dribble or shot and ten for a pass. Grouped cross-validation selected a six-point pass-success bonus for the opening-value estimate. Subsequent decisions still use the learned continuation values from the first study. Both teams use the same decisions with their respective goal directions.

On 32 identical development starts, the original opening converted 19 initial contests. The new opening converted 20, with 25 goals from 45 recoveries overall. Four goals followed a completed pass and 13 followed a controlled dribble. The new opening and original opening ran in a fresh Webots process for every game. The action model was frozen on GitHub before independent testing. [Training episodes and fitted estimates](training.json.gz) · [Both development panels](development.json) · [Frozen weights](policy.json) · [Source hashes](freeze.json)

## Reproduce

Use the project's isolated Python environment with the native Webots binary available. Both launchers accept an explicit binary path via `--webots`. The test measures uninterrupted possession, intended receptions, actual kicks and goal-line crossings at 100 Hz.

```sh
python scripts/verify_shield_training.py
python scripts/run_shield_attack.py qualify runs/new-shield-qualification
python scripts/audit_shield_attack.py runs/new-shield-qualification
python scripts/run_shield_attack.py demo runs/new-shield-demo
python scripts/audit_shield_attack.py runs/new-shield-demo --demo
python scripts/build_recovery_goal_demo.py runs/new-shield-demo --media runs/new-shield-media
python scripts/run_shield_attack.py ordinary-movie runs/new-shield-ordinary --qualification runs/new-shield-qualification
python scripts/check_recovery_showcase.py runs/new-shield-ordinary runs/new-shield-qualification --media runs/new-shield-media
```
