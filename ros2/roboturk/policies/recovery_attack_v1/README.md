# Scoring after a contested recovery in 3v3

The previous RoboTurk team could win a contested ball and find an outlet. Across its twelve published 90-second self-play games, one of sixteen verified recoveries led to a goal under the uninterrupted-possession definition used here. This experiment gives the recovering team a learned choice of passing, carrying the ball or shooting. The same decision policy runs on each of the three players and can choose again after an intended reception or a short dribble.

## Frozen-team evaluation

All 58 games ran in independent native Webots processes. Each contested start used identical first-frame coordinates for the released and trained teams. An opening conversion credits only the first contest of the game. The third column includes later recoveries in the same match. Pass and carry columns count scoring sequences and can overlap.

| Teams and starts | Opening goals / games | Recovery goals / recoveries | With a pass | With a carry |
| --- | ---: | ---: | ---: | ---: |
| Previous team, contested | 2/20 | 5/39 | 3 | 0 |
| New team, contested | 11/20 | 16/30 | 1 | 9 |
| Head-to-head, contested | 3/12 | 8/21 | 1 | 3 |
| New team, ordinary 60-second open play | 3/6 | 3/5 | 1 | 2 |

Two qualification targets were missed. The new team scored from the opening recovery in 11 of 20 contested games against a target of 12. In head-to-head games it scored twice from the opening contest against a target of three. Across all recoveries it scored seven times against the previous team's one. In new-team self-play, both colors scored eight recovery goals. No recorded robot pair came within 0.18 m and no robot left the field in this test. [Every game and physical audit](qualification/audit.json) · [Predeclared thresholds](protocol.json)

The independent replication doubled the sample size with the same teams, durations and proportional thresholds. Its protocol was published before the new games. [Replication plan](replication-plan.json)

## What the policy observes

A native Webots contest begins when an opponent from each side holds the ball in its front mouth for 0.20 seconds. One of those players must then hold it alone for 0.20 seconds. The released jaw and support policies handle that contest. The new attack policy takes over after recovery and stays active while the team retains the ball. An opposing mouth touch, a restart or 18 elapsed seconds ends its opportunity.

The decision values depend on field position, pressure, heading, passing-lane clearance, the receiving player's field position and its shooting lane. The refined model also observes completed passes and previous dribble choices in the current attack. It samples from three learned option values. Its pass option uses the existing receiver weights plus forward progress, alignment and shooting space to choose a teammate, then pivots for a 2.2 m/s kick. The dribble option steers around bodies while keeping the roller on. The shoot option aims through an open part of the opponent's goal and kicks at 4 m/s. The low-level motions, action durations, trigger and safety limits are explicit.

## How the three choices were trained

The first native league alternated the learning side between Blue and Yellow. Its opponent used a sampled historical checkpoint. A ridge regression for each action estimated the return from the actual kicks and subsequent goals. Training used exploratory choices and discounted the reward by the time remaining until the goal. The first 144 games produced 92 recovery goals, of which two were pass-assisted. [Initial league and checkpoints](initial.json.gz) · [Update replay](initial-audit.json)

The second league starts with the learned values from the first. It fits from both sides' observed possessions against historical policies. In these training trials a restart or opposing touch ends the scoring opportunity early. Evaluation games continue to their full time limit. A completed pass earns a small shaping reward. A physically verified goal earns the main reward, with additional credit for a completed pass or a sustained carry preceding it. Action values use a geometric initial estimate and update from native outcomes.

Two further self-play studies compared the three possible openings on 240 seeded layouts apiece. Each study ran 720 native rollouts. A different opening was tried on each repetition, and the trained continuation policy played out the possession. The value estimator uses the actual recovered state in each rollout. In the final study, the pass option evaluated reachable spots ahead of either teammate, accounting for opponent positions and motion, turning time, receiver travel and the next shooting opportunity. It commands the teammate toward that spot while the carrier pivots.

The final controller estimates each opening's return from similar training observations. Five-fold selection kept all repetitions of a layout in the same fold. Among estimates that produced both pass-assisted and carry-assisted goals in validation, the selected estimate used 18 neighbors for dribbling and shooting and four for passing. The selected additional pass-success bonus was zero. Subsequent choices use the learned continuation values. [First opening study](branches-audit.json) · [Lead-pass study](lead-audit.json) · [Selection results](selection.json) · [Frozen policy](policy.json)

## What counts as a conversion

The audit reads the ball and six robot poses at 100 Hz. It checks that the winner's team shoots across the opponent's goal line inside the posts, within 18 seconds of recovery and three seconds of the shot. Each intermediate kick must reach the selected teammate, travel at least 0.55 m and end in at least 0.20 seconds of exclusive receiver control. An opposing mouth touch or a referee restart breaks the sequence. A dribble-assisted goal requires at least 0.18 m of simultaneous robot and ball displacement during one uninterrupted controlled carry. The referee score and the recovery-attributed score are reported separately.

Training, development, qualification and demo seeds use distinct ranges. The main comparison uses matched contested fixtures. Qualification also measures spontaneous recoveries in ordinary 3v3 open play. The Webots world uses ground-truth poses, ideal body velocity, a spring dribbler and fixed kick power. [Evaluation plan](protocol.json)

## Reproduce the experiment

Run these commands from the modern project with its Python environment active. The launcher accepts `--webots /path/to/webots`. The reported matches used the native Webots R2025a nightly from November 17, 2026.

```sh
export PYTHONPATH="$PWD/src/roboturk_core"
python scripts/freeze_recovery_attack.py verify
python scripts/run_recovery_attack.py qualify runs/new-attack-qualification
python scripts/audit_recovery_attack.py runs/new-attack-qualification
python scripts/run_recovery_attack.py demo runs/new-attack-demo
python scripts/audit_recovery_attack.py runs/new-attack-demo --demo
python scripts/build_recovery_goal_demo.py runs/new-attack-demo --media runs/new-attack-media
```

The four compressed training archives are JSON documents containing the native episode records, launch settings, checkpoint data and fitted policy. The replay command checks the learned parameters against those records. Primary seeds start at 9200000 for the initial league, 9220000 for the second league, 9240000 for the first opening study and 9245000 for the lead-pass study. Only development and training data were used to select the controller. [Initial training](initial.json.gz) · [Refined training](refined.json.gz) · [Opening trials](branches.json.gz) · [Lead-pass trials](lead.json.gz) · [Development audit](development.json) · [Source and policy hashes](freeze.json)
