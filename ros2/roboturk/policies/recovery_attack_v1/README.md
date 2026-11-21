# Scoring after a contested recovery in 3v3

The previous RoboTurk team could win a contested ball and find an outlet. Across its twelve published 90-second self-play games, one of sixteen verified recoveries led to a goal under the uninterrupted-possession definition used here. This experiment gives the recovering team a learned choice of passing, carrying the ball or shooting. The same decision policy runs on each of the three players and can choose again after an intended reception or a short dribble.

## What the policy observes

A native Webots contest begins when an opponent from each side holds the ball in its front mouth for 0.20 seconds. One of those players must then hold it alone for 0.20 seconds. The released jaw and support policies handle that contest. The new attack policy takes over after recovery and stays active while the team retains the ball. An opposing mouth touch, a restart or 18 elapsed seconds ends its opportunity.

The decision values depend on field position, pressure, heading, passing-lane clearance, the receiving player's field position and its shooting lane. The refined model also observes completed passes and previous dribble choices in the current attack. It samples from three learned option values. Its pass option uses the existing receiver weights plus forward progress, alignment and shooting space to choose a teammate, then pivots for a 2.2 m/s kick. The dribble option steers around bodies while keeping the roller on. The shoot option aims through an open part of the opponent's goal and kicks at 4 m/s. The low-level motions, action durations, trigger and safety limits are explicit.

## How the three choices were trained

The first native league alternated the learning side between Blue and Yellow. Its opponent used a sampled historical checkpoint. A ridge regression for each action estimated the return from the actual kicks and subsequent goals. Training used exploratory choices and discounted the reward by the time remaining until the goal. The first 144 games produced 92 recovery goals, of which two were pass-assisted. [Initial league and checkpoints](initial.json.gz) · [Update replay](initial-audit.json)

The second league starts with the learned values from the first. It fits from both sides' observed possessions against historical policies. In these training trials a restart or opposing touch ends the scoring opportunity early. Evaluation games continue to their full time limit. A completed pass earns a small shaping reward. A physically verified goal earns the main reward, with additional credit for a completed pass or a sustained carry preceding it. Action values use a geometric initial estimate and update from native outcomes.

## What counts as a conversion

The audit reads the ball and six robot poses at 100 Hz. It checks that the winner's team shoots across the opponent's goal line inside the posts, within 18 seconds of recovery and three seconds of the shot. Each intermediate kick must reach the selected teammate, travel at least 0.55 m and end in at least 0.20 seconds of exclusive receiver control. An opposing mouth touch or a referee restart breaks the sequence. A dribble-assisted goal requires at least 0.18 m of simultaneous robot and ball displacement during one uninterrupted controlled carry. The referee score and the recovery-attributed score are reported separately.

Training, development, qualification and demo seeds use distinct ranges. The main comparison uses matched contested fixtures. Qualification also measures spontaneous recoveries in ordinary 3v3 open play. The Webots world uses ground-truth poses, ideal body velocity, a spring dribbler and fixed kick power. [Evaluation plan](protocol.json)
