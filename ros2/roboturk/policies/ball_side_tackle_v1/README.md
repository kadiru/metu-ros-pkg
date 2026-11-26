# Ball-side tackle policy for the 5v5 Presser

The checkpoint controls approach angle, mouth offset, draw speed, and lateral pull. The motor uses the measured carrier orientation and frame-to-frame velocities, closes from the exposed flank, then draws the ball beyond the opponent roller’s retention region. Close-range braking is pair-specific to the selected carrier. Other robot pairs use the open-play steering policy. The selector activates when an opposing carrier still has the ball and has room for the challenge. Ordinary role and possession logic handles play after the tackle.

## Fit and qualification

A velocity-based pilot explored nine profiles on eight training starts each. The final draw controller fit six profiles on the same eight starts. Selection maximized contact-free possession wins, then minimized body contacts, then median time among clean wins. The selected profile uses a 37° approach, a 0.108 m mouth offset, a 0.22 m/s draw-speed cap, and a forward draw. It won 8 of 8 training starts. This was a parameter search over native Webots rollouts.

The frozen profile finished 24/24 clean takeaways versus 0/24 for the current Presser. Qualification uses the native roller spring and ideal observations. [Both GIFs, per-start results and referee logs](../../media/ball-side-tackle/README.md) · [Fit table](fit.json)

## Run in Webots

From the modern project root with Webots on PATH and the project Python environment:

```sh
steering=policies/open_motion_v1/actor.json
tackle=policies/ball_side_tackle_v1/actor.json
python scripts/run_five_roles.py runs/tackle-match --seed 38644001 --seconds 32 --watch --trajectory-planner --errt-fallback --commit-waypoints --open-play-policy "$steering" --tackle-policy "$tackle"
```

To refit the parametric controller on the eight training seeds:

```sh
python scripts/run_five_roles.py runs/tackle-fit --seed 38710000 --tackle-trials 8 --open-play-policy "$steering" --tackle-sweep policies/ball_side_tackle_v1/training_grid.json
python scripts/fit_tackle.py runs/tackle-fit-output runs/tackle-fit
```

To reproduce the held-out comparison:

```sh
python scripts/run_five_roles.py runs/tackle-current --seed 38712000 --tackle-trials 24 --motion-audit --open-play-policy "$steering"
python scripts/run_five_roles.py runs/tackle-trained --seed 38712000 --tackle-trials 24 --motion-audit --open-play-policy "$steering" --tackle-policy "$tackle"
PYTHONPATH=src/roboturk_core python scripts/autoref/judge_tackle.py runs/tackle-current/current/trials.jsonl.gz runs/tackle-current-ref
PYTHONPATH=src/roboturk_core python scripts/autoref/judge_tackle.py runs/tackle-trained/tackle/trials.jsonl.gz runs/tackle-trained-ref
python scripts/compare_tackle.py runs/tackle-current/current/trials.jsonl.gz runs/tackle-trained/tackle/trials.jsonl.gz runs/tackle-current-ref/result.json runs/tackle-trained-ref/result.json
```

Use the existing passive detector build for adjudication. [TIGERs setup and observer build](../../media/autoref/README.md)
