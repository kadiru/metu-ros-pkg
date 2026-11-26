# Recovering a contested ball and finding a teammate

The first candidate missed three of six qualification checks. On eight precisely matched 90-second starts it reduced joint possession from 358.60 to 7.87 s. It completed four recovery-to-pass sequences in three games. In the eight head-to-head games the released side completed both observed sequences. This version is retained as a comparison for subsequent training. [Full audit](qualification/audit.json) · [Per-game traces](qualification) · [Back-to-back process diagnostic](batched-diagnostic.json)

In a 3v3 game, two opposing dribblers can trap the ball between them. After six consecutive control ticks with an opponent at the ball, a robot uses a learned policy to choose among eleven body motions and roller settings. A second policy chooses where its support player should receive. That player can keep its current position or move to one of six locations relative to the contest. The carrier can select either teammate once it has the ball. The released recipient selector and pass motor make those decisions and execute the kick.

CMDragons gave its primary attacker responsibility for protecting the ball and its support attackers responsibility for finding receiving positions within zones. That division inspired RoboTurk's pairing of a robot working to free the ball with a teammate moving into a passing lane. RoboTurk learns its own motion and position scores from Webots games. [CMDragons 2016 Extended Team Description Paper, section 3.3](https://ssl.robocup.org/wp-content/uploads/2019/01/2016_ETDP_CMDragons.pdf)

## Training

The league ran 96 ten-second contest starts, varying the ball position, orientation, robot identities and support geometry. The learning side alternated colors. It faced the released team every third episode and a sampled historical policy in the other episodes. Both learned policies sampled actions in training and chose the highest-scoring action in evaluation. The motor update rewards sole control and completing a subsequent pass. The support update gives positive credit only for a completed pass to the outlet selected during the contest.

Replay reproduced every logged update, including 279 motor choices and 235 outlet choices. The learning side won the opening contest in 56 episodes, completed the ensuing pass in 34 and delivered to its designated outlet in 16. In 12 separate development games both teams used the trained policies. Nine games contained a recovery and four contained a full recovery-to-pass sequence. [Learned weights](policy.json) · [Training audit](training-audit.json) · [Episode logs and updates](training)

## Evaluation method

Each team has three robots. The matched panels use the same eight ordinary 90-second starts and continue until time expires, including after turnovers. Only a goal or ball leaving the field triggers a restart. One panel uses the released system on both teams, one enables the new policies on both teams, and one enables them on one team with colors alternating. The audit checks matching initial poses and recomputes physical events from the 100 Hz traces.

A contest requires opposing robot mouths to hold the ball together for 0.20 seconds. A recovery requires an original contestant to hold it alone for 0.20 seconds. To count a chain, that robot must then kick within eight seconds, and its intended teammate must receive within four seconds after at least 0.55 m of travel and hold the ball alone for 0.20 seconds. An opposing touch or a restart invalidates the chain. The designated-outlet count measures how often the player who moved into support was the one who received. Late receptions include every intended pass completed in the final 30 seconds. [Thresholds and seed ranges](protocol.json) · [Runtime and checkpoint hashes](freeze.json)

The Webots backend uses ground-truth poses, ideal body velocity, a spring dribbler and a fixed-power pass. These experiments measure simulation performance. The trigger, stabilization window, candidate motions, flank locations and the other 3v3 tactical roles are explicit controllers.

## Reproduce the matches

From the ROS 2 project directory, run with its Python environment active and Webots installed. Pass the native binary to the launcher with `--webots /path/to/webots` when using a different install location. Use a new output directory for each run.

```sh
export PYTHONPATH="$PWD/src/roboturk_core"
python scripts/run_contest.py train runs/new-league --matches 96 --seconds 10
python scripts/run_contest.py develop runs/new-dev --model runs/new-league/policy.json
python scripts/run_isolated_contest.py runs/new-panels
python scripts/run_contest.py demo runs/new-demo
python scripts/audit_contest.py runs/new-demo --demo
python scripts/build_contest_preview.py runs/new-demo --media runs/new-media
```

The matched launcher starts a fresh Webots process for every team configuration and seed. It verifies both freeze manifests before starting, then runs the physical trace audit after all 24 matches. The training logs include the initial seed and the historical opponent of each episode. [Development traces and game logs](development)
