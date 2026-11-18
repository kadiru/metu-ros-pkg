# Validation — November 18, 2026 (America/Los_Angeles)

## Verified locally

- 33 Python tests pass. They include Gymnasium contract checks for all six tasks; seeded replay; coordinate transforms; finite action validation; velocity/actuator limits; HALT/STOP/stale-input overrides; goal-line crossing; receiving/passing success; pass-role persistence; reset clearing; and the deferred Webots reset regression.
- Ruff static checks pass and Python sources compile.
- Both Python packages built and installed as editable packages; `pip check` reports no broken dependencies. Exact local versions are recorded in `requirements-macos-validation.txt` (a macOS validation environment, not a universal ROS lockfile).
- ROS-specific tests are skipped locally because this Mac has no ROS 2 installation. The included Jazzy CI job and generated interface build have **not** been run here.
- The original source checkout remains unchanged.

## Webots measurements

Runtime: Webots R2025a Nightly Build 17/11/2026, commit `e5d1122a3cd59f007ee4b37c8f651b50e7ee83d2`, Apple Silicon macOS. [Runtime provenance](runtime-manifest.json). Original stable R2025a failed on the known macOS 27 TCP-port detection issue.

After diagnosing on seeds 1000–1009, the final analytic baseline ran against **fresh seeds 2000–2009**, ten episodes per skill:

| Skill | Successes | Episodes |
| --- | ---: | ---: |
| Move | 10 | 10 |
| Chase | 10 | 10 |
| Dribble | 10 | 10 |
| Pass and receive | 10 | 10 |
| Receive incoming ball | 10 | 10 |
| Shoot | 10 | 10 |

[Raw Webots episode results](webots-baseline-results.json). These are small, unobstructed curriculum scenarios with analytic policies and approximate dynamics. They do **not** establish robustness against opponents, calibrated physical dribbling, competitive gameplay, or learned-policy performance.

Actual simulator testing caught three issues that the synthetic backend did not reveal: dynamic PROTOs need an `IMPORTABLE EXTERNPROTO` declaration; deferred physics reset erased initial ball velocity; and a loose kick-heading tolerance missed the receiving robot. Passing also needed a hold phase after releasing the ball, matching team-play behavior. All are corrected in the recorded final runs.

Webots emits an Apple GPU capability warning and a texture-loader warning. The simulator parsed the authored world and PROTOs, stepped the Supervisor controller, created robots dynamically, and wrote the recorded episode results successfully. No visual-quality claim is made.

## Synthetic backend

[Recorded analytic baseline results](synthetic-baseline-results.json) are a software comparison only. The synthetic backend deliberately simplifies contacts and snaps a held ball to the robot mouth. It must not be confused with Webots physics or hardware evidence.

## PPO training/checkpoint smoke test

Actual Webots backend, chase skill, seed 7, exactly 2,048 steps, SB3 PPO, CPU device with one training thread. Training completed and saved `runs/webots_smoke/policy.zip` plus its contract/config manifest. Loading and inference passed; loading the chase checkpoint as a passing skill correctly raised a compatibility error.

The reloaded policy achieved **0/10 successes** on held-out seeds 3000–3009 (out-of-bounds or collision). This short run verifies training and persistence, not skill learning quality. No learned policy is qualified for use. [Recorded training manifest and evaluation](ppo-smoke-results.json). Checkpoint binaries remain ignored local artifacts.

## Remaining qualification work

ROS colcon build and live graph on Linux; motor/roller/dribbler/kicker calibration; official vision/referee adapters and full rule enforcement; obstructed and adversarial scenarios; full team match evaluation; qualified learned policies; recurrent policies; multi-agent training and self-play; physical transfer. No GPU training has been launched.
