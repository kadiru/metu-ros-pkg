# Validation — November 18, 2026 (America/Los_Angeles)

## Verified locally

- 50 Python tests pass. They include Gymnasium contracts for all six tasks and capture/dribble/scoring curricula; seeded replay; coordinate/target rotation invariance; finite actions; actuator limits and retain/fire semantics; HALT/STOP/stale-input overrides; full-ball goal crossing in both directions; rejecting premature shots and pushed-in balls; possession loss between control frames; arrival hold; passing/receiving and role persistence; reset clearing; and the deferred Webots reset regression.
- Ruff static checks pass and Python sources compile.
- Both Python packages built and installed as editable packages; `pip check` reports no broken dependencies. Exact local versions are recorded in `requirements-macos-validation.txt` (a macOS validation environment, not a universal ROS lockfile).
- ROS-specific tests are skipped locally because this Mac has no ROS 2 installation. GitHub Actions on Ubuntu 24.04 / ROS 2 Jazzy built all three packages and passed **2/2 ROS bridge/interface tests** at commit `3b543642`. These tests exercise conversions, command/referee watchdogs and reset with a reference backend; a complete Webots/ROS live-graph run is still pending.
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

Webots emits an Apple GPU capability warning and a texture-loader warning. The simulator parsed the authored world and PROTOs, stepped the Supervisor controller, created robots dynamically, and wrote the recorded episode results successfully. Native movie recording also completed; the resulting H.264 video was decoded and inspected.

## Qualified learned capture policy

The new `capture.randomized.v1` task tests full-circle robot headings/bearings, starting distances 0.35–2 m, and stationary or initially rolling balls (0.15–0.6 m/s). Success requires 0.4 s of continuous mouth possession, low relative speed and active dribbler.

The neural checkpoint achieved **200/200** reserved episodes (seeds 50000–50199), comprising **100/100 stationary** and **100/100 moving** cases. Mean capture time: **1.628 s**; Wilson 95% interval: **98.12–100%**. Six consecutive new demo seeds all succeeded and retained the ball for another second under the same policy. [Model card and episode records](../policies/chase_capture_v1/README.md) · [Actual Webots video](../media/chase-capture-demo.mp4).

Training used 300 demonstrations, 150 DAgger episodes and 131,072 PPO steps, seed 7, one CPU thread. Teacher commands are absent from inference. Development was 50/50 both before and after PPO, so these results do not establish PPO's incremental benefit. This is qualification for the stated unobstructed approximate simulation distribution only.

## Qualified learned capture-and-dribble policy

The frozen goal-conditioned policy passed **200/200** reserved episodes (150000–150199), with **zero possession losses across 438,456 physics-step checks** after acquisition. Both attacking directions and stationary/initially rolling balls were included. Mean progress toward the goal-side target was **3.5867 m**; mean duration **13.0177 s**, including the final one-second stationary hold. Wilson 95% success interval: **98.12–100%** for this distribution. [Model card, qualification and all development outcomes](../policies/capture_dribble_v1/README.md).

Development exposed a real regression: unconstrained PPO lost possession during arrival in 60/60 cases. The released experiment restarted from the successful cloned actor and used 65,536 PPO steps with reference-policy anchoring; it passed 60/60 development before the checkpoint was frozen. The clone itself already achieved 60/60 and was faster on development, so no incremental PPO benefit is claimed. Qualification uses actual Webots and the loaded `CheckpointPolicy` inference path, with no teacher fallback.

Core and ROS CI passed for source commit `10089703`, including the new training mode. The [native simulator demo](../media/capture-dribble-demo.mp4) passed **6/6 fresh scenarios**, retaining possession for another 0.8 s after arrival in all six. A camera-only rerender produced identical episode metrics. The complete 81.96 s H.264 video decoded successfully, and route/tracking frames were visually inspected; only the camera and nonphysical target marker affect presentation.

## Qualified learned capture-dribble-score policy

V2 scored **200/200** new reserved scenarios (350000–350199), with **zero invalid shots and zero pre-shot possession losses** across 241,996 physics checks. Stationary/rolling subsets were 112/112 and 88/88; positive/negative goals 106/106 and 94/94. Mean full-sequence time **8.3683 s**, controlled progress **3.1708 m**, shot distance **1.2865 m**. Each goal required exactly one applied kick after acquisition and at least 0.5 m of retained carry, followed by full-ball crossing inside the opponent's posts. [Model card and complete records](../policies/capture_dribble_shoot_v2/README.md).

The earlier v1 scored 192/200 and 5/6 demo scenarios, with premature-shot failures retained. V2 adds explicit radial goal distance and new recovery data; its initial range-only refit was 0/60, while DAgger and anchored PPO each scored 60/60 on new development seeds. All stages remain in the published evidence. This does not establish an incremental PPO success benefit. Qualification uses a single loaded MLP, with dribbler-until-fire and fixed 3.5 m/s kick semantics, against an unguarded goal in approximate dynamics. Core and ROS CI passed at training source commit `5edfd064`.

The [new native simulator demo](../media/capture-dribble-score-demo.mp4) scored **6/6**, on preselected seeds 360000–360005. All outcomes are retained. Full H.264 decoding and visual inspection of route and goal frames verify the recording artifact. The [earlier 5/6 demo](../media/capture-dribble-score-v1-demo.mp4) remains available, including its failure.

## Synthetic backend

[Recorded analytic baseline results](synthetic-baseline-results.json) are a software comparison only. The synthetic backend deliberately simplifies contacts and snaps a held ball to the robot mouth. It must not be confused with Webots physics or hardware evidence.

## Historical PPO training/checkpoint smoke test

Actual Webots backend, chase skill, seed 7, exactly 2,048 steps, SB3 PPO, CPU device with one training thread. Training completed and saved `runs/webots_smoke/policy.zip` plus its contract/config manifest. Loading and inference passed; loading the chase checkpoint as a passing skill correctly raised a compatibility error.

The reloaded policy achieved **0/10 successes** on held-out seeds 3000–3009 (out-of-bounds or collision). This short run verified training and persistence, not skill learning quality. It preceded the specialized capture curriculum and qualified checkpoint above. [Recorded training manifest and evaluation](ppo-smoke-results.json). This obsolete smoke checkpoint remains an ignored local artifact.

## Remaining qualification work

Full Webots/ROS live graph on Linux; motor/roller/dribbler/kicker calibration; official vision/referee adapters and full rule enforcement; guarded goals, obstructed/adversarial scenarios; multiple training seeds and ablations; learned receiving/passing and variable kick power; full team match evaluation; recurrent policies; multi-agent training and self-play; physical transfer. No GPU training has been launched.
