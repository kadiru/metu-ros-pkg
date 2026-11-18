# Chase-and-capture policy v1

Trained and evaluated November 18, 2026 (America/Los_Angeles). This is a learned single-robot capture behavior for the approximate RoboTurk Webots model. It controls body velocity, angular velocity and dribbler duty; no analytic fallback runs during inference.

| Measurement | Result |
| --- | --- |
| Reserved qualification episodes | **200/200 successful**, seeds 50000–50199 |
| Stationary / initially rolling ball | **100/100 / 100/100** |
| Mean capture time | **1.628 s**, including 0.4 s stable possession |
| Wilson 95% interval | **98.12–100%** on the sampled distribution |
| Fresh recorded demo | **6/6**, seeds 60000–60005; all retained possession for another 50/50 control frames (1 s) |
| Development before / after PPO | **50/50 / 50/50**, seeds 10000–10049 |

The target was at least 95% success on the reserved 200 episodes. Success requires 20 consecutive frames with the ball in the mouth region, relative speed below 0.2 m/s, and dribbler output above 0.5. Episodes time out after 10 s. Starting distances are 0.35–2 m, headings and bearings span the full circle, and half the scenarios sample initial ball speed 0.15–0.6 m/s. See the [exact curriculum and reproduction commands](../../docs/CAPTURE_TRAINING.md).

Training used 300 analytic demonstration episodes, actor cloning, 150 DAgger episodes, and 131,072 PPO steps in actual Webots, on one CPU thread. Actor and critic each have two 128-unit layers. Only one training seed (7) was tested. Because cloning already achieved 50/50 on development episodes, this experiment does **not** establish an incremental benefit from PPO or learning from scratch.

## Artifacts and provenance

- [policy.zip](policy.zip): SB3 PPO checkpoint; [policy.json](policy.json): required companion manifest, including full configuration and `roboturk.capture.v1` contract.
- SHA-256: `59d95fe767135d83d3fb3c5eaa4efdee158d0c478a7c8f20739e3bfbf2aac042`.
- [qualification.json](qualification.json): every held-out episode, aggregate metrics and checkpoint hash. [demo.json](demo.json): every recorded episode and post-capture retention.
- [bc-evaluation.json](bc-evaluation.json) and [development.json](development.json): before-PPO and final development results.
- [Demo video](../../media/chase-capture-demo.mp4): actual Webots movie output, six consecutive episodes, normal simulation speed, H.264 re-encoded for playback. No failures removed or generated animation substituted.
- Training implementation: local source commit `0645538`; corresponding published commit [`3b543642`](https://github.com/kadiru/metu-ros-pkg/commit/3b543642be1d2b2652569b4e79e228b39e572e44). Demo recorder and evidence were added afterward without modifying the policy.
- Runtime: Webots R2025a nightly 17/11/2026, commit `e5d1122a3cd59f007ee4b37c8f651b50e7ee83d2`; [runtime manifest](../../docs/runtime-manifest.json), [Python environment](../../requirements-macos-validation.txt).

Load with `CheckpointPolicy(path, Skill.CHASE, Config())` and register with `BehaviorLibrary`, or use `--mode capture-evaluate` / `capture-demo` with `scripts/run_webots.py`. Keep the JSON next to the ZIP. Skill, observation/action contract and complete configuration must match.

## Limits

Qualification uses ground-truth state, one robot, no opponents or obstacles, ideal body velocity and a bounded spring/damper dribbler. It does not establish hardware transfer, wheel/roller realism, noisy perception, latency tolerance, contested possession, match performance or robustness across training seeds. The learned controller currently stops after capture; purposeful dribbling and passing require further skills and evaluation. Intermediate demonstration arrays remain local and can be regenerated with the published training script.
