# Migration audit and design decisions

Inspected November 17, 2026 (America/Los_Angeles). Historical source: `kadiru/metu-ros-pkg`, revision `841298f3daa5c723a183c29ea0d8617c22d66c8a`.

## Archive findings

| Original component | Finding | Modern replacement |
| --- | --- | --- |
| `ssl_msgs`, `ssl_srvs` | ROS 1 message headers and global state; separate ball/robot/field messages | `roboturk_interfaces`, explicit team+ID, SI units and documented frames |
| `ssl_path_planner_` | Substantive OMPL/CGAL implementation; the version without trailing underscore is a separate scaffold | Future collision-aware planner; current analytic baseline is deliberately simple |
| `ssl_game_planner_` | Refbox/game-state code, target poses, back-and-forth experiments; not a finished modern soccer team | `TeamStrategy` → `Play` → role intents → `BehaviorLibrary` |
| `ssl_webots_sim_/protos/SSLRobot.proto` | Old `Servo`, string `controllerArgs`, outdated Physics contact fields | R2025a world/PROTO syntax, current Supervisor API, WorldInfo contact properties |
| `hardware_test/main.cpp` | `N_WHEELS = 3`, radius 0.025 m; four-wheel block commented; old `getServo`/position-velocity control | Body-command contract now; measured wheel model later |
| Historical overview | Describes five four-omniwheel robots | Do not silently assume the simulator is the same hardware revision |
| `Ball.proto` | Generic old model defaults radius 0.043 m, mass 0.026 kg | Explicit prototype ball radius 0.0215 m, mass 0.046 kg |

The historical ball defaults may be overridden in some worlds; this is a source discrepancy to audit, not proof every old simulation used the wrong ball. Similarly, no historical planner benchmark was rerun.

## Version choice

- [Official Webots latest release](https://github.com/cyberbotics/webots/releases/latest) resolves to R2025a. Its Linux assets list Ubuntu 22.04 and 24.04.
- [ROS upstream distribution table](https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases.rst) lists Lyrical (May 2026–May 2031) and Jazzy (May 2024–May 2029). The initial pairing uses Jazzy/Ubuntu 24.04 for the simulator's published platform support; this does not imply Jazzy is the newest ROS.
- Original R2025a failed locally on macOS 27 with all controller ports reported occupied. [Upstream PR #7032](https://github.com/cyberbotics/webots/pull/7032), merged October 3, fixes this detection issue. The official [November 17 nightly](https://github.com/cyberbotics/webots/releases/tag/nightly_17_11_2026) is kept as a distinct project-local validation runtime.
- [Current Supervisor API](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/supervisor.md), [contact properties](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/contactproperties.md), [Python controllers](https://github.com/cyberbotics/webots/blob/R2025a/docs/guide/using-python.md).
- [Gymnasium episode API](https://gymnasium.farama.org/api/env/) and [SB3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html).

## Boundaries

The core imports neither ROS nor Webots. ROS handles coordination/transport; the Webots Supervisor owns stepping/reset and exposes ground truth. Gymnasium owns task reward/termination. Policies consume a versioned observation vector and return normalized actions. A skill can switch from analytic to learned without changing role assignment.

Training and deployment share the encoder, action decoder and skill actuator masks. Termination is initially implemented in the Gym tasks; team play uses possession, a pass timeout and the world state to decide transitions. Full behavior execution actions with cancellation/deadlines and a policy qualification registry are follow-up work.

## Observation/action contract

`roboturk.skills.v1`: 17 base scalars, 7 skill one-hot scalars, and 5 teammate plus 5 opponent slots of 6 scalars each (84 floats by default). Includes body velocity, relative ball state/visibility, target vector, attacking direction, absolute field position, heading, possession, neighbor velocity/position, visibility mask and intended-receiver flag. Neighbor order is distance then team/ID; zero slots have mask zero. Absolute field position and heading are retained to expose boundaries; this version is not rotation-invariant.

Actions: `[vx, vy, omega, dribbler, kick]` in `[-1,1]`. Translation maps to m/s and is radially clipped; omega maps to rad/s; negative dribbler/kick values disable the actuator. Kick requests are edge-triggered in both backends. No kick is available to move/chase/dribble/receive. Inputs and outputs are finite-checked.

## Curriculum and acceptance

Move: reach a target. Chase: acquire the ball at the mouth. Receive: capture an incoming ball. Dribble: carry to target while retaining possession. Pass: the designated receiver captures the ball at low relative velocity; after a valid kick the passer holds, matching the team play's transfer of responsibility. Shoot: the whole ball crosses the opponent's goal line inside the mouth, interpolating the crossing location between frames. Except scoring, success requires three consecutive control frames. Time limits truncate; collisions/out-of-bounds terminate as failures.

Webots resets flush deferred pose/physics resets with one basic physics tick before applying initial velocities. That settling tick is outside episode time. Zero-duration steps alone did not flush the physics reset, which silently erased an incoming ball's velocity. The observed failure is covered by a deferred-physics regression test and actual receiving rollouts.

The seeded initial scenarios are small and mostly unobstructed. Their success is a plumbing baseline, not general soccer proficiency. Before qualification, use fixed held-out scenarios, multiple seeds, randomized dynamics and perception, and failure metrics. Multi-agent curricula should eventually move to a parallel environment API while preserving per-robot inference contracts.
