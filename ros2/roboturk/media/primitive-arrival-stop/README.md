# Braking-aware primitive arrival

The released arrival drill uses position feedback and a terminal-velocity feed-forward. For field position `p`, target `g`, terminal velocity `v_g` and robot heading `psi`, it computes `v = clamp_norm(v_g + 3.2*(g-p), cap)` and rotates the result into the body frame. The selected stop profile caps the request at 2.4 m/s. The heading request is `clip(5*wrap(psi_goal-psi), -8, 8)` rad/s. A moving arrival can route behind the target to line up the requested velocity.

The new `braking` profile is available for stationary arrivals. It takes the smaller of cruise speed and the speed that can stop with a one-tick reaction margin. For remaining usable distance `d = max(0, |g-p|-0.004)`, braking authority `a = 0.7*max_acceleration`, and control period `T = 0.04 s`, it solves `v*T + v^2/(2*a) = d`. The direction is toward the target. The command subtracts 0.65 times measured cross-track velocity, uses a 4 mm positional deadband, and compensates for the rotation expected during half a control tick. The Webots backend slews each 2 ms physics step from measured velocity with a 5 m/s² planar acceleration limit.

This is a deterministic feedback profile. Frozen drill profiles and the 5v5 match select their existing controllers. The articulated four-wheel bench is a separate test of motor and actuator behavior.

## Native Webots comparison

Each of 20 starts from rest and 20 starts with incoming velocity was run with both profiles from identical reset states. The ordinary starts span 0.9–2.3 m and random pose headings. Moving starts arrive at 1.8–2.35 m/s with lateral velocity and start just outside their theoretical stopping distance. A stop must stay within 1 cm of the target pose, 0.05 m/s of zero speed, and 0.04 rad of the desired heading for 0.25 s. Settling time is measured at the start of that confirmed window. Overshoot is measured along the original approach direction at the 2 ms physics rate. [Measurements](measurements.json)

| Initial state | Proportional median | Braking profile median |
| --- | ---: | ---: |
| From rest | 1.721 s | 1.248 s |
| Already moving | 0.927 s | 0.613 s |

Both profiles settled all 40 trials. Neither crossed the target plane in these starts. The new profile shortened the final approach in this panel. The test used the existing body-velocity chassis on an open field.

From the RoboTurk ROS 2 directory with the project environment active:

```sh
python scripts/run_webots.py --webots /path/to/webots --mode primitive-arrival-check \
  --seed 51200 --episodes 20 --timeout 120 --output runs/arrival-check/results.json
```

## How the 5v5 match plans movement

The role allocator supplies a target point. Standard role commands use proportional position and heading feedback, then team-local steering considers the observed positions and velocities of nearby robots. For eligible off-ball roles under the timed planner, `Leg.to()` constructs independent acceleration–cruise–braking profiles for x and y using measured initial velocities. Per-axis speed and acceleration budgets are divided by the square root of two. The direct timed path is checked against field limits and predicted positions. If blocked, it tries four offsets, then an Extended RRT waypoint when enabled, then requests braking if no route is accepted. The first 40 ms of the chosen plan is issued, and the plan is reconsidered on the next observation. The Webots backend applies the same acceleration cap to the body-frame request.
