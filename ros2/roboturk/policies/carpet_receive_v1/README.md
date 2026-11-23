# Receiving a slowing pass

This policy controls the receiver in the unguarded two-robot pass, receive and score play. The v2 transfer network selects the pass and positions both teammates. After an applied pass, a fitted controller takes over the receiver's three motion commands when the ball comes within 0.65 m. It measures ball position and velocity in the receiver's body frame. The existing play requires 0.40 s of stable possession before handing control to the scoring network. Pass and shot requests remain 1.8 m/s and 3.5 m/s.

On 40 matched openings the previous team completed 40 plays on the previous physics and 29 on the new carpet. All 11 failures were reception timeouts after a valid pass. On development seed 32701000 the receiver followed the decelerating ball until they were both stationary, with the ball still 0.229 m from the receiver's center.

The 18 receiving coefficients come from ridge regression on 1,235 labeled Webots states across 24 separate training openings. The existing capture demonstration controller supplied the labels. During collection the learned team drove the robots, with a 35% chance of applying the teacher's local receiver action on eligible steps. Inference loads the fitted coefficients from the manifest. This is supervised imitation learning. [Training episodes and fit error](receiver-fit.json) · [Weights and control contract](policy.json)

Use the checkpoint and its sibling manifest together. The ZIP contains the original transfer network. The manifest contains the fitted receiver weights and pins the ZIP hash. The scorer is the existing capture/dribble/shoot v2 checkpoint.

```sh
python scripts/run_webots.py --mode team-fit-receiver --checkpoint policies/pass_receive_v2/policy.zip --episodes 24 --seed 32710000 --output runs/refit --timeout 1800
python scripts/run_webots.py --mode team-evaluate --checkpoint policies/carpet_receive_v1/policy.zip --shoot-checkpoint policies/capture_dribble_shoot_v2/policy.zip --episodes 200 --seed 32702000 --output runs/carpet-team --timeout 2400
```

Reserved qualification uses seeds 32702000–32702199. The target is at least 190 full goals in 200 trials with zero recorded pre-kick losses or robot collisions. Demo seed 32703000 is separate. The passer starts in possession, both robots see native simulator state, and the goal is unguarded.

## Qualification

On 200 fresh openings the original team scored 163 goals and this team scored 200. The fitted team had 0 pre-kick loss trials and 0 collision trials. [Webots replays and paired results](../../media/carpet-skills/README.md)
