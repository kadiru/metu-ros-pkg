# Scoring after recovery under pressure

The first scoring policy improved on the previous outlet team in two independent studies. It converted 28 of 60 opening contests, below the target of 36. Training records showed that recovering players sometimes lost the ball while turning and sometimes shot out or into an intercepted lane. [First policy and all 174 games](../recovery_attack_v1/README.md)

This experiment gives the ball carrier four additional combinations of turn speed and motion away from a nearby opponent. During a pass or shot the carrier can pivot around the ball while translating. The new shot planner compares seven points inside the goal and estimates where intervening robots will be when the ball reaches them. Every configuration uses the same kick speeds, field, 3v3 defenses and underlying Webots contacts. The original controller is also available in training.

Both colors play each of 120 training layouts under five motor configurations. The learner selects the control with the highest count of actual opening-recovery goals. On a further 300 layouts it tries each of pass, dribble and shoot and fits opening returns from the resulting play. Five-fold validation keeps all trials of a layout in the same fold. The published protocol specifies the development comparison and held-out thresholds. [Protocol and seed ranges](protocol.json)
