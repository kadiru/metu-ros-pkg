# Choosing the carrier motion at first touch

Team 8 scored 30 qualifying goals and lost possession twice before the first pass in 40 fresh Webots openings. The previous team scored 26 and lost possession three times on the same starts. Neither team had an opening pass intercepted. [All 40 matched outcomes](comparison-8.json)

A decision tree fitted to self-play returns chooses one of two carrier responses when the ball winner first takes control. It uses the robot’s angular velocity relative to the side of the roller from which the ball is moving. The learned split is 1.5 rad/s. The selected team scored 23/28 with zero pre-pass losses in development. It also turned two of the five historical losses into pressured receiver goals. [Frozen policy](planned-policy-8.json) · [Development openings](training/development-selector-8.json) · [Historical rematches](historical-8/)

Both teams in the preregistered pair met the threshold. Team 9 earned the release with 31 qualifying goals on its separate 40-opening panel. [Selection rule and seed blocks](paired-challenge.json) · [Team 9](attempt-9.md)
