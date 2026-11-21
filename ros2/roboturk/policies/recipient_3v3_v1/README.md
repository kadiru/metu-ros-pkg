# Choosing a pass recipient in 3v3

Six robots play one-minute Webots matches on a 9 × 6 m field. When the carrier
has a passing opportunity, a shared linear scoring function evaluates both
teammates. Each candidate has six features measured relative to the carrier,
ball, goal and three opponents. These describe passing lane clearance, marking
distance, forward progress, range, turn angle and receiver orientation. Robot
IDs identify recipients and do not enter the score. Choosing deterministically
at deployment uses the same features as sampling a receiver during training.

Each seed starts near uniform and plays 20 native matches. The learning side
alternates colors and faces a sampled historical receiver policy. Both teams
sample receiver choices. The score-function policy gradient uses +1 for an actual
intended reception and −1 for an interception, timeout, restart or end of match
after a kicked pass. Only the learning side updates its six parameters after each
match. Its checkpoint joins the opponent pool every four matches.

The pass motor comes from the previous ten-skill release. The 3v3 experiment
trains recipient selection. Carrying, shooting, support, defensive coverage,
receiver motion and geometric velocity limits use explicit controllers. The
carrier can keep the ball when both lanes are crowded. The game uses a 12-second
possession clock and randomized restarts after expiration, a goal or a ball-out.
Each restart permutes the robot IDs and places a defender near one candidate
lane. All six robots move throughout each match.

An intended reception requires a real kick, 0.55 m of ball travel and at least
0.20 seconds of exclusive mouth possession by the selected robot within four
seconds. An opponent or the other teammate taking the pass ends the attempt.
The audit checks the outcome against 10 ms native position samples and checks
the selected teammate against the policy's recorded candidate scores.
[Protocol and test cases](protocol.json).
