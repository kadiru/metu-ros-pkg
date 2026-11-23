# Receiving a pass on the updated carpet

Both receivers face the same pass on an unguarded Webots opening. The original receiver follows the ball until it stops and times out at 16 s. The fitted receiver secures the ball, carries it forward and scores at 14.82 s. The ring follows the visible ball in the native recordings. [Full original recording](original-timeout.mp4) · [Full fitted replay](fitted-replay.mp4)

![Both receivers on the same opening](reception-replay.gif)

On the original 40 paired team openings the old policy scored 40 goals on the previous simulator and 29 on the updated carpet. The fitted receiver scored 40 on the updated carpet. Its 18 coefficients were learned from 1,235 labeled Webots states on 24 separate training openings. [Policy and training code](../../policies/carpet_receive_v1/README.md)

The fresh demo on seed 32703000 shows the fitted team receiving, carrying and scoring in 14.20 s. The passer starts with the ball and the goal is unguarded. [Uncut fresh demo](fresh-demo.mp4) · [Episode log](fresh-demo.json)

![Blue passes, cyan receives and scores](pass-receive-score.gif)
