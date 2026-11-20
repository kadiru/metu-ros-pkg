# Continuous 2v2 — experiment in progress

This release is **not qualified** and has no new reliable neural demo yet.
The previous qualified decision_self_play_v2 release remains intact.

Seven learned choices select separate neural motion heads: recovery, dribble, pass,
shoot, press, cover and support. Both teams use the same 88-input/35-output contract.
Interceptions and missed shots continue; goals and ball-outs cause logged referee
restarts. Native Webots, ground-truth state and approximate dynamics; no hardware claim.

Thirty 45-second analytic demonstration matches completed:29goals,80passes,
zero contacts/fieldexits. Older reception totals are provisional:metric version2
now invalidates a pass on interception/expiry, counts same-robot recovery and
separates support from coverage distance. These amendments do not change observations.
The original collection and launch-source hashes remain intact.

Independent initializations611/622 each fitted80epochs to135224 native robot
observations. Both failed all six initial neural-only development matches with
field exits and frequent contacts; zero goals. These failures and both checkpoints
are retained. The extended fits add320epochs each using the same data; they are
being tested before corrective DAgger and historical self-play.

Primary seed611 was chosen before testing;622 is an independent replication.
Reserved seeds4500000+ and fresh demo4600000–2 remain unused.
Protocol:[CONTINUOUS_PLAY.md](../../docs/CONTINUOUS_PLAY.md).
