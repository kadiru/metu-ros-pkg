# Continuous 2v2: recovery, support and counterattacks

Started November 20, 2026, America/Los_Angeles. This is a new experiment.
The qualified tactical-decision release and its frozen runtime remain unchanged.

## Runtime contract

Both teams have two robots and opposite goals. The v4 shared policy uses122 robot-centered, attack-normalized observations
(including match clock, score and34additional localgeometry measurements) and35 outputs: seven behavior scores and seven
separate four-output neural motion heads. Argmax selects the executed motion head,
so behavior choice changes the physical command. The selected head supplies body
velocity, rotation and release. It can execute recovery, dribbling, passing, shooting,
pressing, covering or support. Both colors and IDs use the same contract; there is
no permanent goalkeeper or privileged attacking side. Seed, scenario label, teacher
state and match outcome are excluded from observations.

Contact, interception, possession loss and missed shots do not end a match.
Whole-ball goals and ball-outs cause explicit logged referee restarts. These reset
the formation and ball and are simplified rules, not official SSL set pieces.
No other pose reset, action projection or analytic movement fallback is permitted
during neural evaluation. Physics remains the pinned native Webots ideal body
velocity/spring-dribbler/velocity-kick model; no hardware fidelity is claimed.

Possession is exclusive: simultaneous mouth contact by multiple robots is contested,
not assigned by robot iteration order. Stable acquisition requires 0.3 seconds of
exclusive measured mouth control and relative speed below 0.25 m/s. A turnover
requires a later stable owner from the opposing team. Stops alone receive no win
credit and never terminate play.

## Training and evaluation plan

Use native teacher demonstrations followed by mixed DAgger to initialize a neural
policy. Teachers are training-only, and their outcomes are labeled accordingly.
Then use PPO against immutable historical policies, sampling controlled robot/color,
teammate and opponent snapshots. Retain reset/pool records, actual actor changes,
trust-region accept/reject decisions, failures and every historical checkpoint.

Independent training seeds are 611 and 622. Primary release seed 611 is selected
in advance; seed 622 checks training-seed variability and supplies an unseen
cross-training-seed opponent. Report both, without selecting the better held-out result.

| Purpose | Seeds |
| --- | --- |
| Teacher pilot development | 4100000–4100099 |
| Native demonstration collection | 4200000–4200999 |
| DAgger collection | 4210000–4210999 |
| PPO resets only | 4300000–4399999 |
| Development and historical cross-play | 4400000–4499999 |
| Reserved qualification | 4500000–4599999 |
| Fresh consecutive uncut demo | 4600000–4600002 |

Initial plan: two self-play generations of 32,768 learner steps per training seed,
131,072 total, with randomized roles and both attacking directions. Both teams
continue after possession changes. Match duration is 45 simulation seconds.
Pilot/collection changes are allowed on development data and must be logged before
freezing. Numerical release gates will be fixed after the physics/teacher pilot,
before reserved evaluation. Freeze and publish models/runtime before reserved cases.

Report score, win/draw/loss, goal difference, exclusive possession duration,
contested duration, successful acquisitions/recoveries, turnovers and subsequent
play, physical passes/receptions, off-ball movement, contacts and field exits.
Do not substitute valid defensive contact for winning a continuous match.
Check whole-ball crossings, explicit restart ledgers and uninterrupted policy calls.
Compare historical opponents and both training seeds. No claim of an isolated PPO
benefit without a controlled comparison.

The deliverable is an uncut native Webots recording of the three consecutive fresh
matches, including every outcome, plus code, checkpoints and complete evidence on
GitHub. Multiple attacks and possession transitions must occur without contact
triggering resets. Vision noise, full ROS graph integration, calibrated dynamics,
official SSL rules and hardware transfer remain later work.

## Retained development failures

The initial six 30-second teacher games completed with no goals. Multiple dribblers
could grip the ball simultaneously; the old first-match ownership test biased
possession credit and the teacher could deadlock. These results are retained as
a failed, pre-exclusive-possession pilot; their possession totals are not qualified.

The next six 45-second teacher games, using exclusive/contested control and escape
passes, completed with zero goals, eight passes, six receptions and four matches
with turnovers. No robot contacts or field exits. Two-dimensional friction made
steep-angle shots miss; a central shooting setup is being tested. Both pilot files
are native teacher results, not neural-policy performance.

The central-shot pilot completed6/6matches,18passes,8receptions,3turnovermatches,
zero goals/contacts/fieldexits. Restrictive shot eligibility and changing goal-corner
selection delayed shooting. A revised fixed near-corner/angle-compensated teacher
is under development; all changes are training labels, not neural runtime overrides.

The v2 factored-head/angle-compensated pilot completed6x45s:four goals (blue1/yellow3),
21passes,six stable receptions,four matches with turnovers,zero robot contacts or
fieldexits. Three games had goals, including one1–1draw. Teacher collection now uses
30fresh matches4200000–4200029. These are analytic demonstrations, not neural results.

## Preparation results and metric amendment

Native demonstration collection completed30/30matches:29goals and80passes,
zero contacts or fieldexits. Dataset135224observations (see exact count in the
collection receipt). Initial80epoch neural fits failed all6development cases for
both seeds, with zero goals. Extending each fit by320epochs gave611:6/6completed,
1goal,3receptions,17.96%mean contact time,5fieldexitmatches;622:5/6completed,
0goals,1reception,18.77%contacttime,6fieldexitmatches. These are retained failures,
not reliable releases. Corrective native mixedDAgger is now collecting20matches
perseed with50%neural action selection independently for eachrobot/controltick.

Metric version2 invalidates old passes after an opponent acquisition, after5s,
or after the passer recovers its own kick. Stable reacquisition after at least
0.2s without exclusive mouth control is counted as recovery, including own fumbles
and successful pass receptions. Initial acquisition is reported separately. Raw
exclusive mouth occupancy remains the definition of possession seconds; it is
not the stable0.3s acquisition count. Cover and support distances are separated.
Old raw results retain their original metrics. No policy input changes.

The independent auditor counts a counterattack when exclusive stable control
after a turnover advances the ball at least0.75m toward goal within10s, before
the next opposing acquisition or referee restart. It verifies recorded whole-ball
crossings, pass-to-reception provenance, all four policy-call counts and explicit
restart ledgers. Contact is never relabeled as a win or a successful match outcome.

## Predeclared release checks

Before any reserved evaluation, `policies/continuous_play_v1/protocol.json` fixes
six20-match panels: self-play, own historical opponent and the independently trained
other seed, separately for611and622. Each training seed is assessed across60games.
Required:>=95%completion,<=2%mean contact time,<=5%fieldexitmatches,>=90%loose-start
acquisition,20goals (>=5percolor),20passes,15receptions,20shots,10counterattackmatches
and>=1mmean support movement percolor. Every event/hash audit must pass. These
measure functional continuous play and report actual W/D/L; they do not promise
winning against another competent policy. All failures remain, and gates will not
be weakened after reserved results. The fresh uncut demo uses611against622 for the
three preselected consecutive45s matches4600000–2, retaining all outcomes.

## Role-allocation diagnosis

Most V3contactframescamefromtwoteammatesbothcovering. Thetrainingteacher'slocal
hysteresis couldassignneitherrobottochase;mixedDAggerthenreinforcedthebadstate.
It nowassignsexactlyonenearestteammate. V4addsallfourrobot-to-ballrangesandthe
own/teammate difference. Runtimebehaviorandmotionremainneural. Recordednative
trainingstatesarere-labeledoffline withtraining-onlyteachers;theoriginaldatasets
andresultsremain. The0.30mseparation experimentstalledcaptureandisretainedas
afailure;traininglabelsreturnto0.23m withpositive separationwhenrobots overlap.
