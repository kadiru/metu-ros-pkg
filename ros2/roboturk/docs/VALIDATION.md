# Validation — November 19, 2026 (America/Los_Angeles)

## Qualified learned tactical decisions and self-play

Scripted tactical targets and reception-triggered handoffs are replaced by a
75-input/12-output neural manager. Imitation/DAgger supplies initialization and
learned coordinate regression. Two self-play generations update its decision
readouts and both planar neural opponents: 196,608 native PPO steps, 192 accepted
updates. Low-level motion actors and the coordinate regressor remain frozen during
this PPO stage. No isolated PPO benefit is claimed.

| Reserved panel | Goals | Defensive stops | Clean | Collisions | Unforced |
| --- | ---: | ---: | ---: | ---: | ---: |
| Final attackers / final defense | 107 | 126 | **233/240** | 2 | 5 |
| Final attackers / initial defense | 65 | 53 | **118/120** | 0 | 2 |
| Initial attackers / final defense | 60 | 57 | **117/120** | 1 | 2 |

All gates passed after models/runtime were published at
[6906019](https://github.com/kadiru/metu-ros-pkg/commit/690601957abde5cd40980290288a4806c198db26).
The final pair's seven failures are two collisions, four ball-outs and one possession
loss. Clean outcomes include 41 interceptions and 85 keeper saves. Final clean
Wilson 95% interval: 94.10–98.58%; goal interval: 38.43–50.91%. Gates use point estimates.
Every outcome is retained; there was no reserved-case tuning.

The final panel contains 80 actual passes, 35 stable receptions, 114 controlled-carry
episodes, 192 shots and 91 goals without a prior pass. Each family passed its
clean/goal gates. All 62,767 manager decisions and both opponents' continuous
calls/physics checks were audited, along with physical kick/reception, whole-ball
goals, target-update gates and immutable hashes. The manager changed acting robots
in 37 episodes. No teacher planner or scripted handoff runs in deployed inference.

Fresh demo: **9/9 clean**, five goals, two interceptions and two saves. The
[45.6-second native Webots movie](../media/learned-decisions-self-play-demo.mp4)
retains all 1,140 frames at 1280×720/25 fps. Episode 3 (6.84–18.64 s) includes the
complete learned pass/receive/handoff/dribble/shot sequence. Both video decodes,
timing/frame counts and all terminal frames passed QA; native edge-encoding warnings
are retained in provenance. No episodes or timing were edited.

All 270 development cross-play games are retained, plus narrower/broader failures
and the failed three-generation v1 league. Final broader development was 117/120;
extended development 235/240. Passing reserved gates does not erase these failures.

Local checks: 148 tests passed, one ROS-only module skipped, Ruff clean. Core and
ROS CI passed for the exact frozen head in both PR and push workflows. This uses
the native Supervisor, not a complete live ROS graph. Scope remains approximate
ground-truth initial-possession 2v2 with contact-based stops, fixed kick speeds,
three starting families and frozen motion skills; full-match/hardware behavior is
not established.

[Model card and all raw audits](../policies/decision_self_play_v2/README.md) ·
[Video provenance](../policies/decision_self_play_v2/demo-video.json) ·
[Protocol and replay](LEARNED_DECISIONS.md).


## Qualified mobile defense and alternating self-play

Five generations completed 655,360 native PPO steps across four roles, with
637 accepted / 640 attempted updates and six retained historical snapshots. All
four actors changed each generation. Supervised spacing preparations and a scripted
phase-transition correction are separate contributions; no isolated PPO gain is claimed.

| Reserved panel | Goals | Defensive stops | Clean outcomes | Collisions | Unforced errors |
| --- | ---: | ---: | ---: | ---: | ---: |
| Final attackers / final defense | 78 | 116 | **194/200** | 2 | 4 |
| Final attackers / bootstrap defense | 19 | 80 | **99/100** | 0 | 1 |
| Bootstrap attackers / final defense | 47 | 50 | **97/100** | 1 | 2 |

All panels passed unchanged predeclared gates after the freeze was published at
[a143963](https://github.com/kadiru/metu-ros-pkg/commit/a143963da3e7cc7d2061276e4c1b500d0e3b5d8c).
Clean outcomes include goals and legitimate defensive stops. Final-pair goals were
78/200 (39%), with 100 interceptions and 16 keeper saves. Six failures remain:
two defender collisions, possession loss, keeper field exit, invalid pass and attacker
field exit. Clean Wilson 95% interval: 93.61–98.62%; these gates use point estimates.
No reserved-case tuning, outcome selection or runtime/model changes followed freezing.

Both defensive actors made 82,189 calls in the final panel; the audit checks continuous
inference/physics sampling and real pass/reception/carry/shot/whole-ball crossing.
Stops use contact-distance or modeled mouth possession, not sustained-capture or
ODE contact-sensor guarantees. Maximum x excursions: defender 4.076m, keeper 0.670m.
All 720 matched-runtime development cross-play games are retained; final/final was
18/20 clean. The separate development attack/bootstrap panel failed with six collisions.
Passing reserved results do not erase those robustness failures.

[Fresh native movie](../media/mobile-self-play-demo.mp4): 8/8 clean, two goals, three
interceptions, three saves. All 1,830 frames retained, 73.2s, 1280×720, 25fps. Full
native/final decodes and all terminal frames plus passing/dribbling frames passed QA.
[Model card and raw audits](../policies/self_play_v1/README.md) ·
[Video provenance](../policies/self_play_v1/demo-video.json) ·
[Protocol/replay](SELF_PLAY.md).

Local checks: 116 tests passed, one ROS-only module skipped, Ruff clean. Core and
ROS CI passed on the frozen source and subsequent evidence publication. Native
qualification uses the Supervisor directly; live ROS/Webots applied-kick feedback
and complete graph validation remain separate. Scope: ground-truth approximate
2v2 drill, initial passer possession, scripted tactical targets/handoff, learned
motion/release. No full-match, official SSL rules or hardware qualification.

## Earlier qualified play against fixed learned defense

The four frozen policies scored **200/200 reserved native Webots cases**, with
**zero pre-release possession losses or robot collisions**. Mean15.2234s;
Wilson95% interval98.12–100%. Positive/negative goals102/98; passer roles99/101.
Mean ball displacement to reception2.9344m, retained receiver carry3.7730m and
shot distance0.7173m. Every success required an actual pass,0.4s stable reception,
retained carry, an actual shot and whole-ball crossing inside the posts.

The independent release audit verified every reserved seed1650000–1650199,
all four checkpoint/manifest hashes and71 frozen runtime inputs. Both opponents
remained active through the handoff:152,234 neural calls per opponent and608,936
physics checks each. There were487,852 possession checks. No analytic movement
fallback or handoff reset. [Complete model card and raw evidence](../policies/learned_opponents_play_v1/README.md).

Complete-play development passed120/120 before the freeze was published at9fec0371.
Each attacking actor trained from300 demonstrations,150 mixed DAgger episodes and
65,536 anchored PPO steps. Transfer BC/DAgger/PPO60/60; new finisher source0/60,
BC57/60, DAgger/PPO60/60. No incremental PPO success-count benefit is established.
The old full play scored45/60 against these opponents, with15 keeper saves.
Failed close-only teacher32/40 and revised teacher60/60 remain development evidence.

The independently frozen neural defender intercepted100/100 reserved direct passes;
the neural keeper saved99/100 direct shots at3.5m/s longitudinal speed. The event
predicate is center distance within summed radii +2mm or modeled mouth possession;
keeper success also requires no goal through2s. It is not an ODE contact-sensor
measurement. One conceded goal1260065 is retained. Stationary development45/60 and
35/60; neural60/60 and59/60; teachers60/60 and57/60. Each defensive actor completed
32,768 anchored PPO steps. [Opponent model card and audit](../policies/learned_opponents_v1/README.md).

This is a bounded approximate, ground-truth Webots curriculum. The passer starts
with possession. Tactical receiving/finishing targets and role handoff are scripted;
neural actors provide motion and release. The keeper/defender retain fixed role
anchors, speed/acceleration limits and latency context. It is not an evolving
self-play league, unrestricted match play or a calibrated hardware result.

The [fresh native demo](../media/learned-opponents-pass-receive-score-demo.mp4) scored6/6,
zero losses/collisions. All2,346 frames are retained at25fps,1280×720,93.84s. Both
native and published files fully decoded; representative pass, carry, keeper and
goal frames were inspected. All six reserved demo seeds happened to attack +X;
the qualification includes98 successful -X cases. No runtime/model changes after
freeze or qualification. [Video provenance](../policies/learned_opponents_play_v1/demo-video.json).

## Previous analytic-opponent keeper play

Frozen qualification: **199/200 goals**, one defender interception, zero pre-release
possession losses or robot collisions. All199 completed receptions ended in goals.
Wilson95% interval97.22–99.91%; mean successful sequence14.943s. Six fresh consecutive
native Webots demo cases scored6/6;99.16s,2,479 frames,1280×720 at25fps.
[Model card and raw evidence](../policies/keeper_finish_v1/README.md).

The two attackers use the frozen learned transfer actor and a new learned finisher.
The midfield defender and keeper are bounded analytic opponents. Target/phase
selection is scripted, motion and release are learned. Standalone development was
60/60 before and after constrained PPO; no incremental PPO success benefit is claimed.
Separately, [block/recover](../policies/block_shot_v1/README.md) passed200/200 physical-retention cases and3/3 demo cases. [Coverage](../policies/cover_pass_lane_v1/README.md) passed200/200 geometric lane-hold cases and3/3 demos. Mean completion2.2179s and3.4481s respectively. The combined native defensive video preserves all489 frames (19.56s). Their actors are separate from the fixed attacking-test opponents.

## Verified locally

- 90 Python tests pass (including keeper/defensive contracts, mirrored goalkeeper positioning, automatic PPO resets and inference parity). They include Gymnasium contracts for all six tasks and capture/dribble/scoring/team-transfer/defended-transfer curricula; seeded replay; coordinate/target rotation invariance; finite actions; actuator limits and retain/fire semantics; HALT/STOP/stale-input overrides; full-ball goal crossing in both directions; rejecting premature shots and pushed-in balls; possession loss between control frames; arrival hold; passing/receiving and role persistence; reset clearing; the deferred Webots reset regression; actual-pass/reception gating; collision rejection; unchanged physics at the scoring handoff; play-memory agreement with the task monitor; defender interception/collision checks between control frames; continuing opponent control through scoring; body/field action conversion; and fixed, mirrored receiving-point targets shared by the policy and monitor.
- Ruff static checks pass and Python sources compile.
- Both Python packages built and installed as editable packages; `pip check` reports no broken dependencies. Exact local versions are recorded in `requirements-macos-validation.txt` (a macOS validation environment, not a universal ROS lockfile).
- ROS-specific tests are skipped locally because this Mac has no ROS 2 installation. GitHub Actions on Ubuntu 24.04 / ROS 2 Jazzy built all three packages and passed **2/2 ROS bridge/interface tests** at commit `3b543642`. These tests exercise conversions, command/referee watchdogs and reset with a reference backend; a complete Webots/ROS live-graph run is still pending.
- The original source checkout remains unchanged.

## Webots measurements

Runtime: Webots R2025a Nightly Build 17/11/2026, commit `e5d1122a3cd59f007ee4b37c8f651b50e7ee83d2`, Apple Silicon macOS. [Runtime provenance](runtime-manifest.json). Original stable R2025a failed on the known macOS 27 TCP-port detection issue.

After diagnosing on seeds 1000–1009, the final analytic baseline ran against **fresh seeds 2000–2009**, ten episodes per skill:

| Skill | Successes | Episodes |
| --- | ---: | ---: |
| Move | 10 | 10 |
| Chase | 10 | 10 |
| Dribble | 10 | 10 |
| Pass and receive | 10 | 10 |
| Receive incoming ball | 10 | 10 |
| Shoot | 10 | 10 |

[Raw Webots episode results](webots-baseline-results.json). These are small, unobstructed curriculum scenarios with analytic policies and approximate dynamics. They do **not** establish robustness against opponents, calibrated physical dribbling, competitive gameplay, or learned-policy performance.

Actual simulator testing caught three issues that the synthetic backend did not reveal: dynamic PROTOs need an `IMPORTABLE EXTERNPROTO` declaration; deferred physics reset erased initial ball velocity; and a loose kick-heading tolerance missed the receiving robot. Passing also needed a hold phase after releasing the ball, matching team-play behavior. All are corrected in the recorded final runs.

Webots emits an Apple GPU capability warning and a texture-loader warning. The simulator parsed the authored world and PROTOs, stepped the Supervisor controller, created robots dynamically, and wrote the recorded episode results successfully. Native movie recording also completed; the resulting H.264 video was decoded and inspected.

## Qualified learned capture policy

The new `capture.randomized.v1` task tests full-circle robot headings/bearings, starting distances 0.35–2 m, and stationary or initially rolling balls (0.15–0.6 m/s). Success requires 0.4 s of continuous mouth possession, low relative speed and active dribbler.

The neural checkpoint achieved **200/200** reserved episodes (seeds 50000–50199), comprising **100/100 stationary** and **100/100 moving** cases. Mean capture time: **1.628 s**; Wilson 95% interval: **98.12–100%**. Six consecutive new demo seeds all succeeded and retained the ball for another second under the same policy. [Model card and episode records](../policies/chase_capture_v1/README.md) · [Actual Webots video](../media/chase-capture-demo.mp4).

Training used 300 demonstrations, 150 DAgger episodes and 131,072 PPO steps, seed 7, one CPU thread. Teacher commands are absent from inference. Development was 50/50 both before and after PPO, so these results do not establish PPO's incremental benefit. This is qualification for the stated unobstructed approximate simulation distribution only.

## Qualified learned capture-and-dribble policy

The frozen goal-conditioned policy passed **200/200** reserved episodes (150000–150199), with **zero possession losses across 438,456 physics-step checks** after acquisition. Both attacking directions and stationary/initially rolling balls were included. Mean progress toward the goal-side target was **3.5867 m**; mean duration **13.0177 s**, including the final one-second stationary hold. Wilson 95% success interval: **98.12–100%** for this distribution. [Model card, qualification and all development outcomes](../policies/capture_dribble_v1/README.md).

Development exposed a real regression: unconstrained PPO lost possession during arrival in 60/60 cases. The released experiment restarted from the successful cloned actor and used 65,536 PPO steps with reference-policy anchoring; it passed 60/60 development before the checkpoint was frozen. The clone itself already achieved 60/60 and was faster on development, so no incremental PPO benefit is claimed. Qualification uses actual Webots and the loaded `CheckpointPolicy` inference path, with no teacher fallback.

Core and ROS CI passed for source commit `10089703`, including the new training mode. The [native simulator demo](../media/capture-dribble-demo.mp4) passed **6/6 fresh scenarios**, retaining possession for another 0.8 s after arrival in all six. A camera-only rerender produced identical episode metrics. The complete 81.96 s H.264 video decoded successfully, and route/tracking frames were visually inspected; only the camera and nonphysical target marker affect presentation.

## Qualified learned capture-dribble-score policy

V2 scored **200/200** new reserved scenarios (350000–350199), with **zero invalid shots and zero pre-shot possession losses** across 241,996 physics checks. Stationary/rolling subsets were 112/112 and 88/88; positive/negative goals 106/106 and 94/94. Mean full-sequence time **8.3683 s**, controlled progress **3.1708 m**, shot distance **1.2865 m**. Each goal required exactly one applied kick after acquisition and at least 0.5 m of retained carry, followed by full-ball crossing inside the opponent's posts. [Model card and complete records](../policies/capture_dribble_shoot_v2/README.md).

The earlier v1 scored 192/200 and 5/6 demo scenarios, with premature-shot failures retained. V2 adds explicit radial goal distance and new recovery data; its initial range-only refit was 0/60, while DAgger and anchored PPO each scored 60/60 on new development seeds. All stages remain in the published evidence. This does not establish an incremental PPO success benefit. Qualification uses a single loaded MLP, with dribbler-until-fire and fixed 3.5 m/s kick semantics, against an unguarded goal in approximate dynamics. Core and ROS CI passed at training source commit `5edfd064`.

The [new native simulator demo](../media/capture-dribble-score-demo.mp4) scored **6/6**, on preselected seeds 360000–360005. All outcomes are retained. Full H.264 decoding and visual inspection of route and goal frames verify the recording artifact. The [earlier 5/6 demo](../media/capture-dribble-score-v1-demo.mp4) remains available, including its failure.

## Qualified learned two-robot passing and scoring play

The frozen play scored **200/200** reserved scenarios (550000–550199), with **zero premature kicks, possession losses before either release, or robot collisions** across **399,816 physics possession checks**. Negative/positive goals: 95/95 and 105/105. Passer ID0/ID1: 87/87 and 113/113. Every goal followed exactly one passer kick, at least 1 m of ball displacement, 0.4 s stable reception, at least 0.5 m of retained progress toward goal, and one receiver kick. Whole-ball goal crossing is required. Mean full sequence: **12.3278 s**; pass-to-reception ball displacement: **1.9138 m**; receiver carry progress: **2.7488 m**. [Model card, checkpoints and every outcome](../policies/pass_receive_v2/README.md).

The initial 34-input transfer experiment achieved 54/60 cloned, 37/60 DAgger and 56/60 final PPO development receptions. All failures were pass-release timeouts and all 64 actor updates were rejected. V2 adds two explicit signed aiming-error inputs. Refit, new DAgger and constrained PPO each passed 60/60 transfer development; the full play then scored 60/60 before qualification. V2 accepted all 64 actor updates, but no incremental PPO success benefit is established. All stage results, original experiment checkpoints and mixed-collection failures remain published.

The play composes a centralized learned transfer network with the frozen learned scorer. Role assignment, passer hold after release, receiver handoff after stable possession and fixed kick speeds are explicit interface rules. Full evaluation invokes `PassReceiveScorePlay` and checks its memory against an independent task monitor. The ball is never reset at handoff. The passer starts holding a stationary ball; both headings, teammate positions, robot roles and attack direction vary. Ground-truth state, unguarded goals and approximate dynamics only. Source core and ROS CI passed at commit `04a6ee14`; complete live ROS integration still requires actual kick feedback in the message graph.

The [native two-robot demo](../media/pass-receive-score-demo.mp4) scored **6/6**, on consecutive preselected fresh seeds 560000–560005. All outcomes are retained. The 77.64 s, 1280×720, 25 fps recording shows actual Webots physics at normal simulation speed. The complete H.264 file decoded successfully; passing, reception/carry and goal frames were inspected. No generated animation or teacher fallback is used.

## Qualified defended passing and scoring play

The frozen v3 play scored **200/200** reserved scenarios (850000–850199), with **zero interceptions, pre-kick possession losses or robot collisions** across **407,216 possession checks** and **531,960 defender physics checks**. Wilson 95% interval: 98.12–100%. Negative/positive goals: 99/99 and 101/101; passer ID0/ID1: 100/100 each. Mean complete sequence: **13.299 s**; ball displacement to stable reception: **2.8343 m**; controlled receiver carry: **3.2496 m**. Each success required one actual pass and one actual shot, with the whole ball crossing inside the goal posts. [Model card and every outcome](../policies/defended_pass_receive_v3/README.md).

The initial direct lane was blocked (maximum initial clearance 0.02475 m). At release, minimum lane clearance was 0.7453 m; minimum ball/defender center distance throughout a complete sequence was 0.4750 m. The independent analytic lane defender patrols its original passing line and tracks incoming-ball crossings, with randomized speed 0.25–0.55 m/s, period and phase. It remains active during scoring. This is a bounded lane-guard curriculum with ground-truth approximate dynamics.

The play now selects a receiving point on the opposite flank with 0.65 m touchline margin. Target selection and stable-reception handoff are scripted. A 62-input/seven-output centralized neural policy learns both attackers' motion, pass timing and reception; the previous frozen scorer handles carry/shoot. No analytic attacker motion fallback, physics reset or action shield is used. Full learned evaluation invokes `PassReceiveScorePlay` and checks its memory against the task monitor.

Earlier candidates are retained: v1 received 38/60 development passes; v2 received 59/60 but failed full qualification at 185/200 (11 interceptions and four other failures). V3 used 300/300 fresh teacher receptions,150/150 mixed DAgger receptions, and development 59/60 cloned then 60/60 after both DAgger and constrained PPO. All 64 PPO updates were accepted; no incremental success benefit over DAgger is established. The broader full-play development check scored 120/120 before freezing. No reserved qualification/demo observations entered training. Source core and ROS CI passed at `f380ec8a`; complete Webots/ROS live-graph validation remains separate.

The [native defended-play demo](../media/defended-pass-receive-score-demo.mp4) scored **6/6** on consecutive preselected fresh seeds 860000–860005, with no interceptions, losses or collisions. The 84.12 s, 1280×720,25 fps recording includes every outcome at normal simulation speed. Native and final H.264 files decoded completely without errors; actual pass-around-defender, receiver dribble and whole-ball goal frames were inspected. Frozen checkpoint hashes matched after recording. Video provenance is in [demo-video.json](../policies/defended_pass_receive_v3/demo-video.json).

## Synthetic backend

[Recorded analytic baseline results](synthetic-baseline-results.json) are a software comparison only. The synthetic backend deliberately simplifies contacts and snaps a held ball to the robot mouth. It must not be confused with Webots physics or hardware evidence.

## Historical PPO training/checkpoint smoke test

Actual Webots backend, chase skill, seed 7, exactly 2,048 steps, SB3 PPO, CPU device with one training thread. Training completed and saved `runs/webots_smoke/policy.zip` plus its contract/config manifest. Loading and inference passed; loading the chase checkpoint as a passing skill correctly raised a compatibility error.

The reloaded policy achieved **0/10 successes** on held-out seeds 3000–3009 (out-of-bounds or collision). This short run verified training and persistence, not skill learning quality. It preceded the specialized capture curriculum and qualified checkpoint above. [Recorded training manifest and evaluation](ppo-smoke-results.json). This obsolete smoke checkpoint remains an ignored local artifact.

## Remaining qualification work

Full Webots/ROS live graph on Linux; motor/roller/dribbler/kicker calibration; official vision/referee adapters and full rule enforcement; guarded goals, obstructed/adversarial scenarios; multiple training seeds and ablations; passes to moving teammates, coordinated acquisition and variable kick power; full team match evaluation; recurrent policies; multi-agent training and self-play; physical transfer. No GPU training has been launched.
