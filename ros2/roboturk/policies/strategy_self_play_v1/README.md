# Learned skill strategy — integration in progress

This experiment adds the three qualified TDP skill actors to a ten-choice learned
strategy layer, with learned target parameters and target-refresh requests.
Native integration/preparation is underway; no new trained-strategy reliability
result is claimed yet. The prior isolated skill success rates do not establish
composed strategy performance.

See [architecture](../../docs/LEARNED_SKILL_STRATEGY.md) and the [predeclared
protocol](protocol.json). All failed pilots, exact launch sources and later
training/evaluation evidence will be retained. Earlier qualified policies and
videos remain unchanged.

## Preparation development (not reserved qualification)

The first DAgger refits failed to score and are retained. Corrected target-setting
regression and 96 additional native teacher matches produced the following
results on 16 fresh development matches per seed. All matches completed with
zero detected robot contacts and field exits.

| Training seed | Goals | Receive-stop | Redirect goals | Clearances |
|---|---:|---:|---:|---:|
| 811 | 6 | 4 | 2 | 4 |
| 822 | 3 | 3 | 2 | 5 |

Historical self-play training has started. Neither these development results nor
the previous isolated-skill results establish final strategy reliability.

The first historical league (seed 811) completed 65,536 native PPO steps over two
generations, accepting 58 of 64 updates. Both actors changed; the complete target
function and all motor policies remained fixed. Final-generation development:
8/8 completed, two goals, one stopped reception, one redirect goal and two
clearances, with zero detected contacts/exits. Both leagues have now completed: 131,072 total steps and 112/128 accepted updates.
These results do not isolate a PPO benefit from preparation.

The native camera preflight retains both 6-second cases and both rendering
versions. Adding a non-collidable ball ring left all 2,400 recorded physics states
identical. The ring is only a visibility aid for the GIF; the physical ball,
observations and controllers are unchanged. This is development footage, not the
reserved final demonstration.

The broader 64-match development check produced 12 goals with zero robot
contacts/exits. This was not enough margin to freeze the first candidates.
A second preparation is collecting correction data and fitting three separate
learned intent heads. The actor has 31 raw outputs, packed into the same 17-value
deployed action after its neural skill choice. Reserved tests remain untouched.
