# A ball-path veto for the 5v5 Presser

`--tackle-selector` enables the selector alongside the fitted `--tackle-policy` checkpoint and the existing open-play steering policy. The role allocator selects the Presser and keeps its existing guard-box and touchline-recovery priorities. The selector uses the direct draw while it progresses. After 12 consecutive shared-grip samples with little progress, the selector may try a pivot. It can try a safe pivot earlier if the direct draw is vetoed and both rollers already reach the ball. Left and right describe the sweep in the opponent carrier’s frame.

Each control tick, the selector checks ball drift during the 80 ms reaction window and projects drift with each candidate's nominal pull direction. The geometric screen reserves clearance from the lines and checks for a release aimed through the defending goal mouth. An inward pull can counter outward drift. The selector can cancel an active sweep and selects the roller-off contain command when the draw is vetoed and neither pivot is available. The release envelope is a conservative heuristic with a short drift projection. Both the draw and the sweeps use the existing dribbler spring and request zero kick speed.

## Matched Webots panel

The panel compares direct draw, forced left pivot, forced right pivot, contain, and the selector from the same Webots poses and velocities. Each trial starts with both rollers in reach of the ball. Midfield, mirrored touchlines, and mirrored defending goals each have eight fresh starts. Carriers hold still, reverse, slide, or turn according to the matched seed. The keeper and seven other players are parked. The fixed pivots start immediately and use the same controller as the selector. Every option uses the same collision steering and the same post-capture hold. Touchline trials use board-aware steering so a robot can keep its roller on a ball at the line.

These are forced duels on the 5v5 field. A normal match also applies the role and positional eligibility checks before a duel can start. The outcomes measure the short challenge and give no estimate of long-match goals conceded.

- **Clean retained** requires exclusive roller control for the last 240 ms, with zero body contacts, ball-outs, own goals, and goalward releases during the trial.
- **Goalward release** is a loose ball moving at least 0.10 m/s toward the defending goal from within 0.65 m, with its current velocity ray through the mouth. An **own goal** requires a physical ball crossing.
- **Body contact** uses the existing 1 mm body clearance threshold and native 2 ms physics samples.

The forced left pivot put the ball into the defending goal at 0.56 s on the replayed own-goal start. The selector vetoed that pivot, chose the other side, and made body contact. On the replayed touchline start, the direct draw sent the ball out at 0.61 s. The selector switched to the inward pivot and kept the ball in play, with body contact. Those replays test the identified failures separately from the fresh panel.

[Per-zone results](results.json) · [Matched start states and outcomes](panels.json.gz) · [Hazard replays](hazard-replays.json) · [Normal 5v5 integration trace](integration.json)

The selector honored all its action vetoes. Its own-goal failure occurred after it switched to contain on seed 38724203. The ball continued across the line while both pivots and the draw were vetoed. The fixed direct draw kept the ball on that start. The selector stays opt-in because the fresh panel gives the direct draw both better retention near our goal and fewer hazardous outcomes overall.

<!-- results:start -->
### Results

| Zone | Choice | Clean retained | Ball out | Goalward release | Own goal | Body contact |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Midfield | Direct draw | 6/8 | 0 | 0 | 0 | 0 |
| Midfield | Pivot left | 4/8 | 0 | 0 | 0 | 4 |
| Midfield | Pivot right | 4/8 | 0 | 0 | 0 | 4 |
| Midfield | Contain | 0/8 | 0 | 0 | 0 | 0 |
| Midfield | **Selector** | 6/8 | 0 | 0 | 0 | 0 |
| Touchline | Direct draw | 6/8 | 1 | 0 | 0 | 0 |
| Touchline | Pivot left | 3/8 | 3 | 0 | 0 | 2 |
| Touchline | Pivot right | 3/8 | 2 | 0 | 0 | 3 |
| Touchline | Contain | 0/8 | 1 | 0 | 0 | 0 |
| Touchline | **Selector** | 5/8 | 1 | 0 | 0 | 1 |
| Own goal | Direct draw | 6/8 | 0 | 0 | 0 | 0 |
| Own goal | Pivot left | 6/8 | 0 | 0 | 0 | 2 |
| Own goal | Pivot right | 4/8 | 0 | 0 | 1 | 2 |
| Own goal | Contain | 0/8 | 0 | 0 | 1 | 1 |
| Own goal | **Selector** | 2/8 | 0 | 0 | 1 | 1 |
<!-- results:end -->

## Reproduce

Run from the modern stack root with Webots and the project Python environment. Each command evaluates all five choices.

```sh
for zone in midfield touchline own-goal; do
  for block in 0 4; do
    python scripts/run_five_roles.py "runs/panel-$zone-$block" \
      --tackle-panel "$zone" --seed "$((38724200 + block))" --tackle-trials 4 \
      --open-play-policy policies/open_motion_v1/actor.json \
      --tackle-policy policies/ball_side_tackle_v1/actor.json
  done
done
python scripts/compare_tackle_selector.py runs/panel-*/tackle-panel.json
```

To exercise the selector in a regular match, pass `--tackle-selector` with those two policy paths.
