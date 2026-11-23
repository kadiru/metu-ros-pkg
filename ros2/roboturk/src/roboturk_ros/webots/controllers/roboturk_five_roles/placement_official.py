"""Drive native placement trials exclusively from live referee commands."""

from dataclasses import replace
import json
import math
import subprocess
import time

from roboturk.five_roles import CFG
from roboturk.model import Ball, Command, possession
from roboturk.placement import PlacementRestart, stop_formation, xy
from placement_runner import scenario
from traffic_training import crowded, nearest_pair


def exact_start(backend, world):
    """Set a paired fixture after tracker warmup, before either policy acts."""
    backend.reset(world)
    for r in world.robots:
        node = backend.nodes[r.key]
        node.getField("translation").setSFVec3f([r.x, r.y, 0.075])
        node.getField("rotation").setSFRotation([0, 0, 1, r.yaw])
        node.setVelocity([r.vx, r.vy, 0, 0, 0, r.omega])
    ball = world.ball
    backend.ball.getField("translation").setSFVec3f([ball.x, ball.y, CFG.ball_radius])
    backend.ball.setVelocity([ball.vx, ball.vy, 0, 0, 0, 0])
    if backend.supervisor.step(0) == -1:
        raise RuntimeError("Webots closed during paired setup")
    actual = backend.read()
    for r in world.robots:
        q = actual.robot(r.key)
        if math.dist(xy(r), xy(q)) > 1e-6 or abs(math.remainder(r.yaw - q.yaw, 2 * math.pi)) > 1e-6:
            raise RuntimeError("Paired placement start differs from its prescribed pose")
        if math.dist((r.vx, r.vy, r.omega), (q.vx, q.vy, q.omega)) > 1e-6:
            raise RuntimeError("Paired robot velocity differs")
    if math.dist((ball.vx, ball.vy), (actual.ball.vx, actual.ball.vy)) > 1e-6:
        raise RuntimeError("Paired ball velocity differs")
    if math.dist(xy(ball), xy(actual.ball)) > 1e-6:
        raise RuntimeError("Paired ball start differs")
    return actual


def command(kind, team="UNKNOWN"):
    return {"change": {"newCommandChange": {"command": {"type": kind, "forTeam": team.upper()}}}}


def official(sim, backend, root, folder, spec, receiver, vision):
    weights = (
        spec.get("weights")
        or json.loads((root / "policies/placement_v1/actor.json").read_text())["weights"]
    )
    demo = spec["mode"] == "demo"
    strict = spec.get("adjudicate_crashes", False)
    traffic = None
    clear_retreat = spec.get("actor") == "placement_v2_clear"
    if clear_retreat or spec.get("actor") == "placement_v2":
        traffic = json.loads((root / "policies/placement_v2/actor.json").read_text())["traffic"]
    if demo:
        (folder / "frames").mkdir()
    results = []
    origin = time.time() - sim.getTime()
    frame = 0
    with subprocess.Popen(
        [
            "node",
            str(root / "scripts/autoref/placement_table.mjs"),
            "ws://127.0.0.1:21981/api/control",
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    ) as table:
        if table.stdout.readline().strip() != "READY":
            raise RuntimeError("No official table connection")

        def send(msg):
            table.stdin.write(json.dumps(msg) + "\n")
            table.stdin.flush()

        def await_command(name):
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                state = receiver.poll()
                if state.command == name:
                    return state
                time.sleep(0.025)
            raise RuntimeError(f"Table broadcast {state.command} while waiting for {name}")

        starts = [(s, None) for s in spec.get("seeds", [])] + spec.get("archived", [])
        natural = {}
        if spec.get("starts") == "natural":
            from natural_placement import restore

            cases = json.loads((root / "policies/placement_v2/natural-starts.json").read_text())
            natural = {(c["seed"], c["tick"]): c for c in cases}
            starts = list(natural)
        for seed, crossing in starts:
            if (seed, crossing) in natural:
                c = natural[seed, crossing]
                world, team, target = restore(seed, c["sign"], c["state"]), c["team"], c["target"]
            elif spec.get("starts") == "crowded":
                world, team, target = crowded(seed)
            else:
                world, team, target = scenario(root, seed, crossing)
            other = "yellow" if team == "blue" else "blue"
            if demo:
                # Practice an overhit diagonal pass toward the right wing.
                # The receiving robot is offset from the line of flight.
                bx, by = 0.15, 2.48
                angle = 1.22
                passer = next(r.key for r in world.robots if r.key.team == other and r.key.id == 1)
                intended = replace(passer, id=2)
                spots = {
                    passer: (bx - 0.115 * math.cos(angle), by - 0.115 * math.sin(angle), angle),
                    intended: (bx + 0.46, 2.89, angle - math.pi),
                }
                robots = []
                for r in world.robots:
                    if r.key in spots:
                        x, y, yaw = spots[r.key]
                        r = replace(r, x=x, y=y, yaw=yaw)
                    elif abs(r.y) > 2.0:
                        r = replace(r, y=r.y * 0.52)
                    robots.append(r)
                world = replace(world, robots=tuple(robots), ball=Ball(bx, by))
            send(command("HALT"))
            await_command("HALT")
            w = backend.reset(world)
            restart = PlacementRestart(weights, traffic, clear_retreat)
            for _ in range(35):
                state = receiver.poll()
                vision.send(
                    w, origin + sim.getTime(), backend.ball.getPosition()[2], receiver.feed.packet
                )
                w = backend.step({})
            send(command("FORCE_START"))
            await_command("FORCE_START")
            if not demo:
                send(command("STOP"))
                await_command("STOP")
                send(
                    {
                        "change": {
                            "setBallPlacementPosChange": {"pos": {"x": target[0], "y": target[1]}}
                        }
                    }
                )
                send(command("BALL_PLACEMENT", team))
                await_command("BALL_PLACEMENT_" + team.upper())
            if spec.get("exact_start"):
                w = exact_start(backend, replace(world, time=w.time))
            row = dict(
                seed=seed,
                crossing=crossing,
                team=team,
                target=target,
                initial_ball=xy(w.ball),
                vision_start=origin + sim.getTime(),
                event_ids=[],
                commands=[],
                phases=[],
                kicks=[],
                flight_contacts=[],
                opening_robots=[[r.key.team, r.key.id, r.x, r.y, r.yaw] for r in w.robots],
            )
            if spec.get("exact_start"):
                row["opening_robot_velocities"] = [[r.vx, r.vy, r.omega] for r in w.robots]
            if strict:
                row["min_center_m"] = nearest_pair(w)
                row["actor"] = spec.get("actor", "placement_v1")
                row["opening_ball_velocity"] = [w.ball.vx, w.ball.vy]
            if demo:
                row["opening_pass_to"] = [intended.team, intended.id]
            first = None
            active_play = None
            telemetry = []
            # Reproduce the original film's physics times for the two table
            # actions. The live officials still have to detect and award the exit.
            timed = demo and spec.get("replay_film_timing", False)

            def clock(t):
                return abs(w.time - t) < CFG.dt / 2

            for i in range(1600 if demo else 752):
                if timed and clock(4.4):
                    await_command("STOP")
                if timed and clock(4.64):
                    packet = receiver.feed.packet
                    if not any(
                        e.WhichOneof("event") == "ball_left_field_touch_line"
                        for e in packet.game_events
                    ):
                        raise RuntimeError(
                            "Live official did not confirm the recorded touchline exit"
                        )
                    awarded = packet.Command.Name(packet.next_command).split("_")[-1]
                    send({"continueAction": {"type": "BALL_PLACEMENT_START", "forTeam": awarded}})
                    await_command("BALL_PLACEMENT_" + awarded)

                state = receiver.poll()
                packet = receiver.feed.packet
                if state.command and (
                    not row["commands"] or state.command != row["commands"][-1][1]
                ):
                    row["commands"].append([round(w.time, 2), state.command, state.counter])
                event_types = {}
                if state.source:
                    for event in packet.game_events:
                        kind = event.WhichOneof("event")
                        event_types[kind] = getattr(event, kind) if kind else None
                        if event.id and event.id not in row["event_ids"]:
                            row["event_ids"].append(event.id)
                if (
                    strict
                    and {"bot_crash_unique", "bot_crash_drawn", "bot_interfered_placement"}
                    & event_types.keys()
                ):
                    row["stopped_by_foul"] = True
                    break
                if state.placement_target is not None:
                    if first is None:
                        first = w.time
                        row.update(
                            team=state.placement_team,
                            target=state.placement_target,
                            placement_counter=state.counter,
                        )
                    if not demo and (
                        state.placement_team != team
                        or math.dist(state.placement_target, target) > 0.002
                    ):
                        raise RuntimeError("Wrong award or designated point on wire")
                w = replace(w, referee=state.mode, attack_sign=state.attack_sign)
                backend.world = w
                action = restart.commands(w, state)
                out = action[0] if action else {}
                if restart.play:
                    active_play = restart.play
                    row.update(placer=active_play.placer.id, receiver=active_play.receiver.id)
                    phase = active_play.phase if restart.direct_since is None else "free kick"
                    if not row["phases"] or row["phases"][-1][1] != phase:
                        row["phases"].append([round(w.time, 2), phase])
                if demo and state.command == "FORCE_START" and first is None and not row["kicks"]:
                    if i >= 12:
                        out = {passer: Command(kick=1.05)}
                if demo and state.command == "STOP" and first is None:
                    out = stop_formation(w) if not timed or w.time >= 4.4 - 1e-8 else {}
                    if not timed and "ball_left_field_touch_line" in event_types and i % 10 == 0:
                        awarded = (
                            "BLUE"
                            if event_types["ball_left_field_touch_line"].by_team == 1
                            else "YELLOW"
                        )
                        # The table chooses the official follow-up's team.
                        if packet.HasField("next_command"):
                            awarded = packet.Command.Name(packet.next_command).split("_")[-1]
                        send(
                            {"continueAction": {"type": "BALL_PLACEMENT_START", "forTeam": awarded}}
                        )
                if first is not None and "placement_succeeded" in event_types:
                    if "placed_seconds" not in row:
                        row.update(
                            placed_seconds=round(w.time - first, 2),
                            final_error_m=math.dist(xy(w.ball), row["target"]),
                            speed_at_acceptance=math.hypot(w.ball.vx, w.ball.vy),
                        )
                    if demo and i % 10 == 0 and state.command == "STOP":
                        send({"continueAction": {"type": "NEXT_COMMAND", "forTeam": "UNKNOWN"}})
                stamp = origin + sim.getTime()
                vision.send(
                    w, stamp, backend.ball.getPosition()[2], packet if state.source else None
                )
                if demo and i % 2 == 0:
                    telemetry.append(
                        dict(
                            time=w.time,
                            ball=xy(w.ball),
                            robots=[xy(r) for r in w.robots],
                            command=state.command,
                            phase=(row["phases"][-1][1] if row["phases"] else "opening"),
                        )
                    )
                    phase = row["phases"][-1][1] if row["phases"] else "Opening pass"
                    sim.setLabel(
                        0,
                        f"5v5   |   {state.command}   |   {phase}",
                        0.06,
                        0.04,
                        0.055,
                        0xFFFFFF,
                        0,
                        "Arial",
                    )
                    sim.exportImage(str(folder / "frames" / f"{frame:05d}.jpg"), 85)
                    frame += 1
                w = backend.step(out)
                if strict and first is not None:
                    row["min_center_m"] = min(
                        row["min_center_m"],
                        *(nearest_pair(sample) for sample in backend.last_substep_worlds),
                    )
                restart.applied(backend.last_kick_events)
                for k, at, v in backend.last_kick_events:
                    row["kicks"].append(
                        dict(
                            seconds=round(at.time, 2),
                            team=k.team,
                            id=k.id,
                            speed=v,
                            ball=xy(at.ball),
                        )
                    )
                if demo and active_play and active_play.kicked:
                    for sample in backend.last_substep_worlds:
                        for r in sample.robots:
                            if r.key not in (
                                active_play.placer,
                                active_play.receiver,
                            ) and possession(sample, r.key, CFG):
                                row["flight_contacts"].append([sample.time, r.key.team, r.key.id])
                if demo and restart.received >= 5:
                    row.update(received_seconds=round(w.time, 2), received_ball=xy(w.ball))
                    break
                if not demo and "placed_seconds" in row:
                    break
                if first is not None and "placed_seconds" not in row and w.time - first > 30:
                    break
            row["vision_end"] = origin + sim.getTime()
            if demo:
                (folder / "film-trace.json").write_text(json.dumps(telemetry))
            results.append(row)
            (folder / "progress.json").write_text(json.dumps(results))
            print(
                seed,
                row["commands"],
                row.get("placed_seconds"),
                row.get("final_error_m"),
                flush=True,
            )
        table.stdin.close()
    return results
