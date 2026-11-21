"""Webots overhead camera, scoring HUD and ball marker for recovery matches."""

import math


def movie_camera(sim, folder):
    camera = sim.getFromDef("CAMERA")
    camera.getField("position").setSFVec3f([0, 0, 16.5])
    camera.getField("orientation").setSFRotation(
        [-0.577350269, 0.577350269, 0.577350269, 2.094395102]
    )
    pts = ", ".join(
        f"{0.08 * math.cos(i * math.tau / 32)} {0.08 * math.sin(i * math.tau / 32)} 0"
        for i in range(32)
    )
    indices = ", ".join(str(j) for j in [*range(32), 0, -1])
    sim.getRoot().getField("children").importMFNodeFromString(
        -1,
        "DEF BALL_RING Pose { children [ Shape { appearance Appearance { material Material { emissiveColor 1 .8 .1 } } "
        f"geometry IndexedLineSet {{ coord Coordinate {{ point [ {pts} ] }} coordIndex [ {indices} ] }} }} ] }}",
    )
    ring = sim.getFromDef("BALL_RING").getField("translation")
    sim.movieStartRecording(str(folder / "match.mp4"), 1280, 720, 0, 85, 1, False)

    def draw(g):
        ring.setSFVec3f([g.world.ball.x, g.world.ball.y, 0.05])
        sim.setLabel(
            0,
            f"RoboTurk    |    3v3 open play    |    {g.tick * 0.04:05.1f} s",
            0.02,
            0.02,
            0.056,
            0xFFFFFF,
            0,
            "Arial",
        )
        sim.setLabel(
            1,
            f"BLUE   {g.score['blue']} : {g.score['yellow']}   YELLOW",
            0.02,
            0.10,
            0.055,
            0xFFFFFF,
            0,
            "Arial",
        )
        if g.dual_ticks >= 5:
            label = "CONTESTED BALL"
        elif g.flight:
            label = (
                f"{g.flight['team'].upper()} {g.flight['passer']} passes to {g.flight['receiver']}"
            )
        elif g.owner:
            label = f"{g.owner.team.upper()} {g.owner.id} in possession"
        elif g.passes:
            label = g.passes[-1].get("result", "").upper()
        else:
            label = "Ball in play"
        sim.setLabel(2, label, 0.02, 0.83, 0.065, 0xFFDF80, 0, "Arial")
        outlets = [
            f"{team.upper()} {role['mate'].id} opens for {role['actor'].id}"
            for team, role in g.roles.items()
        ]
        if g.active:
            team = g.active[0].upper()
            action = g.option["action"].upper() if g.option else "BALL IN PLAY"
            outlets = [f"{team} RECOVERY   |   {action}"]
        if g.events and g.events[-1]["event"] == "goal" and g.tick - g.events[-1]["tick"] < 28:
            label = "GOAL"
            sim.setLabel(2, label, 0.02, 0.83, 0.065, 0xFFDF80, 0, "Arial")
        sim.setLabel(10, "     ".join(outlets), 0.02, 0.91, 0.045, 0x83EEDD, 0, "Arial")
        for r in g.world.robots:
            sim.setLabel(
                3 + (3 if r.key.team == "yellow" else 0) + r.key.id,
                str(r.key.id),
                0.5 + r.x * 0.073,
                0.483 - r.y * 0.130,
                0.038,
                0xFFFFFF,
                0,
                "Arial",
            )

    return draw
