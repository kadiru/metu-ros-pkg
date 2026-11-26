"""Bridge recorded open-play crossings to the independent TIGERs touchline detector."""

from dataclasses import replace
import time

from roboturk.five_roles import CFG, kickoff
from roboturk.model import Ball
from roboturk.ssl_vision import Packet, practice_command


def restore(seed, sign, values):
    template = kickoff(seed, "blue", sign)
    ball = Ball(*values[:4])
    poses = values[4:]
    robots = tuple(
        replace(r, x=poses[3 * i], y=poses[3 * i + 1], yaw=poses[3 * i + 2])
        for i, r in enumerate(template.robots)
    )
    return replace(template, ball=ball, robots=robots)


def camera(backend, vision, spec):
    start = time.time()
    frame = 0
    records = []
    for case in spec["cases"]:
        seed, sign = case["seed"], case["sign"]
        packet = practice_command(sign)
        packet.command = Packet.HALT
        packet.command_counter = frame + 1

        def emit(w, height):
            nonlocal frame
            stamp = start + CFG.dt * frame
            packet.packet_timestamp = int(stamp * 1e6)
            vision.send(w, stamp, height, packet)
            frame += 1

        first = restore(seed, sign, case["before"][0])
        for _ in range(45):
            emit(first, CFG.ball_radius)
        packet.command = Packet.FORCE_START
        packet.command_counter += 1
        begin = start + CFG.dt * frame
        for v in case["before"]:
            emit(restore(seed, sign, v), CFG.ball_radius)
        # Reconstruct the last measured control sample, then physically coast
        # beyond the line. The open-play match itself reset on the crossing.
        w = backend.reset(restore(seed, sign, case["state"]))
        for _ in range(35):
            emit(w, backend.ball.getPosition()[2])
            w = backend.step({})
        records.append(dict(seed=seed, tick=case["tick"], start=begin, end=start + CFG.dt * frame))
    return records
