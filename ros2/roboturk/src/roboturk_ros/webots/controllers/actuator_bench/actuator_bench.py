"""Exercise and film the articulated robot using the same body-frame Command as skills."""

from dataclasses import asdict
import json
import math
import os
from pathlib import Path

from controller import Supervisor

from roboturk.model import Command, body_vector

from actuators import Actuators
from traction import apply_roller_contact, apply_traction


MOTIONS = (
    ("Forward", Command(vx=0.48)),
    ("Strafe left", Command(vy=0.42)),
    ("Strafe right", Command(vy=-0.42)),
    ("Turn in place", Command(omega=2.4)),
    ("Drive and turn", Command(vx=0.42, vy=-0.25, omega=-1.7)),
)


def point_camera(camera, focus, azimuth, elevation, distance):
    tilt = math.pi / 2 - elevation
    azimuth_rotation = azimuth + math.pi / 2
    a, b = azimuth_rotation / 2, tilt / 2
    qx, qy, qz = math.cos(a) * math.sin(b), math.sin(a) * math.sin(b), math.sin(a) * math.cos(b)
    qw = math.cos(a) * math.cos(b)
    # Webots Viewpoint looks along local +x, with +z up.
    qw, qx, qy, qz = (
        (qw + qx - qy - qz) / 2,
        (-qw + qx + qy - qz) / 2,
        (qw - qx + qy - qz) / 2,
        (qw + qx + qy + qz) / 2,
    )
    norm = math.sqrt(qx**2 + qy**2 + qz**2)
    angle = 2 * math.atan2(norm, qw)
    camera.getField("orientation").setSFRotation([qx / norm, qy / norm, qz / norm, angle])
    x, y, z = focus
    camera.getField("position").setSFVec3f(
        [
            x + distance * math.cos(elevation) * math.cos(azimuth),
            y + distance * math.cos(elevation) * math.sin(azimuth),
            z + distance * math.sin(elevation),
        ]
    )


class Bench:
    def __init__(self, supervisor, film):
        self.supervisor = supervisor
        self.robot = supervisor.getSelf()
        self.ball = supervisor.getFromDef("BALL")
        self.camera = supervisor.getFromDef("CAMERA")
        self.control = Actuators(supervisor)
        self.film = film
        self.index = 0
        self.feedback = None

    def reset(self, ball=(1.6, 0.6), yaw=0.0):
        self.robot.getField("translation").setSFVec3f([0, 0, 0.001])
        self.robot.getField("rotation").setSFRotation([0, 0, 1, yaw])
        self.ball.getField("translation").setSFVec3f([*ball, 0.0215])
        self.robot.resetPhysics()
        self.ball.resetPhysics()
        self.control.reset()
        self.feedback = None
        self.tick(0.28)

    def tick(self, duration, command=Command(), label=None, view=None, *, frame_ms=80):
        n = round(duration / self.control.dt)
        frames = []
        interval = max(1, round(frame_ms / self.control.dt_ms))
        for i in range(n):
            fired = self.control.apply(command, self.feedback) if self.feedback else None
            if self.feedback:
                apply_traction(self.robot, self.feedback.wheel_rad_s)
                if self.control.active is None and command.dribbler > 0:
                    apply_roller_contact(self.robot, self.ball, self.feedback.roller_rad_s)
            if self.supervisor.step(self.control.dt_ms) == -1:
                raise RuntimeError("Webots stopped during the actuator bench")
            self.feedback = self.control.sample()
            if i % 2 == 0 or fired:
                pose = self.robot.getPosition()
                orientation = self.robot.getOrientation()
                yaw = math.atan2(orientation[3], orientation[0])
                vx, vy, _, _, _, omega = self.robot.getVelocity()
                measured = (*body_vector(vx, vy, yaw), omega)
                frames.append(
                    dict(
                        t=self.supervisor.getTime(),
                        pos=pose,
                        yaw=yaw,
                        twist=measured,
                        ball=self.ball.getPosition(),
                        ball_v=self.ball.getVelocity()[:3],
                        feedback=asdict(self.feedback),
                        targets=self.control.wheel_targets,
                        fired=fired,
                    )
                )
            if self.film and label and i % interval == 0:
                self.frame(label, view)
        return frames

    def frame(self, label, view):
        point_camera(self.camera, *view(self))
        beam = (
            "Light sensor · ball detected"
            if self.feedback.ball_at_roller
            else "Light sensor · clear"
        )
        color = 0xBDFADB if self.feedback.ball_at_roller else 0xCBD6E7
        self.supervisor.setLabel(0, label, 0.034, 0.032, 0.122, 0xFFFFFF, 0, "Arial")
        self.supervisor.setLabel(1, beam, 0.034, 0.13, 0.073, color, 0, "Arial")
        self.supervisor.exportImage(str(self.film / f"{self.index:04d}.jpg"), 90)
        self.index += 1


def front(bench):
    x, y, _ = bench.robot.getPosition()
    return (x + 0.039, y, 0.062), -0.55, 0.56, 0.54


def motion_camera(_bench):
    return (0.0, 0.13, 0.060), -1.25, 1.10, 1.17


def release_camera(bench):
    x, y, _ = bench.robot.getPosition()
    bx, by, bz = bench.ball.getPosition()
    progress = max(0, min(bx - x, 1.38))
    return (
        (x + 0.08 + progress / 2, y + (by - y) / 2, 0.066 + bz / 4),
        -1.05,
        0.30,
        0.55 + 0.80 * progress,
    )


def mean(values):
    return sum(values) / len(values)


def motion_bench(bench):
    results = {}
    for yaw in (0.0, 0.79, -1.72):
        for label, command in MOTIONS:
            bench.reset(yaw=yaw)
            visible = yaw == 0 and label in ("Strafe left", "Turn in place")
            rows = bench.tick(
                0.90, command, ("Four omniwheels · " + label) if visible else None, motion_camera
            )
            tail = rows[-65:]
            target = (command.vx, command.vy, command.omega)
            measured = [mean([row["twist"][i] for row in tail]) for i in range(3)]
            shaft_error = max(
                abs(
                    mean([row["feedback"]["wheel_rad_s"][i] for row in tail])
                    - tail[-1]["targets"][i]
                )
                for i in range(4)
            )
            encoder = [float(v) for v in bench.feedback.encoder_twist]
            results[f"{yaw:+.2f}/{label}"] = dict(
                target=target,
                measured=measured,
                encoder=encoder,
                max_shaft_error_rad_s=shaft_error,
                end=tail[-1]["pos"],
            )
    return results


def release_bench(bench, name, offset, serve, speed, *, film=False):
    bench.reset(ball=(0.152 if serve else 0.136, offset))
    if serve:
        bench.ball.setVelocity([-0.29, 0, 0, 0, 0, 0])  # Independent serve into the robot.
    label = "90 mm dribbler · receive and carry" if film and serve else None
    received = bench.tick(0.50, Command(dribbler=0.68), label, front)
    carried = (
        bench.tick(
            0.40,
            Command(vx=0.19, dribbler=0.68),
            label,
            front,
        )
        if serve
        else []
    )
    if serve:
        carried += bench.tick(0.16, Command(dribbler=0.68), label, front)
    label = (
        ("Flat kick · slow motion" if name == "flat" else "Chip kick · slow motion")
        if film
        else None
    )
    pre = bench.tick(0.22, Command(dribbler=0.68), label, release_camera)
    elevation = 0.0 if name == "flat" else math.radians(36)
    command = Command(kick=speed, kick_elevation=elevation)
    rows = bench.tick(0.36, command, label, release_camera, frame_ms=20)
    rows += bench.tick(0.46, command)
    final = bench.tick(0.21)
    best = max(rows[:40], key=lambda s: s["ball_v"][0])
    fires = [s["fired"] for s in rows if s["fired"]]
    return dict(
        requested_m_s=speed,
        ball_offset_m=offset,
        served=serve,
        saw_beam=any(row["feedback"]["ball_at_roller"] for row in received),
        retained_carry=bool(carried) and all(row["feedback"]["ball_at_roller"] for row in carried),
        carry_beam_fraction=mean([row["feedback"]["ball_at_roller"] for row in carried])
        if carried
        else None,
        carry_distance_m=carried[-1]["pos"][0] - carried[0]["pos"][0] if carried else 0,
        pre_ball=pre[-1]["ball"],
        pre_robot=pre[-1]["pos"],
        beam_before=pre[-1]["feedback"]["ball_at_roller"],
        beam_after=rows[-1]["feedback"]["ball_at_roller"],
        fires=fires,
        launch_m_s=math.hypot(*best["ball_v"]),
        launch_z_m_s=best["ball_v"][2],
        apex_m=max(s["ball"][2] for s in rows),
        flat_stroke_m=max(s["feedback"]["flat_position_m"] for s in rows),
        chip_stroke_m=max(s["feedback"]["chip_position_m"] for s in rows),
        final_flat_m=final[-1]["feedback"]["flat_position_m"],
        final_chip_m=final[-1]["feedback"]["chip_position_m"],
    )


def check(motion, releases, empty_fires):
    failures = []
    for name, result in motion.items():
        error = [abs(a - b) for a, b in zip(result["target"], result["measured"], strict=True)]
        if math.hypot(*error[:2]) > 0.075 or error[2] > 0.20:
            failures.append(f"{name}: chassis motion missed its target")
        if result["max_shaft_error_rad_s"] > 1.0:
            failures.append(f"{name}: wheel shafts missed their targets")
    carry = releases["flat"][0]
    if not carry["saw_beam"] or not carry["retained_carry"] or carry["carry_distance_m"] < 0.06:
        failures.append("The served ball was not captured and held through the carry")
    for name, rows in releases.items():
        for index, row in enumerate(rows, start=1):
            if not row["beam_before"] or row["beam_after"] or row["fires"] != [name]:
                failures.append(f"{name} {index}: incorrect beam or firing sequence")
            if row[f"{name}_stroke_m"] < 0.025 or abs(row[f"final_{name}_m"]) > 0.002:
                failures.append(f"{name} {index}: striker failed to extend and retract")
            if row["launch_m_s"] < 0.78 * row["requested_m_s"]:
                failures.append(f"{name} {index}: weak ball release")
            if name == "flat" and row["apex_m"] > 0.037:
                failures.append(f"flat {index}: ball left the ground")
            if name == "chip" and row["apex_m"] < 0.044:
                failures.append(f"chip {index}: insufficient loft")
    if empty_fires:
        failures.append("An empty roller permitted a kick")
    return failures


def main(supervisor):
    output = Path(os.environ.get("ROBOTURK_OUTPUT", "actuator-bench.json")).resolve()
    film = output.with_suffix("") if os.environ.get("ROBOTURK_FILM_ACTUATORS") else None
    output.parent.mkdir(parents=True, exist_ok=True)
    if film:
        film.mkdir(exist_ok=True)
    bench = Bench(supervisor, film)
    bench.reset(ball=(0.146, 0))
    start = supervisor.getTime()
    bench.tick(
        1.6,
        label="RoboTurk · flat-front robot",
        view=lambda _: ((0.019, 0.0, 0.055), 0.78 - (supervisor.getTime() - start), 0.49, 0.465),
    )
    motion = motion_bench(bench)
    releases = {
        name: [
            release_bench(bench, name, offset, serve, speed, film=index == 0 and film)
            for index, (offset, serve, speed) in enumerate(
                [
                    (0, name == "flat", 3.2 if name == "flat" else 3.8),
                    (-0.018, False, 2.3),
                    (0.018, False, 3.6),
                ]
            )
        ]
        for name in ("flat", "chip")
    }
    bench.reset()
    empty = bench.tick(0.2, Command(kick=3.2))
    empty_fires = sum(bool(s["fired"]) for s in empty)
    failures = check(motion, releases, empty_fires)
    output.write_text(
        json.dumps(
            dict(
                motion=motion,
                releases=releases,
                empty_fires=empty_fires,
                failures=failures,
            ),
            indent=2,
            allow_nan=False,
        )
        + "\n"
    )
    if failures:
        raise RuntimeError("; ".join(failures))
    print(f"Saved articulated actuator bench to {output}", flush=True)


if __name__ == "__main__":
    supervisor = Supervisor()
    try:
        main(supervisor)
    except Exception:
        import traceback

        traceback.print_exc()
        supervisor.simulationQuit(1)
    else:
        supervisor.simulationQuit(0)
