"""Device-level controller for the articulated Webots SSL robot."""

from dataclasses import dataclass
import math

from roboturk.wheel_drive import limit_by_shafts, shafts_for_twist, twist_for_shafts


BEAM_CLEAR_M = 0.088
BEAM_DEBOUNCE_S = 0.006
ROLLER_MAX_RAD_S = 560.0
STRIKER_STROKE_M = {"flat": 0.032, "chip": 0.034}
MAX_KICK_M_S = 10.0
# Local speed fit for the bench striker at moderate power.
FLAT_SPEED_RATIO = 1.22
COOLDOWN_S = 0.18


@dataclass(frozen=True)
class Feedback:
    wheel_rad_s: tuple[float, ...]
    roller_rad_s: float
    beam_distance_m: float
    ball_at_roller: bool
    flat_position_m: float
    chip_position_m: float

    @property
    def encoder_twist(self):
        return twist_for_shafts(self.wheel_rad_s)


class Actuators:
    """Turn a robot-local Command into motor targets and return onboard feedback."""

    def __init__(self, robot):
        self.dt_ms = int(robot.getBasicTimeStep())
        self.dt = self.dt_ms / 1000
        self.wheels = tuple(robot.getDevice(f"wheel_{i}") for i in range(4))
        self.roller = robot.getDevice("dribbler")
        self.strikers = {name: robot.getDevice(f"{name}_striker") for name in STRIKER_STROKE_M}
        self.encoders = tuple(robot.getDevice(f"wheel_{i}_encoder") for i in range(4)) + (
            robot.getDevice("dribbler_encoder"),
        )
        self.striker_encoders = tuple(
            robot.getDevice(f"{name}_striker_encoder") for name in self.strikers
        )
        self.beam = robot.getDevice("ball_beam")
        self.indicator = robot.getDevice("beam_indicator")
        for sensor in (*self.encoders, *self.striker_encoders, self.beam):
            sensor.enable(self.dt_ms)
        for motor in (*self.wheels, self.roller):
            motor.setPosition(math.inf)
            motor.setVelocity(0)
        self.shaft_limit = min(motor.getMaxVelocity() for motor in self.wheels)
        self.now = robot.getTime
        self.reset()

    def reset(self):
        self.previous_angles = None
        self.beam_samples = 0
        self.ball_at_roller = False
        self.kick_requested = False
        self.active = None
        self.retract_at = -math.inf
        self.cooldown_until = -math.inf
        self.wheel_targets = (0.0,) * 4
        for motor in (*self.wheels, self.roller):
            motor.setVelocity(0)
        for motor in self.strikers.values():
            self.retract(motor)
        self.indicator.set(0)

    @staticmethod
    def retract(motor):
        motor.setControlPID(45, 0, 0)
        motor.setVelocity(2.0)
        motor.setPosition(0.0)

    def sample(self):
        angles = tuple(sensor.getValue() for sensor in self.encoders)
        rates = (0.0,) * len(angles)
        if self.previous_angles is not None:
            rates = tuple(
                (angle - before) / self.dt
                for angle, before in zip(angles, self.previous_angles, strict=True)
            )
        self.previous_angles = angles
        distance = self.beam.getValue()
        obstructed = distance < BEAM_CLEAR_M
        if obstructed == self.ball_at_roller:
            self.beam_samples = 0
        else:
            self.beam_samples += 1
            if self.beam_samples >= math.ceil(BEAM_DEBOUNCE_S / self.dt):
                self.ball_at_roller = obstructed
                self.beam_samples = 0
                self.indicator.set(int(obstructed))
        flat, chip = (sensor.getValue() for sensor in self.striker_encoders)
        return Feedback(rates[:4], rates[4], distance, self.ball_at_roller, flat, chip)

    def apply(self, command, feedback):
        """Return which striker fired this tick, if any."""
        if not all(
            math.isfinite(v)
            for v in (
                command.vx,
                command.vy,
                command.omega,
                command.dribbler,
                command.kick,
                command.kick_elevation,
            )
        ):
            raise ValueError("Actuator commands must be finite")
        limited = limit_by_shafts(command, self.shaft_limit)
        self.wheel_targets = shafts_for_twist(limited.vx, limited.vy, limited.omega)
        for motor, target in zip(self.wheels, self.wheel_targets, strict=True):
            motor.setVelocity(float(target))
        time = self.now()
        if self.active is not None and time >= self.retract_at:
            self.retract(self.strikers[self.active])
            self.active = None
        wants_kick = command.kick > 0
        new_kick = wants_kick and not self.kick_requested
        self.kick_requested = wants_kick
        fired = None
        if new_kick and feedback.ball_at_roller and time >= self.cooldown_until:
            fired = "chip" if command.kick_elevation > 0 else "flat"
            motor = self.strikers[fired]
            speed = min(MAX_KICK_M_S, command.kick)
            if fired == "flat":
                speed /= FLAT_SPEED_RATIO
            stroke = STRIKER_STROKE_M[fired]
            motor.setControlPID(400, 0, 0)
            motor.setVelocity(speed)
            motor.setPosition(stroke)
            self.active = fired
            self.retract_at = time + min(0.12, stroke / max(0.001, speed) + 0.018)
            self.cooldown_until = max(time + COOLDOWN_S, self.retract_at + 0.03)
        duty = min(1.0, max(0.0, command.dribbler))
        self.roller.setVelocity(0.0 if self.active else ROLLER_MAX_RAD_S * duty)
        return fired
