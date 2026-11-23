"""Direction-independent traction, rolling resistance and straight-ball prediction."""

from math import hypot

# Coulomb friction and rolling-friction length (torque per unit normal force).
SLIDE_FRICTION = 0.30
ROLL_FRICTION = 0.0008
GRAVITY = 9.81
BALL_RADIUS = 0.0215
SLIDE_DECEL = GRAVITY * SLIDE_FRICTION
# A solid sphere has I = 2/5 m r². While rolling, both rotation and translation slow.
ROLL_DECEL = GRAVITY * ROLL_FRICTION / (1.4 * BALL_RADIUS)
# During initial slip, friction spins up the initially non-spinning ball while
# rolling resistance opposes that spin. The switch occurs at v = omega * r.
SWITCH_RATIO = 1 - SLIDE_DECEL / (3.5 * (SLIDE_DECEL - ROLL_DECEL))


def ground_step(speed, switch_speed, dt):
    """Return distance and final speed, resolving slide/roll/rest within a step."""
    sliding = min(dt, max(0.0, speed - switch_speed) / SLIDE_DECEL)
    middle = speed - SLIDE_DECEL * sliding
    rolling = min(dt - sliding, middle / ROLL_DECEL)
    end = max(0.0, middle - ROLL_DECEL * rolling)
    return (speed + middle) * sliding / 2 + (middle + end) * rolling / 2, end


def carpet_velocity(velocity, dt, radius=BALL_RADIUS):
    """Apply isotropic friction impulses to a grounded solid sphere in world coordinates."""
    vx, vy, vz, wx, wy, wz = velocity
    # u is the translational velocity that would exactly match this spin in pure roll.
    ux, uy = radius * wy, -radius * wx
    rotation = hypot(ux, uy)
    if max(hypot(vx, vy), rotation) <= ROLL_DECEL * dt:
        return [0.0, 0.0, vz, 0.0, 0.0, wz]
    if rotation:
        # Rolling-resistance couple = normal force times the rolling-friction length.
        loss = min(1.0, GRAVITY * ROLL_FRICTION * dt / (0.4 * radius * rotation))
        ux, uy = ux * (1 - loss), uy * (1 - loss)
    sx, sy = vx - ux, vy - uy
    slip = hypot(sx, sy)
    if slip:
        # A tangential impulse changes v by J/m and u by -2.5 J/m.
        # Limit it to the impulse that cancels slip, including static friction while rolling.
        fraction = min(1 / 3.5, SLIDE_DECEL * dt / slip)
        jx, jy = -sx * fraction, -sy * fraction
        vx, vy = vx + jx, vy + jy
        ux, uy = ux - 2.5 * jx, uy - 2.5 * jy
    return [vx, vy, vz, -uy / radius, ux / radius, wz]
