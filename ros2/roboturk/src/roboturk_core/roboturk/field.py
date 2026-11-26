"""Nominal SSL Division B dimensions, in meters between line centers."""

LENGTH, WIDTH = 9.0, 6.0
CARPET_MARGIN = 0.7
WALL_MARGIN = 0.3
GOAL_WIDTH, GOAL_DEPTH, GOAL_HEIGHT = 1.0, 0.18, 0.16
GOAL_WALL = 0.02
DEFENSE_DEPTH, DEFENSE_WIDTH = 1.0, 2.0
CIRCLE_RADIUS, LINE_WIDTH = 0.5, 0.01
GOAL_TO_PENALTY_MARK = 6.0


def field_lines():
    x, y = LENGTH / 2, WIDTH / 2
    yield "TopTouchLine", (-x, y), (x, y)
    yield "BottomTouchLine", (-x, -y), (x, -y)
    yield "LeftGoalLine", (-x, -y), (-x, y)
    yield "RightGoalLine", (x, -y), (x, y)
    yield "HalfwayLine", (0, -y), (0, y)
    yield "CenterLine", (-x, 0), (x, 0)
    for end, sign in (("Left", -1), ("Right", 1)):
        back, front = sign * x, sign * (x - DEFENSE_DEPTH)
        half = DEFENSE_WIDTH / 2
        yield end + "PenaltyStretch", (front, -half), (front, half)
        for side, ordinate in (("Left", sign * half), ("Right", -sign * half)):
            yield end + "Field" + side + "PenaltyStretch", (back, ordinate), (front, ordinate)
