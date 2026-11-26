from io import BytesIO

import pytest

from roboturk.five_roles import kickoff
from roboturk.ssl_vision import Wrapper, detection, geometry, read_delimited, write_delimited


def test_overhead_camera_matches_field_frame_and_lengths():
    world = kickoff(13, "blue", -1)
    g = geometry().geometry
    cam = g.calib[0]
    assert (g.field.field_length, g.field.field_width, g.field.goal_width) == (9000, 6000, 1000)
    assert (cam.q0, cam.q1, cam.q2, cam.q3) == (1, 0, 0, 0)
    frame = detection(world, 3, 1.5, 0.0215)
    assert frame.IsInitialized()
    d = frame.detection
    assert d.frame_number == 3 and d.t_capture == d.t_sent == 1.5
    assert d.balls[0].z == pytest.approx(21.5)
    for team, seen in (("blue", d.robots_blue), ("yellow", d.robots_yellow)):
        expected = {r.key.id: r for r in world.robots if r.key.team == team}
        assert len(seen) == len(expected) == 5
        for robot in seen:
            r = expected[robot.robot_id]
            assert (robot.x, robot.y) == pytest.approx((1000 * r.x, 1000 * r.y))
            assert robot.pixel_x == pytest.approx(
                cam.principal_point_x + cam.focal_length * robot.x / (cam.tz - robot.height)
            )
            assert robot.pixel_y == pytest.approx(
                cam.principal_point_y - cam.focal_length * robot.y / (cam.tz - robot.height)
            )


def test_delimited_protobuf_preserves_frame_boundaries():
    stream = BytesIO()
    for v in (geometry(), detection(kickoff(1, "yellow", 1), 1, 1.0, 0.0215)):
        write_delimited(stream, v)
    stream.seek(0)
    assert read_delimited(stream, Wrapper).HasField("geometry")
    assert read_delimited(stream, Wrapper).HasField("detection")
    with pytest.raises(EOFError):
        read_delimited(stream, Wrapper)
