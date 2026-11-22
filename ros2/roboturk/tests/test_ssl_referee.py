import pytest

from roboturk.model import Ball, Referee
from roboturk.ssl_referee import Packet, RefereeFeed, RestartGate


def packet(command, counter, timestamp, source="one", stage="NORMAL_FIRST_HALF"):
    p = Packet(
        packet_timestamp=timestamp,
        command_timestamp=timestamp,
        command_counter=counter,
        command=Packet.Command.Value(command),
        stage=Packet.Stage.Value(stage),
        source_identifier=source,
    )
    for team in (p.blue, p.yellow):
        team.name = "test"
        team.score = team.red_cards = team.yellow_cards = 0
        team.timeouts = team.timeout_time = team.goalkeeper = 0
    return p


def send(feed, p, now):
    feed.accept(p.SerializeToString(), "127.0.0.1", now, p.packet_timestamp / 1e6)
    return feed.current(now)


@pytest.mark.parametrize(
    ("command", "mode", "team"),
    [
        ("HALT", Referee.HALT, ""),
        ("STOP", Referee.STOP, ""),
        ("FORCE_START", Referee.RUNNING, ""),
        ("PREPARE_KICKOFF_BLUE", Referee.STOP, ""),
        ("PREPARE_PENALTY_YELLOW", Referee.STOP, ""),
        ("BALL_PLACEMENT_BLUE", Referee.STOP, ""),
        ("DIRECT_FREE_YELLOW", Referee.RUNNING, "yellow"),
        ("INDIRECT_FREE_BLUE", Referee.RUNNING, "blue"),
        ("TIMEOUT_YELLOW", Referee.STOP, ""),
        ("NORMAL_START", Referee.STOP, ""),
    ],
)
def test_referee_commands(command, mode, team):
    s = send(RefereeFeed(), packet(command, 42, 100), 0)
    assert (s.mode, s.restart_team) == (mode, team)


def test_score_side_and_stage_can_change_without_a_new_command():
    feed = RefereeFeed()
    p = packet("FORCE_START", 4, 1)
    p.blue.score = 2
    p.blue_team_on_positive_half = True
    state = send(feed, p, 1)
    assert (state.score, state.attack_sign) == ((2, 0), -1)
    p.packet_timestamp += 1
    p.blue.score = 1
    p.stage = Packet.NORMAL_HALF_TIME
    assert send(feed, p, 1.1).mode == Referee.HALT
    p.packet_timestamp += 1
    p.stage = Packet.NORMAL_SECOND_HALF
    p.blue_team_on_positive_half = False
    state = send(feed, p, 1.2)
    assert (state.mode, state.score, state.attack_sign) == (Referee.RUNNING, (1, 0), 1)


def test_kickoff_history_wrap_and_missing_prepare():
    feed = RefereeFeed()
    send(feed, packet("PREPARE_KICKOFF_BLUE", 2**32 - 1, 1), 1)
    assert send(feed, packet("NORMAL_START", 0, 2), 1.1).restart_team == "blue"
    assert send(feed, packet("NORMAL_START", 0, 3), 1.2).restart_team == "blue"
    send(feed, packet("PREPARE_KICKOFF_YELLOW", 1, 4), 1.3)
    assert send(feed, packet("NORMAL_START", 3, 5), 1.4).mode == Referee.STOP
    send(feed, packet("PREPARE_PENALTY_BLUE", 4, 6), 1.5)
    assert send(feed, packet("NORMAL_START", 5, 7), 1.6).mode == Referee.STOP


def test_replays_garbage_and_foreign_feeds_do_not_extend_motion():
    feed = RefereeFeed()
    assert feed.current(0).mode == Referee.HALT
    p = packet("FORCE_START", 8, 10)
    send(feed, p, 1)
    send(feed, p, 1.4)
    send(feed, packet("HALT", 7, 11), 1.41)
    send(feed, packet("HALT", 0, 12, source="other"), 1.42)
    feed.accept(b"\xff", "local", 1.43, 0)
    feed.accept(b"", "local", 1.44, 0)
    feed.accept(packet("FORCE_START", 9, 13).SerializeToString(), "local", 1.45, 20)
    assert feed.current(1.49).mode == Referee.RUNNING
    assert feed.current(1.51).mode == Referee.HALT
    assert send(feed, packet("STOP", 0, 14, source="other"), 1.6).source == "other"


def test_restart_releases_once_per_command():
    gate = RestartGate()

    def team(counter, x):
        return gate.restricted_to("a", counter, "yellow", Ball(x, 0))

    assert team(2, 1) == team(2, 1.04) == "yellow"
    assert team(2, 1.06) == team(2, 1) == ""
    assert team(3, 1.06) == "yellow"


def test_placement_target_uses_metres_and_expires_on_halt_or_stale_feed():
    feed = RefereeFeed()
    p = packet("BALL_PLACEMENT_YELLOW", 8, 10)
    p.designated_position.x, p.designated_position.y = -1234, 2800
    state = send(feed, p, 1)
    assert state.placement_target == pytest.approx((-1.234, 2.8))
    assert state.placement_team == "yellow" and state.mode == Referee.STOP
    assert feed.current(1.51).placement_target is None
    p.packet_timestamp += 1
    p.stage = Packet.NORMAL_HALF_TIME
    assert send(feed, p, 1.6).placement_target is None
    p.packet_timestamp += 1
    p.stage = Packet.NORMAL_FIRST_HALF
    p.designated_position.x = float("nan")
    assert send(feed, p, 1.7).placement_target is None


def test_restart_retains_assignment_and_rejects_wrong_free_kick():
    from roboturk.five_roles import kickoff
    from roboturk.placement import PlacementRestart

    feed = RefereeFeed()
    w = kickoff(23, "blue", 1)
    actor = PlacementRestart([1.2, 0.6, 2.0, 0.04, 0.02])
    p = packet("BALL_PLACEMENT_BLUE", 8, 10)
    p.designated_position.x = 200
    p.designated_position.y = 2500
    state = send(feed, p, 1)
    commands, _ = actor.commands(w, state)
    assert len(commands) == 10 and all(c.kick == 0 for c in commands.values())
    play = actor.play
    actor.commands(w, state)
    assert actor.play is play and play.placer.team == play.receiver.team == "blue"
    assert play.placer.id != 0 and play.receiver != play.placer
    assert actor.commands(w, feed.current(1.6)) is None
    p.packet_timestamp += 1
    assert actor.commands(w, send(feed, p, 1.7)) is not None
    assert actor.commands(w, send(feed, packet("DIRECT_FREE_YELLOW", 9, 12), 1.8)) is None
    assert actor.play is None
