from dataclasses import replace
from roboturk.five_roles import FiveRoles, Flight, TEAMS, kickoff
from roboturk.model import RobotKey


def test_ten_roles_and_goalkeeper_identity():
    world = kickoff(12, "blue", 1)
    tactics = FiveRoles()
    for _ in range(3):
        owner = tactics.observe(world)
    for team in TEAMS:
        phase, roles = tactics.assign(world, team)
        assert phase == ("Attack" if team == "blue" else "Defend")
        assert len(roles) == len({a.role for a in roles.values()}) == 5
        assert roles[RobotKey(team, 0)].role == "Keeper"
        if team == "blue":
            assert roles[owner].role == "Carrier"


def test_pass_recipient_is_bound_until_opponent_gains_control():
    w = kickoff(15, "yellow", -1)
    t = FiveRoles()
    for _ in range(3):
        holder = t.observe(w)
    receiver = next(r.key for r in w.robots if r.key.team == "blue" and r.key.id != 0)
    t.flight = Flight("blue", receiver, (1.0, -0.6), w.time + 1)
    phase, roles = t.assign(w, "blue")
    assert phase == "Attack" and roles[receiver].role == "Receiver"
    t.observe(w)
    assert t.flight is None and t.control == holder
    assert t.assign(w, "blue")[0] == "Defend"


def test_assignment_moves_with_the_players_and_respects_attack_direction():
    w = kickoff(9, "blue", 1)
    a = FiveRoles()
    _, first = a.assign(w, "blue")
    keys = [k for k in first if k.id]
    swapped = {keys[0]: keys[1], keys[1]: keys[0]}
    alt = replace(
        w,
        robots=tuple(replace(r, key=swapped.get(r.key, r.key), x=-r.x, y=-r.y) for r in w.robots),
        ball=replace(w.ball, x=-w.ball.x, y=-w.ball.y),
        attack_sign=-1,
    )
    _, second = FiveRoles().assign(alt, "blue")
    for key, role in first.items():
        other = second[swapped.get(key, key)]
        assert other.role == role.role and other.target == tuple(-v for v in role.target)
