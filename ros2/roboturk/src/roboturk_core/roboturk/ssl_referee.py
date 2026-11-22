"""Receive the SSL Game Controller broadcast and resolve its team restart commands."""

from dataclasses import dataclass, replace
from importlib.resources import files
from ipaddress import IPv4Address
import math
import socket
import time

from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
from google.protobuf.message import DecodeError

from .model import Referee

GROUP = "224.5.23.1"
PORT = 10003
TIMEOUT = 0.5


def _referee_type():
    schema = files(__package__).joinpath("ssl_protocol/referee.pb").read_bytes()
    pool = descriptor_pool.DescriptorPool()
    for descriptor in descriptor_pb2.FileDescriptorSet.FromString(schema).file:
        pool.Add(descriptor)
    descriptor = pool.FindMessageTypeByName("Referee")
    if hasattr(message_factory, "GetMessageClass"):
        return message_factory.GetMessageClass(descriptor)
    return message_factory.MessageFactory(pool).GetPrototype(descriptor)  # ROS Jazzy protobuf


Packet = _referee_type()


@dataclass(frozen=True)
class Decision:
    mode: Referee = Referee.HALT
    command: str = ""
    stage: str = ""
    counter: int = 0
    source: str = ""
    attack_sign: int = 1
    score: tuple[int, int] = (0, 0)  # blue, yellow
    restart_team: str = ""
    placement_team: str = ""
    placement_target: tuple[float, float] | None = None


def resolve(command, previous, consecutive):
    if command == "FORCE_START":
        return Referee.RUNNING, ""
    if command == "NORMAL_START" and consecutive and previous.startswith("PREPARE_KICKOFF_"):
        return Referee.RUNNING, previous.split("_")[-1].lower()
    if command.startswith(("DIRECT_FREE_", "INDIRECT_FREE_")):
        return Referee.RUNNING, command.split("_")[-1].lower()
    if command == "HALT":
        return Referee.HALT, ""
    return Referee.STOP, ""


class RefereeFeed:
    def __init__(self):
        self.packet = None
        self.last = Decision()
        self.received = -math.inf
        self.timestamp = -1

    def accept(self, data, sender, now, unix_time):
        try:
            p = Packet.FromString(data)
        except DecodeError:
            return
        if not p.IsInitialized() or abs(unix_time - p.packet_timestamp / 1e6) > TIMEOUT:
            return
        source = p.source_identifier or sender
        same = source == self.last.source
        if not same and now - self.received <= TIMEOUT:
            return
        if same and p.packet_timestamp <= self.timestamp:
            return
        delta = (p.command_counter - self.last.counter) % (1 << 32)
        if same and delta >= 1 << 31:
            return
        command = Packet.Command.Name(p.command)
        stage = Packet.Stage.Name(p.stage)
        if same and delta == 0 and command == self.last.command:
            mode, team = self.last.mode, self.last.restart_team
        else:
            mode, team = resolve(command, self.last.command, same and delta == 1)
        sign = self.last.attack_sign if same else 1
        if p.HasField("blue_team_on_positive_half"):
            sign = -1 if p.blue_team_on_positive_half else 1
        placing = ""
        target = None
        if command.startswith("BALL_PLACEMENT_") and p.HasField("designated_position"):
            x, y = p.designated_position.x / 1000, p.designated_position.y / 1000
            if math.isfinite(x) and math.isfinite(y):
                placing, target = command.split("_")[-1].lower(), (x, y)
        self.packet = p
        self.last = Decision(
            mode,
            command,
            stage,
            p.command_counter,
            source,
            sign,
            (p.blue.score, p.yellow.score),
            team,
            placing,
            target,
        )
        self.timestamp, self.received = p.packet_timestamp, now

    def current(self, now):
        if now - self.received > TIMEOUT:
            return Decision(attack_sign=self.last.attack_sign, score=self.last.score)
        if self.last.stage.endswith(("HALF_TIME", "BREAK")) or self.last.stage == "POST_GAME":
            return replace(
                self.last,
                mode=Referee.HALT,
                restart_team="",
                placement_team="",
                placement_target=None,
            )
        return self.last


class RefereeSocket:
    def __init__(self, address=GROUP, port=PORT, interface="0.0.0.0"):
        group = IPv4Address(address)
        if not 1 <= port <= 65535:
            raise ValueError("Expected a UDP port between 1 and 65535")
        self.feed = RefereeFeed()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if group.is_multicast and hasattr(socket, "SO_REUSEPORT"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            sock.bind(("" if group.is_multicast else str(group), port))
            if group.is_multicast:
                membership = group.packed + IPv4Address(interface).packed
                sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
            sock.setblocking(False)
        except Exception:
            sock.close()
            raise
        self.socket = sock

    def poll(self):
        for _ in range(256):
            try:
                data, (sender, _) = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            self.feed.accept(data, sender, time.monotonic(), time.time())
        return self.feed.current(time.monotonic())

    def close(self):
        self.socket.close()


class RestartGate:
    """Only the awarded team plays until the ball moves 5 cm on its restart."""

    def __init__(self):
        self.key = None
        self.origin = None
        self.released = False

    def restricted_to(self, source, counter, restart_team, ball):
        if not restart_team:
            self.key = None
            return ""
        key = (source, counter, restart_team)
        if key != self.key:
            self.key, self.origin, self.released = key, (ball.x, ball.y), False
        if math.dist(self.origin, (ball.x, ball.y)) >= 0.05:
            self.released = True
        return "" if self.released else restart_team
