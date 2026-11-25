"""Ground-truth SSL-Vision camera and length-delimited TIGERs simulator transport."""

from importlib.resources import files
import math
from select import select
import socket

from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

from . import field as pitch
from .carpet import BALL_RADIUS, ROLL_DECEL, SLIDE_DECEL, SWITCH_RATIO
from .ssl_referee import Packet

pool = descriptor_pool.DescriptorPool()
for item in descriptor_pb2.FileDescriptorSet.FromString(
    files(__package__).joinpath("ssl_protocol/vision.pb").read_bytes()
).file:
    pool.Add(item)


def message_type(name):
    desc = pool.FindMessageTypeByName(name)
    if hasattr(message_factory, "GetMessageClass"):
        return message_factory.GetMessageClass(desc)
    return message_factory.MessageFactory(pool).GetPrototype(desc)


Wrapper = message_type("SSL_WrapperPacket")
CiInput = message_type("AutoRefCiInput")
CiOutput = message_type("AutoRefCiOutput")
GameEvent = message_type("GameEvent")

# Timestamp-only subset of the official SSL Game Controller CiInput.
clock_schema = descriptor_pb2.FileDescriptorProto(name="game_controller_clock.proto")
clock_schema.message_type.add(name="GameControllerTick").field.add(
    name="timestamp", number=1, type=descriptor_pb2.FieldDescriptorProto.TYPE_INT64
)
pool.Add(clock_schema)
GameControllerTick = message_type("GameControllerTick")


def geometry():
    packet = Wrapper(source=1)
    field = packet.geometry.field

    def mm(value):
        return round(1000 * value)

    field.field_length, field.field_width = mm(pitch.LENGTH), mm(pitch.WIDTH)
    field.goal_width = mm(pitch.GOAL_WIDTH)
    field.goal_depth, field.goal_height = mm(pitch.GOAL_DEPTH), mm(pitch.GOAL_HEIGHT)
    field.boundary_width = field.boundary_width_goal_line = mm(pitch.WALL_MARGIN)
    field.line_thickness, field.center_circle_radius = mm(pitch.LINE_WIDTH), mm(pitch.CIRCLE_RADIUS)
    field.penalty_area_depth, field.penalty_area_width = (
        mm(pitch.DEFENSE_DEPTH),
        mm(pitch.DEFENSE_WIDTH),
    )
    field.goal_center_to_penalty_mark = mm(pitch.GOAL_TO_PENALTY_MARK)
    field.ball_radius, field.max_robot_radius = 1000 * BALL_RADIUS, 90
    model = packet.geometry.models.straight_two_phase
    model.acc_slide, model.acc_roll, model.k_switch = -SLIDE_DECEL, -ROLL_DECEL, SWITCH_RATIO
    for name, start, end in pitch.field_lines():
        line = field.field_lines.add(name=name, type=name, thickness=field.line_thickness)
        line.p1.x, line.p1.y = map(mm, start)
        line.p2.x, line.p2.y = map(mm, end)
    arc = field.field_arcs.add(
        name="CenterCircle",
        type="CenterCircle",
        radius=field.center_circle_radius,
        a1=0,
        a2=2 * math.pi,
        thickness=field.line_thickness,
    )
    arc.center.x = arc.center.y = 0
    # Ideal overhead pinhole with x right, field y up, optical axis down.
    packet.geometry.calib.add(
        camera_id=0,
        focal_length=1000,
        principal_point_x=1000,
        principal_point_y=700,
        distortion=0,
        q0=1,
        q1=0,
        q2=0,
        q3=0,
        tx=0,
        ty=0,
        tz=6000,
        derived_camera_world_tx=0,
        derived_camera_world_ty=0,
        derived_camera_world_tz=6000,
        pixel_image_width=2000,
        pixel_image_height=1400,
    )
    return packet


def detection(world, frame, seconds, ball_height):
    packet = Wrapper(source=1)
    d = packet.detection
    d.frame_number, d.camera_id, d.t_capture, d.t_sent = frame, 0, seconds, seconds

    def observe(entity, z):
        x, y = entity.x * 1000, entity.y * 1000
        scale = 1000 / (6000 - z)
        return dict(confidence=1, x=x, y=y, pixel_x=1000 + x * scale, pixel_y=700 - y * scale)

    d.balls.add(**observe(world.ball, ball_height * 1000), z=ball_height * 1000)
    for r in world.robots:
        robots = d.robots_blue if r.key.team == "blue" else d.robots_yellow
        robots.add(**observe(r, 150), robot_id=r.key.id, orientation=r.yaw, height=150)
    return packet


def practice_command(sign):
    p = Packet(
        stage=Packet.NORMAL_FIRST_HALF,
        command=Packet.FORCE_START,
        command_counter=1,
        command_timestamp=0,
        packet_timestamp=0,
        blue_team_on_positive_half=sign < 0,
    )
    for team in (p.blue, p.yellow):
        team.name = ""
        team.score = team.red_cards = team.yellow_cards = 0
        team.timeouts = team.timeout_time = team.goalkeeper = 0
    return p


def write_delimited(stream, message):
    data = message.SerializeToString()
    n = len(data)
    prefix = bytearray()
    while n >= 128:
        prefix.append((n & 127) | 128)
        n >>= 7
    stream.write(bytes(prefix) + bytes([n]) + data)
    stream.flush()


def read_delimited(stream, cls):
    size = 0
    for shift in range(0, 35, 7):
        byte = stream.read(1)
        if not byte:
            raise EOFError("AutoRef disconnected")
        size |= (byte[0] & 127) << shift
        if byte[0] < 128:
            if size > 1 << 20:
                break
            data = stream.read(size)
            if len(data) != size:
                raise EOFError("Incomplete AutoRef response")
            return cls.FromString(data)
    raise ValueError("Invalid AutoRef frame length")


class GameControllerClock:
    def __init__(self, address):
        self.socket = socket.create_connection(address, timeout=5)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.stream = self.socket.makefile("wb")

    def send(self, seconds):
        # Referee packets still arrive over UDP. Drain the CI copy so that
        # the Game Controller can keep publishing on both transports.
        if select([self.socket], [], [], 0)[0] and not self.socket.recv(65536):
            raise ConnectionError("Game Controller clock disconnected")
        write_delimited(self.stream, GameControllerTick(timestamp=round(seconds * 1e9)))

    def close(self):
        self.stream.close()
        self.socket.close()


class VisionOutput:
    def __init__(self, udp, ci, tracker=None, *, gc_clock=None):
        self.udp = tuple(udp) if udp else None
        self.tracker = tuple(tracker) if tracker else None
        self.datagrams = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.datagrams.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        self.stream = None
        if ci:
            with socket.create_connection(ci, timeout=5) as connection:
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.stream = connection.makefile("rwb")
        self.ci_started = False
        self.sequence = 0
        self.geometry = geometry()
        self.gc_clock = GameControllerClock(gc_clock) if gc_clock else None

    def send(self, world, seconds, height, referee):
        return self.send_frame(detection(world, self.sequence, seconds, height), seconds, referee)

    def send_frame(self, frame, seconds, referee):
        if self.gc_clock:
            self.gc_clock.send(seconds)
        frame.detection.frame_number = self.sequence
        frame.detection.t_capture = frame.detection.t_sent = seconds
        send_geometry = self.sequence % 25 == 0
        self.sequence += 1
        if self.udp:
            if send_geometry:
                self.datagrams.sendto(self.geometry.SerializeToString(), self.udp)
            self.datagrams.sendto(frame.SerializeToString(), self.udp)
        if self.stream and referee is not None:
            msg = CiInput()
            msg.detection.add().CopyFrom(frame.detection)
            if send_geometry or not self.ci_started:
                msg.geometry.CopyFrom(self.geometry.geometry)
            self.ci_started = True
            msg.referee_message.ParseFromString(referee.SerializeToString())
            write_delimited(self.stream, msg)
            output = read_delimited(self.stream, CiOutput)
            if self.tracker and output.HasField("tracker_wrapper_packet"):
                self.datagrams.sendto(
                    output.tracker_wrapper_packet.SerializeToString(), self.tracker
                )
            if output.HasField("tracker_wrapper_packet"):
                return output.tracker_wrapper_packet.tracked_frame

    def close(self):
        if self.stream:
            self.stream.close()
        if self.gc_clock:
            self.gc_clock.close()
        self.datagrams.close()
