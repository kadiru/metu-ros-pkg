"""Ground-truth SSL-Vision camera and length-delimited TIGERs simulator transport."""

from importlib.resources import files
import math
import socket

from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

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


def geometry():
    packet = Wrapper(source=1)
    field = packet.geometry.field
    field.field_length, field.field_width = 9000, 6000
    field.goal_width, field.goal_depth, field.goal_height = 1000, 190, 160
    field.boundary_width = 500
    field.line_thickness, field.center_circle_radius = 10, 500
    field.penalty_area_depth, field.penalty_area_width = 950, 2000
    field.ball_radius, field.max_robot_radius = 21.5, 90
    for name, start, end in (
        ("TopTouchLine", (-4500, 3000), (4500, 3000)),
        ("BottomTouchLine", (-4500, -3000), (4500, -3000)),
        ("LeftGoalLine", (-4500, -3000), (-4500, 3000)),
        ("RightGoalLine", (4500, -3000), (4500, 3000)),
        ("HalfwayLine", (0, -3000), (0, 3000)),
    ):
        line = field.field_lines.add(name=name, thickness=10)
        line.p1.x, line.p1.y = start
        line.p2.x, line.p2.y = end
    arc = field.field_arcs.add(name="CenterCircle", radius=500, a1=0, a2=2 * math.pi, thickness=10)
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


class VisionOutput:
    def __init__(self, udp, ci, tracker=None):
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

    def send(self, world, seconds, height, referee):
        return self.send_frame(detection(world, self.sequence, seconds, height), seconds, referee)

    def send_frame(self, frame, seconds, referee):
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
        self.datagrams.close()
