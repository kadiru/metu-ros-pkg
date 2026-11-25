"""Generate the reusable Division B Webots pitch from the SSL-Vision dimensions."""

import math
from pathlib import Path
from struct import pack
from zlib import compress, crc32

import numpy as np

from roboturk import field as f

PROTO = (
    Path(__file__).resolve().parents[1] / "src/roboturk_ros/webots/protos/SslDivisionBField.proto"
)
CARPET_TEXTURE = PROTO.with_name("division_b_carpet.png")
CARPET_SIZE = (f.LENGTH + 2 * f.CARPET_MARGIN, f.WIDTH + 2 * f.CARPET_MARGIN)
WHITE, BLACK = "0.96 0.96 0.96", "0.033 0.04 0.046"
BLUE, YELLOW = "0.07 0.26 1", "1 0.79 0.05"
PAINT_GLOW, FOAM_GLOW = "0.27 0.27 0.27", "0.15 0.15 0.15"


def vec(values):
    return " ".join(f"{v:.8g}" for v in values)


def box(name, center, size, color, material=None, *, glow=None):
    body = "Solid" if material else "Pose"
    geometry = f"Box {{ size {vec(size)} }}"
    physics = (
        f'\n      name "{name.lower()}" contactMaterial "{material}" boundingObject {geometry}'
        if material
        else ""
    )
    emission = f" emissiveColor {glow}" if glow else ""
    return f"""    DEF {name} {body} {{
      translation {vec(center)}
      children [ Shape {{
        appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0{emission} }}
        geometry {geometry}
      }} ]{physics}
    }}"""


def carpet_texture():
    """Render a matte green carpet with subtle alternating nap and fine grain."""
    width, height = 2048, 1024
    x = CARPET_SIZE[0] * ((np.arange(width) + 0.5) / width - 0.5)
    y = CARPET_SIZE[1] * ((np.arange(height) + 0.5) / height - 0.5)[:, None]
    on_pitch = (abs(x) < f.LENGTH / 2) & (abs(y) < f.WIDTH / 2)
    stripe_width = f.LENGTH / 10
    nap = np.tanh(12 * np.sin(math.pi * (x + f.LENGTH / 2) / stripe_width))
    variation = np.sin(2.1 * x + 1.7 * y) + 0.5 * np.sin(4.4 * x - 2.2 * y)
    grain = np.random.default_rng(904).standard_normal((height, width))
    grain = 0.7 * grain + 0.3 * np.roll(grain, 1, axis=0)
    tone = np.where(on_pitch, 11.0 * nap, -18.0) + 1.6 * variation + 2.0 * grain
    rgb = np.rint((67, 139, 68) + tone[..., None] * (0.62, 1.0, 0.64)).astype(np.uint8)

    def chunk(kind, payload):
        return pack(">I", len(payload)) + kind + payload + pack(">I", crc32(kind + payload))

    scanlines = b"".join(b"\0" + row.tobytes() for row in rgb)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", compress(scanlines, level=9))
        + chunk(b"IEND", b"")
    )


def carpet_node():
    width, height = (m / 2 for m in CARPET_SIZE)
    corners = [(-width, -height), (width, -height), (width, height), (-width, height)]
    points = ", ".join(vec((*xy, 0.025)) for xy in corners)
    return f"""    DEF CARPET Solid {{
      translation 0 0 -0.025
      children [ Shape {{
        appearance PBRAppearance {{
          baseColorMap ImageTexture {{
            url [ "{CARPET_TEXTURE.name}" ]
            repeatS FALSE
            repeatT FALSE
            filtering 4
          }}
          roughness 1
          metalness 0
        }}
        geometry IndexedFaceSet {{
          coord Coordinate {{ point [ {points} ] }}
          coordIndex [ 0 1 2 3 -1 ]
          texCoord TextureCoordinate {{ point [ 0 0, 1 0, 1 1, 0 1 ] }}
        }}
      }} ]
      name "carpet" contactMaterial "floor" boundingObject Box {{ size {vec((*CARPET_SIZE, 0.05))} }}
    }}"""


def center_circle():
    # The same 10 mm stroke as the painted straight markings.
    points, faces = [], []
    segments = 128
    for i in range(segments):
        a = math.tau * i / segments
        for r in (f.CIRCLE_RADIUS - f.LINE_WIDTH / 2, f.CIRCLE_RADIUS + f.LINE_WIDTH / 2):
            points.append(vec((r * math.cos(a), r * math.sin(a), 0.0012)))
        j = 2 * ((i + 1) % segments)
        faces.append(f"{2 * i} {2 * i + 1} {j + 1} {j} -1")
    return f"""    DEF CENTER_CIRCLE Shape {{
      appearance PBRAppearance {{ baseColor {WHITE} roughness 1 metalness 0 emissiveColor {PAINT_GLOW} }}
      geometry IndexedFaceSet {{
        coord Coordinate {{ point [ {", ".join(points)} ] }}
        coordIndex [ {", ".join(faces)} ]
      }}
    }}"""


def field_proto():
    x, y = f.LENGTH / 2, f.WIDTH / 2
    margin = f.WALL_MARGIN
    nodes = [carpet_node()]
    for name, (x1, y1), (x2, y2) in f.field_lines():
        nodes.append(
            box(
                name,
                ((x1 + x2) / 2, (y1 + y2) / 2, 0.0005),
                (abs(x2 - x1) or f.LINE_WIDTH, abs(y2 - y1) or f.LINE_WIDTH, 0.001),
                WHITE,
                glow=PAINT_GLOW,
            )
        )
    nodes.append(center_circle())
    board_width, board_height = 0.02, 0.12
    for sign, end in ((-1, "LEFT"), (1, "RIGHT")):
        nodes.append(
            box(
                end + "_END_BOARD",
                (sign * (x + margin + board_width / 2), 0, board_height / 2),
                (board_width, 2 * (y + margin), board_height),
                BLACK,
                "board",
            )
        )
        nodes.append(
            box(
                end + "_TOUCH_BOARD",
                (0, sign * (y + margin + board_width / 2), board_height / 2),
                (2 * (x + margin + board_width), board_width, board_height),
                BLACK,
                "board",
            )
        )
        start = x + f.LINE_WIDTH / 2
        depth = margin - f.LINE_WIDTH / 2
        wall, height = f.GOAL_WALL, f.GOAL_HEIGHT
        rear = start + f.GOAL_DEPTH
        marker = "IS leftGoalMarkerColor" if sign < 0 else "IS rightGoalMarkerColor"
        nodes.append(
            box(
                end + "_GOAL_MARKER",
                (sign * (x + margin + board_width + 0.075), 0, 0.001),
                (0.15, 1.00, 0.002),
                marker,
            )
        )
        spot_x = sign * (f.GOAL_TO_PENALTY_MARK - x)
        nodes.append(f"""    DEF {end}_PENALTY_MARK Pose {{
      translation {vec((spot_x, 0, 0.002))}
      children [ Shape {{
        appearance PBRAppearance {{ baseColor {WHITE} roughness 1 metalness 0 emissiveColor {PAINT_GLOW} }}
        geometry Cylinder {{ radius 0.018 height 0.001 subdivision 24 }}
      }} ]
    }}""")
        nodes.append(
            box(
                end + "_GOAL_BACK",
                (sign * (rear + wall / 2), 0, height / 2),
                (wall, f.GOAL_WIDTH, height),
                BLACK,
                "goal",
            )
        )
        # The lining is visual. The solids keep the original collision faces.
        skin = 0.0001
        nodes.append(
            box(
                end + "_BACK_LINING",
                (sign * (rear - skin / 2), 0, height / 2),
                (skin, f.GOAL_WIDTH, height),
                WHITE,
                glow=FOAM_GLOW,
            )
        )
        for side, label in ((-1, "BOTTOM"), (1, "TOP")):
            cx = sign * (start + depth / 2)
            nodes.append(
                box(
                    end + "_GOAL_" + label,
                    (cx, side * (f.GOAL_WIDTH + wall) / 2, height / 2),
                    (depth, wall, height),
                    BLACK,
                    "goal",
                )
            )
            nodes.append(
                box(
                    end + "_" + label + "_LINING",
                    (
                        sign * (start + f.GOAL_DEPTH / 2),
                        side * (f.GOAL_WIDTH - skin) / 2,
                        height / 2,
                    ),
                    (f.GOAL_DEPTH, skin, height),
                    WHITE,
                    glow=FOAM_GLOW,
                )
            )
    return (
        "#VRML_SIM R2025a utf8\n# Generated by scripts/build_ssl_field.py\n"
        "PROTO SslDivisionBField [\n"
        f"  field SFColor leftGoalMarkerColor {BLUE}\n"
        f"  field SFColor rightGoalMarkerColor {YELLOW}\n"
        "] {\n  Group { children [\n" + "\n".join(nodes) + "\n  ] }\n}\n"
    )


if __name__ == "__main__":
    PROTO.write_text(field_proto())
    CARPET_TEXTURE.write_bytes(carpet_texture())
