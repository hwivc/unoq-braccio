"""Workspace accuracy checks. Pure Python: run with ``python test_workspace.py``
or ``pytest``; no ROS, numpy or Gazebo needed.

Covers: world file matches braccio_workspace.py, every arm target is reachable
and IK agrees with the URDF forward kinematics, the overhead camera sees the
whole workspace at a usable resolution, and pixel->table error stays small.
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from unoq_braccio_driver import braccio_workspace as ws  # noqa: E402
from unoq_braccio_driver.braccio_kinematics import (  # noqa: E402
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    forward_kinematics,
    servo_to_urdf,
    solve_ik,
)

WORLD = os.path.join(
    os.path.dirname(__file__), "..", "..", "unoq_braccio_sim", "worlds", "workspace.world"
)
URDF = os.path.join(
    os.path.dirname(__file__), "..", "..", "unoq_braccio_sim", "urdf", "braccio.urdf.xacro"
)


def _models():
    root = ET.parse(WORLD).getroot()
    return {m.get("name"): m for m in root.iter("model")}


def _pose(model):
    return [float(v) for v in model.find("pose").text.split()]


def test_world_matches_workspace_module():
    models = _models()
    for color, (x, y) in ws.CUBES.items():
        m = models[f"{color}_cube"]
        px, py, pz = _pose(m)[:3]
        assert (px, py) == (x, y), color
        assert abs(pz - ws.CUBE_CENTRE_Z) < 1e-9
        size = [float(v) for v in m.find(".//collision/geometry/box/size").text.split()]
        assert size == [ws.CUBE_SIZE] * 3, color
    for b in ws.BINS:
        m = models[f"bin_{b.name}"]
        px, py, pz = _pose(m)[:3]
        assert (px, py) == b.centre, b.name
        assert abs(pz - b.height / 2) < 1e-9
        size = [float(v) for v in m.find(".//visual/geometry/box/size").text.split()]
        assert size == [b.size, b.size, b.height], b.name
    cam = _pose(models["overhead_camera"])
    assert tuple(cam[:3]) == ws.CAMERA_XYZ
    assert abs(cam[4] - math.pi / 2) < 1e-3  # pitched straight down
    sensor = models["overhead_camera"].find(".//sensor/camera")
    assert float(sensor.find("horizontal_fov").text) == ws.CAMERA_HFOV
    assert int(sensor.find("image/width").text) == ws.CAMERA_WIDTH
    assert int(sensor.find("image/height").text) == ws.CAMERA_HEIGHT


def test_cubes_start_in_pick_sector_and_clear_of_bins():
    for color, (x, y) in ws.CUBES.items():
        assert ws.sector_of(x, y) == "pick", color
        for b in ws.BINS:
            assert not b.sector.contains(x, y, ws.CUBE_SIZE), (color, b.name)
    # cubes must not touch each other
    pts = list(ws.CUBES.values())
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            assert math.dist(pts[i], pts[j]) > 2 * ws.CUBE_SIZE


def test_bins_do_not_overlap_and_are_distinct_colours():
    for i, a in enumerate(ws.BINS):
        for b in ws.BINS[i + 1:]:
            assert math.dist(a.centre, b.centre) > a.size, (a.name, b.name)
    assert not {b.name for b in ws.BINS} & set(ws.CUBE_HSV)
    assert len({b.cube_color for b in ws.BINS}) == len(ws.BINS)


def _hue_gaps():
    def spans(table):
        return {k: [(lo[0], hi[0]) for lo, hi in v] for k, v in table.items()}

    cube, binn = spans(ws.CUBE_HSV), spans(ws.BIN_HSV)
    for cname, cs in cube.items():
        for bname, bs in binn.items():
            for c_lo, c_hi in cs:
                for b_lo, b_hi in bs:
                    assert c_hi < b_lo or b_hi < c_lo, (cname, bname)


def test_cube_and_bin_hue_ranges_do_not_overlap():
    _hue_gaps()


def _urdf_joint(xacro, name):
    joint = ET.fromstring(
        xacro[xacro.index(f'<joint name="{name}"') : xacro.index("</joint>", xacro.index(f'<joint name="{name}"')) + 8]
        .replace("xacro:", "xacro_")
    )
    origin = joint.find("origin")
    limit = joint.find("limit")
    return (
        [float(v) for v in origin.get("xyz").split()],
        [float(v) for v in origin.get("rpy").split()],
        [float(v) for v in joint.find("axis").get("xyz").split()],
        float(limit.get("lower")),
        float(limit.get("upper")),
    )


def _finger_tip(origin, rpy, axis, angle, length=0.08):
    """Fingertip (x, z) in the wrist_roll frame. Both fingers have rpy only about
    y and axis +-y, and their meshes extend along the link's +x."""
    pitch = rpy[1] + axis[1] * angle
    return origin[0] + length * math.cos(pitch), origin[2] - length * math.sin(pitch)


def test_gripper_fingers_mirror_each_other():
    """Regression test for two real bugs. First, a left finger target outside
    its URDF limits, which ros2_control clamped so only one finger moved.
    Second, "left = offset - right", which kept the left target in range but
    swung both fingers to the same side together instead of pinching.

    This reads the finger joints straight from the URDF, then checks that every
    servo value stays in range, that the fingertips are mirror images, and
    that they move towards each other as the gripper closes.
    """
    xacro = open(URDF, encoding="utf-8").read()
    right = _urdf_joint(xacro, "gripper")
    left = _urdf_joint(xacro, "left_gripper")

    previous_gap = None
    for servo in range(GRIPPER_OPEN, 111):
        r_angle = servo_to_urdf("gripper", servo)
        l_angle = servo_to_urdf("left_gripper", servo)
        for angle, (_, _, _, lower, upper) in ((r_angle, right), (l_angle, left)):
            assert lower - 1e-6 <= angle <= upper + 1e-6, (servo, angle, lower, upper)
        rx, rz = _finger_tip(*right[:3], r_angle)
        lx, lz = _finger_tip(*left[:3], l_angle)
        assert abs(rx + lx) < 1e-3 and abs(rz - lz) < 1e-3, (servo, (rx, rz), (lx, lz))
        assert rx > 0 > lx, (servo, rx, lx)
        gap = rx - lx
        if previous_gap is not None:
            assert gap < previous_gap, "fingers must close towards each other"
        previous_gap = gap


def test_finger_pads_pinch_a_cube_flat():
    """The finger collision pads are what actually holds a cube in Gazebo.
    They must sit centred on the finger plates (y = 0, where the IK puts the
    cube), be parallel when they first touch a 30 mm cube, and touch it just
    before GRIPPER_CLOSED so the gripper squeezes rather than misses."""
    xacro = open(URDF, encoding="utf-8").read()
    faces = {}
    for joint, link in (("gripper", "right_gripper_link"), ("left_gripper", "left_gripper_link")):
        origin, rpy, axis, _, _ = _urdf_joint(xacro, joint)
        start = xacro.index(f'<link name="{link}">')
        col = ET.fromstring(xacro[xacro.index("<collision>", start) : xacro.index("</collision>", start) + 12])
        cxyz = [float(v) for v in col.find("origin").get("xyz").split()]
        cpitch = float(col.find("origin").get("rpy").split()[1])
        size = [float(v) for v in col.find("geometry/box").get("size").split()]
        assert abs(cxyz[1]) < 1e-6, f"{link} pad is off the finger plate"
        for servo in range(GRIPPER_OPEN, 111):
            angle = rpy[1] + axis[1] * servo_to_urdf(joint, servo)
            corners = []
            for u in (-size[0] / 2, size[0] / 2):
                for v in (-size[2] / 2, size[2] / 2):
                    lx = cxyz[0] + u * math.cos(cpitch) + v * math.sin(cpitch)
                    lz = cxyz[2] - u * math.sin(cpitch) + v * math.cos(cpitch)
                    corners.append(origin[0] + lx * math.cos(angle) + lz * math.sin(angle))
            inner = min(abs(c) for c in corners)
            faces.setdefault(servo, []).append((inner, math.degrees(angle + cpitch) % 180.0))
    touch = next(s for s in range(GRIPPER_OPEN, 111) if faces[s][0][0] + faces[s][1][0] <= ws.CUBE_SIZE)
    assert GRIPPER_CLOSED - 10 <= touch < GRIPPER_CLOSED, (touch, GRIPPER_CLOSED)
    for x, pitch in faces[touch]:
        assert abs(pitch - 90.0) < 3.0, f"pad not vertical at contact: {pitch:.1f} deg"


def test_arm_targets_reachable_and_ik_matches_urdf():
    worst = 0.0
    targets = []
    for color, (x, y) in ws.CUBES.items():
        b = ws.BIN_BY_CUBE_COLOR[color]
        targets += [(x, y, ws.HOVER_Z, GRIPPER_OPEN), (x, y, ws.CUBE_CENTRE_Z, GRIPPER_OPEN),
                    (x, y, ws.CUBE_CENTRE_Z, GRIPPER_CLOSED)]
        for dx, dy in ws.BIN_SLOT_OFFSETS:
            bx, by = b.centre[0] + dx, b.centre[1] + dy
            targets += [(bx, by, ws.HOVER_Z, GRIPPER_CLOSED),
                        (bx, by, ws.release_z(b), GRIPPER_CLOSED),
                        (bx, by, ws.release_z(b), GRIPPER_OPEN)]
    for x, y, z, g in targets:
        pose = solve_ik(x, y, z, g)
        assert pose is not None, (x, y, z)
        tip = forward_kinematics(pose[:5])
        worst = max(worst, math.dist(tip, (x, y, z)))
    assert worst < 0.005, worst


def test_camera_covers_workspace_at_usable_resolution():
    fx = ws.CAMERA_FX
    cx, cy = ws.CAMERA_WIDTH / 2, ws.CAMERA_HEIGHT / 2
    cam_x, cam_y, cam_z = ws.CAMERA_XYZ
    corners = []
    for rect in [ws.PICK_SECTOR] + [b.sector for b in ws.BINS]:
        for sx in (-1, 1):
            for sy in (-1, 1):
                corners.append((rect.x + sx * rect.size_x / 2, rect.y + sy * rect.size_y / 2))
    for x, y in corners:
        u, v = ws.table_to_pixel(x, y, 0.0, fx, fx, cx, cy, cam_x, cam_y, cam_z)
        assert 10 <= u <= ws.CAMERA_WIDTH - 10 and 10 <= v <= ws.CAMERA_HEIGHT - 10, (x, y, u, v)
    mm_per_px = (cam_z - ws.CUBE_CENTRE_Z) / fx * 1000
    assert mm_per_px < 1.5, mm_per_px
    assert ws.CUBE_SIZE * 1000 / mm_per_px > 20  # cube at least ~20 px wide


def _project(p):
    cx, cy = ws.CAMERA_WIDTH / 2, ws.CAMERA_HEIGHT / 2
    return ws.table_to_pixel(p[0], p[1], p[2], ws.CAMERA_FX, ws.CAMERA_FX, cx, cy,
                             ws.CAMERA_XYZ[0], ws.CAMERA_XYZ[1], ws.CAMERA_XYZ[2])


def _hull(points):
    points = sorted(set(points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _centroid(poly):
    area = sx = sy = 0.0
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        c = x0 * y1 - x1 * y0
        area += c
        sx += (x0 + x1) * c
        sy += (y0 + y1) * c
    return sx / (3 * area), sy / (3 * area)


def test_pixel_to_table_error_under_2mm_across_pick_sector():
    """Silhouette centroid of a perspective-projected cube, back-projected at mid height."""
    fx = ws.CAMERA_FX
    cx, cy = ws.CAMERA_WIDTH / 2, ws.CAMERA_HEIGHT / 2
    cam_x, cam_y, cam_z = ws.CAMERA_XYZ
    half = ws.CUBE_SIZE / 2
    worst = 0.0
    for i in range(7):
        for j in range(7):
            x = ws.PICK_SECTOR.x + (i / 6 - 0.5) * (ws.PICK_SECTOR.size_x - ws.CUBE_SIZE)
            y = ws.PICK_SECTOR.y + (j / 6 - 0.5) * (ws.PICK_SECTOR.size_y - ws.CUBE_SIZE)
            pts = [_project((x + a * half, y + b * half, c))
                   for a in (-1, 1) for b in (-1, 1) for c in (0.0, ws.CUBE_SIZE)]
            u, v = _centroid(_hull(pts))
            ex, ey = ws.pixel_to_table(u, v, fx, fx, cx, cy, cam_x, cam_y,
                                       cam_z - ws.CUBE_CENTRE_Z)
            worst = max(worst, math.hypot(ex - x, ey - y))
    assert worst < 0.002, worst


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as exc:
                failed += 1
                print(f"FAIL {name}: {exc!r}")
    sys.exit(1 if failed else 0)
