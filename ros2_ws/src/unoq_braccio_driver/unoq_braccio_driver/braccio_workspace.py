"""Single source of truth for the simulated workspace layout.

Everything that has to agree about where things are (the Gazebo world, the
overhead cube detector, the pick-and-place planner and the RViz markers) reads
this module. ``test/test_workspace.py`` checks that ``worlds/workspace.world``
still matches it.

Frame: world, metres. The arm base is at the origin and, at servo base=90,
faces +x. +y is to the arm's left.
"""

import math
from dataclasses import dataclass

# --- cubes --------------------------------------------------------------------
CUBE_SIZE = 0.03            # 30 mm cube
CUBE_CENTRE_Z = CUBE_SIZE / 2.0

# --- overhead camera ------------------------------------------------------------
# Looks straight down (pitch +90 deg): image right = world -y, image down = world -x.
CAMERA_XYZ = (0.20, 0.0, 0.60)
CAMERA_HFOV = 1.0           # rad
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FX = (CAMERA_WIDTH / 2.0) / math.tan(CAMERA_HFOV / 2.0)


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle on the table: centre and size."""

    x: float
    y: float
    size_x: float
    size_y: float

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (
            abs(x - self.x) <= self.size_x / 2.0 + margin
            and abs(y - self.y) <= self.size_y / 2.0 + margin
        )


# --- sectors ----------------------------------------------------------------------
# Where cubes start. Cubes outside this sector are never picked.
PICK_SECTOR = Rect(x=0.225, y=-0.09, size_x=0.15, size_y=0.23)


@dataclass(frozen=True)
class Bin:
    name: str               # colour of the bin itself
    cube_color: str         # colour of the cube that belongs in it
    centre: tuple           # (x, y)
    size: float = 0.08
    height: float = 0.02

    @property
    def sector(self) -> Rect:
        return Rect(self.centre[0], self.centre[1], self.size, self.size)


# Bin colours are deliberately not cube colours, so the camera cannot confuse a
# bin with a cube. Bins sit on an arc of radius 0.24 m around the arm base.
BINS = (
    Bin("green", "red", (0.2255, 0.082)),
    Bin("cyan", "blue", (0.154, 0.184)),
    Bin("magenta", "yellow", (0.0417, 0.236)),
)
BIN_BY_CUBE_COLOR = {b.cube_color: b for b in BINS}
BIN_BY_NAME = {b.name: b for b in BINS}

# Cubes as placed in worlds/workspace.world (all inside PICK_SECTOR).
CUBES = {
    "red": (0.20, -0.15),
    "blue": (0.20, -0.08),
    "yellow": (0.20, -0.01),
}

# --- simulated grasp -----------------------------------------------------------------
# Gazebo does not hold a cube by finger friction reliably, so in simulation a
# grasped cube is welded to the wrist with a DetachableJoint (one per cube in
# braccio.urdf.xacro) and let go again on release. sim_grasp_attacher drives
# them; these names must match the xacro (test/test_workspace.py checks it).
GRASP_PARENT_LINK = "wrist_roll_link"


def cube_model(color: str) -> str:
    """Gazebo model name of the cube of ``color``."""
    return f"{color}_cube"


def grasp_topic(color: str, action: str) -> str:
    """Gazebo/ROS topic of a cube's DetachableJoint: attach, detach or state."""
    return f"/sim/grasp/{cube_model(color)}/{action}"


# Slots inside a bin so several cubes do not land on the same spot.
BIN_SLOT_OFFSETS = ((-0.02, -0.02), (0.02, -0.02), (-0.02, 0.02), (0.02, 0.02))

# --- colours (OpenCV HSV, H in 0-179) ------------------------------------------------
# Cube and bin hues are separated by at least ~20 units. The arm is orange
# (H ~ 10), so red is kept tight around 0.
CUBE_HSV = {
    "red": [((0, 130, 60), (5, 255, 255)), ((174, 130, 60), (179, 255, 255))],
    "blue": [((100, 130, 50), (130, 255, 255))],
    "yellow": [((22, 130, 60), (35, 255, 255))],
}
BIN_HSV = {
    "green": [((45, 130, 40), (75, 255, 255))],
    "cyan": [((82, 130, 40), (96, 255, 255))],
    "magenta": [((140, 130, 40), (168, 255, 255))],
}

# --- arm targets ---------------------------------------------------------------------
HOVER_Z = 0.14              # fingertip height while travelling
RELEASE_MARGIN = 0.02       # gap above a bin top when the cube is let go


def release_z(bin_: Bin) -> float:
    """Fingertip height that puts the cube ``RELEASE_MARGIN`` above the bin."""
    return bin_.height + CUBE_CENTRE_Z + RELEASE_MARGIN


def sector_of(x: float, y: float, margin: float = 0.015) -> str:
    """Which sector a table point lies in: a bin name, ``pick`` or ``none``."""
    for b in BINS:
        if b.sector.contains(x, y, margin):
            return b.name
    if PICK_SECTOR.contains(x, y, margin):
        return "pick"
    return "none"


def pixel_to_table(u, v, fx, fy, cx, cy, cam_x, cam_y, height):
    """Pixel -> table xy for a camera looking straight down.

    ``height`` is the camera height above the plane the pixel is projected
    onto. Use the cube's mid-height plane: the silhouette centroid of a cube
    seen in perspective matches its centre there to under 1 mm, against up
    to ~6 mm if the top face plane is used.
    """
    return (
        cam_x - (v - cy) / fy * height,
        cam_y - (u - cx) / fx * height,
    )


def table_to_pixel(x, y, z, fx, fy, cx, cy, cam_x, cam_y, cam_z):
    depth = cam_z - z
    return (cx - (y - cam_y) / depth * fx, cy - (x - cam_x) / depth * fy)
