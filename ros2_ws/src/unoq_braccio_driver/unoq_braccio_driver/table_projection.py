"""Pixel <-> table mapping for the overhead camera (numpy only, no ROS).

Two ways to know where a pixel is on the table:

* ``PinholeDownProjection`` - simulation. The camera looks straight down from
  a known position with a known focal length, so the mapping is exact.

* ``HomographyProjection`` - real camera. Nobody mounts a phone exactly level
  at an exactly known spot, and its focal length is unknown, so instead of
  measuring the camera we measure the *table*: put a cube on 4+ marked points
  whose (x, y) from the arm base is known, note where each appears in the
  image, and fit a homography (a 3x3 matrix that maps any pixel to a table
  point on that plane, for any camera angle). ``table_calibration`` does this
  and saves the result; see docs/camera.md.

  The homography is fitted to cube *centres*, so it maps onto the plane at
  cube mid-height - exactly what the detector needs.

Math: a homography H maps pixel (u, v) to table (x, y) via
    [x', y', w]^T = H [u, v, 1]^T,   x = x'/w,  y = y'/w
It has 8 unknowns (H is scale-free), so 4 point pairs fix it; more points are
solved in a least-squares sense (DLT with SVD, as in OpenCV's findHomography).
"""

import math

import numpy as np


def fit_homography(pixels, table):
    """3x3 H mapping pixel (u, v) -> table (x, y), from >= 4 point pairs.

    Points are normalised first (centred, mean distance sqrt(2)) so the
    solution is well conditioned when pixels are in hundreds and metres in
    tenths. Raises ValueError for fewer than 4 points or a degenerate layout
    (e.g. three points on a line).
    """
    src = np.asarray(pixels, dtype=float)
    dst = np.asarray(table, dtype=float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 2 or len(src) < 4:
        raise ValueError("need at least 4 matching (u, v) / (x, y) pairs")

    def normaliser(points):
        centre = points.mean(axis=0)
        spread = np.sqrt(((points - centre) ** 2).sum(axis=1)).mean()
        if spread < 1e-12:
            raise ValueError("points are all in one place")
        s = math.sqrt(2.0) / spread
        return np.array([[s, 0, -s * centre[0]], [0, s, -s * centre[1]], [0, 0, 1]])

    t_src, t_dst = normaliser(src), normaliser(dst)
    src_n = (t_src @ np.c_[src, np.ones(len(src))].T).T
    dst_n = (t_dst @ np.c_[dst, np.ones(len(dst))].T).T

    rows = []
    for (u, v, _), (x, y, _) in zip(src_n, dst_n):
        rows.append([-u, -v, -1, 0, 0, 0, x * u, x * v, x])
        rows.append([0, 0, 0, -u, -v, -1, y * u, y * v, y])
    _, singular, vt = np.linalg.svd(np.asarray(rows))
    if len(src) == 4 and singular[-2] < 1e-9 * singular[0]:
        raise ValueError("degenerate point layout (are three points on a line?)")
    h_n = vt[-1].reshape(3, 3)
    h = np.linalg.inv(t_dst) @ h_n @ t_src
    if abs(h[2, 2]) < 1e-12:
        raise ValueError("degenerate point layout")
    return h / h[2, 2]


def apply_homography(h, u, v):
    """Table (x, y) for pixel (u, v)."""
    x, y, w = np.asarray(h) @ np.array([u, v, 1.0])
    return float(x / w), float(y / w)


def reprojection_errors(h, pixels, table):
    """Distance in metres between each measured table point and H(pixel)."""
    return [
        math.dist(apply_homography(h, u, v), (x, y))
        for (u, v), (x, y) in zip(pixels, table)
    ]


class PinholeDownProjection:
    """Simulation camera: straight down from (cam_x, cam_y, cam_z)."""

    def __init__(self, fx, fy, cx, cy, cam_x, cam_y, cam_z):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.cam_x, self.cam_y, self.cam_z = cam_x, cam_y, cam_z

    def to_table(self, u, v, plane_z):
        """Table (x, y) of pixel (u, v) on the horizontal plane at ``plane_z``.

        Image right is world -y and image down is world -x (camera pitched
        +90 degrees, see braccio_workspace.CAMERA_XYZ).
        """
        height = self.cam_z - plane_z
        return (
            self.cam_x - (v - self.cy) / self.fy * height,
            self.cam_y - (u - self.cx) / self.fx * height,
        )

    def pixels_per_metre(self, plane_z):
        return self.fx / (self.cam_z - plane_z)


class HomographyProjection:
    """Real camera: calibrated homography onto the cube mid-height plane.

    ``image_size`` is the (width, height) the calibration was made at; frames
    of another size are scaled to it, so changing the camera resolution later
    does not silently break the mapping (as long as the aspect ratio and the
    camera's position stay the same).
    """

    def __init__(self, h, image_size):
        self.h = np.asarray(h, dtype=float)
        self.width, self.height = image_size

    def _scaled(self, u, v, frame_size):
        if frame_size is None:
            return u, v
        fw, fh = frame_size
        return u * self.width / fw, v * self.height / fh

    def to_table(self, u, v, plane_z=None, frame_size=None):
        """Table (x, y) of pixel (u, v). ``plane_z`` is ignored: the
        homography is only valid on the plane it was calibrated on (cube
        mid-height)."""
        return apply_homography(self.h, *self._scaled(u, v, frame_size))

    def pixels_per_metre(self, plane_z=None, frame_size=None):
        """Image scale near the middle of the frame, pixels per metre.

        Used only to size-filter colour blobs, so an average is fine.
        """
        u, v = self.width / 2.0, self.height / 2.0
        step = 10.0
        x0, y0 = apply_homography(self.h, u, v)
        x1, y1 = apply_homography(self.h, u + step, v)
        x2, y2 = apply_homography(self.h, u, v + step)
        metres = (math.dist((x0, y0), (x1, y1)) + math.dist((x0, y0), (x2, y2))) / 2.0
        scale = step / metres
        if frame_size is not None:
            scale *= frame_size[0] / self.width
        return scale


def load_calibration(path):
    """Read a calibration file written by ``table_calibration``.

    Returns a ``HomographyProjection``. Raises OSError / ValueError if the
    file is missing or malformed.
    """
    import yaml

    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    try:
        h = np.asarray(data["homography"], dtype=float).reshape(3, 3)
        size = (int(data["image_width"]), int(data["image_height"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{path} is not a table calibration file: {exc}") from exc
    return HomographyProjection(h, size)


def save_calibration(path, h, image_size, pixels, table, errors_m):
    """Write a calibration file (YAML) with the points kept for reference."""
    import os

    import yaml

    data = {
        "homography": [[float(v) for v in row] for row in np.asarray(h)],
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "rms_error_mm": round(1000.0 * math.sqrt(sum(e * e for e in errors_m) / len(errors_m)), 2),
        "points": [
            {"pixel": [round(float(u), 1), round(float(v), 1)],
             "table": [float(x), float(y)],
             "error_mm": round(1000.0 * e, 2)}
            for (u, v), (x, y), e in zip(pixels, table, errors_m)
        ],
    }
    folder = os.path.dirname(os.path.abspath(path))
    os.makedirs(folder, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(
            "# Overhead camera -> table calibration, written by table_calibration.\n"
            "# Redo it whenever the camera moves or its zoom changes.\n"
        )
        yaml.safe_dump(data, handle, sort_keys=False)
