"""Camera calibration and real-workspace checks. Needs numpy and PyYAML, no
ROS: run with ``python test_camera_calibration.py`` or ``pytest``.

Covers: the homography fit recovers a tilted camera exactly and stays
accurate with click noise, bad layouts are rejected, frames of another size
are scaled, the simulation mapping is unchanged, and the real workspace YAML
loads, is reachable, and does not leak into the simulated layout.
"""

import importlib
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from unoq_braccio_driver import braccio_workspace as ws  # noqa: E402
from unoq_braccio_driver.braccio_kinematics import solve_ik  # noqa: E402
from unoq_braccio_driver.table_projection import (  # noqa: E402
    HomographyProjection,
    PinholeDownProjection,
    apply_homography,
    fit_homography,
    reprojection_errors,
)

REAL_CONFIG = os.path.join(
    os.path.dirname(__file__), "..", "..", "unoq_braccio_bringup", "config", "real_workspace.yaml"
)

# A tilted, off-centre "phone" camera: table (x, y) -> pixel (u, v).
TRUE_TABLE_TO_PIXEL = np.array([
    [-150.0, -1900.0, 380.0],
    [-1750.0, 120.0, 700.0],
    [0.35, 0.25, 1.0],
])
POINTS = [(0.15, -0.15), (0.30, -0.15), (0.30, 0.10), (0.15, 0.10), (0.22, -0.02)]


def _to_pixel(x, y):
    u, v, w = TRUE_TABLE_TO_PIXEL @ np.array([x, y, 1.0])
    return u / w, v / w


def test_homography_recovers_tilted_camera_exactly():
    pixels = [_to_pixel(x, y) for x, y in POINTS]
    h = fit_homography(pixels, POINTS)
    assert max(reprojection_errors(h, pixels, POINTS)) < 1e-9
    # and points that were not used in the fit
    for x, y in [(0.25, 0.0), (0.18, -0.08), (0.28, 0.05)]:
        assert math.dist(apply_homography(h, *_to_pixel(x, y)), (x, y)) < 1e-9


def test_homography_with_click_noise_stays_within_a_few_mm():
    rng = np.random.default_rng(1)
    pixels = [np.add(_to_pixel(x, y), rng.normal(0, 1.5, 2)) for x, y in POINTS]
    h = fit_homography(pixels, POINTS)
    worst = 0.0
    for x in np.linspace(0.15, 0.30, 6):
        for y in np.linspace(-0.15, 0.10, 6):
            worst = max(worst, math.dist(apply_homography(h, *_to_pixel(x, y)), (x, y)))
    assert worst < 0.004, worst  # 1.5 px click noise -> under 4 mm anywhere in the area


def test_bad_layouts_are_rejected():
    for table in ([(0.1, 0.0), (0.2, 0.0), (0.3, 0.0), (0.2, 0.1)][:3],   # too few
                  [(0.1, 0.0), (0.2, 0.0), (0.3, 0.0), (0.4, 0.0)]):    # all on a line
        try:
            fit_homography([_to_pixel(*p) for p in table], table)
        except ValueError:
            continue
        raise AssertionError(f"accepted {table}")


def test_other_frame_size_is_scaled():
    pixels = [_to_pixel(x, y) for x, y in POINTS]
    projection = HomographyProjection(fit_homography(pixels, POINTS), (640, 480))
    u, v = _to_pixel(0.25, 0.0)
    x, y = projection.to_table(u * 2, v * 2, frame_size=(1280, 960))
    assert math.dist((x, y), (0.25, 0.0)) < 1e-9
    assert projection.pixels_per_metre(frame_size=(1280, 960)) > projection.pixels_per_metre()


def test_simulation_mapping_unchanged():
    fx = ws.CAMERA_FX
    cx, cy = ws.CAMERA_WIDTH / 2.0, ws.CAMERA_HEIGHT / 2.0
    cam_x, cam_y, cam_z = ws.CAMERA_XYZ
    pinhole = PinholeDownProjection(fx, fx, cx, cy, cam_x, cam_y, cam_z)
    for u, v in [(10, 20), (320, 240), (600, 400)]:
        for plane_z in (ws.CUBE_CENTRE_Z, 0.02):
            old = ws.pixel_to_table(u, v, fx, fx, cx, cy, cam_x, cam_y, cam_z - plane_z)
            assert math.dist(pinhole.to_table(u, v, plane_z), old) < 1e-12
    assert abs(pinhole.pixels_per_metre(0.0) - fx / cam_z) < 1e-12


def test_real_workspace_config_loads_and_is_reachable():
    try:
        config = ws.load_config(REAL_CONFIG)
        assert len(ws.CALIBRATION_POINTS) >= 4
        assert set(ws.BIN_BY_CUBE_COLOR) <= set(ws.CUBE_HSV), "a bin for a colour with no HSV range"
        assert ws.CUBE_CENTRE_Z == config["cube_size"] / 2
        pick = ws.PICK_SECTOR
        targets = [(pick.x + a * pick.size_x / 2, pick.y + b * pick.size_y / 2)
                   for a in (-1, 0, 1) for b in (-1, 0, 1)]
        for x, y in targets:
            for z in (ws.CUBE_CENTRE_Z, ws.HOVER_Z):
                assert solve_ik(x, y, z, ws.GRIPPER_CLOSED) is not None, ("pick", x, y, z)
        for b in ws.BINS:
            for z in (ws.release_z(b), ws.HOVER_Z):
                assert solve_ik(*b.centre, z, ws.GRIPPER_CLOSED) is not None, (b.name, z)
            assert not pick.contains(*b.centre, ws.CUBE_SIZE), f"{b.name} overlaps the pick area"
    finally:
        importlib.reload(ws)


def test_simulation_layout_untouched_by_real_config():
    importlib.reload(ws)
    sim_bins = [b.name for b in ws.BINS]
    ws.load_config(REAL_CONFIG)
    importlib.reload(ws)
    assert [b.name for b in ws.BINS] == sim_bins == ["green", "cyan", "magenta"]
    assert ws.CUBE_SIZE == 0.03


def test_bad_config_entries_give_readable_errors():
    for bad in ({"bins": [{"name": "x", "x": 0.1}]},
                {"pick_area": {"x": 0.2}},
                {"calibration_points": [[0.1, 0.1], [0.2, 0.2]]}):
        try:
            ws.apply_config(bad)
        except ValueError:
            continue
        finally:
            importlib.reload(ws)
        raise AssertionError(f"accepted {bad}")


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
