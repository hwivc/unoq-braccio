"""Cube colours for the web dashboard: show, measure, add and remove them. No ROS.

The colours live in ``cube_hsv`` of ~/.ros/braccio_setup.yaml, the same entry
``real_setup --ros-args -p steps:=colors`` writes, measured the same way
(real_setup.find_cube / measure_color on one cube under the camera).
"""

from __future__ import annotations

import colorsys
import os
import re

NAME = re.compile(r"^[a-z][a-z0-9]{0,15}$")


def ranges_hue(ranges):
    """(centre hue, half width) of a colour's HSV ranges (OpenCV hue 0-179, may wrap)."""
    spans = [(float(low[0]), float(high[0])) for low, high in ranges]
    if len(spans) == 2 and min(s[0] for s in spans) <= 0 and max(s[1] for s in spans) >= 179:
        top = max(spans, key=lambda s: s[1])      # e.g. (170, 179)
        bottom = min(spans, key=lambda s: s[0])   # e.g. (0, 6)
        width = (179 - top[0]) + bottom[1] + 1
        return (top[0] + width / 2.0) % 180.0, width / 2.0
    lo, hi = min(s[0] for s in spans), max(s[1] for s in spans)
    return (lo + hi) / 2.0, (hi - lo) / 2.0


def swatch(hue, saturation=0.82, value=0.92):
    """CSS colour for an OpenCV hue."""
    r, g, b = colorsys.hsv_to_rgb((float(hue) % 180.0) / 180.0, saturation, value)
    return "#{:02x}{:02x}{:02x}".format(round(r * 255), round(g * 255), round(b * 255))


def color_list(cube_hsv):
    """[{name, swatch, hue}] for the page."""
    out = []
    for name, ranges in cube_hsv.items():
        hue, _ = ranges_hue(ranges)
        out.append({"name": name, "swatch": swatch(hue), "hue": round(hue)})
    return out


def blob_at(hsv, x, y, hue_tolerance=12, min_frac=0.0005, max_frac=0.06):
    """The coloured blob a person tapped at pixel (x, y): (box, None) or (None, why).

    Grows from the tapped pixel through saturated pixels of about the same
    hue, so a blue cube on a wooden table next to the orange arm comes out
    as just the cube.
    """
    import cv2
    import numpy as np

    from unoq_braccio_driver.real_setup import MIN_SATURATION, MIN_VALUE

    rows, cols = hsv.shape[:2]
    if not (0 <= x < cols and 0 <= y < rows):
        return None, "That is outside the picture."
    patch = hsv[max(0, y - 3):y + 4, max(0, x - 3):x + 4].reshape(-1, 3).astype(float)
    patch = patch[(patch[:, 1] >= MIN_SATURATION) & (patch[:, 2] >= MIN_VALUE)]
    if len(patch) < 5:
        return None, "That spot has no strong colour (white, grey, black or shadow). Tap the middle of the cube."
    angles = patch[:, 0] * (np.pi / 90.0)
    hue = (np.degrees(np.arctan2(np.sin(angles).mean(), np.cos(angles).mean())) / 2.0) % 180.0
    diff = np.abs(hsv[:, :, 0].astype(float) - hue) % 180.0
    mask = ((np.minimum(diff, 180.0 - diff) <= hue_tolerance)
            & (hsv[:, :, 1] >= MIN_SATURATION) & (hsv[:, :, 2] >= MIN_VALUE)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    label = labels[y, x]
    if label == 0:  # tapped a speck the clean-up removed: take the nearest blob close by
        near = labels[max(0, y - 12):y + 13, max(0, x - 12):x + 13]
        found = near[near > 0]
        if not len(found):
            return None, "No solid colour there. Tap the middle of the cube."
        label = int(np.bincount(found).argmax())
    bx, by, bw, bh, area = stats[label]
    if area > max_frac * rows * cols:
        return None, "That patch is too big for a cube (the arm, the table or something behind?). Tap the cube itself."
    if area < min_frac * rows * cols:
        return None, "That patch is too small. Move the cube closer to the middle, on the table."
    return (int(bx), int(by), int(bx + bw), int(by + bh), float(area)), None


def preview(rgb, cube_hsv, point=None):
    """What adding a colour now would save.

    ``point`` (x, y in picture pixels) is where a person tapped the cube; the
    blob there is measured. Without it, the most cube-like coloured blob in
    view is taken (the biggest coloured thing is often the arm or a cable).

    Returns {seen, name, hue, swatch, box, clash, ranges} (``seen`` False
    with ``why`` when there is nothing to measure). ``clash`` names an
    existing colour whose hue range overlaps, other than the one with the
    suggested name (saving a colour again replaces it).
    """
    from unoq_braccio_driver.color_vision import to_hsv
    from unoq_braccio_driver.real_setup import hue_name, measure_color

    if rgb is None:
        return {"seen": False, "why": "No camera picture yet."}
    hsv = to_hsv(rgb)
    if point is not None:
        box, why = blob_at(hsv, int(point[0]), int(point[1]))
    else:
        box, why = most_cube_like(hsv), "Tap the cube in the camera picture."
    measured = measure_color(hsv, box) if box else None
    if not measured:
        return {"seen": False, "why": why or "Tap the middle of the cube."}
    hue, half, ranges = measured
    name = hue_name(hue)
    return {"seen": True, "name": name, "hue": round(hue), "swatch": swatch(hue),
            "box": [int(v) for v in box[:4]], "width": int(rgb.shape[1]), "height": int(rgb.shape[0]),
            "clash": clash(hue, half, cube_hsv, name), "ranges": ranges, "half": half,
            "tapped": point is not None}


def most_cube_like(hsv, min_frac=0.001, max_frac=0.06, min_fill=0.75):
    """The biggest solid, square-ish, cube-sized coloured blob not cut off by the edge, or None."""
    import cv2
    import numpy as np

    from unoq_braccio_driver.real_setup import MIN_SATURATION, MIN_VALUE

    rows, cols = hsv.shape[:2]
    mask = ((hsv[:, :, 1] >= MIN_SATURATION) & (hsv[:, :, 2] >= MIN_VALUE)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        if not min_frac * rows * cols <= area <= max_frac * rows * cols:
            continue
        if x <= 1 or y <= 1 or x + w >= cols - 1 or y + h >= rows - 1 or max(w, h) > 1.6 * min(w, h):
            continue  # cut off, or long and thin (a cable, an edge)
        (_, _), (rw, rh), _ = cv2.minAreaRect(contour)
        if rw * rh <= 0 or area / (rw * rh) < min_fill:
            continue
        if best is None or area > best[4]:
            best = (x, y, x + w, y + h, area)
    return best


def clash(hue, half, cube_hsv, replacing=""):
    """The existing colour too close to ``hue`` to tell apart, or None."""
    from unoq_braccio_driver.real_setup import hue_distance

    for other, ranges in cube_hsv.items():
        if other == replacing:
            continue
        other_hue, other_half = ranges_hue(ranges)
        if hue_distance(hue, other_hue) < half + other_half + 2:
            return other
    return None


def save_cube_hsv(path, cube_hsv):
    """Write ``cube_hsv`` into the set-up file, keeping everything else in it."""
    import yaml

    path = os.path.expanduser(path)
    setup, header = {}, ""
    if os.path.exists(path):
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
        for line in text.splitlines():   # the comment block at the top
            if not line.startswith("#"):
                break
            header += line + "\n"
        setup = yaml.safe_load(text) or {}
    setup["cube_hsv"] = {name: [[list(map(int, low)), list(map(int, high))] for low, high in ranges]
                         for name, ranges in cube_hsv.items()}
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(header or "# Written by real_setup. Re-run it rather than editing by hand:\n"
                     "#   ros2 run unoq_braccio_driver real_setup\n")
        yaml.safe_dump(setup, handle, sort_keys=False)
    os.replace(tmp, path)
