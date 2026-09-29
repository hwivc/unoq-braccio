"""Colour blob helpers shared by the overhead and gripper detectors."""

import numpy as np

_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}


def image_to_rgb(msg):
    """sensor_msgs/Image -> HxWx3 uint8 RGB array, or None if unsupported."""
    channels = _CHANNELS.get(msg.encoding)
    if channels is None:
        return None
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    frame = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding.startswith("bgr"):
        frame = frame[:, :, [2, 1, 0]]
    return frame[:, :, :3]


def to_hsv(rgb):
    import cv2

    return cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)


def best_color(hsv_crop, ranges_by_color):
    """Colour whose HSV range covers the most of ``hsv_crop``.

    Used to classify the colour *inside* a box a cube-finding model already
    returned (the model finds "a cube"; this decides which one). Returns
    ``(name, fraction)``, or ``(None, 0.0)`` for an empty crop.
    """
    import cv2

    total = hsv_crop.shape[0] * hsv_crop.shape[1]
    if total == 0:
        return None, 0.0
    best_name, best_frac = None, 0.0
    for name, ranges in ranges_by_color.items():
        mask = None
        for low, high in ranges:
            part = cv2.inRange(hsv_crop, np.array(low), np.array(high))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        frac = float(np.count_nonzero(mask)) / total
        if frac > best_frac:
            best_name, best_frac = name, frac
    return best_name, best_frac


def find_blobs(hsv, ranges, min_area, max_area):
    """Blobs inside any HSV range: list of (u, v, area) in pixels.

    The centroid comes from the outer contour, so a cube resting on a bin does
    not pull the bin's centre off.
    """
    import cv2

    mask = None
    for low, high in ranges:
        part = cv2.inRange(hsv, np.array(low), np.array(high))
        mask = part if mask is None else cv2.bitwise_or(mask, part)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not min_area <= area <= max_area:
            continue
        m = cv2.moments(contour)
        if m["m00"] <= 0:
            continue
        blobs.append((m["m10"] / m["m00"], m["m01"] / m["m00"], area))
    return blobs
