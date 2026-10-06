"""Read config/real_camera.yaml (millimetres) into node parameters (metres)."""

import os

KEYS = ("camera_height_mm", "camera_x_mm", "camera_y_mm", "cube_size_mm")


def read_camera_config(path):
    """camera_x/y/z, cube_size (metres) and camera_fx (pixels, 0 = measure)."""
    import yaml

    with open(os.path.expanduser(path), encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    missing = [k for k in KEYS if k not in config]
    if missing:
        raise RuntimeError(f"{path} is missing {', '.join(missing)}")
    return {
        "camera_x": float(config["camera_x_mm"]) / 1000.0,
        "camera_y": float(config["camera_y_mm"]) / 1000.0,
        "camera_z": float(config["camera_height_mm"]) / 1000.0,
        "cube_size": float(config["cube_size_mm"]) / 1000.0,
        "camera_fx": float(config.get("camera_fx_px", 0.0)),
    }
