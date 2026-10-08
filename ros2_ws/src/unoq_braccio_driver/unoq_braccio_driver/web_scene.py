"""URDF -> a scene the web dashboard's browser can draw. No ROS (ament only for package:// paths)."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET


def _floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


def resolve_mesh(filename):
    """file:// or package:// mesh path -> absolute path on disk (or None)."""
    if filename.startswith("file://"):
        return filename[len("file://"):]
    if filename.startswith("package://"):
        from ament_index_python.packages import get_package_share_directory

        package, _, rest = filename[len("package://"):].partition("/")
        try:
            return os.path.join(get_package_share_directory(package), rest)
        except LookupError:
            return None
    return filename if os.path.isabs(filename) else None


def urdf_scene(xml_text):
    """Links with their visuals, and the meshes they use.

    Returns ``(scene, meshes)``: scene = {"root", "links": {name: [visual]}},
    each visual {"origin": [x, y, z, roll, pitch, yaw], "color": [r, g, b, a],
    "geometry": {"type": "mesh", "mesh": index, "scale": [...]} | box | cylinder |
    sphere}; meshes = list of absolute file paths, served by index only.
    """
    robot = ET.fromstring(xml_text)
    colors = {}
    for material in robot.findall("material"):
        color = material.find("color")
        if color is not None:
            colors[material.get("name")] = _floats(color.get("rgba"), [0.8, 0.8, 0.8, 1])
    children = {j.find("child").get("link") for j in robot.findall("joint") if j.find("child") is not None}
    names = [link.get("name") for link in robot.findall("link")]
    root = next((n for n in names if n not in children), names[0] if names else "world")
    meshes, links = [], {}
    for link in robot.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            geometry = visual.find("geometry")
            if geometry is None or len(geometry) == 0:
                continue
            shape = geometry[0]
            material = visual.find("material")
            color = [0.8, 0.8, 0.8, 1.0]
            if material is not None:
                inline = material.find("color")
                color = (_floats(inline.get("rgba"), color) if inline is not None
                         else colors.get(material.get("name"), color))
            if shape.tag == "mesh":
                path = resolve_mesh(shape.get("filename", ""))
                if not path or not os.path.exists(path):
                    continue
                if path not in meshes:
                    meshes.append(path)
                geom = {"type": "mesh", "mesh": meshes.index(path),
                        "scale": _floats(shape.get("scale"), [1, 1, 1])}
            elif shape.tag == "box":
                geom = {"type": "box", "size": _floats(shape.get("size"), [0.01] * 3)}
            elif shape.tag == "cylinder":
                geom = {"type": "cylinder", "radius": float(shape.get("radius", 0.01)),
                        "length": float(shape.get("length", 0.01))}
            elif shape.tag == "sphere":
                geom = {"type": "sphere", "radius": float(shape.get("radius", 0.01))}
            else:
                continue
            xyz = _floats(origin.get("xyz") if origin is not None else None, [0, 0, 0])
            rpy = _floats(origin.get("rpy") if origin is not None else None, [0, 0, 0])
            visuals.append({"origin": xyz + rpy, "color": color, "geometry": geom})
        if visuals:
            links[link.get("name")] = visuals
    return {"root": root, "links": links}, meshes
