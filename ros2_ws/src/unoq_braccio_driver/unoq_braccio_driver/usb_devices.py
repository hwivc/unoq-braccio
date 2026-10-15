"""Which USB device is the arm and which is the camera, and manual choices. No ROS.

Auto-detection goes by USB vendor / product id, never by port number
(ttyACM0 and ttyACM1 swap with plug-in order) and never by opening a port to
ask what it is (opening the UNO's port restarts the arm). When it cannot
decide (nothing plugged in, two possible boards, a clone it does not know),
the web dashboard's Devices section can pin either device to a port. That
choice is kept in ~/.ros/braccio_devices.yaml, which serial_bridge and
camera_node re-read while they run, so no restart is needed:

    arm: auto                    # or a serial port, e.g. /dev/ttyACM1
    camera: auto                 # or grove:/dev/ttyACM0, /dev/video0, 0, http://...
    reconnect: {arm: 3, camera: 1}   # bumped by the Reconnect buttons
"""

from __future__ import annotations

import glob
import os
import re

DEVICES_FILE = "~/.ros/braccio_devices.yaml"
STATUS_TOPIC = "/braccio/devices"

# USB ids of the arm's board. Keep in step with scripts/99-braccio-uno.rules.
ARM_IDS = {
    (0x2341, 0x0043): "Arduino Uno R3",
    (0x2341, 0x0001): "Arduino Uno",
    (0x2341, 0x0069): "Arduino Uno R4 Minima",
    (0x2341, 0x1002): "Arduino Uno R4 WiFi",
    (0x2A03, 0x0043): "Arduino Uno",
    (0x1A86, 0x7523): "Arduino Uno clone (CH340)",
}
# The Grove Vision AI V2 sits behind a WCH CH343; any Seeed board counts too.
CAMERA_IDS = {(0x1A86, 0x55D3): "Grove Vision AI V2"}
CAMERA_VENDORS = {0x2886: "Seeed board"}
ALIASES = ("/dev/braccio", "/dev/grove_vision")
URL = re.compile(r"^(https?|rtsp)://[^\s]{1,300}$")


def classify(vid, pid):
    """("arm" | "camera" | "other", name or None) for a USB id."""
    if (vid, pid) in ARM_IDS:
        return "arm", ARM_IDS[(vid, pid)]
    if (vid, pid) in CAMERA_IDS:
        return "camera", CAMERA_IDS[(vid, pid)]
    if vid in CAMERA_VENDORS:
        return "camera", CAMERA_VENDORS[vid]
    return "other", None


def serial_ports(comports=None):
    """USB serial ports as dicts: port, name, kind, usb id, serial number, aliases."""
    if comports is None:
        from serial.tools import list_ports

        comports = list_ports.comports()
    aliases = {os.path.realpath(a): a for a in ALIASES if os.path.exists(a)}
    out = []
    for p in comports:
        if p.vid is None:
            continue  # not USB (built-in UARTs)
        kind, name = classify(p.vid, p.pid)
        out.append({
            "port": p.device, "kind": kind,
            "name": name or (p.product or p.description or "USB serial device"),
            "usb_id": f"{p.vid:04x}:{p.pid or 0:04x}", "serial": p.serial_number or "",
            "alias": aliases.get(os.path.realpath(p.device), ""),
        })
    return sorted(out, key=lambda d: d["port"])


def _read(path, default=""):
    try:
        with open(path, encoding="utf-8") as handle:
            return handle.read().strip()
    except OSError:
        return default


def video_devices(dev_glob="/dev/video*", sys_root="/sys/class/video4linux"):
    """Cameras on USB that OpenCV can open: [{port, name, usb_id, builtin}].

    Only USB cameras are listed: plugged-in webcams and a laptop's built-in
    one (``builtin``: USB marks it "fixed"). The other /dev/video nodes, such
    as a Raspberry Pi 5's video decoder and image processor, are not cameras.
    Each camera's extra nodes (metadata, index > 0) are left out too.
    """
    out = []
    for path in sorted(glob.glob(dev_glob), key=lambda p: int(re.sub(r"\D", "", p) or 0)):
        sysfs = os.path.join(sys_root, os.path.basename(path))
        device = os.path.realpath(os.path.join(sysfs, "device"))
        if "/usb" not in device or _read(os.path.join(sysfs, "index"), "0") != "0":
            continue
        usb = device
        while usb != os.path.dirname(usb) and not os.path.exists(os.path.join(usb, "idVendor")):
            usb = os.path.dirname(usb)   # interface -> the USB device that owns it
        vid, pid = _read(os.path.join(usb, "idVendor")), _read(os.path.join(usb, "idProduct"))
        out.append({
            "port": path, "name": _read(os.path.join(sysfs, "name"), "camera"),
            "usb_id": f"{vid}:{pid}" if vid else "",
            "builtin": _read(os.path.join(usb, "removable")) == "fixed",
        })
    return out


def _find(kind, ports, what, plug):
    found = [p for p in ports if p["kind"] == kind]
    if len(found) == 1:
        return found[0]["port"], None
    if not found:
        return None, f"No {what} found: {plug}"
    return None, (f"Several possible {what}s ({', '.join(p['port'] for p in found)}): "
                  "choose one in the web page's Settings > Devices")


def find_arm(ports=None):
    """(port, None) for the one arm board plugged in, else (None, why)."""
    return _find("arm", serial_ports() if ports is None else ports, "arm",
                 "plug the UNO in (check the USB cable or hub)")


def find_camera(ports=None):
    """(port, None) for the one Grove Vision AI V2 plugged in, else (None, why)."""
    return _find("camera", serial_ports() if ports is None else ports, "Grove camera",
                 "plug the Grove Vision AI V2 in (check the USB cable or hub)")


def camera_port_problem(port, ports=None):
    """Why ``port`` must not be opened as the camera, or None if it may be.

    Manual choices may name a board auto-detect does not know, but never the
    arm's: opening that restarts the arm and garbles its commands.
    """
    ports = serial_ports() if ports is None else ports
    real = os.path.realpath(port)
    found = next((p for p in ports if os.path.realpath(p["port"]) == real), None)
    if found is None:
        return f"{port} does not exist (unplugged?)"
    if found["kind"] == "arm" or real == os.path.realpath("/dev/braccio"):
        return f"{port} is the arm's board ({found['name']}), not a camera"
    return None


def check_arm_choice(port, ports=None):
    """(ok, message) for pinning the arm to ``port`` ("auto" always works)."""
    if port == "auto":
        return True, ""
    ports = serial_ports() if ports is None else ports
    found = next((p for p in ports if p["port"] == port), None)
    if found is None:
        return False, f"{port} is not plugged in."
    if found["kind"] == "camera":
        return False, f"{port} is the camera ({found['name']}), not the arm."
    return True, ""


def check_camera_choice(source, ports=None, videos=None):
    """(ok, message) for the camera source a person typed or picked."""
    source = str(source or "").strip()
    if source == "auto":
        return True, ""
    if source.startswith("grove:"):
        problem = camera_port_problem(source[len("grove:"):], ports)
        return (False, problem) if problem else (True, "")
    if source.isdigit() and int(source) < 64:
        return True, ""
    if source.startswith("/dev/video"):
        videos = video_devices() if videos is None else videos
        if source in [v["port"] for v in videos]:
            return True, ""
        return False, f"{source} is not plugged in."
    if URL.match(source):
        return True, ""
    return False, "Use grove:<port>, a /dev/videoN camera, a camera number or an http(s):// / rtsp:// stream."


# -- the choices file ----------------------------------------------------------

def load_choices(path=DEVICES_FILE):
    import yaml

    choices = {"arm": "auto", "camera": "auto", "reconnect": {"arm": 0, "camera": 0}}
    try:
        with open(os.path.expanduser(path), encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except (OSError, ValueError):
        return choices
    for key in ("arm", "camera"):
        if data.get(key):
            choices[key] = str(data[key])
    reconnect = data.get("reconnect") or {}
    for key in ("arm", "camera"):
        try:
            choices["reconnect"][key] = int(reconnect.get(key, 0))
        except (TypeError, ValueError):
            pass
    return choices


def save_choice(device, source=None, reconnect=False, path=DEVICES_FILE):
    """Pin ``device`` ("arm" / "camera") to ``source`` ("auto" to un-pin) and/or ask it to reconnect."""
    import yaml

    choices = load_choices(path)
    if source is not None:
        choices[device] = source
    if reconnect:
        choices["reconnect"][device] += 1
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write("# Set from the web dashboard (Settings > Devices). auto = find by USB id.\n")
        yaml.safe_dump(choices, handle, sort_keys=False)
    os.replace(tmp, path)
    return choices


class Choices:
    """The choices file, re-read only when it changes (cheap to call often)."""

    def __init__(self, path=DEVICES_FILE):
        self.path = os.path.expanduser(path)
        self.stamp = None
        self.value = load_choices(self.path)

    def get(self):
        try:
            stamp = os.stat(self.path).st_mtime_ns
        except OSError:
            stamp = None
        if stamp != self.stamp:
            self.stamp = stamp
            self.value = load_choices(self.path)
        return self.value
