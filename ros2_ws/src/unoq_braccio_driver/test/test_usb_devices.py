"""Which USB device is the arm and which the camera, and the web page's manual
choices. Needs PyYAML only, no ROS or hardware: ``python test_usb_devices.py`` or ``pytest``.
"""

import os
import sys
import tempfile
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from unoq_braccio_driver import usb_devices as ud  # noqa: E402

UNO = SimpleNamespace(device="/dev/ttyACM1", vid=0x2341, pid=0x0043, serial_number="A1",
                      product="Arduino Uno", description="ttyACM1")
GROVE = SimpleNamespace(device="/dev/ttyACM0", vid=0x1A86, pid=0x55D3, serial_number="5AF7",
                        product="USB Single Serial", description="ttyACM0")
CLONE = SimpleNamespace(device="/dev/ttyUSB0", vid=0x1A86, pid=0x7523, serial_number=None,
                        product="USB2.0-Serial", description="ttyUSB0")
OTHER = SimpleNamespace(device="/dev/ttyUSB1", vid=0x0403, pid=0x6001, serial_number=None,
                        product="FT232R", description="ttyUSB1")
BUILTIN = SimpleNamespace(device="/dev/ttyS0", vid=None, pid=None, serial_number=None,
                          product=None, description="n/a")


def test_labels_each_board_by_usb_id():
    ports = ud.serial_ports([UNO, GROVE, OTHER, BUILTIN])
    kinds = {p["port"]: (p["kind"], p["name"]) for p in ports}
    assert kinds == {"/dev/ttyACM0": ("camera", "Grove Vision AI V2"),
                     "/dev/ttyACM1": ("arm", "Arduino Uno R3"),
                     "/dev/ttyUSB1": ("other", "FT232R")}          # built-in UART left out


def test_auto_finds_each_device_whichever_port_it_is_on():
    ports = ud.serial_ports([GROVE, UNO])
    assert ud.find_arm(ports) == ("/dev/ttyACM1", None)
    assert ud.find_camera(ports) == ("/dev/ttyACM0", None)
    swapped = ud.serial_ports([
        SimpleNamespace(**{**vars(GROVE), "device": "/dev/ttyACM1"}),
        SimpleNamespace(**{**vars(UNO), "device": "/dev/ttyACM0"})])
    assert ud.find_arm(swapped) == ("/dev/ttyACM0", None)
    assert ud.find_camera(swapped) == ("/dev/ttyACM1", None)


def test_never_guesses_when_it_cannot_tell():
    port, why = ud.find_arm(ud.serial_ports([]))
    assert port is None and "No arm found" in why
    port, why = ud.find_arm(ud.serial_ports([UNO, CLONE]))       # two possible arms
    assert port is None and "Several" in why and "Devices" in why
    assert ud.find_camera(ud.serial_ports([UNO, CLONE]))[0] is None   # a CH340 clone is never the camera


def test_manual_choices_never_open_the_arm_as_a_camera():
    ports = ud.serial_ports([UNO, GROVE, OTHER])
    assert ud.camera_port_problem("/dev/ttyACM1", ports).endswith("not a camera")
    assert ud.camera_port_problem("/dev/ttyUSB1", ports) is None  # unknown board: allowed by hand
    assert "does not exist" in ud.camera_port_problem("/dev/ttyACM9", ports)
    assert ud.check_camera_choice("grove:/dev/ttyACM1", ports)[0] is False
    assert ud.check_camera_choice("grove:/dev/ttyACM0", ports)[0] is True
    assert ud.check_camera_choice("auto", ports)[0] is True
    assert ud.check_camera_choice("http://192.168.1.5:8080/video", ports)[0] is True
    assert ud.check_camera_choice("/dev/video0", ports, [{"port": "/dev/video0", "name": "cam"}])[0] is True
    assert ud.check_camera_choice("/dev/video4", ports, [])[0] is False
    assert ud.check_camera_choice("file:///etc/passwd", ports)[0] is False
    assert ud.check_arm_choice("/dev/ttyACM0", ports)[0] is False  # that is the camera
    assert ud.check_arm_choice("/dev/ttyUSB1", ports)[0] is True
    assert ud.check_arm_choice("/dev/ttyACM9", ports)[0] is False


def test_choices_file_round_trip_and_reconnect_counter():
    with tempfile.TemporaryDirectory() as root:
        path = os.path.join(root, "devices.yaml")
        watcher = ud.Choices(path)
        assert watcher.get() == {"arm": "auto", "camera": "auto", "reconnect": {"arm": 0, "camera": 0}}
        ud.save_choice("camera", "grove:/dev/ttyACM0", path=path)
        ud.save_choice("arm", reconnect=True, path=path)
        ud.save_choice("arm", reconnect=True, path=path)
        got = watcher.get()
        assert got["camera"] == "grove:/dev/ttyACM0" and got["arm"] == "auto"
        assert got["reconnect"] == {"arm": 2, "camera": 0}
        ud.save_choice("camera", "auto", path=path)
        assert ud.load_choices(path)["camera"] == "auto"


def _fake_sysfs(root):
    """/dev/video* and sysfs for: a USB webcam (2 nodes), a built-in webcam,
    and a Raspberry Pi 5 image processor that is not a camera."""
    def write(path, text):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as handle:
            handle.write(text + "\n")

    cameras = {  # node: (sysfs device dir, name, index)
        "video0": ("devices/pci/usb1/1-6/1-6:1.0", "Integrated_Webcam_HD: Integrate", "0"),
        "video2": ("devices/pci/usb1/1-2/1-2:1.0", "HD Pro Webcam C920", "0"),
        "video3": ("devices/pci/usb1/1-2/1-2:1.0", "HD Pro Webcam C920", "1"),
        "video20": ("devices/platform/1000880000.pisp_be", "pispbe", "0"),
    }
    write(os.path.join(root, "devices/pci/usb1/1-6/idVendor"), "0bda")
    write(os.path.join(root, "devices/pci/usb1/1-6/idProduct"), "565a")
    write(os.path.join(root, "devices/pci/usb1/1-6/removable"), "fixed")
    write(os.path.join(root, "devices/pci/usb1/1-2/idVendor"), "046d")
    write(os.path.join(root, "devices/pci/usb1/1-2/idProduct"), "082d")
    write(os.path.join(root, "devices/pci/usb1/1-2/removable"), "removable")
    for node, (device, name, index) in cameras.items():
        os.makedirs(os.path.join(root, device), exist_ok=True)
        write(os.path.join(root, "dev", node), "")
        write(os.path.join(root, "sys", node, "name"), name)
        write(os.path.join(root, "sys", node, "index"), index)
        os.symlink(os.path.join(root, device), os.path.join(root, "sys", node, "device"))


def test_lists_usb_cameras_but_not_pi_video_engines():
    with tempfile.TemporaryDirectory() as root:
        _fake_sysfs(root)
        found = ud.video_devices(os.path.join(root, "dev", "video*"), os.path.join(root, "sys"))
        assert [(os.path.basename(v["port"]), v["usb_id"], v["builtin"]) for v in found] == [
            ("video0", "0bda:565a", True),       # the laptop's own camera
            ("video2", "046d:082d", False),      # a plugged-in USB webcam (video3 is its metadata node)
        ]
        assert found[1]["name"] == "HD Pro Webcam C920"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
