"""Web dashboard pieces that need no ROS: accounts, control lease, URDF scene.
``python test_web_dashboard.py`` or ``pytest``.
"""

import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from unoq_braccio_driver.web_auth import Accounts  # noqa: E402


def _accounts(root):
    return Accounts(os.path.join(root, "users.yaml"))


def test_first_person_pairs_as_owner_and_is_saved():
    with tempfile.TemporaryDirectory() as root:
        accounts = _accounts(root)
        code = accounts.first_run_code()
        assert code and len(code) == 6
        assert accounts.join("000000" if code != "000000" else "111111", "ana", "password1")[0] is False
        assert accounts.join(code, "ana", "short")[0] is False
        ok, role = accounts.join(code, "Ana", "password1")
        assert ok and role == "owner"
        assert accounts.join(code, "bob", "password1")[0] is False      # a code works once
        assert accounts.first_run_code() is None
        again = _accounts(root)                                          # reloads from disk
        assert again.login("ana", "password1") and not again.login("ana", "wrong-pass")
        assert oct(os.stat(again.path).st_mode & 0o777) == "0o600"
        assert "password1" not in open(again.path).read()


def test_wrong_passwords_are_throttled():
    with tempfile.TemporaryDirectory() as root:
        accounts = _accounts(root)
        accounts.join(accounts.first_run_code(), "ana", "password1")
        for _ in range(5):
            assert accounts.login("ana", "nope", ip="10.0.0.9") is None
        assert accounts.login("ana", "password1", ip="10.0.0.9") is None   # locked for a minute
        assert accounts.login("ana", "password1", ip="10.0.0.8")           # other devices fine


def test_one_driver_at_a_time_and_viewers_cannot_drive():
    with tempfile.TemporaryDirectory() as root:
        accounts = _accounts(root)
        accounts.join(accounts.first_run_code(), "ana", "password1")
        accounts.join(accounts.new_code("operator"), "bob", "password2")
        accounts.join(accounts.new_code("viewer"), "cat", "password3")
        ana, bob, cat = (accounts.login(n, p) for n, p in
                         (("ana", "password1"), ("bob", "password2"), ("cat", "password3")))
        assert accounts.take(cat)[0] is False
        assert accounts.take(ana)[0] and accounts.driving(ana)
        ok, holder = accounts.take(bob)
        assert not ok and holder["user"] == "ana"
        assert accounts.take(bob, force=True)[0] and not accounts.driving(ana)
        accounts.lease["seen"] = time.time() - 999                      # idle lease lapses
        assert accounts.holder() is None
        accounts.remove("bob")
        assert accounts.session(bob) is None


def test_urdf_scene_reads_links_meshes_and_materials():
    from unoq_braccio_driver.web_scene import urdf_scene

    with tempfile.TemporaryDirectory() as root:
        mesh = os.path.join(root, "part.stl")
        open(mesh, "wb").write(b"solid x\nendsolid x\n")
        xml = f"""<robot name="r">
          <material name="orange"><color rgba="1 0.5 0 1"/></material>
          <link name="world"/>
          <link name="a"><visual><origin xyz="0 0 0.1" rpy="0 0 1.57"/>
            <geometry><mesh filename="file://{mesh}" scale="0.001 0.001 0.001"/></geometry>
            <material name="orange"/></visual></link>
          <link name="b"><visual><geometry><cylinder radius="0.02" length="0.1"/></geometry></visual>
            <visual><geometry><mesh filename="file:///missing.stl"/></geometry></visual></link>
          <joint name="j1" type="fixed"><parent link="world"/><child link="a"/></joint>
          <joint name="j2" type="revolute"><parent link="a"/><child link="b"/></joint>
        </robot>"""
        scene, meshes = urdf_scene(xml)
        assert scene["root"] == "world" and meshes == [mesh]
        a = scene["links"]["a"][0]
        assert a["origin"] == [0, 0, 0.1, 0, 0, 1.57] and a["color"] == [1, 0.5, 0, 1]
        assert a["geometry"] == {"type": "mesh", "mesh": 0, "scale": [0.001, 0.001, 0.001]}
        assert [v["geometry"]["type"] for v in scene["links"]["b"]] == ["cylinder"]   # missing mesh skipped


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
