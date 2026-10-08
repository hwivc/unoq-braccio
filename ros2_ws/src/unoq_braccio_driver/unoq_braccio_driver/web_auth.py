"""Accounts, sessions and the control lease for the web dashboard. No ROS.

People are kept in ``~/.ros/braccio_web.yaml`` with salted PBKDF2 password
hashes. A device joins with a pairing code shown in the dashboard terminal
(the first person) or by an owner in Settings, the way a new device pairs
with a phone: being on the same network is not enough.

Roles: ``owner`` (control, advanced settings, invite people), ``operator``
(control) and ``viewer`` (watch only). Only one person drives at a time:
the control lease. Anyone who can control may always press Stop.
"""

from __future__ import annotations

import hashlib
import hmac
import os
import secrets
import threading
import time

ROLES = ("owner", "operator", "viewer")
CAN_CONTROL = ("owner", "operator")
USERS_FILE = "~/.ros/braccio_web.yaml"
ITERATIONS = 200_000


def hash_password(password, salt=None):
    salt = salt or secrets.token_hex(16)
    digest = hashlib.pbkdf2_hmac("sha256", password.encode(), bytes.fromhex(salt), ITERATIONS)
    return salt, digest.hex()


def check_password(password, salt, expected):
    return hmac.compare_digest(hash_password(password, salt)[1], expected)


class Accounts:
    """People, sessions, pairing codes, login throttling and the control lease."""

    def __init__(self, path=USERS_FILE, session_hours=12.0):
        self.path = os.path.expanduser(path)
        self.session_s = session_hours * 3600.0
        self.lock = threading.Lock()
        self.users = self._load()
        self.sessions = {}      # token -> {"user", "expires", "device"}
        self.codes = {}         # code -> {"role", "expires"}
        self.failures = {}      # ip -> [times]
        self.lease = None       # {"token", "user", "device", "since", "seen"}

    # -- storage -------------------------------------------------------------

    def _load(self):
        import yaml

        if not os.path.exists(self.path):
            return {}
        with open(self.path, encoding="utf-8") as handle:
            return (yaml.safe_load(handle) or {}).get("users", {})

    def _save(self):
        import yaml

        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + ".tmp"
        with open(os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600), "w",
                  encoding="utf-8") as handle:
            handle.write("# Web dashboard accounts (salted PBKDF2). Delete a person's entry to remove them.\n")
            yaml.safe_dump({"users": self.users}, handle, sort_keys=True)
        os.replace(tmp, self.path)

    # -- pairing codes ---------------------------------------------------------

    def new_code(self, role="operator", minutes=10.0):
        if role not in ROLES:
            raise ValueError(role)
        code = f"{secrets.randbelow(1_000_000):06d}"
        with self.lock:
            self.codes[code] = {"role": role, "expires": time.time() + minutes * 60.0}
        return code

    def first_run_code(self):
        """A code for the first person (who becomes owner), or None if someone exists."""
        if self.users:
            return None
        live = [c for c, v in self.codes.items() if v["role"] == "owner" and v["expires"] > time.time()]
        return live[0] if live else self.new_code("owner", minutes=24 * 60.0)

    def join(self, code, name, password):
        """Create an account with a pairing code. Returns (ok, message)."""
        name = (name or "").strip().lower()
        if not name.isalnum() or not 2 <= len(name) <= 24:
            return False, "Use 2-24 letters or digits for the name."
        if len(password or "") < 8:
            return False, "Use at least 8 characters for the password."
        with self.lock:
            entry = self.codes.get((code or "").strip())
            if entry is None or entry["expires"] < time.time():
                return False, "That code is not valid. Ask for a new one."
            if name in self.users:
                return False, "That name is taken."
            del self.codes[code.strip()]
            salt, digest = hash_password(password)
            self.users[name] = {"role": entry["role"], "salt": salt, "hash": digest,
                                "created": time.strftime("%Y-%m-%d %H:%M:%S")}
            self._save()
        return True, entry["role"]

    # -- sessions --------------------------------------------------------------

    def throttled(self, ip, limit=5, window=60.0):
        now = time.time()
        recent = [t for t in self.failures.get(ip, []) if now - t < window]
        self.failures[ip] = recent
        return len(recent) >= limit

    def login(self, name, password, ip="", device=""):
        """A session token, or None. Wrong passwords from one address are throttled."""
        if self.throttled(ip):
            return None
        user = self.users.get((name or "").strip().lower())
        if user is None or not check_password(password or "", user["salt"], user["hash"]):
            self.failures.setdefault(ip, []).append(time.time())
            return None
        token = secrets.token_urlsafe(32)
        with self.lock:
            self.sessions[token] = {"user": name.strip().lower(), "device": device[:60],
                                    "expires": time.time() + self.session_s}
        return token

    def session(self, token):
        """{"user", "role", "device"} for a live token, else None."""
        with self.lock:
            s = self.sessions.get(token or "")
            if s is None or s["expires"] < time.time() or s["user"] not in self.users:
                self.sessions.pop(token or "", None)
                return None
            return {"user": s["user"], "role": self.users[s["user"]]["role"], "device": s["device"]}

    def logout(self, token):
        with self.lock:
            self.sessions.pop(token or "", None)
            if self.lease and self.lease["token"] == token:
                self.lease = None

    def people(self):
        return [{"name": n, "role": u["role"], "created": u.get("created", "")}
                for n, u in sorted(self.users.items())]

    def remove(self, name):
        with self.lock:
            if self.users.pop(name, None) is None:
                return False
            self.sessions = {t: s for t, s in self.sessions.items() if s["user"] != name}
            if self.lease and self.lease["user"] == name:
                self.lease = None
            self._save()
            return True

    # -- control lease ----------------------------------------------------------

    def holder(self, idle_s=120.0):
        """Who is driving, or None. A lease nobody touched for ``idle_s`` lapses."""
        with self.lock:
            if self.lease and time.time() - self.lease["seen"] > idle_s:
                self.lease = None
            return dict(self.lease) if self.lease else None

    def take(self, token, force=False):
        """Take the controls. Returns (ok, holder). ``force`` takes them from someone else."""
        s = self.session(token)
        if s is None or s["role"] not in CAN_CONTROL:
            return False, self.holder()
        current = self.holder()
        if current and current["token"] != token and not force:
            return False, current
        with self.lock:
            self.lease = {"token": token, "user": s["user"], "device": s["device"],
                          "since": time.time(), "seen": time.time()}
            return True, dict(self.lease)

    def release(self, token):
        with self.lock:
            if self.lease and self.lease["token"] == token:
                self.lease = None

    def driving(self, token):
        """True if ``token`` holds the lease (and refreshes it)."""
        with self.lock:
            if self.lease and self.lease["token"] == token:
                self.lease["seen"] = time.time()
                return True
            return False
