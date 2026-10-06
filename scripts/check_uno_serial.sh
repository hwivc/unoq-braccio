#!/usr/bin/env bash
# Find out why the Braccio's UNO serial port drops or stays silent.
# Stop any ROS launch using the arm first.
#   scripts/check_uno_serial.sh                 # /dev/braccio
#   scripts/check_uno_serial.sh /dev/ttyACM0
set -u

PORT="${1:-/dev/braccio}"

echo "== 1. Serial ports =="
FOUND="$(ls -l /dev/braccio /dev/ttyACM* /dev/ttyUSB* 2>/dev/null)"
if [ -n "$FOUND" ]; then echo "$FOUND"; else echo "No serial ports at all: is the UNO plugged in?"; fi
if command -v lsusb >/dev/null; then
  echo "Board: $(lsusb | grep -iE '2341:|1a86:|arduino' || echo 'no Arduino or CH340 found by lsusb')"
fi
if [ ! -e "$PORT" ]; then
  echo "$PORT does not exist. Try scripts/check_uno_serial.sh /dev/ttyACM0 (or ttyUSB0)."
  exit 1
fi
REAL="$(readlink -f "$PORT")"
echo "$PORT -> $REAL"

echo
echo "== 2. Other programs holding the port (should be none) =="
if command -v fuser >/dev/null; then
  fuser -v "$REAL" 2>&1 || echo "none"
else
  echo "(fuser not installed)"
fi

echo
echo "== 3. ModemManager / brltty =="
if systemctl is-active --quiet ModemManager 2>/dev/null; then
  if udevadm info -q property -n "$REAL" 2>/dev/null | grep -q "ID_MM_DEVICE_IGNORE=1"; then
    echo "ModemManager running, but told to ignore this board: OK"
  else
    echo "PROBLEM: ModemManager is running and may probe the board."
    echo "  Fix: scripts/setup_uno_serial.sh, then unplug/replug the UNO"
    echo "  (or: sudo systemctl stop ModemManager)"
  fi
else
  echo "ModemManager not running: OK"
fi
if dpkg -s brltty >/dev/null 2>&1; then
  echo "brltty installed: harmless for a genuine UNO; for a CH340 clone run  sudo apt remove brltty"
fi

echo
echo "== 4. Permission =="
if [ -r "$REAL" ] && [ -w "$REAL" ]; then echo "read/write OK"; else
  echo "PROBLEM: no access. Run scripts/setup_uno_serial.sh, then log out and back in."; fi

if stty -F "$REAL" 2>/dev/null | grep -q -- "-hupcl"; then
  echo "Note: HUPCL is off on $REAL, so just opening the port does not reset the UNO."
  echo "      The bridge pulses reset itself, so this is fine. To restore: stty -F $REAL hupcl"
fi

echo
echo "== 5. Listening to the board for 15 s (resetting it first) =="
START="$(date '+%Y-%m-%d %H:%M:%S')"
python3 - "$REAL" <<'PY'
import sys, time
import serial

port = sys.argv[1]
t0 = time.monotonic()
stamp = lambda: f"[{time.monotonic() - t0:5.1f} s]"
try:
    s = serial.Serial(port, 115200, timeout=0.2)
except Exception as exc:
    print("OPEN FAILED:", exc)
    sys.exit(1)
# Pulse DTR like the ROS bridge does, so a standard UNO restarts even if
# Linux left DTR asserted from an earlier session.
s.dtr = False
time.sleep(0.1)
s.reset_input_buffer()
s.dtr = True
print(stamp(), "opened and pulsed reset; expect READY BRACCIO_UNO 1 and a STAT line within ~8 s")
asked = False
heard = False
while time.monotonic() - t0 < 15:
    try:
        raw = s.readline()
    except Exception as exc:
        print(stamp(), "LOST THE PORT:", exc)
        break
    if raw:
        heard = True
        print(stamp(), raw.decode("ascii", errors="replace").strip())
    if not asked and time.monotonic() - t0 > 10:
        asked = True
        try:
            # The bare newline first clears any half line of junk on the board.
            s.write(b"\nI\nS\n")
            print(stamp(), "sent I and S (identify, status)")
        except Exception as exc:
            print(stamp(), "LOST THE PORT on write:", exc)
            break
if not heard:
    print(stamp(), "nothing received from the board")
PY

echo
echo "== 6. Kernel USB messages during that test =="
if sudo -n true 2>/dev/null || sudo -v; then
  sudo journalctl -k --since "$START" --no-pager 2>/dev/null \
    | grep -iE "usb|tty|cdc_acm|ch34|brltty|over-current" || echo "none (no USB disconnects)"
fi

cat <<'EOF'

== How to read this ==
- READY and STAT printed, nothing lost  -> board and port are fine. If step 2 or 3
  showed a problem, that was it; fix it and launch again.
- "LOST THE PORT" and step 6 shows "USB disconnect" (often ~2 s in, when the
  servos power up) -> POWER. The board browns out when the servos switch on.
  Power the servos from the Braccio shield's own 5 V supply (4 A or more), not
  from USB, and check its plug and polarity.
- "LOST THE PORT" with no USB disconnect -> another program is reading the port
  (step 2 / step 3). Close it, or run scripts/setup_uno_serial.sh and replug.
- No READY at first, but "READY BRACCIO_UNO 1" and STAT after "sent I and S"
  -> the board did not restart even when reset was pulsed. The arm works, but
  it keeps its last pose at launch instead of standing up. On a standard UNO
  this points to a clone missing the reset capacitor; press the board's RESET
  button (or power-cycle it) to get the standing-up start.
- "ERR unknown_command" right after "sent I and S", then READY and STAT ->
  leftover junk from before (e.g. ModemManager); harmless once cleared.
- Nothing received, nothing lost -> wrong sketch on the board. Reflash:
  bash scripts/flash_uno.sh /dev/ttyACM0
EOF
