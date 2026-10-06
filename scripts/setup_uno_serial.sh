#!/usr/bin/env bash
# One-time Linux setup for the Arduino UNO serial link:
#  - installs a udev rule so the board always appears as /dev/braccio
#  - adds you to the dialout group so ROS can open the port without sudo
# Log out and back in afterwards for the group change to apply.
set -euo pipefail

RULES="$(cd "$(dirname "$0")" && pwd)/99-braccio-uno.rules"

sudo cp "$RULES" /etc/udev/rules.d/99-braccio-uno.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"

# brltty (braille display support) grabs CH340 USB-serial chips, which most
# UNO clones use, and makes the port vanish.
if dpkg -s brltty >/dev/null 2>&1; then
  echo "NOTE: brltty is installed. If your UNO is a clone (CH340 chip) and"
  echo "      /dev/ttyUSB0 disappears, remove it:  sudo apt remove brltty"
fi

echo "Done. Unplug and replug the UNO, then check:  ls -l /dev/braccio"
echo "If you were just added to 'dialout', log out and back in first."
