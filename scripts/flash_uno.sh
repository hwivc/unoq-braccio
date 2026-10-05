#!/usr/bin/env bash
# Compile and upload firmware/braccio_uno_firmware to an Arduino UNO.
#   scripts/flash_uno.sh                       # UNO R3 on /dev/ttyACM0
#   scripts/flash_uno.sh /dev/ttyUSB0          # clone board (CH340)
#   scripts/flash_uno.sh /dev/ttyACM0 arduino:renesas_uno:minima   # UNO R4 Minima
set -euo pipefail

PORT="${1:-/dev/ttyACM0}"
FQBN="${2:-arduino:avr:uno}"
SKETCH_DIR="$(cd "$(dirname "$0")/../firmware/braccio_uno_firmware" && pwd)"
CORE="${FQBN%:*}"

arduino-cli core update-index
arduino-cli core install "$CORE"
arduino-cli lib install Servo
arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR"
