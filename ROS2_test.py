#!/usr/bin/env python3
"""Use a Grove Vision AI V2 as a plain webcam and show it in a window.

    python ROS2_test.py                         # auto-detect the board
    python ROS2_test.py --port /dev/ttyACM0     # or COM5 on Windows
    python ROS2_test.py --resolution 2          # 640x480 (slower)

No model, no ROS 2 - just frames from the camera on the board, pulled over
USB serial with AT+SAMPLE. Press q or Esc in the window to quit.
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2

from sscma import SSCMAClient, SSCMAError, find_port

RESOLUTIONS = {0: "240x240", 1: "480x480", 2: "640x480"}


def main() -> int:
    parser = argparse.ArgumentParser(description="Grove Vision AI V2 as a webcam.")
    parser.add_argument("--port", help="Serial port. Auto-detected when omitted.")
    parser.add_argument("--resolution", type=int, choices=sorted(RESOLUTIONS),
                        default=0, help="0 = 240x240, 1 = 480x480, 2 = 640x480.")
    parser.add_argument("--scale", type=float, default=2.0,
                        help="Window upscale factor (default: 2).")
    args = parser.parse_args()

    try:
        port = args.port or find_port()
        print(f"Opening {port}, {RESOLUTIONS[args.resolution]}...")

        with SSCMAClient(port=port) as client:
            client.set_resolution(args.resolution)

            window = "Grove Vision AI V2"
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            last = time.monotonic()
            fps = 0.0
            frames = 0

            for frame in client.stream(detect=False):
                frames += 1
                now = time.monotonic()
                # Smoothed so the number is readable rather than flickering.
                fps = 0.9 * fps + 0.1 * (1.0 / max(now - last, 1e-6))
                last = now

                image = frame.image
                if args.scale != 1.0:
                    image = cv2.resize(image, None, fx=args.scale, fy=args.scale,
                                       interpolation=cv2.INTER_NEAREST)
                cv2.putText(image, f"{fps:4.1f} fps", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow(window, image)

                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    break

            cv2.destroyAllWindows()

        if frames == 0:
            print("No frames arrived - check the cable carries data and the "
                  "board runs SSCMA firmware.")
            return 1
        print(f"Stopped after {frames} frames.")
        return 0

    except SSCMAError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
