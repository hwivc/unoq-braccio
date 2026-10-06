"""Download the Edge Impulse cube model (float32, ~156 MB).

    ros2 run unoq_braccio_driver download_model
    ros2 run unoq_braccio_driver download_model --dest ~/models --url <other model link>

The model is too big for GitHub, so it is fetched from the Edge Impulse
project (https://studio.edgeimpulse.com/studio/975321) into ~/unoq-braccio/,
where the cube detector looks for it. Run it once; it does nothing if the
model is already there.

It downloads to a ``.part`` file, shows progress, retries if the connection
drops (resuming where it stopped when the server allows it, otherwise
starting again), checks the result is a TFLite model of the expected size,
and only then renames it - a half-downloaded file is never used.
"""

import argparse
import os
import sys
import time
import urllib.error
import urllib.request

MODEL_URL = "https://studio.edgeimpulse.com/v1/api/975321/learn-data/3/model/tflite-float"
MODEL_NAME = "ei-cube-detection-object-detection-tensorflow-lite-float32-model.3.lite"
DEFAULT_DEST = "~/unoq-braccio"
CHUNK = 256 * 1024


def is_tflite(path):
    """TFLite files carry the identifier 'TFL3' at bytes 4-8."""
    with open(path, "rb") as handle:
        return handle.read(8)[4:8] == b"TFL3"


def download(url, final_path, attempts=20):
    part = final_path + ".part"
    total = None
    for attempt in range(1, attempts + 1):
        have = os.path.getsize(part) if os.path.exists(part) else 0
        request = urllib.request.Request(url, headers={"Range": f"bytes={have}-"} if have else {})
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                if have and response.status != 206:
                    print("\nServer cannot resume; starting again from the beginning.")
                    have = 0
                length = response.headers.get("Content-Length")
                if length is not None:
                    total = have + int(length)
                started = time.monotonic()
                done = 0
                with open(part, "ab" if have else "wb") as out:
                    while True:
                        chunk = response.read(CHUNK)
                        if not chunk:
                            break
                        out.write(chunk)
                        done += len(chunk)
                        speed = done / max(1e-6, time.monotonic() - started)
                        got = have + done
                        if total:
                            left = (total - got) / max(speed, 1.0)
                            print(f"\r  {got / 1e6:6.1f} / {total / 1e6:.1f} MB  "
                                  f"{100 * got / total:5.1f} %  {speed / 1e3:6.0f} kB/s  "
                                  f"~{left / 60:4.0f} min left ", end="", flush=True)
                        else:
                            print(f"\r  {got / 1e6:6.1f} MB  {speed / 1e3:6.0f} kB/s ", end="", flush=True)
            size = os.path.getsize(part)
            if total is not None and size < total:
                raise OSError(f"connection ended early ({size} of {total} bytes)")
            break
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            if isinstance(exc, urllib.error.HTTPError) and exc.code in (401, 403, 404):
                print(f"\nDownload refused ({exc}). Check the link, or download the model by "
                      "hand from https://studio.edgeimpulse.com/studio/975321 (Dashboard -> "
                      f"download the TensorFlow Lite float32 model) into {os.path.dirname(final_path)}")
                return False
            if attempt == attempts:
                print(f"\nGave up after {attempts} attempts: {exc}. Run the command again to continue.")
                return False
            print(f"\n  connection problem ({exc}); retrying in 5 s (attempt {attempt + 1}/{attempts})")
            time.sleep(5)

    print()
    if not is_tflite(part):
        os.remove(part)
        print("The downloaded file is not a TensorFlow Lite model; removed it. Check the link.")
        return False
    os.replace(part, final_path)
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description="Download the Edge Impulse cube model.")
    parser.add_argument("--url", default=MODEL_URL)
    parser.add_argument("--dest", default=DEFAULT_DEST, help="folder to save it in")
    parser.add_argument("--name", default=MODEL_NAME, help="file name to save it as")
    args, _ = parser.parse_known_args(argv)  # ignore ROS's own arguments

    folder = os.path.expanduser(args.dest)
    final_path = os.path.join(folder, args.name)
    if os.path.exists(final_path) and is_tflite(final_path):
        print(f"Model already there: {final_path}")
        return 0
    os.makedirs(folder, exist_ok=True)
    print(f"Downloading the cube model (~156 MB) to {final_path}")
    print("This can take a while on a slow connection; Ctrl+C is safe, run again to continue.")
    try:
        ok = download(args.url, final_path)
    except KeyboardInterrupt:
        print("\nStopped. Run the same command again to continue.")
        return 1
    if ok:
        print(f"Done: {final_path}")
        print("Needs the TFLite runtime once:  "
              "pip install --break-system-packages ai-edge-litert 'numpy<2'")
        print("(numpy<2 matters: Ubuntu's OpenCV breaks with NumPy 2.)")
        print("Then restart the launch; it logs 'Cube detector: Edge Impulse model ...'.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
