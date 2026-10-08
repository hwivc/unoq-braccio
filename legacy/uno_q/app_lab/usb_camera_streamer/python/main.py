from arduino.app_utils import App
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2
import threading
import time


CAMERA_INDEX = 0
PORT = 8080
frame_lock = threading.Lock()
latest_jpeg = None


def camera_loop():
    global latest_jpeg
    capture = cv2.VideoCapture(CAMERA_INDEX)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    capture.set(cv2.CAP_PROP_FPS, 15)

    while True:
        ok, frame = capture.read()
        if not ok:
            time.sleep(0.2)
            continue
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            with frame_lock:
                latest_jpeg = encoded.tobytes()
        time.sleep(0.02)


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/", "/stream"):
            self.send_error(404)
            return

        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"UNO Q USB camera streamer. Use /stream\n")
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        while True:
            with frame_lock:
                frame = latest_jpeg
            if frame is None:
                time.sleep(0.1)
                continue
            try:
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(0.07)
            except (BrokenPipeError, ConnectionResetError):
                break

    def log_message(self, format, *args):
        return


def loop():
    threading.Thread(target=camera_loop, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), StreamHandler)
    print(f"USB camera stream listening on port {PORT}")
    server.serve_forever()


App.run(user_loop=loop)
