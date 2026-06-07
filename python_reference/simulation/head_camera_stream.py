import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import mujoco
import numpy as np


class _StreamState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_jpg = None
        self.latest_png = None
        self.last_ts = None
        self.seq = 0


class HeadCameraStreamServer:
    """Publish MuJoCo camera frames as HTTP stream + RGBA PNG snapshots.

    Endpoints:
      - /health      : JSON status
      - /frame.jpg   : latest RGB jpeg frame
      - /frame.png   : latest RGBA png frame (alpha=255)
      - /stream.mjpg : multipart JPEG stream
    """

    def __init__(
        self,
        model,
        data,
        camera_name="head_rgba_camera",
        host="0.0.0.0",
        port=18080,
        width=640,
        height=480,
        jpeg_quality=80,
    ):
        self.model = model
        self.data = data
        self.camera_name = camera_name
        self.host = host
        self.port = int(port)
        self.width = int(width)
        self.height = int(height)
        self.jpeg_quality = int(jpeg_quality)

        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if camera_id < 0:
            raise ValueError(f"camera '{camera_name}' not found in model")

        self.renderer = mujoco.Renderer(model, width=self.width, height=self.height)
        self.state = _StreamState()

        self._httpd = None
        self._thread = None

    def start(self):
        state = self.state

        class Handler(BaseHTTPRequestHandler):
            def _send_bytes(self, data: bytes, ctype: str):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path == "/health":
                    with state.lock:
                        payload = {
                            "ok": True,
                            "seq": state.seq,
                            "timestamp": state.last_ts,
                        }
                    raw = json.dumps(payload).encode("utf-8")
                    self._send_bytes(raw, "application/json")
                    return

                if self.path == "/frame.jpg":
                    with state.lock:
                        buf = state.latest_jpg
                    if buf is None:
                        self.send_error(503, "no frame yet")
                        return
                    self._send_bytes(buf, "image/jpeg")
                    return

                if self.path == "/frame.png":
                    with state.lock:
                        buf = state.latest_png
                    if buf is None:
                        self.send_error(503, "no frame yet")
                        return
                    self._send_bytes(buf, "image/png")
                    return

                if self.path == "/stream.mjpg":
                    self.send_response(200)
                    self.send_header("Age", "0")
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    try:
                        while True:
                            with state.lock:
                                buf = state.latest_jpg
                            if buf is not None:
                                self.wfile.write(b"--frame\r\n")
                                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                                self.wfile.write(f"Content-Length: {len(buf)}\r\n\r\n".encode("utf-8"))
                                self.wfile.write(buf)
                                self.wfile.write(b"\r\n")
                            time.sleep(0.03)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    return

                self.send_error(404, "not found")

            def log_message(self, fmt, *args):
                # keep deploy console clean
                return

        self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        print(f"[CameraStream] serving at http://{self.host}:{self.port} (camera={self.camera_name})")

    def update(self):
        self.renderer.update_scene(self.data, camera=self.camera_name)
        rgb = self.renderer.render()  # HxWx3, RGB

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        ok_jpg, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])

        alpha = np.full((rgb.shape[0], rgb.shape[1], 1), 255, dtype=np.uint8)
        rgba = np.concatenate([rgb, alpha], axis=2)
        bgra = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA)
        ok_png, png = cv2.imencode(".png", bgra)

        if ok_jpg and ok_png:
            with self.state.lock:
                self.state.latest_jpg = jpg.tobytes()
                self.state.latest_png = png.tobytes()
                self.state.last_ts = time.time()
                self.state.seq += 1

    def close(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        self.renderer.close()
