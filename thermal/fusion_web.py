"""Web front-end for fusion_pi.py, in its OWN process so it can never be starved by the Hailo loop.

  :8081  fused view (MJPEG)        :8082  thermal view (MJPEG)
  GET /calibrate  /calibrate/status  /calibrate/reset   (JSON, CORS open for the dashboard)
Reads frames/status that fusion_pi.py writes to /dev/shm/team16_fusion and sends commands back
through a file there. Standard library only.
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

SHM = Path("/dev/shm/team16_fusion")
PORTS = {8081: "fused", 8082: "thermal"}


def read_status():
    try:
        s = json.loads((SHM / "status.json").read_text())
        if time.time() - s.get("t", 0) > 5:
            s.update(thermal_ok=False, state="idle", message="Fusion process is not producing frames")
        return s
    except (FileNotFoundError, ValueError):
        return {"state": "idle", "samples": 0, "needed": 12, "secs_left": 0, "calibrated": False,
                "thermal_ok": False, "message": "Fusion process is not running"}


def send_cmd(text):
    SHM.mkdir(parents=True, exist_ok=True)
    tmp = SHM / "cmd.tmp"
    tmp.write_text(text)
    tmp.replace(SHM / "cmd")


def make_handler(chan):
    class H(BaseHTTPRequestHandler):
        timeout = 15  # a stalled/dead client can only hold its own thread for this long

        def log_message(self, *a):
            pass

        def do_GET(self):
            path = self.path.split("?")[0]
            if path.startswith("/calibrate"):
                s = read_status()
                if path == "/calibrate":
                    send_cmd("calibrate")
                    s.update(state="collecting", samples=0, secs_left=30, message="")
                elif path == "/calibrate/reset":
                    send_cmd("reset")
                    s.update(calibrated=False, message="Reset to rough guess")
                body = json.dumps(s).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            f, last = SHM / f"{chan}.jpg", 0
            try:
                while True:
                    try:
                        m = f.stat().st_mtime_ns
                        if m != last:
                            data = f.read_bytes()
                            last = m
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(data)
                                             + data + b"\r\n")
                    except FileNotFoundError:
                        pass
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
    return H


class Server(ThreadingHTTPServer):
    request_queue_size = 64
    daemon_threads = True


def main():
    for port, chan in PORTS.items():
        srv = Server(("0.0.0.0", port), make_handler(chan))
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"{chan}: http://<pi address>:{port}/", flush=True)
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
