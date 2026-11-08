"""Hold the latest top-down map posted by the lidar laptop, and remember every position it has posted.

POST /map     with the JSON document.  GET /map returns that document plus "trail": every distinct
              locked pose posted so far (oldest first), so a new post never erases the earlier ones.
POST /clear   forgets the trail (the map itself stays).
Listens on 0.0.0.0:8086. State is saved to map_feed_state.json so a restart keeps the map and the trail.
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
STATE = os.path.join(HERE, "map_feed_state.json")
LATEST = None          # last posted document (dict)
TRAIL = []             # [{"x_m","y_m","z_m","yaw_cw_deg","t","seq"}, ...]
MAX_TRAIL = 20000
LOCK = threading.Lock()
LOCK_BODY = {"view": "topdown", "locked": False, "pose": None, "outline": []}


def load():
    global LATEST, TRAIL
    try:
        with open(STATE) as f:
            s = json.load(f)
        LATEST, TRAIL = s.get("latest"), s.get("trail", [])
    except (OSError, ValueError):
        pass


def save():
    tmp = STATE + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"latest": LATEST, "trail": TRAIL}, f)
    os.replace(tmp, STATE)


def remember(doc):
    p = doc.get("pose") if isinstance(doc, dict) else None
    if not isinstance(p, dict) or doc.get("locked") is False:
        return
    try:
        x, y = float(p["x_m"]), float(p["y_m"])
    except (KeyError, TypeError, ValueError):
        return
    if TRAIL and abs(TRAIL[-1]["x_m"] - x) < 1e-6 and abs(TRAIL[-1]["y_m"] - y) < 1e-6:
        return
    TRAIL.append({"x_m": x, "y_m": y, "z_m": p.get("z_m", 0.0), "yaw_cw_deg": p.get("yaw_cw_deg"),
                  "t": round(time.time(), 2), "seq": doc.get("seq")})
    del TRAIL[:-MAX_TRAIL]


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        return

    def _send(self, code, body, content_type="application/json"):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self._send(204, b"")

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path not in ("/", "/map"):
            self._send(404, b'{"error":"not found"}')
            return
        with LOCK:
            doc = dict(LATEST or LOCK_BODY)
            doc["trail"] = list(TRAIL)
        self._send(200, json.dumps(doc))

    def do_POST(self):
        global LATEST, TRAIL
        path = self.path.split("?", 1)[0]
        if path == "/clear":
            with LOCK:
                TRAIL = []
                save()
            self._send(200, b'{"ok":true}')
            return
        length = int(self.headers.get("Content-Length", "0") or 0)
        if path not in ("/", "/map") or length <= 0 or length > 2_000_000:
            self._send(400, b'{"error":"bad request"}')
            return
        body = self.rfile.read(length)
        try:
            doc = json.loads(body)
        except json.JSONDecodeError:
            self._send(400, b'{"error":"json"}')
            return
        if not isinstance(doc, dict):
            self._send(400, b'{"error":"json object expected"}')
            return
        doc.pop("trail", None)
        with LOCK:
            LATEST = doc
            remember(doc)
            save()
        self._send(200, b'{"ok":true}')


def main():
    load()
    server = ThreadingHTTPServer(("0.0.0.0", 8086), Handler)
    print("map feed on :8086 (trail points: %d)" % len(TRAIL), flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
