"""Hold the latest top-down map posted by the lidar laptop.

POST /map with the JSON document. GET /map returns that same document.
Listens on 0.0.0.0:8086.
"""

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LATEST = b""
LOCK_BODY = b'{"view":"topdown","locked":false,"pose":null,"outline":[]}'


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
        self._send(200, LATEST or LOCK_BODY)

    def do_POST(self):
        global LATEST
        path = self.path.split("?", 1)[0]
        length = int(self.headers.get("Content-Length", "0") or 0)
        if path not in ("/", "/map") or length <= 0 or length > 2_000_000:
            self._send(400, b'{"error":"bad request"}')
            return
        body = self.rfile.read(length)
        try:
            json.loads(body)
        except json.JSONDecodeError:
            self._send(400, b'{"error":"json"}')
            return
        LATEST = body
        self._send(200, b'{"ok":true}')


def main():
    server = ThreadingHTTPServer(("0.0.0.0", 8086), Handler)
    print("map feed on :8086", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
