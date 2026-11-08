#!/usr/bin/env python3
"""rfhits hub: per-category JSON logs + live JSON ports for the dashboard.

Tails the pipeline logs (monitor_hits.jsonl, rf_hits.jsonl) and fans
out by record kind into --dir (default ~/rfhits):

  kind bt                 -> bt_classic.jsonl   0.0.0.0:8091  cat "bt"
  kind ble                -> ble.jsonl          0.0.0.0:8092  cat "ble"
  hz, no kind (rf file)   -> cellular.jsonl     0.0.0.0:8093  cat "cell"
  kind wifi-probe/assoc   -> wifi.jsonl         0.0.0.0:8094  cat "wifi"

Any other kind (coax-window, unknown) is ignored: not written, not
streamed. Output line = source record + "t_ns" (hub-ingest nanoseconds)
+ "cat"; identifiers/signal (mac, name, adapter, ssid, hz, power_db,
rssi) ride along from the source. TCP clients get newline-delimited
JSON, live only (no history replay); multiple clients per port.
run_pod.sh rm -f's the source logs each demo; offsets reset when a file
shrinks. Writes README.md into --dir on first run. Stdlib only; the hub
never opens the Pluto.
"""
import argparse
import json
import os
import socket
import socketserver
import sys
import threading
import time

from phone_phone import read_new

# (cat, filename, port) -- ports 8091-8094 confirmed free on the Pi.
CATS = [
    ("bt", "bt_classic.jsonl", 8091),
    ("ble", "ble.jsonl", 8092),
    ("cell", "cellular.jsonl", 8093),
    ("wifi", "wifi.jsonl", 8094),
]

README_TEXT = """\
# rfhits -- phone-detect hit logs + live ports

Every phone-detect hit lands here, split by category. `rf_hub.py`
(systemd: `rf-hub.service`) tails `~/team16_sdr/monitor_hits.jsonl` and
`~/team16_sdr/rf_hits.jsonl` and fans records out by kind.

## Feeds (start on boot, stay up)

| Unit | Runs | Feeds |
|---|---|---|
| `rf-feed-monitor.service` (root) | `monitor_scan.py` (Alfa ch1 monitor + hci0/hci1) | wifi, bt, ble |
| `rf-feed-cell.service` (hacker) | `phone_scan.py` (Pluto uplink dwell) | cell |

`run_pod.sh` pkills `phone_scan.py` for its demo window; the cell feed
service then flaps (Pluto lock busy, rc=1) until the demo ends and
restarts itself into ownership. Harmless.

## Files (newline-delimited JSON, one hit per line)

| File | Source records |
|---|---|
| `bt_classic.jsonl` | `kind: "bt"` (public-block MAC sightings, hci0/hci1) |
| `ble.jsonl` | `kind: "ble"` (locally-administered MAC sightings) |
| `cellular.jsonl` | rf hits from the Pluto (`hz`, `power_db`, `pose`) |
| `wifi.jsonl` | `kind: "wifi-probe"` / `"wifi-assoc"` (Alfa monitor) |

## Live ports (bound to all interfaces; remote machines can connect)

| Port | Category |
|---|---|
| 8091 | bt_classic |
| 8092 | ble |
| 8093 | cellular |
| 8094 | wifi |

Each port streams new hits as JSON lines the moment the hub ingests
them. Example client:

    nc 192.168.8.163 8092

Streams are **live only**: a client sees hits arriving after it
connects, never history. Multiple clients per port are fine.

## Record shape

Every line is the source record plus two hub fields:

- `t` -- capture time (float seconds, from the feed)
- `t_ns` -- hub-ingest time (int nanoseconds; sub-second tail lag)
- `cat` -- `"bt"`, `"ble"`, `"wifi"`, or `"cell"`

Identifiers and signal ride along from the source: `mac`, `name`,
`adapter`, `ssid` for bt/ble/wifi (`rssi` is the signal); `hz`,
`power_db`, `pose` for cell (`power_db` is the signal).

## Gotchas

- `rssi: 0` from hci0 is a controller artifact meaning "unknown", not
  a real reading.
- `run_pod.sh` truncates the source logs at each demo start; the hub
  notices the shrink and resets its read offsets automatically.
- Each `*.jsonl` is purged (truncated to empty) once it reaches
  `--max-mb` (default 5120 = 5 GiB) so the disk never fills up.

## Ideas (not implemented)

- Null-out `rssi: 0` from hci0 (measured garbage) instead of trusting it.
- A fifth combined port (8095) streaming all categories with `cat` as
  the discriminator.
- `--replay N` to send the last N file lines on connect once the
  dashboard wants history.
- Log rotation for `~/rfhits/*.jsonl` (the 5 GiB purge is a blunt
  instrument; ~120 lines/min at demo rates).
"""


def route(rec):
    """cat for a source record, or None to ignore it."""
    kind = rec.get("kind")
    if kind == "bt":
        return "bt"
    if kind == "ble":
        return "ble"
    if kind in ("wifi-probe", "wifi-assoc"):
        return "wifi"
    if kind is None and "hz" in rec:
        return "cell"
    return None


class Broadcaster:
    """Append lines to one category file; push each to its TCP clients."""

    SEND_TIMEOUT = 5.0  # drop a client whose send stalls this long

    def __init__(self, path, max_bytes):
        self.max_bytes = max_bytes
        self.lock = threading.Lock()
        self.clients = []
        self.f = open(path, "a", encoding="utf-8")

    def publish(self, line):
        data = (line + "\n").encode("utf-8")
        with self.lock:
            self.f.write(line + "\n")
            self.f.flush()
            if self.f.tell() >= self.max_bytes:
                self.f.truncate(0)  # purge in place (same inode)
                self.f.seek(0)
            clients = list(self.clients)  # snapshot; send outside the lock
        dead = []
        for sock in clients:
            try:
                sock.sendall(data)
            except OSError:  # closed, or SEND_TIMEOUT on a stalled reader
                dead.append(sock)
        if dead:
            with self.lock:
                for sock in dead:
                    if sock in self.clients:
                        self.clients.remove(sock)
            for sock in dead:
                try:
                    sock.close()
                except OSError:
                    pass

    def add(self, sock):
        with self.lock:
            self.clients.append(sock)

    def remove(self, sock):
        with self.lock:
            if sock in self.clients:
                self.clients.remove(sock)


class Handler(socketserver.BaseRequestHandler):
    """Hold the connection, drain until the client closes."""

    def handle(self):
        bcast = self.server.broadcaster
        self.request.settimeout(bcast.SEND_TIMEOUT)
        bcast.add(self.request)
        try:
            while True:
                try:
                    if self.request.recv(4096) == b"":
                        break
                except socket.timeout:
                    continue  # idle is fine; publish() bounds the send
        except OSError:
            pass
        finally:
            bcast.remove(self.request)


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    ap = argparse.ArgumentParser(
        description="Fan pipeline hit logs out to ~/rfhits + live ports.")
    ap.add_argument("--monitor", default="~/team16_sdr/monitor_hits.jsonl")
    ap.add_argument("--rf", default="~/team16_sdr/rf_hits.jsonl")
    ap.add_argument("--dir", default="~/rfhits")
    ap.add_argument("--poll", type=float, default=0.25,
                    help="tail poll interval, s")
    ap.add_argument("--max-mb", type=float, default=5120.0,
                    help="purge a category file at this size, MiB")
    args = ap.parse_args()
    monitor = os.path.expanduser(args.monitor)
    rf = os.path.expanduser(args.rf)
    outdir = os.path.expanduser(args.dir)

    os.makedirs(outdir, exist_ok=True)
    readme = os.path.join(outdir, "README.md")
    if not os.path.exists(readme):
        with open(readme, "w", encoding="utf-8") as f:
            f.write(README_TEXT)

    bcasts = {}
    servers = []
    for cat, fname, port in CATS:
        srv = Server(("0.0.0.0", port), Handler)
        srv.broadcaster = Broadcaster(os.path.join(outdir, fname),
                                      int(args.max_mb * 1024 * 1024))
        bcasts[cat] = srv.broadcaster
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        servers.append(srv)

    offsets = {monitor: 0, rf: 0}
    try:
        while True:
            for path in (monitor, rf):
                try:
                    if os.path.exists(path) and \
                            os.path.getsize(path) < offsets[path]:
                        offsets[path] = 0  # run_pod.sh rm -f'd the log
                    recs, offsets[path] = read_new(path, offsets[path])
                except OSError:
                    continue
                for rec in recs:
                    cat = route(rec)
                    if cat is None:
                        continue
                    out = dict(rec)
                    out["t_ns"] = time.time_ns()
                    out["cat"] = cat
                    bcasts[cat].publish(json.dumps(out, ensure_ascii=False))
            time.sleep(args.poll)
    except KeyboardInterrupt:
        pass
    finally:
        for srv in servers:
            srv.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
