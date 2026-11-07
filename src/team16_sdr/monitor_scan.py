#!/usr/bin/env python3
"""Passive WiFi + Bluetooth monitor harness (Alfa USB card).

Feeds (each independent; a dead feed prints `ERROR: <feed> unavailable`
and the harness continues):
  wifi: tcpdump on the Alfa monitor interface, probe/assoc requests.
  bt:   bluetoothctl active scan (BR/EDR inquiry + BLE scan requests).

JSON lines appended to --out:
  {"t":…, "kind": "wifi-probe"|"wifi-assoc"|"bt"|"ble", "mac":…,
   "rssi":…, "ssid":…|null, "name":…|null}
Dedupe: wifi one line per (mac,kind)/5 s; bt one line per mac/10 s.
bt `kind` heuristic: locally-administered (private) MAC -> "ble",
public block -> "bt".
"""
import argparse
import json
import re
import subprocess
import sys
import threading
import time

MAC_RE = re.compile(r"([0-9a-f]{2}:){5}[0-9a-f]{2}", re.I)


def emit(out, lock, rec):
    line = json.dumps(rec, ensure_ascii=False)
    with lock:
        out.write(line + "\n")
        out.flush()
    print(line, flush=True)


def wifi_feed(out, lock, iface, stop):
    """tcpdump -Q in -e on management frames; parse probe/assoc requests."""
    last = {}  # (mac, kind) -> t
    cmd = ["tcpdump", "-i", iface, "-Q", "in", "-e", "-l",
           "type mgt subtype probe-req or type mgt subtype assoc-req"]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, bufsize=1)
    except FileNotFoundError:
        print("ERROR: wifi unavailable (tcpdump missing)", file=sys.stderr)
        return

    def err_check():
        time.sleep(1.0)
        if proc.poll() is not None:
            err = proc.stderr.read().strip().splitlines()
            print(f"ERROR: wifi unavailable ({iface}: {err[-1] if err else 'exited'})",
                  file=sys.stderr)
            return True
        return False

    if err_check():
        return
    try:
        for line in proc.stdout:
            if stop.is_set():
                break
            if "probe request" in line:
                kind = "wifi-probe"
            elif "assoc request" in line:
                kind = "wifi-assoc"
            else:
                continue
            src = line.split("src", 1)
            m = MAC_RE.search(src[1]) if len(src) > 1 else MAC_RE.search(line)
            if not m:
                continue
            mac = m.group(0).lower()
            sm = re.search(r"SSID:\"([^\"]*)\"", line)
            ssid = sm.group(1) if sm else None
            rm = re.search(r"(\-?\d+) dBm", line)
            rssi = int(rm.group(1)) if rm else None
            now = time.time()
            key = (mac, kind)
            if now - last.get(key, 0) < 5.0:
                continue
            last[key] = now
            emit(out, lock, {"t": round(now, 3), "kind": kind, "mac": mac,
                             "rssi": rssi, "ssid": ssid})
    finally:
        proc.terminate()


def resolve_controller(adapter):
    """Map hciN -> controller BD Address via hciconfig (None if unknown)."""
    try:
        out = subprocess.run(["hciconfig", adapter, "-a"], capture_output=True,
                             text=True, timeout=5).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    m = re.search(r"BD Address:\s*((?:[0-9A-F]{2}:){5}[0-9A-F]{2})", out, re.I)
    return m.group(1) if m else None


def is_private_mac(mac):
    return bool(int(mac.split(":")[0], 16) & 0x02)


def bt_sightings(adapter, seconds):
    """One bluetoothctl scan window; yield bt/ble sighting dicts.

    bluetoothctl prints `[NEW] Device MAC [alias]` then separate
    `[CHG] Device MAC RSSI: -N` lines; we emit on the first RSSI per
    device, and flush RSSI-less devices at window end.
    """
    ctrl = resolve_controller(adapter)
    try:
        proc = subprocess.Popen(["bluetoothctl"],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, bufsize=1)
    except FileNotFoundError:
        raise RuntimeError("bluetoothctl missing (sudo apt-get install -y bluez)")
    proc.stdin.write((f"select {ctrl}\n" if ctrl else "") + "scan on\n")
    proc.stdin.flush()
    devices = {}   # mac -> {"name":…, "rssi":…}
    emitted = set()
    t_end = time.time() + seconds
    try:
        for raw in proc.stdout:
            if time.time() >= t_end:
                break
            line = re.sub(r"\x1b\[[0-9;]*m", "", raw)
            m = re.search(r"\[NEW\] Device ((?:[0-9A-F]{2}:){5}[0-9A-F]{2})(?: (.*))?",
                          line, re.I)
            if m:
                mac = m.group(1).lower()
                name = (m.group(2) or "").strip() or None
                if name and re.fullmatch(r"(?:[0-9A-F]{2}[-_]){5}[0-9A-F]{2}",
                                         name, re.I):
                    name = None  # placeholder alias derived from the MAC
                devices[mac] = {"name": name, "rssi": None}
                continue
            m = re.search(r"\[CHG\] Device ((?:[0-9A-F]{2}:){5}[0-9A-F]{2}) "
                          r"RSSI: (-?\d+)", line, re.I)
            if m:
                mac = m.group(1).lower()
                d = devices.setdefault(mac, {"name": None, "rssi": None})
                d["rssi"] = int(m.group(2))
                if mac in emitted:
                    continue
                emitted.add(mac)
                yield {"t": round(time.time(), 3),
                       "kind": "ble" if is_private_mac(mac) else "bt",
                       "mac": mac, "name": d["name"], "rssi": d["rssi"]}
                continue
            m = re.search(r"\[CHG\] Device ((?:[0-9A-F]{2}:){5}[0-9A-F]{2}) "
                          r"(?:Alias|Name): (.+)", line, re.I)
            if m:
                d = devices.get(m.group(1).lower())
                if d and m.group(2).strip():
                    d["name"] = m.group(2).strip()
        for mac, d in devices.items():  # flush devices seen without RSSI
            if mac not in emitted:
                emitted.add(mac)
                yield {"t": round(time.time(), 3),
                       "kind": "ble" if is_private_mac(mac) else "bt",
                       "mac": mac, "name": d["name"], "rssi": d["rssi"]}
    finally:
        proc.stdin.write("scan off\nexit\n")
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()


def bt_feed(out, lock, adapter, stop):
    """Continuous bt/ble sightings, deduped one line per mac per 10 s."""
    last = {}
    try:
        while not stop.is_set():
            for rec in bt_sightings(adapter, 15):
                if stop.is_set():
                    return
                if time.time() - last.get(rec["mac"], 0) < 10.0:
                    continue
                last[rec["mac"]] = time.time()
                emit(out, lock, rec)
    except RuntimeError as e:
        print(f"ERROR: bt unavailable ({e})", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iface", default="mon1",
                    help="Alfa monitor interface (create with iw first)")
    ap.add_argument("--adapter", default="hci0", help="BT adapter, e.g. hci0")
    ap.add_argument("--out", default="monitor_hits.jsonl")
    ap.add_argument("--no-bt", action="store_true")
    ap.add_argument("--no-wifi", action="store_true")
    args = ap.parse_args()

    out = open(args.out, "a")
    lock = threading.Lock()
    stop = threading.Event()
    threads = []
    if not args.no_wifi:
        threads.append(threading.Thread(target=wifi_feed,
                                        args=(out, lock, args.iface, stop),
                                        daemon=True))
    if not args.no_bt:
        threads.append(threading.Thread(target=bt_feed,
                                        args=(out, lock, args.adapter, stop),
                                        daemon=True))
    for t in threads:
        t.start()
    try:
        while any(t.is_alive() for t in threads):
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop.set()
        return 0
    stop.set()
    return 0


if __name__ == "__main__":
    sys.exit(main())
