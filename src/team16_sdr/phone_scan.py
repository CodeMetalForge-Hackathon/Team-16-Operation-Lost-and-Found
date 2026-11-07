#!/usr/bin/env python3
"""Cellular uplink band scanner for the ADALM-PLUTO (receive-only).

Dwells on cellular uplink channels, FFT-detects energy above the noise
floor, and appends rf_hit JSON lines:
  {"t": <epoch>, "hz": <center Hz>, "power_db": <dBFS>, "pose": null}

Never touches the TX chain (altvoltage1) or ensm_mode.
"""
import argparse
import json
import sys
import time

import numpy as np

GROUP_HZ = 1_000_000  # bin-group width for hit detection
FULL_SCALE = 2048.0   # 12-bit IQ full scale


def open_device(uri, freq, samplerate, bw, gain):
    import iio
    ctx = iio.Context(uri)
    phy = ctx.find_device("ad9361-phy")
    phy.find_channel("altvoltage0", True).attrs["frequency"].value = str(int(freq))
    v0 = phy.find_channel("voltage0", False)
    v0.attrs["gain_control_mode"].value = "manual"
    v0.attrs["hardwaregain"].value = str(int(gain))
    v0.attrs["sampling_frequency"].value = str(int(samplerate))
    v0.attrs["rf_bandwidth"].value = str(int(bw))
    lpc = ctx.find_device("cf-ad9361-lpc")
    lpc.find_channel("voltage0", False).enabled = True  # I
    lpc.find_channel("voltage1", False).enabled = True  # Q
    return phy, lpc


def dwell(lpc, samplerate, seconds, buf_limit=32768):
    """Capture `seconds` of complex IQ from cf-ad9361-lpc (I/Q int16).

    The Pi5 xHCI sometimes arms the first DMA buffer dead (reads all-zero
    forever); recreate the buffer until the first read carries signal.
    """
    import iio
    n = int(samplerate * seconds)
    size = min(n, buf_limit)
    buf = None
    for _ in range(10):
        buf = iio.Buffer(lpc, size, cyclic=False)
        first = buf.read()
        if np.abs(np.frombuffer(first, dtype="<i2")).max() > 100:
            break
        del buf
    chunks = [np.frombuffer(first, dtype="<i2")]
    got = len(chunks[0]) // 2
    fails = 0
    while got < n:
        try:
            data = buf.read()
            fails = 0
        except Exception:
            fails += 1
            if fails >= 2:
                raise
            continue
        raw = np.frombuffer(data, dtype="<i2")
        chunks.append(raw)
        got += len(raw) // 2
    iq = np.concatenate(chunks) if len(chunks) > 1 else chunks[0]
    iq = iq[: 2 * (len(iq) // 2)]
    i = (iq[0::2] >> 4).astype(np.float64)  # S12/16 left-aligned -> 12-bit
    q = (iq[1::2] >> 4).astype(np.float64)
    return i + 1j * q


def analyze(iq, samplerate, margin_db):
    """Welch-averaged periodogram in dBFS; return (floor_db, best_hz, best_db)
    or None when no group exceeds floor + margin_db."""
    n = len(iq) - (len(iq) % 1024)
    if n < 8192:
        return None
    seg = 8192
    nseg = min(8, n // seg)
    win = np.hanning(seg)
    psd = np.zeros(seg)
    for k in range(nseg):
        x = iq[k * seg:(k + 1) * seg] * win
        f = np.fft.fftshift(np.fft.fft(x))
        psd += np.abs(f) ** 2
    psd /= nseg * (np.sum(win ** 2) * seg)
    db = 10 * np.log10(np.maximum(psd, 1e-30)) - 20 * np.log10(FULL_SCALE)
    floor = float(np.median(db))
    per_bin = samplerate / seg
    group = max(1, int(round(GROUP_HZ / per_bin)))
    lo, hi = 20, seg - 20
    usable = (hi - lo) // group
    gm = psd[lo:lo + usable * group].reshape(usable, group).mean(axis=1)
    gm_db = 10 * np.log10(np.maximum(gm, 1e-30)) - 20 * np.log10(FULL_SCALE)
    dc = seg // 2  # fftshift puts DC at center; blank its group
    for g in range(usable):
        b0, b1 = lo + g * group, lo + (g + 1) * group
        if b0 <= dc < b1:
            gm_db[g] = -np.inf
    k = int(np.argmax(gm_db))
    best_db = float(gm_db[k])
    if best_db < floor + margin_db:
        return None
    center_bin = lo + k * group + group // 2
    best_hz = int(round((center_bin - seg // 2) * per_bin))
    return floor, best_hz, best_db


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uri", default="usb:")
    ap.add_argument("--freq", type=int, default=2_620_000_000,
                    help="single channel for one-shot dwells (walk_log)")
    ap.add_argument("--samplerate", type=float, default=20e6)
    ap.add_argument("--gain", type=float, default=30)
    ap.add_argument("--dwell", type=float, default=2.0)
    ap.add_argument("--channels", default="2620000000,1750000000,850000000")
    ap.add_argument("--margin-db", type=float, default=8.0)
    ap.add_argument("--out", default="rf_hits.jsonl")
    ap.add_argument("--once", action="store_true",
                    help="single dwell on --freq, print result, exit")
    args = ap.parse_args()

    channels = [int(c) for c in args.channels.split(",") if c.strip()]
    phy, lpc = open_device(args.uri, channels[0], args.samplerate,
                           args.samplerate, args.gain)

    def one_shot(freq, seconds):
        phy.find_channel("altvoltage0", True).attrs["frequency"].value = str(int(freq))
        time.sleep(0.05)
        iq = dwell(lpc, args.samplerate, seconds)
        return analyze(iq, args.samplerate, args.margin_db)

    if args.once:
        res = one_shot(args.freq, max(0.5, args.dwell))
        if res:
            floor, hz, db = res
            line = json.dumps({"t": round(time.time(), 3), "hz": hz,
                               "power_db": round(db, 2), "pose": None})
            print(line, flush=True)
        return 0

    timeouts = 0
    try:
        with open(args.out, "a") as out:
            while True:
                for hz0 in channels:
                    try:
                        res = one_shot(hz0, args.dwell)
                    except Exception:
                        timeouts += 1
                        if timeouts >= 2:
                            print("ERROR: iio read timeout", file=sys.stderr)
                            return 1
                        continue
                    timeouts = 0
                    if res is None:
                        continue
                    floor, hz, db = res
                    line = json.dumps({"t": round(time.time(), 3),
                                       "hz": int(hz0 + hz),
                                       "power_db": round(db, 2),
                                       "pose": None})
                    out.write(line + "\n")
                    out.flush()
                    print(line, flush=True)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
