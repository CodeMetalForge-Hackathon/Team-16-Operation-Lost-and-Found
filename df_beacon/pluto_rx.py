#!/usr/bin/env python3
"""PlutoSDR receive-only monitor for the DF beacon's LoRa channel (915 MHz by default).

Tunes the Pluto's receiver, measures the power inside the beacon's 125 kHz channel several times a
second, and sends one reading per interval to the dashboard's beacon feed. It never transmits: the
Pluto's TX attenuation is set to maximum as a precaution and no TX buffer is ever created.

  python3 pluto_rx.py                                  # Pluto on its default USB address, post to the local feed
  python3 pluto_rx.py --uri ip:192.168.2.1 --sink post:http://192.168.8.163:8083/ingest   # run from a PC
  python3 pluto_rx.py --sink file:/home/hacker/team16_beacon/sdr.jsonl                     # for beacon_feed --sdr-file
  python3 pluto_rx.py --selftest                       # check the maths on synthetic data, no hardware

Readings are RELATIVE power (dBFS plus --cal-db). Calibrate --cal-db against a known signal if you
need true dBm; for finding the beacon only the change in level matters.

The LoRa beacon is a short burst (about 0.3 s of every second), so the monitor takes many short
captures and the dashboard shows the peak of the last couple of seconds.
"""
import argparse
import json
import sys
import time
import urllib.request

import numpy as np

FULL_SCALE = 2048.0  # Pluto samples are 12-bit


def channel_power_dbfs(iq, fs, centre_off_hz, bw_hz):
    """Power (dBFS) inside [centre_off +/- bw/2] and the per-bin noise floor outside it (dBFS per channel)."""
    x = iq - iq.mean()
    n = len(x)
    w = np.hanning(n)
    spec = np.fft.fftshift(np.fft.fft(x * w))
    p = (np.abs(spec) ** 2) / (np.sum(w ** 2) * n)  # sums to the mean power of x
    f = np.fft.fftshift(np.fft.fftfreq(n, 1.0 / fs))
    inband = np.abs(f - centre_off_hz) <= bw_hz / 2
    away = np.abs(f - centre_off_hz) > 2 * bw_hz
    away &= np.abs(f) > 20e3  # skip the DC spike
    sig = p[inband].sum()
    floor_bin = np.median(p[away]) / np.log(2)  # median of noise power is ln2 x its mean
    floor_chan = floor_bin * inband.sum() * 1.0
    return 10 * np.log10(sig + 1e-20), 10 * np.log10(floor_chan + 1e-20)


class Sink:
    def __init__(self, spec):
        kind, _, target = spec.partition(":")
        if kind not in ("post", "file") or not target:
            raise SystemExit("--sink must be post:URL or file:PATH")
        self.kind, self.target, self.warned = kind, target, False

    def send(self, t, freq_hz, power):
        rec = {"t": t, "freq_hz": freq_hz, "power_dbm": round(float(power), 2), "unit": "dBFS"}
        try:
            if self.kind == "file":
                with open(self.target, "a") as f:
                    f.write(json.dumps(rec) + "\n")
            else:
                rec["type"] = "sdr"
                req = urllib.request.Request(self.target, json.dumps(rec).encode(),
                                             {"Content-Type": "application/json"})
                urllib.request.urlopen(req, timeout=2).read()
            self.warned = False
        except Exception as exc:
            if not self.warned:
                print(f"sink error: {exc}", flush=True)
                self.warned = True


def selftest():
    rng = np.random.default_rng(1)
    fs, n, off, bw = 1e6, 65536, -250e3, 125e3
    noise = lambda a: a * (rng.standard_normal(n) + 1j * rng.standard_normal(n)) / np.sqrt(2)
    t = np.arange(n) / fs
    # LoRa-like up-chirp sweeping the 125 kHz channel repeatedly (8.2 ms symbol at SF10)
    sym = 1024 / bw
    k = bw / sym
    ph = 2 * np.pi * (off - bw / 2) * t + np.pi * k * (t % sym) ** 2
    chirp = np.exp(1j * ph)
    quiet, floor_q = channel_power_dbfs(noise(0.01), fs, off, bw)
    print(f"noise only        : channel {quiet:6.1f} dBFS, floor estimate {floor_q:6.1f}")
    ok = True
    for amp_db in (-3, 0, 6, 12):  # chirp power relative to the channel noise power
        chan_noise = 10 ** (quiet / 20)
        sig = chirp * chan_noise * 10 ** (amp_db / 20)
        p, fl = channel_power_dbfs(noise(0.01) + sig, fs, off, bw)
        exp = 10 * np.log10(10 ** (quiet / 10) + 10 ** ((quiet + amp_db) / 10))
        good = abs(p - exp) < 1.0 and abs(fl - floor_q) < 1.5
        ok &= good
        print(f"chirp {amp_db:+3d} dB over  : channel {p:6.1f} dBFS (expected {exp:6.1f}), floor {fl:6.1f}  {'ok' if good else 'MISMATCH'}")
    off_channel = np.exp(2j * np.pi * (+300e3) * t) * 0.05  # strong tone far from the beacon channel
    p, _ = channel_power_dbfs(noise(0.01) + off_channel, fs, off, bw)
    good = abs(p - quiet) < 1.0
    ok &= good
    print(f"tone elsewhere     : channel {p:6.1f} dBFS (should stay near {quiet:.1f})  {'ok' if good else 'MISMATCH'}")
    print("SELFTEST PASSED" if ok else "SELFTEST FAILED")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uri", default="usb:", help='libiio URI: "usb:" (auto), "ip:192.168.2.1", ...')
    ap.add_argument("--freq", type=float, default=915e6, help="beacon frequency, Hz")
    ap.add_argument("--offset", type=float, default=250e3, help="tune this far ABOVE the beacon, Hz (keeps it off the DC spike)")
    ap.add_argument("--rate", type=float, default=2.4e6, help="sample rate, Hz (AD9363 minimum is about 2.08e6)")
    ap.add_argument("--bw", type=float, default=125e3, help="beacon channel bandwidth, Hz")
    ap.add_argument("--gain", type=float, default=40.0, help="receiver gain, dB (0-70). Lower it if readings reach 0 dBFS (clipping)")
    ap.add_argument("--samples", type=int, default=131072, help="samples per capture")
    ap.add_argument("--interval", type=float, default=0.25, help="seconds between readings")
    ap.add_argument("--cal-db", type=float, default=0.0, help="added to dBFS (calibration offset)")
    ap.add_argument("--sink", default="post:http://127.0.0.1:8083/ingest")
    ap.add_argument("--selftest", action="store_true", help="test the maths on synthetic data and exit")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())

    import iio
    ctx = iio.Context(a.uri)
    phy = ctx.find_device("ad9361-phy")
    rxdev = ctx.find_device("cf-ad9361-lpc")
    if phy is None or rxdev is None:
        raise SystemExit("No PlutoSDR found at " + a.uri)
    phy.find_channel("altvoltage0", True).attrs["frequency"].value = str(int(a.freq + a.offset))  # RX LO
    rx = phy.find_channel("voltage0", False)
    rx.attrs["sampling_frequency"].value = str(int(a.rate))
    rx.attrs["rf_bandwidth"].value = str(int(min(a.rate * 0.9, 2e6)))
    rx.attrs["gain_control_mode"].value = "manual"
    rx.attrs["hardwaregain"].value = str(a.gain)
    try:  # receive-only: park the transmitter at maximum attenuation
        phy.find_channel("voltage0", True).attrs["hardwaregain"].value = "-89.75"
    except Exception:
        pass
    for name in ("voltage0", "voltage1"):
        rxdev.find_channel(name).enabled = True
    buf = iio.Buffer(rxdev, a.samples, False)
    sink = Sink(a.sink)
    print(f"PlutoSDR receiving: beacon {a.freq / 1e6:.3f} MHz (LO {(a.freq + a.offset) / 1e6:.3f}), "
          f"{a.rate / 1e6:.2f} MS/s, gain {a.gain} dB, sink {a.sink}", flush=True)
    last_print = 0.0
    while True:
        t0 = time.time()
        buf.refill()
        raw = np.frombuffer(buf.read(), dtype=np.int16).astype(np.float32) / FULL_SCALE
        iq = raw[0::2] + 1j * raw[1::2]
        power, floor = channel_power_dbfs(iq, a.rate, -a.offset, a.bw)
        sink.send(t0, a.freq, power + a.cal_db)
        if t0 - last_print > 5:
            last_print = t0
            print(f"channel {power + a.cal_db:6.1f} dBFS   floor {floor + a.cal_db:6.1f}   (+{power - floor:4.1f} dB)", flush=True)
        time.sleep(max(0.0, a.interval - (time.time() - t0)))


if __name__ == "__main__":
    main()
