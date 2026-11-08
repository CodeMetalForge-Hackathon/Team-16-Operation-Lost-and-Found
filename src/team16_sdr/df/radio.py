"""Receive-only RTL-SDR access, plus a synthetic radio for the demo.

The NESDR cannot transmit. This module never turns on the bias tee.
"""

from __future__ import annotations

import math
import os
import urllib.request
import zipfile
from pathlib import Path

import numpy as np

from .lora import SAMPLE_RATE, symbol_chirp

RUNTIME = Path(os.environ.get("LOCALAPPDATA", ".")) / "team16-df" / "rtl-sdr"
LIBRARY_URL = (
    "https://downloads.osmocom.org/binaries/windows/rtl-sdr/rtl-sdr-64bit-20261018.zip"
)
# R820T2 list used until the dongle reports its own steps.
FALLBACK_GAINS = (
    0.0, 0.9, 1.4, 2.7, 3.7, 7.7, 8.7, 12.5, 14.4, 15.7, 16.6, 19.7, 20.7,
    22.9, 25.4, 28.0, 29.7, 32.8, 33.8, 36.4, 37.2, 38.6, 40.2, 42.1, 43.4,
    43.9, 44.5, 48.0, 49.6,
)


class RadioError(RuntimeError):
    pass


def bytes_to_iq(raw: np.ndarray) -> np.ndarray:
    values = raw.astype(np.float32)
    values -= 127.5
    values *= 1.0 / 128.0
    return values[0::2] + 1j * values[1::2]


def _angdiff(a_deg: float, b_deg: float) -> float:
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


class DemoRadio:
    """Synthetic strong LoRa beacon with a cardioid antenna pattern.

    True bearing is 60 degrees. Amplitude follows the heading the UI sets.
    """

    def __init__(self) -> None:
        self.name = "demo LoRa, true bearing 60 deg"
        self.needs_settle = False
        self.sample_rate = SAMPLE_RATE
        self.center = 915_000_000
        self.gain_db = 20.7
        self.gains = list(FALLBACK_GAINS)
        self.gain_index = self.gains.index(20.7)
        self.last_rail = 0.0
        self.heading = 0.0
        self.signal_hz = 915_000_000
        self.bw = 125_000.0
        self.sf = 9
        self.true_bearing = 60.0
        self._chirp_key = None
        self._chirp = None
        self._rng = np.random.default_rng(4)

    def open(self) -> DemoRadio:
        return self

    def close(self) -> None:
        return None

    def set_sample_rate(self, hz: int) -> None:
        self.sample_rate = int(hz)
        self._chirp_key = None

    def set_center_freq(self, hz: int) -> None:
        self.center = int(hz)

    def set_gain_db(self, db: float) -> None:
        self.gain_db = min(self.gains, key=lambda gain: abs(gain - db))
        self.gain_index = self.gains.index(self.gain_db)

    def bump_gain(self, steps: int) -> None:
        self.gain_index = min(max(self.gain_index + steps, 0), len(self.gains) - 1)
        self.gain_db = self.gains[self.gain_index]

    def _unit_chirp(self, n: int) -> np.ndarray:
        key = (self.sample_rate, self.bw, self.sf, n, self.center)
        if key == self._chirp_key and self._chirp is not None:
            return self._chirp
        offset = self.signal_hz - self.center
        if abs(offset) > self.sample_rate / 2:
            wave = np.zeros(n, dtype=np.complex64)
        else:
            wave = synthesize_at(n, self.sample_rate, self.bw, self.sf, offset)
        self._chirp_key = key
        self._chirp = wave
        return wave

    def read(self, nsamp: int) -> np.ndarray:
        err = math.radians(_angdiff(self.heading, self.true_bearing))
        cardioid = 0.5 + 0.5 * math.cos(err)
        amp = 0.4 * cardioid
        noise = self._rng.standard_normal(nsamp) + 1j * self._rng.standard_normal(nsamp)
        iq = amp * self._unit_chirp(nsamp) + (0.02 * noise).astype(np.complex64)
        self.last_rail = 0.0
        return iq.astype(np.complex64)


def synthesize_at(n: int, fs: float, bw: float, sf: int, f_offset_hz: float) -> np.ndarray:
    chirp = symbol_chirp(fs, bw, sf)
    if f_offset_hz:
        t = np.arange(len(chirp), dtype=np.float64) / fs
        chirp = chirp * np.exp(1j * 2.0 * np.pi * f_offset_hz * t).astype(np.complex64)
    reps = int(np.ceil(n / len(chirp)))
    return np.tile(chirp, reps)[:n]


def ensure_library() -> Path:
    if os.name != "nt":
        raise RadioError("This build opens the NESDR with the Windows rtl-sdr library.")
    existing = list(RUNTIME.rglob("librtlsdr.dll"))
    if existing:
        return existing[0].parent
    RUNTIME.mkdir(parents=True, exist_ok=True)
    archive = RUNTIME / "rtl-sdr.zip"
    try:
        urllib.request.urlretrieve(LIBRARY_URL, archive)
    except OSError as exc:
        raise RadioError(f"Could not download rtl-sdr ({exc}).") from exc
    with zipfile.ZipFile(archive) as bundle:
        bundle.extractall(RUNTIME)
    found = list(RUNTIME.rglob("librtlsdr.dll"))
    if not found:
        raise RadioError("The rtl-sdr archive did not contain librtlsdr.dll.")
    return found[0].parent


class RtlRadio:
    def __init__(self) -> None:
        self.name = "NESDR"
        self.needs_settle = True
        self.sample_rate = SAMPLE_RATE
        self.center = 915_000_000
        self.gain_db = 20.7
        self.gains = list(FALLBACK_GAINS)
        self.gain_index = 0
        self.last_rail = 0.0
        self.heading = 0.0
        self._lib = None
        self._dev = None
        self._folder: Path | None = None

    def open(self) -> RtlRadio:
        import ctypes

        folder = ensure_library()
        self._folder = folder
        if hasattr(os, "add_dll_directory"):
            os.add_dll_directory(str(folder))
        lib = ctypes.CDLL(str(folder / "librtlsdr.dll"))
        self._bind(lib)
        count = lib.rtlsdr_get_device_count()
        if count < 1:
            raise RadioError(
                "No RTL-SDR opened. Install the WinUSB driver for USB ID 0BDA:2838, then reopen."
            )
        dev = ctypes.c_void_p()
        rc = lib.rtlsdr_open(ctypes.byref(dev), 0)
        if rc < 0 or not dev.value:
            raise RadioError(f"rtl-sdr open failed ({rc}). Install WinUSB for USB ID 0BDA:2838.")
        self._lib = lib
        self._dev = dev
        name = lib.rtlsdr_get_device_name(0)
        if name:
            self.name = name.decode("utf-8", "replace")
        self._disable_bias_tee()
        self._load_gains()
        lib.rtlsdr_set_tuner_gain_mode(self._dev, 1)
        lib.rtlsdr_set_agc_mode(self._dev, 0)
        self.set_sample_rate(SAMPLE_RATE)
        self.set_gain_db(20.7)
        self.set_center_freq(915_000_000)
        return self

    def _bind(self, lib) -> None:
        import ctypes

        lib.rtlsdr_get_device_count.restype = ctypes.c_uint32
        lib.rtlsdr_get_device_name.argtypes = [ctypes.c_uint32]
        lib.rtlsdr_get_device_name.restype = ctypes.c_char_p
        lib.rtlsdr_open.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint32]
        lib.rtlsdr_open.restype = ctypes.c_int
        lib.rtlsdr_close.argtypes = [ctypes.c_void_p]
        lib.rtlsdr_close.restype = ctypes.c_int
        lib.rtlsdr_set_sample_rate.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        lib.rtlsdr_set_sample_rate.restype = ctypes.c_int
        lib.rtlsdr_set_center_freq.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        lib.rtlsdr_set_center_freq.restype = ctypes.c_int
        lib.rtlsdr_set_tuner_gain_mode.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.rtlsdr_set_tuner_gain_mode.restype = ctypes.c_int
        lib.rtlsdr_set_agc_mode.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.rtlsdr_set_agc_mode.restype = ctypes.c_int
        lib.rtlsdr_set_tuner_gain.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.rtlsdr_set_tuner_gain.restype = ctypes.c_int
        lib.rtlsdr_get_tuner_gains.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
        lib.rtlsdr_get_tuner_gains.restype = ctypes.c_int
        lib.rtlsdr_reset_buffer.argtypes = [ctypes.c_void_p]
        lib.rtlsdr_reset_buffer.restype = ctypes.c_int
        lib.rtlsdr_read_sync.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
        ]
        lib.rtlsdr_read_sync.restype = ctypes.c_int
        if hasattr(lib, "rtlsdr_set_bias_tee"):
            lib.rtlsdr_set_bias_tee.argtypes = [ctypes.c_void_p, ctypes.c_int]
            lib.rtlsdr_set_bias_tee.restype = ctypes.c_int

    def _disable_bias_tee(self) -> None:
        if hasattr(self._lib, "rtlsdr_set_bias_tee"):
            self._lib.rtlsdr_set_bias_tee(self._dev, 0)

    def _load_gains(self) -> None:
        import ctypes

        count = self._lib.rtlsdr_get_tuner_gains(self._dev, None)
        if count <= 0:
            return
        arr = (ctypes.c_int * count)()
        got = self._lib.rtlsdr_get_tuner_gains(self._dev, arr)
        if got <= 0:
            return
        self.gains = [arr[i] / 10.0 for i in range(got)]

    def close(self) -> None:
        if self._lib is not None and self._dev is not None:
            self._lib.rtlsdr_close(self._dev)
        self._dev = None

    def set_sample_rate(self, hz: int) -> None:
        rc = self._lib.rtlsdr_set_sample_rate(self._dev, int(hz))
        if rc < 0:
            raise RadioError(f"sample rate {hz} was rejected ({rc}).")
        self.sample_rate = int(hz)
        self._lib.rtlsdr_reset_buffer(self._dev)

    def set_center_freq(self, hz: int) -> None:
        rc = self._lib.rtlsdr_set_center_freq(self._dev, int(hz))
        if rc < 0:
            raise RadioError(f"tune to {hz} Hz failed ({rc}).")
        self.center = int(hz)
        self._lib.rtlsdr_reset_buffer(self._dev)

    def set_gain_db(self, db: float) -> None:
        best = min(self.gains, key=lambda gain: abs(gain - db))
        rc = self._lib.rtlsdr_set_tuner_gain(self._dev, int(round(best * 10)))
        if rc < 0:
            raise RadioError(f"gain {best} dB was rejected ({rc}).")
        self.gain_db = best
        self.gain_index = self.gains.index(best)

    def bump_gain(self, steps: int) -> None:
        index = min(max(self.gain_index + steps, 0), len(self.gains) - 1)
        self.set_gain_db(self.gains[index])

    def read(self, nsamp: int) -> np.ndarray:
        import ctypes

        need = int(nsamp) * 2
        raw = np.empty(need, dtype=np.uint8)
        got = 0
        while got < need:
            chunk = min(256 * 1024, need - got)
            buf = (ctypes.c_uint8 * chunk)()
            nread = ctypes.c_int()
            rc = self._lib.rtlsdr_read_sync(self._dev, buf, chunk, ctypes.byref(nread))
            if rc < 0 or nread.value <= 0:
                raise RadioError(f"read failed ({rc}).")
            take = min(nread.value, chunk)
            raw[got : got + take] = np.ctypeslib.as_array(buf)[:take]
            got += take
        self.last_rail = float(np.mean((raw <= 1) | (raw >= 254)))
        return bytes_to_iq(raw)
