"""Find a strong LoRa chirp in a band and measure its power.

LoRa's preamble is a repeated upchirp. Mixing the capture with the matching
downchirp collapses that preamble to a tone. A carrier or noise does not.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

SAMPLE_RATE = 2_048_000
# The beacon sends one packet a second. Dwell long enough to catch it,
# and hold the peak past the gap so a mark still sees that packet.
DWELL_S = 1.15
LISTEN_S = 1.25
BURST_HOLD_S = 1.4
TARGET_HZ = 915_000_000
# Keep the signal off the RTL-SDR's DC spur.
TUNE_OFFSET = 250_000
US915 = (902_000_000, 928_000_000)
EU868 = (863_000_000, 870_000_000)
CONFIRM_SCORE = 40.0
BANDWIDTHS = (125_000.0, 250_000.0, 500_000.0)
SPREAD_FACTORS = (7, 8, 9, 10, 11, 12)


@dataclass(frozen=True)
class Blob:
    offset_hz: float
    width_hz: float
    power_dbfs: float
    excess_db: float


@dataclass(frozen=True)
class Hit:
    hz: float
    bw_hz: float
    sf: int | None
    power_dbfs: float
    score: float
    kind: str


def symbol_length(fs: float, bw: float, sf: int) -> int:
    return int(round(fs * (2**sf) / bw))


def symbol_chirp(fs: float, bw: float, sf: int) -> np.ndarray:
    sym = symbol_length(fs, bw, sf)
    tau = np.arange(sym, dtype=np.float64) / fs
    rate = (bw * bw) / (2**sf)
    phase = 2.0 * np.pi * ((-bw / 2.0) * tau + 0.5 * rate * tau * tau)
    return np.exp(1j * phase).astype(np.complex64)


def synthesize(
    n: int,
    fs: float,
    bw: float,
    sf: int,
    amp: float,
    f_offset_hz: float = 0.0,
) -> np.ndarray:
    chirp = symbol_chirp(fs, bw, sf)
    if f_offset_hz:
        t = np.arange(len(chirp), dtype=np.float64) / fs
        tone = np.exp(1j * 2.0 * np.pi * f_offset_hz * t).astype(np.complex64)
        chirp = chirp * tone
    reps = int(np.ceil(n / len(chirp)))
    return (amp * np.tile(chirp, reps)[:n]).astype(np.complex64)


def dechirp_score(iq: np.ndarray, fs: float, bw: float, sf: int) -> float:
    sym = symbol_length(fs, bw, sf)
    if sym < 32 or len(iq) < sym * 4:
        return 0.0
    down = np.conj(symbol_chirp(fs, bw, sf))
    window = np.hanning(sym).astype(np.float32)
    step = max(1, sym // 4)
    best = 0.0
    limit = len(iq) - sym * 3
    for off in range(0, sym, step):
        if off > limit:
            break
        scores = []
        base = iq[off : off + sym * 3]
        for k in range(3):
            seg = base[k * sym : (k + 1) * sym]
            mag = np.abs(np.fft.fft(seg * down * window))
            scores.append(float(mag.max() / (mag.mean() + 1e-12)))
        best = max(best, sum(scores) / len(scores))
    return best


def best_dechirp(
    iq: np.ndarray, fs: float, measured_hz: float | None = None
) -> tuple[int, float, float]:
    bandwidths = BANDWIDTHS
    if measured_hz is not None:
        ordered = sorted(BANDWIDTHS, key=lambda bw: abs(bw - measured_hz))
        bandwidths = [ordered[0]]
        if abs(ordered[1] - measured_hz) <= abs(ordered[0] - measured_hz) * 1.5 + 20_000:
            bandwidths.append(ordered[1])
    winner = (7, float(bandwidths[0]), 0.0)
    for bw in bandwidths:
        for sf in SPREAD_FACTORS:
            score = dechirp_score(iq, fs, bw, sf)
            if score > winner[2]:
                winner = (sf, float(bw), score)
    return winner


def _periodogram(iq: np.ndarray, fs: float, nfft: int = 8192):
    nseg = max(1, min(len(iq) // nfft, 8))
    window = np.hanning(nfft)
    scale = float(np.sum(window) ** 2)
    acc = np.zeros(nfft, dtype=np.float64)
    for i in range(nseg):
        seg = iq[i * nfft : (i + 1) * nfft]
        spec = np.fft.fftshift(np.fft.fft(seg * window))
        acc += (np.abs(spec) ** 2) / scale
    acc /= nseg
    freqs = np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0 / fs))
    db = 10.0 * np.log10(acc + 1e-20)
    return freqs, db, acc


def _regions(mask: np.ndarray) -> list[np.ndarray]:
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    groups = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
    merged: list[np.ndarray] = [groups[0]]
    for group in groups[1:]:
        if group[0] - merged[-1][-1] <= 8:
            merged[-1] = np.arange(merged[-1][0], group[-1] + 1)
        else:
            merged.append(group)
    return merged


def find_blobs(iq: np.ndarray, fs: float) -> list[Blob]:
    freqs, db, ps = _periodogram(iq, fs)
    df = float(freqs[1] - freqs[0])
    leveled = db.copy()
    dc = np.abs(freqs) < 12_000.0
    if np.any(dc):
        near = np.abs(freqs) < 80_000.0
        leveled[dc] = np.median(db[near]) if np.count_nonzero(near) > np.count_nonzero(dc) else np.median(db)
    floor = float(np.median(leveled))

    def collect(excess: float, lo: float, hi: float) -> list[Blob]:
        blobs: list[Blob] = []
        for group in _regions(leveled >= floor + excess):
            width = float(group[-1] - group[0]) * df + df
            if width < lo or width > hi:
                continue
            sl = slice(int(group[0]), int(group[-1]) + 1)
            weight = ps[sl]
            total = float(np.sum(weight))
            if total <= 0.0:
                continue
            offset = float(np.sum(freqs[sl] * weight) / total)
            power = float(10.0 * np.log10(total + 1e-20))
            excess_db = float(np.mean(leveled[sl]) - floor)
            blobs.append(Blob(offset, width, power, excess_db))
        blobs.sort(key=lambda blob: blob.power_dbfs, reverse=True)
        return blobs

    for excess in (8.0, 14.0, 22.0, 30.0):
        wide = collect(excess, 60_000.0, 700_000.0)
        if wide:
            return wide
    return collect(12.0, 2_000.0, 60_000.0)


def _matching_blob(blobs: list[Blob], bw: float) -> Blob | None:
    wide = [blob for blob in blobs if blob.width_hz >= 60_000.0]
    if not wide:
        return blobs[0] if blobs else None
    return min(wide, key=lambda blob: abs(blob.width_hz - bw))


def loudest_slice(iq: np.ndarray, fs: float, span_s: float = 0.4) -> np.ndarray:
    """Capture from the start of the loud packet.

    The chirp check only looks at the beginning of what it is given, so this
    slice has to start on the packet, not on the quiet gap in front of it.
    """
    chunk_s = 0.008
    chunk = max(256, int(fs * chunk_s))
    n = len(iq) // chunk
    span = max(3, int(round(span_s / chunk_s)))
    if n < span:
        return iq
    power = np.mean(np.abs(iq[: n * chunk].reshape(n, chunk)) ** 2, axis=1)
    peak = int(np.argmax(power))
    floor = float(power[peak]) / (10 ** 0.6)  # within 6 dB of the peak
    onset = peak
    while onset > 0 and power[onset - 1] >= floor:
        onset -= 1
    start = max(0, onset - 1)
    stop = min(n, start + span)
    return iq[start * chunk : stop * chunk]


def burst_excess_db(iq: np.ndarray, fs: float) -> tuple[float, float]:
    """When the loudest 8 ms chunk happened, and how far it sits above the median."""
    chunk_s = 0.008
    chunk = max(256, int(fs * chunk_s))
    n = len(iq) // chunk
    if n < 2:
        return 0.0, 0.0
    power = np.mean(np.abs(iq[: n * chunk].reshape(n, chunk)) ** 2, axis=1)
    db = 10.0 * np.log10(power + 1e-20)
    peak = int(np.argmax(db))
    return peak * chunk_s, float(db[peak] - np.median(db))


def analyze(iq: np.ndarray, fs: float, center_hz: float) -> Hit | None:
    view = loudest_slice(iq, fs)
    blobs = find_blobs(view, fs)
    if not blobs:
        return None
    wide = [blob for blob in blobs if blob.width_hz >= 40_000.0]
    if not wide:
        blob = blobs[0]
        return Hit(float(center_hz) + blob.offset_hz, blob.width_hz, None, blob.power_dbfs, 0.0, "tone")
    measured = max(blob.width_hz for blob in wide)
    sf, bw, score = best_dechirp(view, fs, measured)
    blob = _matching_blob(wide, bw)
    if blob is None:
        return None
    hz = float(center_hz) + blob.offset_hz
    if score >= CONFIRM_SCORE:
        return Hit(hz, float(bw), int(sf), blob.power_dbfs, score, "lora")
    return Hit(hz, blob.width_hz, None, blob.power_dbfs, score, "energy")


def describe(hit: Hit | None) -> str:
    if hit is None:
        return "No signal above the noise floor."
    mhz = hit.hz / 1e6
    if hit.kind == "lora":
        return (
            f"LoRa {mhz:.3f} MHz   SF{hit.sf}   {hit.bw_hz / 1e3:.0f} kHz"
            f"   {hit.power_dbfs:.1f} dBFS"
        )
    if hit.kind == "tone":
        return f"Tone {mhz:.3f} MHz   {hit.power_dbfs:.1f} dBFS   (not a LoRa chirp)"
    return (
        f"Energy {mhz:.3f} MHz   {hit.bw_hz / 1e3:.0f} kHz"
        f"   {hit.power_dbfs:.1f} dBFS   (chirp not confirmed)"
    )


def hop_centers(f0: float, f1: float, fs: float, overlap: float = 400_000.0) -> list[int]:
    step = fs - overlap
    guard = fs / 2.0
    centers: list[int] = []
    c = f0 + guard
    while c - guard < f1:
        centers.append(int(round(c)))
        c += step
    if not centers:
        centers.append(int(round((f0 + f1) / 2.0)))
    if centers[-1] + guard < f1:
        centers.append(int(round(f1 - guard)))
    return centers


def channel_power_dbfs(
    iq: np.ndarray, fs: float, bw_hz: float, offset_hz: float = 0.0
) -> tuple[float, np.ndarray]:
    nfft = 4096 if len(iq) >= 4096 else len(iq)
    seg = iq[:nfft]
    window = np.hanning(nfft)
    spec = np.fft.fftshift(np.fft.fft(seg * window))
    scale = float(np.sum(window) ** 2)
    ps = (np.abs(spec) ** 2) / scale
    freqs = np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0 / fs))
    half = max(bw_hz, 20_000.0) / 2.0
    used = ps[np.abs(freqs - offset_hz) <= half]
    power = float(10.0 * np.log10(float(np.sum(used)) + 1e-20))
    db = 10.0 * np.log10(ps + 1e-20)
    return power, db


def peak_channel_power_dbfs(
    iq: np.ndarray, fs: float, bw_hz: float, offset_hz: float = 0.0, block: int = 8192
) -> float:
    """Loudest short block. A one-second listen is mostly the gap between packets."""
    if len(iq) < 256:
        power, _ = channel_power_dbfs(iq, fs, bw_hz, offset_hz)
        return power
    block = min(block, len(iq))
    best = -200.0
    for start in range(0, len(iq) - block + 1, block):
        power, _ = channel_power_dbfs(iq[start : start + block], fs, bw_hz, offset_hz)
        if power > best:
            best = power
    return best


def survey(radio, band: tuple[int, int], stop=None, status=None) -> Hit | None:
    """Sweep a LoRa band and lock the strongest confirmed chirp."""
    f0, f1 = band
    radio.set_sample_rate(SAMPLE_RATE)
    gain = 20.7
    best: Hit | None = None
    raised = False
    for _attempt in range(4):
        if stop is not None and stop.is_set():
            return None
        radio.set_gain_db(gain)
        centers = hop_centers(f0, f1, SAMPLE_RATE)
        candidates: list[Hit] = []
        clipped = False
        for index, center in enumerate(centers):
            if stop is not None and stop.is_set():
                return None
            if status is not None:
                status(f"Search {index + 1}/{len(centers)} at {center / 1e6:.3f} MHz, gain {radio.gain_db:.1f} dB")
            radio.set_center_freq(center)
            if getattr(radio, "needs_settle", False):
                time.sleep(0.02)
                radio.read(8192)
            iq = radio.read(int(radio.sample_rate * DWELL_S))
            if getattr(radio, "last_rail", 0.0) > 0.005:
                clipped = True
                break
            hit = analyze(iq, radio.sample_rate, center)
            if hit is None or hit.kind == "tone":
                continue
            if f0 - 100_000 <= hit.hz <= f1 + 100_000:
                candidates.append(hit)
        if clipped and gain > 1.0:
            gain = max(0.0, gain - 10.0)
            best = None
            continue
        confirmed = [hit for hit in candidates if hit.kind == "lora"]
        pool = confirmed or candidates
        if pool:
            best = max(pool, key=lambda hit: (hit.score, hit.power_dbfs))
            break
        if not raised:
            gain = 40.2
            raised = True
            continue
        break
    return best
