"""Direction finder for a strong LoRa radio at 915 MHz.

Aim a directional antenna, pause, and mark the heading it points. The peak
mark is the bearing. The dongle only receives.
"""

from __future__ import annotations

import json
import math
import queue
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import tkinter as tk

from .bearing import Bearing, RecentPeak, add_sample, bearing_report
from .lora import (
    BURST_HOLD_S,
    LISTEN_S,
    SAMPLE_RATE,
    TARGET_HZ,
    TUNE_OFFSET,
    US915,
    channel_power_dbfs,
    describe,
    peak_channel_power_dbfs,
    survey,
)
from .radio import DemoRadio, RadioError, RtlRadio

BG = "#1b1e24"
FG = "#e7e2d6"
DIM = "#9aa3ad"
ACCENT = "#e2a23a"
PANEL = "#262b33"
HOT = "#f05a28"
WF_W = 256
WF_H = 150
LOG_NAME = "df-log.jsonl"


def _lut() -> list[str]:
    colors = []
    for level in range(64):
        x = level / 63.0
        r = int(max(0, min(255, (1.6 * x - 0.55) * 255)))
        g = int(max(0, min(255, (1.4 * x - 0.15) * 255)))
        b = int(max(0, min(255, (0.35 + 0.9 * x) * max(0.0, 1.15 - x) * 255)))
        colors.append(f"#{r:02x}{g:02x}{b:02x}")
    return colors


COLORS = _lut()


class Engine(threading.Thread):
    def __init__(self, radio) -> None:
        super().__init__(daemon=True)
        self.radio = radio
        self.stop = threading.Event()
        self.cmd: queue.Queue = queue.Queue()
        self.lock = threading.Lock()
        self.snap: dict = {"status": "Opening the radio", "searching": False}
        self.hit = None
        self.bw = 200_000.0
        self.offset_hz = float(TUNE_OFFSET)
        self.recent = RecentPeak(BURST_HOLD_S)
        self.frame = 0

    def publish(self, **fields) -> None:
        with self.lock:
            self.snap.update(fields)

    def copy(self) -> dict:
        with self.lock:
            return dict(self.snap)

    def run(self) -> None:
        try:
            self.radio.open()
        except (RadioError, OSError) as exc:
            self.publish(status=str(exc), error=True, searching=False)
            return
        self.publish(status=f"Listening at 915.000 MHz on {self.radio.name}", error=False, gain=self.radio.gain_db)
        try:
            self.radio.set_sample_rate(SAMPLE_RATE)
            self.radio.set_gain_db(8.7)
            self._tune(TARGET_HZ)
            self._confirm_target()
            while not self.stop.is_set():
                self._drain()
                if self.stop.is_set():
                    break
                iq = self.radio.read(8192)
                self._publish_block(iq)
        except (RadioError, OSError) as exc:
            self.publish(status=str(exc), error=True, searching=False)
        finally:
            try:
                self.radio.close()
            except (RadioError, OSError):
                pass

    def _tune(self, hz: int) -> None:
        self.signal_hz = int(hz)
        self.radio.set_center_freq(int(hz) - TUNE_OFFSET)

    def _confirm_target(self) -> None:
        """The beacon packets once a second. Listen long enough to catch one."""
        self.publish(status="Waiting for a 915 MHz burst", searching=True)
        self._tune(TARGET_HZ)
        iq = None
        for _ in range(4):
            if self.stop.is_set():
                return
            if self.radio.needs_settle:
                time.sleep(0.02)
                self.radio.read(8192)
            iq = self.radio.read(int(self.radio.sample_rate * LISTEN_S))
            if self.radio.last_rail > 0.002 and self.radio.gain_index > 0:
                self.radio.bump_gain(-2)
                continue
            power = peak_channel_power_dbfs(iq, self.radio.sample_rate, 500_000.0, TUNE_OFFSET)
            if power < -55.0 and self.radio.gain_index < len(self.radio.gains) - 1:
                self.radio.bump_gain(2)
                continue
            break
        hit = None if iq is None else _listen_hit(iq, self.radio.sample_rate, self.radio.center)
        self.hit = hit
        if hit is not None and hit.kind == "lora":
            self.bw = hit.bw_hz
        elif hit is not None and hit.kind == "energy":
            self.bw = max(hit.bw_hz, 125_000.0)
        elif hit is not None:
            self.bw = max(hit.bw_hz, 25_000.0)
        else:
            self.bw = 500_000.0
        if hit is not None:
            self.offset_hz = float(hit.hz - self.radio.center)
            text = describe(hit)
        else:
            self.offset_hz = float(TUNE_OFFSET)
            text = "915.000 MHz is quiet."
        self.publish(
            status=text,
            searching=False,
            gain=self.radio.gain_db,
            hit_hz=None if hit is None else hit.hz,
        )

    def _drain(self) -> None:
        survey_band = None
        while True:
            try:
                cmd = self.cmd.get_nowait()
            except queue.Empty:
                break
            kind = cmd[0]
            if kind == "survey":
                survey_band = cmd[1]
            elif kind == "gain":
                self.radio.bump_gain(cmd[1])
                self.publish(gain=self.radio.gain_db)
            elif kind == "heading":
                self.radio.heading = cmd[1]
            elif kind == "retarget":
                self._confirm_target()
        if survey_band is not None:
            self.publish(searching=True, status="Searching the band")
            hit = survey(self.radio, survey_band, stop=self.stop, status=lambda text: self.publish(status=text, searching=True))
            self.hit = hit
            if hit is None:
                self._tune(TARGET_HZ)
                self.offset_hz = float(TUNE_OFFSET)
                self.bw = 500_000.0
                text = "No chirp in the band. Back on 915.000 MHz."
            else:
                self._tune(int(round(hit.hz)))
                self.offset_hz = float(hit.hz - self.radio.center)
                self.bw = hit.bw_hz if hit.kind == "lora" else max(hit.bw_hz, 125_000.0)
                text = describe(hit)
            self.publish(status=text, searching=False, gain=self.radio.gain_db, hit_hz=None if hit is None else hit.hz)

    def _publish_block(self, iq: np.ndarray) -> None:
        power, db = channel_power_dbfs(iq, self.radio.sample_rate, self.bw, self.offset_hz)
        row = db.reshape(WF_W, -1).max(axis=1).astype(np.float32)
        now = time.monotonic()
        peak = self.recent.push(now, power)
        self.frame += 1
        self.publish(
            row=row,
            power=power,
            recent_peak=peak,
            frame=self.frame,
            center=self.radio.center,
            fs=self.radio.sample_rate,
            gain=self.radio.gain_db,
            rail=self.radio.last_rail,
            name=self.radio.name,
        )


def _listen_hit(iq: np.ndarray, fs: float, center_hz: float):
    from .lora import analyze

    return analyze(iq, fs, center_hz)


class App(tk.Tk):
    def __init__(self, demo: bool = False) -> None:
        super().__init__()
        self.demo = demo
        self.title("LoRa direction finder")
        self.configure(bg=BG)
        self.geometry("980x740")
        self.minsize(860, 660)
        self.samples: list[tuple[float, float]] = []
        self._last_gain = None
        self._wf = np.full((WF_H, WF_W), -90.0, dtype=np.float32)
        self._drawn = -1
        self.log_path = Path.cwd() / LOG_NAME
        self.engine = Engine(DemoRadio() if demo else RtlRadio())
        self.stop_requested = False
        self._build()
        self.engine.start()
        self.after(50, self._tick)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build(self) -> None:
        title = tk.Frame(self, bg=BG)
        title.pack(fill="x", padx=16, pady=(12, 4))
        tk.Label(title, text="LoRa direction finder", bg=BG, fg=FG, font=("Segoe UI", 18, "bold")).pack(side="left")
        if self.demo:
            tk.Label(title, text="DEMO  true bearing 60 deg", bg=BG, fg=ACCENT, font=("Segoe UI", 12, "bold")).pack(side="left", padx=12)
        tk.Label(
            title,
            text="915 MHz  receive only",
            bg=BG,
            fg=DIM,
            font=("Segoe UI", 12),
        ).pack(side="right")

        self.status = tk.Label(self, text="Opening the radio", bg=BG, fg=FG, font=("Segoe UI", 13), anchor="w")
        self.status.pack(fill="x", padx=16, pady=(0, 8))

        controls = tk.Frame(self, bg=BG)
        controls.pack(fill="x", padx=16, pady=(0, 8))
        self._button(controls, "Check 915", self._retarget).pack(side="left")
        self._button(controls, "Search 902-928", lambda: self.engine.cmd.put(("survey", US915))).pack(side="left", padx=(8, 0))
        self._button(controls, "Gain -", lambda: self._bump(-1)).pack(side="left", padx=(16, 0))
        self._button(controls, "Gain +", lambda: self._bump(1)).pack(side="left", padx=(8, 0))
        self.gain_label = tk.Label(controls, text="gain", bg=BG, fg=DIM, font=("Segoe UI", 11))
        self.gain_label.pack(side="left", padx=8)
        if not self.demo:
            self._button(controls, "Demo", self._swap_demo).pack(side="right")

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=16)

        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="both", expand=True)
        self.photo = tk.PhotoImage(width=WF_W, height=WF_H)
        tk.Label(left, image=self.photo, bg=PANEL, bd=0).pack(anchor="w")
        scale = tk.Frame(left, bg=BG)
        scale.pack(fill="x", pady=(2, 0))
        self.f_lo = tk.Label(scale, text="914.0", bg=BG, fg=DIM, font=("Segoe UI", 9))
        self.f_lo.pack(side="left")
        self.f_mid = tk.Label(scale, text="915.0 MHz", bg=BG, fg=DIM, font=("Segoe UI", 9))
        self.f_mid.pack(side="left", expand=True)
        self.f_hi = tk.Label(scale, text="916.0", bg=BG, fg=DIM, font=("Segoe UI", 9))
        self.f_hi.pack(side="right")
        tk.Label(
            left,
            text="The beacon bursts once a second. Aim, wait for the number to jump, then mark that heading.",
            wraplength=640,
            bg=BG,
            fg=DIM,
            font=("Segoe UI", 11),
            anchor="w",
        ).pack(fill="x", pady=(10, 4))

        marks = tk.Frame(left, bg=BG)
        marks.pack(fill="x")
        for name, deg in (("N", 0), ("NE", 45), ("E", 90), ("SE", 135), ("S", 180), ("SW", 225), ("W", 270), ("NW", 315)):
            self._button(marks, name, lambda d=deg: self._mark(d)).pack(side="left", padx=(0, 4))
        fine = tk.Frame(left, bg=BG)
        fine.pack(fill="x", pady=(8, 0))
        tk.Label(fine, text="Degrees", bg=BG, fg=DIM, font=("Segoe UI", 11)).pack(side="left")
        self.degrees = tk.Spinbox(fine, from_=0, to=359, width=6, font=("Segoe UI", 12), justify="right")
        self.degrees.pack(side="left", padx=8)
        self._button(fine, "Mark", self._mark_spin).pack(side="left")
        self._button(fine, "Clear", self._clear).pack(side="left", padx=(8, 0))

        if self.demo:
            slider = tk.Frame(left, bg=BG)
            slider.pack(fill="x", pady=(10, 0))
            tk.Label(slider, text="Antenna heading", bg=BG, fg=DIM, font=("Segoe UI", 11)).pack(side="left")
            self.heading = tk.Scale(
                slider, from_=0, to=359, orient="horizontal", showvalue=True,
                bg=BG, fg=FG, highlightthickness=0, troughcolor=PANEL, length=280,
                command=self._heading,
            )
            self.heading.pack(side="left", padx=8)

        right = tk.Frame(body, bg=BG)
        right.pack(side="right", fill="y", padx=(16, 0))
        self.power_label = tk.Label(right, text="--", bg=BG, fg=ACCENT, font=("Segoe UI", 28, "bold"))
        self.power_label.pack(anchor="e")
        tk.Label(right, text="last burst dBFS", bg=BG, fg=DIM, font=("Segoe UI", 10)).pack(anchor="e")
        self.now_label = tk.Label(right, text="", bg=BG, fg=FG, font=("Segoe UI", 11))
        self.now_label.pack(anchor="e", pady=(2, 8))
        self.meter = tk.Canvas(right, width=220, height=18, bg=PANEL, highlightthickness=0)
        self.meter.pack(anchor="e")
        self.polar = tk.Canvas(right, width=260, height=260, bg=PANEL, highlightthickness=0)
        self.polar.pack(anchor="e", pady=(12, 8))
        self.bearing_label = tk.Label(right, text="No marks yet", bg=BG, fg=FG, font=("Segoe UI", 13), wraplength=260, justify="right")
        self.bearing_label.pack(anchor="e")
        self.sample_label = tk.Label(right, text="", bg=BG, fg=DIM, font=("Segoe UI", 10), wraplength=260, justify="right")
        self.sample_label.pack(anchor="e", pady=(4, 0))
        self._draw_polar(None)

    def _button(self, parent, text, command) -> tk.Button:
        return tk.Button(
            parent, text=text, command=command, bg=PANEL, fg=FG, activebackground=ACCENT,
            activeforeground="#1b1e24", relief="flat", padx=10, pady=4, font=("Segoe UI", 11),
        )

    def _heading(self, value) -> None:
        self.engine.cmd.put(("heading", float(value)))

    def _retarget(self) -> None:
        self.engine.cmd.put(("retarget",))

    def _bump(self, steps: int) -> None:
        self.engine.cmd.put(("gain", steps))

    def _swap_demo(self) -> None:
        self._close(reopen_demo=True)

    def _mark_spin(self) -> None:
        try:
            deg = float(self.degrees.get())
        except ValueError:
            return
        self._mark(deg)

    def _mark(self, deg: float) -> None:
        snap = self.engine.copy()
        power = snap.get("recent_peak")
        if power is None:
            return
        gain = snap.get("gain")
        if self.samples and self._last_gain is not None and gain != self._last_gain:
            self.samples.clear()
            self.status.configure(text="Gain changed. Marks cleared so the powers stay comparable.")
        self._last_gain = gain
        add_sample(self.samples, deg, float(power))
        report = bearing_report(self.samples)
        self._show_bearing(report)
        self._draw_polar(report)
        self._log(snap, deg, float(power), report)

    def _clear(self) -> None:
        self.samples.clear()
        self.bearing_label.configure(text="No marks yet")
        self.sample_label.configure(text="")
        self._draw_polar(None)

    def _show_bearing(self, report: Bearing | None) -> None:
        lines = ",  ".join(f"{deg:.0f} deg {power:.1f}" for deg, power in self.samples)
        self.sample_label.configure(text=lines)
        if report is None:
            if len(self.samples) < 2:
                self.bearing_label.configure(text="Marked. One more heading gives a side.")
            else:
                self.bearing_label.configure(text="Powers match. No bearing yet.")
            return
        if report.method == "two-sample":
            text = f"Stronger toward {report.deg:.0f} deg, by {report.contrast_db:.1f} dB"
        else:
            text = f"Bearing {report.deg:.0f} deg    contrast {report.contrast_db:.1f} dB"
        self.bearing_label.configure(text=text)

    def _log(self, snap, deg, power, report: Bearing | None) -> None:
        hit = self.engine.hit
        record = {
            "type": "df_sample",
            "t": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "hz": None if hit is None else int(round(hit.hz)),
            "center_hz": snap.get("center"),
            "bearing_deg": round(deg, 1),
            "power_db": round(power, 2),
            "gain_db": snap.get("gain"),
        }
        if report is not None:
            record["fix_deg"] = round(report.deg, 1)
            record["method"] = report.method
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")

    def _tick(self) -> None:
        snap = self.engine.copy()
        status = snap.get("status")
        if status:
            self.status.configure(text=status)
        gain = snap.get("gain")
        rail = snap.get("rail")
        if gain is not None:
            clip = ""
            if rail is not None and rail > 0.005:
                clip = "  CLIPPING"
            self.gain_label.configure(text=f"{gain:.1f} dB{clip}")
        peak = snap.get("recent_peak")
        power = snap.get("power")
        if peak is not None:
            self.power_label.configure(text=f"{peak:.1f}")
        if power is not None:
            self.now_label.configure(text=f"now {power:.1f} dBFS")
            self._draw_meter(peak if peak is not None else power)
        row = snap.get("row")
        frame = snap.get("frame", -1)
        if row is not None and frame != self._drawn:
            self._drawn = frame
            self._push_row(row)
            center = snap.get("center", TARGET_HZ)
            fs = snap.get("fs", SAMPLE_RATE)
            self.f_lo.configure(text=f"{(center - fs / 2) / 1e6:.3f}")
            self.f_mid.configure(text=f"signal {(center + TUNE_OFFSET) / 1e6:.3f} MHz")
            self.f_hi.configure(text=f"{(center + fs / 2) / 1e6:.3f}")
        if not self.stop_requested:
            self.after(50, self._tick)

    def _push_row(self, row: np.ndarray) -> None:
        self._wf[1:] = self._wf[:-1]
        self._wf[0] = row
        vmin = float(np.percentile(self._wf, 30))
        vmax = max(vmin + 8.0, float(np.percentile(self._wf, 99)))
        levels = np.clip(((self._wf - vmin) / (vmax - vmin) * 63.0).astype(np.int32), 0, 63)
        # One put of the whole image is simpler than 150 separate calls.
        rows = [" ".join(COLORS[int(v)] for v in line) for line in levels]
        self.photo.put("{" + "} {".join(rows) + "}")

    def _draw_meter(self, db: float) -> None:
        frac = max(0.0, min(1.0, (db + 80.0) / 80.0))
        self.meter.delete("all")
        self.meter.create_rectangle(0, 0, 220 * frac, 18, fill=ACCENT, width=0)

    def _draw_polar(self, report: Bearing | None) -> None:
        c = self.polar
        c.delete("all")
        cx, cy, radius = 130, 130, 100
        c.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#445", width=1)
        c.create_line(cx, cy - radius, cx, cy + radius, fill="#334")
        c.create_line(cx - radius, cy, cx + radius, cy, fill="#334")
        c.create_text(cx, cy - radius - 10, text="N", fill=DIM, font=("Segoe UI", 10))
        if not self.samples:
            return
        powers = [power for _, power in self.samples]
        lo, hi = min(powers), max(powers)
        span = max(hi - lo, 1.0)
        for deg, power in self.samples:
            r = 28 + 72 * (power - lo) / span
            rad = math.radians(deg)
            x = cx + r * math.sin(rad)
            y = cy - r * math.cos(rad)
            c.create_oval(x - 4, y - 4, x + 4, y + 4, fill=FG, width=0)
        if report is not None:
            rad = math.radians(report.deg)
            x = cx + radius * math.sin(rad)
            y = cy - radius * math.cos(rad)
            c.create_line(cx, cy, x, y, fill=HOT, width=3)

    def _close(self, reopen_demo: bool = False) -> None:
        self.stop_requested = True
        self.engine.stop.set()
        self.engine.join(timeout=2.0)
        self.destroy()
        if reopen_demo:
            App(demo=True).mainloop()


def run_check() -> int:
    radio = RtlRadio()
    try:
        radio.open()
    except (RadioError, OSError) as exc:
        print(exc)
        return 1
    try:
        radio.set_sample_rate(SAMPLE_RATE)
        radio.set_gain_db(8.7)
        radio.set_center_freq(TARGET_HZ - TUNE_OFFSET)
        if radio.needs_settle:
            time.sleep(0.05)
            radio.read(8192)
        iq = radio.read(int(SAMPLE_RATE * LISTEN_S))
        while radio.last_rail > 0.002 and radio.gain_index > 0:
            radio.bump_gain(-2)
            radio.read(8192)
            iq = radio.read(int(SAMPLE_RATE * LISTEN_S))
        power = peak_channel_power_dbfs(iq, radio.sample_rate, 500_000.0, TUNE_OFFSET)
        from .lora import analyze, burst_excess_db

        when_s, excess_db = burst_excess_db(iq, radio.sample_rate)
        print(f"device: {radio.name}")
        print(f"tune: {radio.center} Hz  signal: {TARGET_HZ} Hz  gain: {radio.gain_db:.1f} dB  rail: {radio.last_rail:.4f}")
        print(f"burst at {when_s:.3f} s in the listen, {excess_db:.1f} dB above the gap, peak {power:.1f} dBFS")

        hit = analyze(iq, radio.sample_rate, radio.center)
        print(describe(hit))
        if hit is not None:
            print(
                f"hz={hit.hz:.0f} kind={hit.kind} bw={hit.bw_hz:.0f} "
                f"sf={hit.sf} score={hit.score:.1f} power={hit.power_dbfs:.1f}"
            )
    except (RadioError, OSError) as exc:
        print(exc)
        return 1
    finally:
        radio.close()
    return 0


def main(argv: list[str] | None = None) -> None:
    args = sys.argv[1:] if argv is None else argv
    if "--check" in args:
        raise SystemExit(run_check())
    App(demo="--demo" in args).mainloop()
