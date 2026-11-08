"""Detector and bearing checks. No radio required."""

from __future__ import annotations

import unittest

import numpy as np

from df.bearing import RecentPeak, add_sample, angdiff, bearing_report
from df.lora import (
    CONFIRM_SCORE,
    SAMPLE_RATE,
    US915,
    best_dechirp,
    dechirp_score,
    hop_centers,
    survey,
    synthesize,
)


def _noise(n, std, seed):
    rng = np.random.default_rng(seed)
    return ((rng.standard_normal(n) + 1j * rng.standard_normal(n)) * std).astype(np.complex64)


class BearingTests(unittest.TestCase):
    def test_recent_peak_forgets_old_power(self):
        peak = RecentPeak(0.6)
        peak.push(0.0, -10.0)
        self.assertAlmostEqual(peak.push(1.0, -30.0), -30.0)

    def test_recent_peak_keeps_a_burst(self):
        peak = RecentPeak(0.6)
        peak.push(0.5, -12.0)
        self.assertAlmostEqual(peak.push(1.0, -28.0), -12.0)

    def test_flat_powers_have_no_bearing(self):
        samples = [(0.0, -20.0), (90.0, -20.2), (180.0, -19.8)]
        self.assertIsNone(bearing_report(samples))

    def test_two_samples_pick_the_stronger_heading(self):
        report = bearing_report([(40.0, -18.0), (220.0, -31.0)])
        self.assertEqual(report.method, "two-sample")
        self.assertAlmostEqual(report.deg, 40.0)
        self.assertAlmostEqual(report.contrast_db, 13.0)

    def test_peak_interpolates_between_marks(self):
        samples = []
        for deg in range(0, 360, 15):
            error = angdiff(deg, 52.0)
            samples.append((float(deg), -(error / 30.0) ** 2))
        report = bearing_report(samples)
        self.assertEqual(report.method, "peak")
        self.assertLess(abs(angdiff(report.deg, 52.0)), 2.0)

    def test_nearby_mark_replaces(self):
        samples = [(10.0, -20.0)]
        add_sample(samples, 12.0, -15.0)
        self.assertEqual(len(samples), 1)
        self.assertAlmostEqual(samples[0][1], -15.0)


class LoraTests(unittest.TestCase):
    def test_hops_cover_the_us_band(self):
        centers = hop_centers(*US915, SAMPLE_RATE)
        self.assertLessEqual(centers[0] - SAMPLE_RATE / 2, US915[0] + 1)
        self.assertGreaterEqual(centers[-1] + SAMPLE_RATE / 2, US915[1] - 1)

    def test_chirp_scores_above_a_tone_and_noise(self):
        n = int(SAMPLE_RATE * 0.16)
        lora = synthesize(n, SAMPLE_RATE, 125_000, 9, 0.35, f_offset_hz=25_000) + _noise(n, 0.02, 1)
        tone = (0.35 * np.exp(1j * 2 * np.pi * 25_000 * np.arange(n) / SAMPLE_RATE)).astype(np.complex64)
        tone = tone + _noise(n, 0.02, 2)
        noise = _noise(n, 0.05, 3)
        lora_sf, lora_bw, lora_score = best_dechirp(lora, SAMPLE_RATE)
        tone_score = dechirp_score(tone, SAMPLE_RATE, 125_000, 9)
        noise_score = dechirp_score(noise, SAMPLE_RATE, 125_000, 9)
        self.assertEqual(lora_sf, 9)
        self.assertEqual(lora_bw, 125_000)
        self.assertGreater(lora_score, CONFIRM_SCORE, f"lora {lora_score}")
        self.assertLess(tone_score, CONFIRM_SCORE, f"tone {tone_score}")
        self.assertLess(noise_score, CONFIRM_SCORE, f"noise {noise_score}")
        self.assertGreater(lora_score, tone_score * 3)

    def test_survey_locks_915(self):
        class Scripted:
            def __init__(self):
                self.sample_rate = SAMPLE_RATE
                self.center = 0
                self.gain_db = 20.7
                self.gains = [0.0, 20.7, 40.2]
                self.last_rail = 0.0
                self.needs_settle = False

            def set_sample_rate(self, hz):
                self.sample_rate = hz

            def set_center_freq(self, hz):
                self.center = int(hz)

            def set_gain_db(self, db):
                self.gain_db = db

            def read(self, nsamp):
                if abs(self.center - 915_000_000) < 1_000_000:
                    sig = synthesize(nsamp, self.sample_rate, 125_000, 9, 0.4, 915_000_000 - self.center)
                    return sig + _noise(nsamp, 0.02, 7)
                return _noise(nsamp, 0.03, 8)

        hit = survey(Scripted(), US915)
        self.assertIsNotNone(hit)
        self.assertEqual(hit.kind, "lora")
        self.assertEqual(hit.sf, 9)
        self.assertLess(abs(hit.hz - 915_000_000), 150_000)

    def test_survey_is_quiet_without_a_chirp(self):
        class Quiet:
            sample_rate = SAMPLE_RATE
            center = 0
            gain_db = 20.7
            last_rail = 0.0
            needs_settle = False

            def set_sample_rate(self, hz):
                self.sample_rate = hz

            def set_center_freq(self, hz):
                self.center = int(hz)

            def set_gain_db(self, db):
                self.gain_db = db

            def read(self, nsamp):
                return _noise(nsamp, 0.03, 9)

        self.assertIsNone(survey(Quiet(), US915))


if __name__ == "__main__":
    unittest.main()
