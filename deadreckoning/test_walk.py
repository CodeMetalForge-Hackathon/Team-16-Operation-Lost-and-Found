"""Check walking mode against a simulated walk with a known path. Run: python test_walk.py

A person carries the board level, x arrow forward, and walks a square (20 steps per side, 0.7 m per step)
with 1-second turns. Gyro offset, noise and a disturbed magnetometer are added.
"""
import math
import random

from drcore import DeadReckoner, G, format_dd

HZ, STEP_HZ, STEP_M = 100, 1.8, 0.7


def walk_samples(legs, seed=1, gyro_bias=(1.2, 1.5, 0.2), mag_noise=1.0, mag_offset=(0.0, 0.0, 0.0)):
    rnd = random.Random(seed)
    B, inc = 50.0, math.radians(65)
    bh, bz = B * math.cos(inc), B * math.sin(inc)
    t, h = 0.0, math.radians(legs[0][1])
    pos = [0.0, 0.0]
    samples, truth_steps = [], []
    dt = 1.0 / HZ

    def emit(h, hdot, a_vert):
        a = ((rnd.gauss(0, 0.05)) / G, rnd.gauss(0, 0.05) / G, (G + a_vert + rnd.gauss(0, 0.05)) / G)
        gyro = (gyro_bias[0] + rnd.gauss(0, 0.1), gyro_bias[1] + rnd.gauss(0, 0.1),
                gyro_bias[2] - math.degrees(hdot) + rnd.gauss(0, 0.1))
        mag = (bh * math.cos(h) + mag_offset[0] + rnd.gauss(0, mag_noise),
               bh * math.sin(h) + mag_offset[1] + rnd.gauss(0, mag_noise),
               -bz + mag_offset[2] + rnd.gauss(0, mag_noise))
        samples.append((t, a, gyro, mag))

    for _ in range(int(3 * HZ)):                       # stand still first (bias + gravity calibration)
        emit(h, 0.0, 0.0)
        t += dt
    for kind, target_deg, steps in legs:
        target = math.radians(target_deg)
        diff = (target - h + math.pi) % (2 * math.pi) - math.pi
        for _ in range(HZ):                            # 1 s turn
            hdot = diff / 1.0
            h += hdot * dt
            emit(h, hdot, 0.0)
            t += dt
        h = target
        for k in range(int(steps * HZ / STEP_HZ)):     # walk: vertical bounce, one peak per step
            ph = 2 * math.pi * STEP_HZ * (k * dt) - math.pi / 2
            emit(h, 0.0, 2.5 * math.sin(ph) * (1 if k * dt * STEP_HZ > 0.25 else 0))
            if k > 0 and abs(((k * dt * STEP_HZ) % 1.0) - 0.75) < dt * STEP_HZ / 2 and k * dt * STEP_HZ > 0.5:
                pos[0] += STEP_M * math.sin(h)
                pos[1] += STEP_M * math.cos(h)
                truth_steps.append((t, tuple(pos)))
            t += dt
    for _ in range(2 * HZ):
        emit(h, 0.0, 0.0)
        t += dt
    return samples, pos, len(truth_steps)


def run(label, **kw):
    legs = [("walk", 0, 20), ("walk", 90, 20), ("walk", 180, 20), ("walk", 270, 20)]
    samples, truth, n_true = walk_samples(legs, **kw)
    dr = DeadReckoner(lat=42.3601, lon=-71.0589, mag_aligned=True)
    dr.set_mode("walking")
    for t, a, g, m in samples:
        dr.feed(t, a, g, m)
    err = math.hypot(dr.p[0] - truth[0], dr.p[1] - truth[1])
    walked = n_true * STEP_M
    print(f"{label}: steps {dr.steps} (true {n_true}), walked {dr.distance:.1f} m, final ({dr.p[0]:.1f}, {dr.p[1]:.1f}) m "
          f"vs true ({truth[0]:.1f}, {truth[1]:.1f}); error {err:.1f} m = {100 * err / walked:.1f}% of {walked:.0f} m"
          f"  | DD {format_dd(*dr.latlon())}")
    return dr


if __name__ == "__main__":
    run("clean magnetometer      ")
    run("mag offset +12 uT (bad) ", mag_offset=(12.0, -8.0, 5.0))
    run("different noise seed     ", seed=7)
