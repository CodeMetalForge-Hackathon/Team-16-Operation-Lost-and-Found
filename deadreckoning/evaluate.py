"""Measure the INS against simulated 100 m flights with a known path. Run: python evaluate.py

The drone's flight controller supplies orientation (a quaternion, PX4/ArduPilot NED/FRD convention) with a
small error; the MPU9250 supplies acceleration. Lidar pose_fix messages arrive once a second at baseline.
Team budget: up to 250 m of drift in one minute of coasting.
"""
import math
import random

from drcore import DeadReckoner, format_dd, parse_coord
from sim import simulate

ALT = 100.0
ROUTE = [(0, 0, ALT), (400, 0, ALT), (400, 400, ALT), (0, 400, ALT), (0, 0, ALT)]   # metres E, N, up from takeoff
START = (38.897700, -77.036500)  # decimal degrees


def run(samples, truth, quats, fix_hz=0.0, fix_sigma=5.0, outage=None, seed=5):
    dr = DeadReckoner(lat=START[0], lon=START[1], init_heading_deg=0.0, mag_aligned=True)
    rnd = random.Random(seed)
    errs, next_fix, t_fly = [], 0.0, None
    for (t, a, g, m), tr, (_, q) in zip(samples, truth, quats):
        dr.feed_attitude(t, q, "ned_frd")
        dr.feed(t, a, g, m, agl=tr[2] + rnd.gauss(0, 0.3))      # lidar height above (flat) ground
        if not dr.grounded:
            t_fly = t if t_fly is None else t_fly
            if fix_hz and t >= next_fix and not (outage and outage[0] <= t - t_fly < outage[1]):
                next_fix = t + 1.0 / fix_hz
                dr.apply_fix(tr[0] + rnd.gauss(0, fix_sigma), tr[1] + rnd.gauss(0, fix_sigma),
                             tr[2] + rnd.gauss(0, 2.0), sigma=fix_sigma)
        errs.append((t, math.dist(dr.p[:2], tr[:2]), dr.sigma_h(), dr.grounded))
    return dr, errs, t_fly


def report(title, samples, truth, quats, window=None, **kw):
    dr, errs, t_fly = run(samples, truth, quats, **kw)
    print(f"\n{title}")
    if t_fly is None:
        print("  never detected as flying")
        return
    flying = [e for e in errs if not e[3] and e[0] > t_fly]
    rms = math.sqrt(sum(e[1] ** 2 for e in flying) / len(flying))
    worst = max(flying, key=lambda e: e[1])
    marks = []
    for after in (10, 30, 60, 120, 180):
        i = int((t_fly + after) * 100)
        if i < len(errs) - 400:
            marks.append(f"{after}s {errs[i][1]:.1f} m (est. 1-sigma {errs[i][2]:.1f})")
    print("  error after takeoff -> " + ";  ".join(marks))
    print(f"  while flying: RMS {rms:.1f} m, worst {worst[1]:.1f} m at {worst[0] - t_fly:.0f} s;"
          f" fixes used {dr.fixes}, rejected {dr.fixes_rejected}")
    if window:
        a, b = window
        sel = [e for e in flying if a <= e[0] - t_fly <= b]
        print(f"  during the {b - a:.0f} s outage: error reached {max(e[1] for e in sel):.1f} m "
              f"(team budget 250 m per minute)")


if __name__ == "__main__":
    ideal, truth_i, q_i = simulate(ROUTE, ideal=True, vmax=8.0, return_quat=True)
    print(f"flight: {len(truth_i) / 100:.0f} s at {ALT:.0f} m, 400 m square, start {format_dd(*START)} (DD)")
    report("A. IDEAL sensors + perfect orientation, no fixes (should be ~0 m)", ideal, truth_i, q_i)
    for err in (0.5, 1.0, 2.0):
        real, truth, quats = simulate(ROUTE, vmax=8.0, return_quat=True, att_err_deg=err)
        report(f"B{err}. realistic MPU9250, orientation error ~{err} deg, COAST ONLY", real, truth, quats)
    real, truth, quats = simulate(ROUTE, vmax=8.0, return_quat=True, att_err_deg=1.0)
    report("C. orientation error ~1 deg + lidar pose_fix every 1 s (+/-5 m)", real, truth, quats, fix_hz=1.0)
    report("D. as C, but lidar drops out for 60 s", real, truth, quats, fix_hz=1.0, outage=(40, 100), window=(40, 100))
    print("\ncoordinate parsing:", parse_coord("38.8977"), parse_coord("77.0365 W", "lon"),
          round(parse_coord("38°53'51.7\"N"), 6), round(parse_coord("-77 2 11.4"), 6))
