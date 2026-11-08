#!/usr/bin/env python3
"""
Simulated pod flight for the ground-station map (replay data so a dead sensor can't sink the demo).

The pod searches adaptively, the same way the map's "search next" box works:
  1. pick the 500 m box that still holds the most probability (planner.py),
  2. announce it (`plan` message -> the yellow box on the map), fly there and sweep it,
  3. if the hiker wasn't seen, lower the probability of everything it looked at (Bayesian update)
     and repeat. When the camera spots the hiker it logs `detection`s, drops beacon B1
     (`drop_event`), then flies home while hearing the beacon (`rf_hit`s).
The camera can miss: inside the footprint it detects the hiker with probability POD (0.7).

  python ground_station/simulate_flight.py --run out/franconia_real --live 20   # watch it at 20x speed
  python ground_station/simulate_flight.py --run out/franconia_real             # write it all at once

Output: ground_station/logs/sim_flight.jsonl (overwritten). The hiker is hidden at a random likely
spot drawn from the probability map (change with --seed), or exactly where you say with --target LAT,LON.
"""
import argparse, json, math, os, random, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from geo import dist_m
from planner import Planner

M_PER_DEG = 111_320.0


def offset(lat, lon, dn, de):
    return lat + dn / M_PER_DEG, lon + de / (M_PER_DEG * math.cos(math.radians(lat)))


def lawnmower(lat, lon, box_m, lane_m):
    pts, n = [], int(box_m / lane_m) + 1
    for i in range(n):
        de = -box_m / 2 + i * lane_m
        a, b = (-box_m / 2, box_m / 2) if i % 2 == 0 else (box_m / 2, -box_m / 2)
        pts += [offset(lat, lon, a, de), offset(lat, lon, b, de)]
    return pts


def densify(waypoints, step_m):
    out = []
    for a, b in zip(waypoints, waypoints[1:]):
        n = max(1, int(dist_m(a, b) / step_m))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", default=os.path.join(ROOT, "out", "franconia_real"))
    ap.add_argument("--out", default=os.path.join(HERE, "logs", "sim_flight.jsonl"))
    ap.add_argument("--max-boxes", type=int, default=12, help="search boxes before the pod must return (battery)")
    ap.add_argument("--box-m", type=float, default=500)
    ap.add_argument("--lane-m", type=float, default=100)
    ap.add_argument("--footprint-m", type=float, default=60, help="camera footprint radius (same as server.py)")
    ap.add_argument("--pod", type=float, default=0.7, help="chance the camera sees the hiker when over them")
    ap.add_argument("--speed", type=float, default=12, help="m/s")
    ap.add_argument("--alt", type=float, default=60, help="m above ground (logged as z)")
    ap.add_argument("--target", help="LAT,LON of the missing hiker (default: random likely spot)")
    ap.add_argument("--beacon-hz", type=float, default=915.0e6,
                    help="placeholder - use the frequency Stefan clears for the real beacon")
    ap.add_argument("--live", type=float, default=0, help="stream in real time at this speed-up (0 = instant)")
    ap.add_argument("--seed", type=int, default=3, help="where the hiker hides (3 = found after 4 boxes on Franconia)")
    a = ap.parse_args()
    rnd = random.Random(a.seed)

    plan = Planner(a.run, a.footprint_m, a.pod, a.box_m)
    hs = [json.loads(l) for l in open(os.path.join(a.run, "hotspots.jsonl")) if l.strip()]
    trail = [h for h in hs if h.get("zone") == "trail"]
    start = (trail[-1]["lat"], trail[-1]["lon"]) if trail else (hs[0]["lat"], hs[0]["lon"])   # launch site
    target = tuple(map(float, a.target.split(","))) if a.target else plan.sample_location(np.random.default_rng(a.seed))
    other_tx = offset(start[0], start[1], 1500, 900)                                         # some other transmitter

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    clock = {"t": time.time() - (0 if a.live else 4000), "i": 0}
    f = open(a.out, "w")

    def emit(rec):
        rec["sim"] = True                                          # marks simulated data on the map
        f.write(json.dumps(rec) + "\n"); f.flush()

    state = dict(pos=start, found=False, beacon=None, seen_this_pass=False)

    def fly(points, searching):
        """Fly through densified points, logging poses; returns True once the hiker is found."""
        for lat, lon in points:
            clock["i"] += 1
            t = time.time() if a.live else clock["t"] + clock["i"]
            i = clock["i"]
            src = "lidar" if i % 15 == 0 else "coast"              # periodic terrain-match fix, INS coast between
            jit = 0 if src == "lidar" else 6
            plat, plon = offset(lat, lon, rnd.gauss(0, jit), rnd.gauss(0, jit))
            pose = dict(lat=round(plat, 6), lon=round(plon, 6), z=a.alt)
            emit(dict(type="pose_fix", t=round(t, 3), source=src, **pose))
            state["pos"] = (lat, lon)
            if not state["found"]:
                plan.see(lat, lon)
                if dist_m((lat, lon), target) < a.footprint_m * 0.75 and not state["seen_this_pass"]:
                    state["seen_this_pass"] = True                 # one detection roll per pass over the hiker
                    if rnd.random() < a.pod:
                        state["found"] = True
                        emit(dict(type="detection", t=round(t, 3), cls="unnatural-color", color="orange",
                                  confidence=0.91, bbox=[612, 340, 38, 30], pose=pose))
                        emit(dict(type="detection", t=round(t + 0.4, 3), cls="person", confidence=0.74,
                                  bbox=[600, 330, 60, 90], pose=pose))
                        emit(dict(type="drop_event", t=round(t + 1, 3), pose=pose, beacon_id="B1"))
                        state["beacon"] = (plat, plon)
            if state["beacon"] and i % 20 == 0:                    # pod hears its own beacon on the cleared band
                d = max(30.0, dist_m((lat, lon), state["beacon"]))
                emit(dict(type="rf_hit", t=round(t, 3), hz=a.beacon_hz, power_db=round(-30 - 20 * math.log10(d / 30), 1),
                          pose=pose, note="own beacon B1"))
            if dist_m((lat, lon), other_tx) < 1200 and i % 30 == 0:
                emit(dict(type="rf_hit", t=round(t, 3), hz=a.beacon_hz + 0.35e6, power_db=round(-75 + rnd.gauss(0, 3), 1),
                          pose=pose, note="unknown transmitter"))
            if a.live:
                time.sleep(1.0 / a.live)
            if searching and state["found"]:
                return True
        return state["found"]

    boxes = 0
    while boxes < a.max_boxes and not state["found"]:
        boxes += 1
        blat, blon, mass = plan.next_box()
        t = time.time() if a.live else clock["t"] + clock["i"]
        emit(dict(type="plan", t=round(t, 3), box=boxes, lat=round(blat, 6), lon=round(blon, 6),
                  size_m=a.box_m, poa=round(mass, 4), searched=round(plan.searched_share(), 4)))
        sweep = lawnmower(blat, blon, a.box_m, a.lane_m)
        state["seen_this_pass"] = False
        if fly(densify([state["pos"]] + sweep, a.speed), searching=True):
            break
        plan.end_pass()                                            # not found: Bayesian update, pick again

    t = time.time() if a.live else clock["t"] + clock["i"]
    emit(dict(type="plan", t=round(t, 3), status="located" if state["found"] else "return",
              lat=round(start[0], 6), lon=round(start[1], 6), size_m=0, searched=round(plan.searched_share(), 4)))
    fly(densify([state["pos"], start], a.speed), searching=False)  # return to launch
    f.close()
    print(f"{boxes} search boxes, {clock['i']} poses -> {a.out}  "
          f"(hiker {'FOUND' if state['found'] else 'NOT found'} at {target[0]:.5f},{target[1]:.5f}; "
          f"{plan.searched_share():.0%} of the probability searched)")


if __name__ == "__main__":
    main()
