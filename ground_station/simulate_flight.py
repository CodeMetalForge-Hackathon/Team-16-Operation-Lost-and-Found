#!/usr/bin/env python3
"""
Simulated pod flight for the ground-station map (replay data so a dead sensor can't sink the demo).

The pod searches adaptively, the same way the map's "search next" box works:
  1. pick the 500 m box that still holds the most probability (planner.py),
  2. announce it (`plan` message -> the yellow box on the map), fly there and sweep it,
  3. if the hiker wasn't seen, lower the probability of everything it looked at (Bayesian update)
     and repeat. The pod carries 10 beacons (--beacons). Anything that looks like the hiker
     (orange tarp, gear) gets a "candidate" beacon for a ground team to check and the search goes on;
     when the camera sees the person it drops a "confirmed" beacon and heads home.
The camera can miss: inside the footprint it detects the hiker with probability POD (0.7).

  python ground_station/simulate_flight.py --run out/franconia_real --live 20   # watch it at 20x speed
  python ground_station/simulate_flight.py --run out/franconia_real             # write it all at once

Output: ground_station/logs/sim_flight.jsonl (overwritten). Where the hiker hides (--hiker):
  realistic (default): independent of the search model. A real case is drawn from
             hiker_distances.json (how far 36 real White Mountains finds were from the route the person was
             on), that distance is applied in a random direction from a random point on the route.
  model:     drawn from the search model's own probability map (most-likely 60%), i.e. the person
             behaves exactly as the model expects (optimistic).
  --target LAT,LON puts the hiker at an exact spot. --seed repeats the same hiding spot.
After the search ends a `sim_truth` message reveals where the hiker actually was.
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


def realistic_location(run_dir, plan, rng, tries=200):
    """Hide the hiker WITHOUT using the search model: distance-from-route of a random real case
    (jittered by that case's location uncertainty), random direction, random point on the route."""
    cases = json.load(open(os.path.join(HERE, "hiker_distances.json")))["cases"]
    rp = os.path.join(run_dir, "route.npy")
    route = np.argwhere(np.load(rp)) if os.path.exists(rp) else np.empty((0, 2))
    rows, cols = plan.prior.shape
    for _ in range(tries):
        c = cases[int(rng.integers(len(cases)))]
        d = max(0.0, c["dist_m"] + rng.normal(0, c["uncertainty_m"] / 2))
        r0, c0 = route[int(rng.integers(len(route)))] if len(route) else (rows // 2, cols // 2)
        lat0, lon0 = plan.latlon(int(r0), int(c0))
        bearing = rng.uniform(0, 2 * math.pi)
        lat, lon = offset(lat0, lon0, d * math.cos(bearing), d * math.sin(bearing))
        r, cc = plan.rc(lat, lon)
        if 0 <= r < rows and 0 <= cc < cols:
            return (lat, lon), dict(method="realistic", dist_from_route_m=round(d),
                                    based_on=f"case #{c['case']} ({c['name']}), found {c['dist_m']} m from their route")
    return plan.sample_location(rng), dict(method="model (fallback)")


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
    ap.add_argument("--target", help="LAT,LON of the missing hiker (overrides --hiker)")
    ap.add_argument("--hiker", choices=["realistic", "model"], default="realistic",
                    help="how the hiker's true location is chosen (see above)")
    ap.add_argument("--beacon-hz", type=float, default=915.0e6,
                    help="placeholder - use the frequency Stefan clears for the real beacon")
    ap.add_argument("--beacons", type=int, default=10, help="beacons the pod carries")
    ap.add_argument("--decoy-rate", type=float, default=0.3,
                    help="chance a search box holds something that looks like the hiker (orange tarp, gear, flagging)")
    ap.add_argument("--live", type=float, default=0, help="stream in real time at this speed-up (0 = instant)")
    ap.add_argument("--seed", type=int, default=3, help="changes where the hiker hides (try 1-10)")
    a = ap.parse_args()
    rnd = random.Random(a.seed)

    plan = Planner(a.run, a.footprint_m, a.pod, a.box_m)
    hs = [json.loads(l) for l in open(os.path.join(a.run, "hotspots.jsonl")) if l.strip()]
    trail = [h for h in hs if h.get("zone") == "trail"]
    start = (trail[-1]["lat"], trail[-1]["lon"]) if trail else (hs[0]["lat"], hs[0]["lon"])   # launch site
    nrng = np.random.default_rng(a.seed)
    truth = dict(method="target")
    if a.target:
        target = tuple(map(float, a.target.split(",")))
    elif a.hiker == "model":
        target = plan.sample_location(nrng); truth = dict(method="model")
    else:
        target, truth = realistic_location(a.run, plan, nrng)
    other_tx = offset(start[0], start[1], 1500, 900)                                         # some other transmitter

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    clock = {"t": time.time() - (0 if a.live else 4000), "i": 0}
    f = open(a.out, "w")

    def emit(rec):
        rec["sim"] = True                                          # marks simulated data on the map
        f.write(json.dumps(rec) + "\n"); f.flush()

    state = dict(pos=start, found=False, seen_this_pass=False, left=a.beacons, dropped=[], decoys=[])

    def drop(t, pose, kind, why):
        """Release the next beacon on a sighting. kind: 'confirmed' (person seen) or 'candidate' (needs a ground check)."""
        if state["left"] <= 0:
            emit(dict(type="detection_note", t=round(t, 3), note="no beacons left to mark this sighting", pose=pose))
            return
        state["left"] -= 1
        bid = f"B{a.beacons - state['left']}"
        emit(dict(type="drop_event", t=round(t, 3), pose=pose, beacon_id=bid, kind=kind, reason=why,
                  beacons_left=state["left"]))
        state["dropped"].append((bid, (pose["lat"], pose["lon"])))

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
            if searching and not state["found"]:
                plan.see(lat, lon)
                if dist_m((lat, lon), target) < a.footprint_m and not state["seen_this_pass"]:
                    state["seen_this_pass"] = True                 # one detection roll per pass over the hiker
                    if rnd.random() < a.pod:
                        state["found"] = True
                        emit(dict(type="detection", t=round(t, 3), cls="unnatural-color", color="orange",
                                  confidence=0.91, bbox=[612, 340, 38, 30], pose=pose))
                        emit(dict(type="detection", t=round(t + 0.4, 3), cls="person", confidence=0.74,
                                  bbox=[600, 330, 60, 90], pose=pose))
                        drop(t + 1, pose, "confirmed", "person detected")
                for dcy in state["decoys"]:                        # things that look like the hiker but aren't
                    if not dcy["seen"] and dist_m((lat, lon), dcy["pos"]) < a.footprint_m:
                        dcy["seen"] = True
                        if rnd.random() < a.pod:
                            emit(dict(type="detection", t=round(t, 3), cls="unnatural-color", color=dcy["color"],
                                      confidence=dcy["conf"], bbox=[480, 300, 30, 26], pose=pose))
                            drop(t + 1, pose, "candidate", f"possible sighting ({dcy['color']})")
            if i % 20 == 0:                                        # pod hears its dropped beacons on the cleared band
                for bid, bpos in state["dropped"]:
                    d = max(30.0, dist_m((lat, lon), bpos))
                    if d < 4000:
                        emit(dict(type="rf_hit", t=round(t, 3), hz=a.beacon_hz, power_db=round(-30 - 20 * math.log10(d / 30), 1),
                                  pose=pose, note=f"own beacon {bid}"))
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
        if rnd.random() < a.decoy_rate:                            # something orange/bright in this box
            state["decoys"].append(dict(pos=offset(blat, blon, rnd.uniform(-200, 200), rnd.uniform(-200, 200)),
                                        color=rnd.choice(["orange", "red", "blue", "pink"]),
                                        conf=round(rnd.uniform(0.55, 0.85), 2), seen=False))
        if fly(densify([state["pos"]] + sweep, a.speed), searching=True):
            break
        plan.end_pass()                                            # not found: Bayesian update, pick again

    t = time.time() if a.live else clock["t"] + clock["i"]
    emit(dict(type="plan", t=round(t, 3), status="located" if state["found"] else "return",
              lat=round(start[0], 6), lon=round(start[1], 6), size_m=0, searched=round(plan.searched_share(), 4)))
    emit(dict(type="sim_truth", t=round(t, 3), lat=round(target[0], 6), lon=round(target[1], 6),
              found=state["found"], **truth))                     # revealed only after the search ends
    fly(densify([state["pos"], start], a.speed), searching=False)  # return to launch
    f.close()
    print(f"{boxes} search boxes, {clock['i']} poses -> {a.out}  "
          f"({truth['method']} hiker {'FOUND' if state['found'] else 'NOT found'} at {target[0]:.5f},{target[1]:.5f}; "
          f"{plan.searched_share():.0%} of the probability searched; {a.beacons - state['left']} of {a.beacons} beacons used)")


if __name__ == "__main__":
    main()
