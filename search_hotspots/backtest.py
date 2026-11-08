#!/usr/bin/env python3
"""
Back-test the hot-spot model on historical lost/missing cases with a known find location.

For every Medium/High-confidence case in data/missing_persons.csv:
  1. Rebuild the probability map from the route the person was on, leaving out that case's own
     incident records (anything within 3 days of the find date), so the answer doesn't leak in.
  2. Measure AREA-TO-FIND: if searchers cleared cells from most to least likely, how many km2
     they would cover before reaching the find. Smaller is better.
       point : the cell at the estimated find coordinate
       best  : the best cell within the case's uncertainty radius (the true spot could be anywhere in it)
  3. Compare against a distance-only baseline (the classic "rings around the IPP" approach) and
     random search (on average you clear half the window).
  4. Check whether the find lies within max(500 m, uncertainty) of one of the 13 listed hot spots.

Writes out/backtest/backtest_results.csv, summary.txt and cdf.png.

Run:  python search_hotspots/backtest.py --excel data/white_mountains_trails.xlsx
      (uses dem.py to find height_map.tif; add --dem PATH to override)
"""
import argparse, csv, datetime as dt, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE); sys.path.insert(0, ROOT)
import hotspots as H

# Case (row number in missing_persons.csv, 1-based) -> route the person was on.
# Strings are gazetteer names; tuples are explicit (lat, lon) starting points.
FRANCONIA = H.TRAIL_ROUTES["Franconia Ridge Loop"]
TUCK = H.TRAIL_ROUTES["Mt. Washington via Tuckerman"]
CASE_ROUTES = {
    11: TUCK, 31: TUCK, 48: TUCK, 53: TUCK, 75: TUCK,
    15: H.TRAIL_ROUTES["Huntington Ravine"], 101: H.TRAIL_ROUTES["Huntington Ravine"],
    20: FRANCONIA, 44: FRANCONIA, 45: FRANCONIA, 51: FRANCONIA, 55: FRANCONIA, 60: FRANCONIA,
    65: FRANCONIA, 81: FRANCONIA, 96: FRANCONIA,
    95: ["Liberty Spring TH", "Mount Liberty", "Little Haystack"],
    14: ["Lincoln Woods TH", "South Twin Mountain"],
    85: ["Gale River TH", "South Twin Mountain", "Mount Guyot"],
    56: H.TRAIL_ROUTES["Pemi Loop"],
    67: H.TRAIL_ROUTES["Bonds Traverse"],
    86: ["Lincoln Woods TH", "Owl's Head"],
    88: ["Lincoln Woods TH", "Pemigewasset Wilderness"],
    46: ["Appalachia TH", "Mount Madison", "Mount Adams", "Mount Jefferson"],
    68: ["Crawford Notch Depot", "Mount Pierce", "Mount Eisenhower", "Mount Monroe"],
    82: H.TRAIL_ROUTES["Ammonoosuc Ravine-Jewell"], 99: H.TRAIL_ROUTES["Ammonoosuc Ravine-Jewell"],
    76: ["Mount Washington"], 89: ["Mount Washington"],          # wandered off the summit
    36: H.TRAIL_ROUTES["Mt. Chocorua"], 77: H.TRAIL_ROUTES["Mt. Chocorua"], 97: H.TRAIL_ROUTES["Mt. Chocorua"],
    64: ["Kearsarge North"],
    58: [(44.1580, -71.6830)],                                   # Old Man viewing area
    90: ["Baldface"],
    91: [(44.1500, -71.3580)],                                   # Davis Path trailhead
}
# Left out: 5 (plane crash), 13 (homicide), 50 (walked off the mountain to a hotel).


def parse_date(s):
    for fmt in ("%Y-%m-%d", "%Y-%m"):
        try:
            return dt.datetime.strptime(str(s)[:len(fmt) + 2].split(" ")[0], fmt)
        except ValueError:
            pass
    return None


def area_to_find(prob, r, c, rad_cells, cell_km2):
    if not (0 <= r < prob.shape[0] and 0 <= c < prob.shape[1]):
        return None, None                                        # find outside the window: a miss
    v_pt = prob[r, c]
    win = prob[max(0, r - rad_cells):r + rad_cells + 1, max(0, c - rad_cells):c + rad_cells + 1]
    v_best = win.max()
    return float((prob > v_pt).sum() * cell_km2), float((prob > v_best).sum() * cell_km2)


def tune(cache, n_sets, out_dir, seed=0):
    """Score random weight sets on every case, then estimate honest performance with
    leave-one-case-out CV: pick the best set on 35 cases, evaluate it on the held-out one."""
    rng = np.random.default_rng(seed)
    names = list(H.WEIGHTS)
    sets = [dict(H.WEIGHTS), {k: 0.0 for k in names}]                # current defaults, distance-only
    sets += [{k: float(rng.choice([0, 0.25, 0.5, 1, 2, 4])) for k in names} for _ in range(n_sets)]
    S = np.zeros((len(sets), len(cache)))
    for j, cc in enumerate(cache):
        r, c = cc["rc"]
        for i, w in enumerate(sets):
            p = cc["base"] * (1 + sum(w[k] * cc["L"][k] for k in names))
            S[i, j] = (p > p[r, c]).sum() * cc["km2"]
    logS = np.log10(S + 0.01)
    cv = []
    for j in range(len(cache)):
        others = np.delete(logS, j, axis=1).mean(axis=1)
        cv.append(S[int(np.argmin(others)), j])
    cv = np.array(cv)
    best = int(np.argmin(logS.mean(axis=1)))
    off = np.array([cc["off"] for cc in cache])
    lines = ["", f"WEIGHT TUNING ({len(sets)} weight sets, {len(cache)} cases, leave-one-case-out CV)",
             f"  Median area-to-find: defaults {np.median(S[0]):.2f} | distance-only {np.median(S[1]):.2f} | "
             f"tuned (CV, honest) {np.median(cv):.2f} km2",
             f"  Off-trail cases:     defaults {np.median(S[0][off]):.2f} | distance-only {np.median(S[1][off]):.2f} | "
             f"tuned (CV) {np.median(cv[off]):.2f} km2" if off.any() else "",
             f"  Within 2 km2: defaults {np.mean(S[0] <= 2):.0%} | distance-only {np.mean(S[1] <= 2):.0%} | tuned (CV) {np.mean(cv <= 2):.0%}",
             f"  Best weights on all cases (paste into WEIGHTS if CV beats distance-only): {sets[best]}"]
    txt = "\n".join(l for l in lines if l)
    print(txt)
    open(os.path.join(out_dir, "summary.txt"), "a").write(txt + "\n")
    json_path = os.path.join(out_dir, "best_weights.json")
    import json; json.dump(sets[best], open(json_path, "w"), indent=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--excel", default=os.path.join(ROOT, "data", "white_mountains_trails.xlsx"))
    ap.add_argument("--cases", default=os.path.join(ROOT, "data", "missing_persons.csv"))
    ap.add_argument("--dem")
    ap.add_argument("--out", default=os.path.join(ROOT, "out", "backtest"))
    ap.add_argument("--tune", type=int, default=0, metavar="N",
                    help="also try N random layer-weight sets, scored with leave-one-case-out cross-validation")
    a = ap.parse_args()
    if not a.dem:
        for cand in (os.environ.get("DEM_PATH", ""), os.path.join(ROOT, "data", "height_map.tif"),
                     os.path.join(ROOT, "dem", "height_map.tif"),
                     os.path.join(os.path.expanduser("~"), "Documents", "height_map.tif")):
            if cand and os.path.exists(cand):
                a.dem = cand; break
    print("DEM:", a.dem or "NONE -> synthetic terrain (results meaningless; install rasterio / set DEM_PATH)")
    os.makedirs(a.out, exist_ok=True)

    gaz = H.load_gazetteer()
    byname = {g["name"]: g for g in gaz}
    kinds = {(g["lat"], g["lon"]): g["kind"] for g in gaz}
    incidents = H.load_incidents(a.excel)
    rows = list(csv.DictReader(open(a.cases, encoding="utf-8")))
    if rows and "Est. Lat" not in rows[0]:
        raise SystemExit(f"{a.cases} has no 'Est. Lat' column - it looks like the wrong file "
                         f"(columns: {list(rows[0])[:5]}...). Replace it with the missing_persons.csv that has find coordinates.")

    results, cache = [], []
    for idx, route in CASE_ROUTES.items():
        case = rows[idx - 1]
        if not case.get("Est. Lat") or case["Confidence"] not in ("Medium", "High"):
            continue
        flat, flon = float(case["Est. Lat"]), float(case["Est. Lon"])
        unc = float(case["Uncertainty (m)"])
        pts = [p if isinstance(p, tuple) else (byname[p]["lat"], byname[p]["lon"]) for p in route]

        # leave-one-out: drop this case's own incident records
        d0 = parse_date(case["Date Found"]) or parse_date(case["Date Missing"])
        inc = [d for d in incidents if not (d0 and isinstance(d.get("date"), dt.datetime)
                                            and abs((d["date"] - d0).days) <= 3)]
        hist = H.geocode_incidents(inc, gaz)

        lats, lons = zip(*(pts + [(flat, flon)]))                 # window must contain route AND find
        span_km = max(8.0, 111 * max(max(lats) - min(lats), (max(lons) - min(lons)) * 0.72) + 8)
        center = ((max(lats) + min(lats)) / 2, (max(lons) + min(lons)) / 2)
        summits = [p for p in pts if kinds.get(p) == "peak"]
        grid, src = H.load_dem(a.dem, center=center, size_km=span_km, peaks=summits)

        prior, mob, L, aux = H.build_layers(grid, pts, hist)
        prob = H.combine(prior, mob, L)
        base = prior * mob; base = base / base.sum()

        cell_m = (grid.dx + grid.dy) / 2
        cell_km2 = grid.dx * grid.dy / 1e6
        win_km2 = prob.size * cell_km2
        r, c = grid.rc(flat, flon)
        rad = max(1, int(unc / cell_m))
        m_pt, m_best = area_to_find(prob, r, c, rad, cell_km2)
        b_pt, b_best = area_to_find(base, r, c, rad, cell_km2)

        from scipy import ndimage
        d_route = ndimage.distance_transform_edt(~aux["route"], sampling=(grid.dy, grid.dx))
        off_m = float(d_route[r, c]) if grid.inside(r, c) else float("nan")
        hs = H.find_hotspots(grid, prob, L, aux["route"], k=10)
        tol = max(500.0, unc)
        def dist_m(h):
            return np.hypot((h["lat"] - flat) * 111_320, (h["lon"] - flon) * 111_320 * np.cos(np.radians(flat)))
        near = min(hs, key=dist_m) if hs else None
        hit_rank = near["rank"] if near and dist_m(near) <= tol else None

        res = dict(case=idx, name=case["Name"], date=case["Date Missing"][:10], area=case["Mountain / Area"],
                   confidence=case["Confidence"], uncertainty_m=int(unc), window_km2=round(win_km2, 1),
                   model_area_km2_point=None if m_pt is None else round(m_pt, 2),
                   model_area_km2_best=None if m_best is None else round(m_best, 2),
                   dist_only_area_km2_point=None if b_pt is None else round(b_pt, 2),
                   dist_only_area_km2_best=None if b_best is None else round(b_best, 2),
                   random_area_km2=round(win_km2 / 2, 1),
                   find_dist_from_route_m=None if off_m != off_m else int(off_m),
                   hotspot_hit_rank=hit_rank, nearest_hotspot_m=None if not near else int(dist_m(near)),
                   terrain=src)
        results.append(res)
        if a.tune and grid.inside(r, c):
            ds = 2                                                   # 40 m cells keep the search fast
            cache.append(dict(base=base[::ds, ::ds].astype(np.float32),
                              L={k: v[::ds, ::ds].astype(np.float32) for k, v in L.items()},
                              rc=(r // ds, c // ds), km2=cell_km2 * ds * ds, case=idx, off=(off_m or 0) > 250))
        print(f"#{idx:3d} {case['Name'][:28]:28s} model {res['model_area_km2_best']} km2 | "
              f"dist-only {res['dist_only_area_km2_best']} km2 | hotspot {hit_rank or '-'} ({res['nearest_hotspot_m']} m)")

    if not results:
        raise SystemExit(f"No testable cases: {a.cases} needs the Est. Lat / Est. Lon / Confidence columns "
                         "(use the missing_persons.csv with estimated find coordinates).")
    with open(os.path.join(a.out, "backtest_results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys())); w.writeheader(); w.writerows(results)

    def arr(k, rs=None):
        return np.array([r[k] if r[k] is not None else np.inf for r in (rs or results)])
    n = len(results)
    off = [r for r in results if (r["find_dist_from_route_m"] or 0) > 250]
    lines = [f"Cases tested: {n} (Medium/High confidence, leave-one-out history); {len(off)} found >250 m off the route",
             f"Terrain: {results[0]['terrain']}",
             "Metric: km2 searched (most-likely cells first) before reaching the estimated find point. Lower is better."]
    for label, rs in (("ALL CASES", results), ("OFF-TRAIL FINDS (>250 m from route)", off)):
        if not rs:
            continue
        m, b, rnd = arr("model_area_km2_point", rs), arr("dist_only_area_km2_point", rs), arr("random_area_km2", rs)
        lines.append(f"\n{label} (n={len(rs)})")
        lines.append(f"  Median area-to-find: model {np.median(m):.2f} km2 | distance-only {np.median(b):.2f} km2 | random {np.median(rnd):.1f} km2")
        for k in (0.5, 1, 2, 5):
            lines.append(f"  Found within first {k:>3} km2: model {np.mean(m <= k):.0%} | distance-only {np.mean(b <= k):.0%} | random ~{np.mean(np.minimum(k/(2*rnd),1)):.0%}")
        lines.append(f"  Model needs less area than distance-only on {np.mean(m < b):.0%} of cases (ties {np.mean(m == b):.0%})")
    m, b, rnd = arr("model_area_km2_point"), arr("dist_only_area_km2_point"), arr("random_area_km2")
    hits = [r for r in results if r["hotspot_hit_rank"]]
    lines.append(f"Find within tolerance of a listed hot spot: {len(hits)}/{n} ({len(hits)/n:.0%})"
                 f"; of those in the top 5: {sum(r['hotspot_hit_rank'] <= 5 for r in hits)}")
    lines.append("Caveat: find coordinates are estimates from text and share the gazetteer with the routes; "
                 "cases found AT a hut/summit on the route are easy. Treat as indicative, not validation.")
    summary = "\n".join(lines)
    print("\n" + summary)
    open(os.path.join(a.out, "summary.txt"), "w").write(summary + "\n")

    if a.tune and cache:
        tune(cache, a.tune, a.out)

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        xs = np.logspace(-2, 2.5, 200)
        fig, ax = plt.subplots(figsize=(7, 4.5))
        for vals, lab, col in ((m, "Hot-spot model", "#d1495b"), (b, "Distance rings only", "#2e86ab"),
                               (rnd, "Random search", "#999999")):
            ax.plot(xs, [np.mean(vals <= x) for x in xs], label=lab, color=col, lw=2.2)
        ax.set_xscale("log"); ax.set_xlabel("Area searched, most-likely first (km², log scale)")
        ax.set_ylabel("Share of historical finds located"); ax.set_ylim(0, 1)
        ax.set_title(f"Back-test on {n} White Mountains lost-person cases"); ax.grid(alpha=0.3); ax.legend()
        fig.savefig(os.path.join(a.out, "cdf.png"), dpi=130, bbox_inches="tight")
    except Exception as e:
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
