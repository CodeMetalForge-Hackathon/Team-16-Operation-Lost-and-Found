#!/usr/bin/env python3
"""
Missing-person probability map + hot spots for the White Mountains (Operation Lost and Found).

Idea (classic SAR "probability of area" map, Bayesian search theory):
    P(cell) ∝ distance_prior(cell) * mobility(cell) * (1 + Σ w_i * layer_i(cell))

Layers (each scaled 0..1):
    downstream  - simulated "lost hiker walks downhill / follows water" paths from the planned route (D8 flow)
    stream      - proximity to drainages derived from the DEM (flow accumulation), no external data needed
    trail       - proximity to the planned route (people are often found on/near trails)
    alpine      - above-treeline cells near the route (weather/whiteout strandings on ridges)
    trap        - terrain traps: steep stream reaches (falls/cascades) and big-stream crossings
    history     - kernel density of past lost/missing incidents from the Excel incident database

Weights are calibrated loosely from the team's spreadsheet (80 lost/missing records out of 547):
    ~30% mention water (brook/river/drainage), ~34% found on/near a trail, ~44% summit/ridge/above treeline.

Inputs
    --excel   workbook with sheet 'wmnf-incident-database' (and optionally 'white-mountain-trails')
    --trail   trail name from the trails sheet (picks route + IPP), or --ipp LAT,LON
    --dem     GeoTIFF (USGS 3DEP 1/3 arc-second clip). If omitted or rasterio missing -> synthetic terrain.
Outputs (in --out)
    hotspots.jsonl   one JSON line per hot spot  {"type":"hotspot","rank","lat","lon","poa","reasons",...}
    hotspots.geojson same, for any map
    probability.npy  normalized grid (sums to 1)
    heatmap.png      quick-look image

Also exposes bayes_update(prob, searched_mask, pod) so the pod/drone can down-weight searched cells.
Deps: numpy, scipy, matplotlib, openpyxl (rasterio optional for real DEMs).
"""
import argparse, csv, heapq, json, math, os, re, time
import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))

# ----------------------------------------------------------------------------- config
# Distance-from-route percentiles in miles. PLACEHOLDERS: consistent with "66% of hikers found within
# 2 mi of PLS" (Hill 1997 via BSAR) and the incident DB (median ~4 mi from road/trailhead).
# Replace with Koester ISRID "Hiker / temperate / mountainous" numbers if the team has the book.
DIST_PCT_MI = {25: 0.8, 50: 1.6, 75: 3.0, 95: 6.5}

WEIGHTS = dict(downstream=1.5, stream=0.6, trail=1.0, alpine=0.8, trap=0.8, history=1.0)

STREAM_AREA_KM2 = 0.25      # contributing area that counts as a stream
TREELINE_M = 1340           # ~4,400 ft in the Whites
DOWNHILL_DECAY_M = 2500     # how far a "walk downstream" path keeps its weight
CELL_M_SYNTH = 30.0

# Trail -> ordered gazetteer waypoints (first = trailhead / IPP)
TRAIL_ROUTES = {
    "Franconia Ridge Loop": ["Lafayette Place (Old Bridle Path TH)", "Little Haystack", "Mount Lincoln", "Mount Lafayette", "Lafayette Place (Old Bridle Path TH)"],
    "Mt. Washington via Tuckerman": ["Pinkham Notch Visitor Center", "Tuckerman Ravine", "Mount Washington"],
    "Huntington Ravine": ["Pinkham Notch Visitor Center", "Huntington Ravine", "Mount Washington"],
    "Ammonoosuc Ravine-Jewell": ["Ammonoosuc Ravine TH (Cog Base)", "Lakes of the Clouds Hut", "Mount Washington", "Ammonoosuc Ravine TH (Cog Base)"],
    "Presidential Traverse": ["Appalachia TH", "Mount Madison", "Mount Adams", "Mount Jefferson", "Mount Washington", "Mount Monroe", "Mount Eisenhower", "Mount Pierce", "Crawford Notch Depot"],
    "Mt. Adams": ["Appalachia TH", "Mount Adams"],
    "Mt. Madison": ["Appalachia TH", "Mount Madison"],
    "Mt. Jefferson": ["Caps Ridge TH", "Mount Jefferson"],
    "Mt. Eisenhower": ["Edmands Path TH", "Mount Eisenhower"],
    "Mt. Liberty & Mt. Flume": ["Liberty Spring TH", "Mount Flume", "Mount Liberty", "Liberty Spring TH"],
    "Mt. Garfield": ["Gale River TH", "Mount Garfield"],
    "North & South Twin": ["Gale River TH", "South Twin Mountain", "North Twin Mountain"],
    "Pemi Loop": ["Lincoln Woods TH", "Mount Flume", "Mount Liberty", "Little Haystack", "Mount Lincoln", "Mount Lafayette", "Mount Garfield", "South Twin Mountain", "Mount Guyot", "Mount Bond", "Lincoln Woods TH"],
    "Bonds Traverse": ["Zealand Rd TH", "Zealand / Zeacliff", "Mount Guyot", "Mount Bond", "Lincoln Woods TH"],
    "Zeacliff": ["Zealand Rd TH", "Zealand / Zeacliff"],
    "Mt. Carrigain": ["Sawyer River Rd TH", "Mount Carrigain"],
    "Mt. Moosilauke": ["Moosilauke Ravine Lodge", "Mount Moosilauke"],
    "Mt. Osceola": ["Tripoli Rd Osceola TH", "Mount Osceola"],
    "Mt. Chocorua": ["Champney Falls TH", "Mount Chocorua"],
    "Mt. Willard": ["Crawford Notch Depot", "Mount Willard"],
    "Webster Cliff": ["Crawford Notch Depot", "Mount Webster"],
    "Carter Dome": ["Nineteen Mile Brook TH", "Carter Dome"],
    "Welch-Dickey": ["Welch-Dickey TH", "Welch-Dickey"],
    "Arethusa Falls": ["Arethusa Falls"],
    "Diana's Bath": ["Diana's Bath"],
}

# ----------------------------------------------------------------------------- geo helpers
class Grid:
    """Raster georeferencing. Supports geographic (lat/lon) rasters and a local metric frame for synthetic DEMs."""
    def __init__(self, z, lat0, lon0, dlat, dlon):
        self.z = z.astype(np.float64)
        self.lat0, self.lon0, self.dlat, self.dlon = lat0, lon0, dlat, dlon   # upper-left corner, deg per cell (dlat<0)
        mid = lat0 + dlat * z.shape[0] / 2
        self.dy = abs(dlat) * 111_320.0
        self.dx = abs(dlon) * 111_320.0 * math.cos(math.radians(mid))

    def rc(self, lat, lon):
        return int((lat - self.lat0) / self.dlat), int((lon - self.lon0) / self.dlon)

    def latlon(self, r, c):
        return self.lat0 + (r + 0.5) * self.dlat, self.lon0 + (c + 0.5) * self.dlon

    def inside(self, r, c):
        return 0 <= r < self.z.shape[0] and 0 <= c < self.z.shape[1]


def load_dem(path, center=None, size_km=14, peaks=(), cell_m=20.0):
    """Read only a size_km x size_km window around `center` from a (possibly huge) DEM, in any CRS
    (e.g. wmnf_dem_utm.tif, EPSG:26919), and resample it to a lat/lon grid of ~cell_m metres."""
    if path:
        try:
            import rasterio
            from rasterio.windows import from_bounds
            from rasterio.warp import reproject, Resampling, transform as warp_xy
            from rasterio.transform import from_origin
            lat0, lon0 = center
            half_lat = size_km * 500 / 111_320.0
            half_lon = size_km * 500 / (111_320.0 * math.cos(math.radians(lat0)))
            box = (lon0 - half_lon, lat0 - half_lat, lon0 + half_lon, lat0 + half_lat)
            with rasterio.open(path) as src:
                xs, ys = warp_xy("EPSG:4326", src.crs, [box[0], box[2], box[0], box[2]], [box[1], box[1], box[3], box[3]])
                pad = 0.05 * (max(xs) - min(xs))
                win = from_bounds(min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad, src.transform)
                win = win.round_offsets().round_lengths()
                if win.width <= 0 or win.height <= 0:
                    raise SystemExit("The trail is outside this DEM's extent")
                data = src.read(1, window=win, boundless=True, fill_value=src.nodata if src.nodata is not None else -9999)
                wt = src.window_transform(win)
                dlat = cell_m / 111_320.0
                dlon = cell_m / (111_320.0 * math.cos(math.radians(lat0)))
                h, w = int(2 * half_lat / dlat), int(2 * half_lon / dlon)
                dst_t = from_origin(box[0], box[3], dlon, dlat)
                z = np.full((h, w), np.nan, np.float32)
                reproject(data.astype(np.float32), z, src_transform=wt, src_crs=src.crs,
                          src_nodata=src.nodata if src.nodata is not None else -9999,
                          dst_transform=dst_t, dst_crs="EPSG:4326", dst_nodata=np.nan,
                          resampling=Resampling.bilinear)
            if np.isnan(z).all():
                raise SystemExit("No elevation data in the window around this trail")
            z = np.where(np.isnan(z), np.nanmin(z), z)
            print(f"DEM window {w}x{h} cells @ {cell_m:.0f} m, elev {z.min():.0f}-{z.max():.0f} m")
            return Grid(z, box[3], box[0], -dlat, dlon), "dem"
        except ImportError:
            print("rasterio not installed (pip install rasterio) -> synthetic terrain")
    return synthetic_dem(center or (44.15, -71.6), size_km, peaks), "synthetic"


def synthetic_dem(center, size_km, peaks=(), seed=7):
    """Stand-in terrain when no DEM tile is on disk: ridges joining the given summit points (lat, lon),
    valleys elsewhere, plus fractal noise so the D8 drainage network looks natural. Shapes are FAKE."""
    n = int(size_km * 1000 / CELL_M_SYNTH)
    rng = np.random.default_rng(seed)
    dlat = -CELL_M_SYNTH / 111_320.0
    dlon = CELL_M_SYNTH / (111_320.0 * math.cos(math.radians(center[0])))
    g = Grid(np.zeros((n, n)), center[0] - dlat * n / 2, center[1] - dlon * n / 2, dlat, dlon)
    rr, cc = np.mgrid[0:n, 0:n]
    z = np.full((n, n), 420.0)
    pk = [g.rc(*p) for p in peaks]
    ridge = np.zeros((n, n), bool)
    for (r0, c0), (r1, c1) in zip(pk, pk[1:]):                      # ridge line between consecutive summits
        for t in np.linspace(0, 1, max(abs(r1 - r0), abs(c1 - c0), 1) + 1):
            r, c = int(r0 + t * (r1 - r0)), int(c0 + t * (c1 - c0))
            if g.inside(r, c):
                ridge[r, c] = True
    if ridge.any():
        d = ndimage.distance_transform_edt(~ridge) * CELL_M_SYNTH
        z += 950 * np.exp(-d / 1800.0)
    for r, c in pk:                                                  # summit bumps
        z += 220 * np.exp(-((rr - r) ** 2 + (cc - c) ** 2) * CELL_M_SYNTH ** 2 / (2 * 500.0 ** 2))
    noise = sum(ndimage.zoom(rng.standard_normal((k, k)), n / k, order=3)[:n, :n] * (45 / i)
                for i, k in enumerate((6, 12, 24, 48), 1))
    g.z = z + noise
    return g


# ----------------------------------------------------------------------------- hydrology
OFFS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

def fill_depressions(z):
    """Priority-flood (Barnes 2014) so water always has a path off the map."""
    h, w = z.shape
    out = z.copy(); seen = np.zeros_like(z, bool); pq = []
    for r in range(h):
        for c in (0, w - 1):
            heapq.heappush(pq, (out[r, c], r, c)); seen[r, c] = True
    for c in range(1, w - 1):
        for r in (0, h - 1):
            heapq.heappush(pq, (out[r, c], r, c)); seen[r, c] = True
    while pq:
        e, r, c = heapq.heappop(pq)
        for dr, dc in OFFS:
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and not seen[rr, cc]:
                seen[rr, cc] = True
                out[rr, cc] = max(out[rr, cc], e + 1e-3)
                heapq.heappush(pq, (out[rr, cc], rr, cc))
    return out


def d8(z, dx, dy):
    """Steepest-descent receiver (flat index, -1 = outlet) and flow accumulation (cells)."""
    h, w = z.shape
    zp = np.pad(z, 1, constant_values=-1e9)              # edges drain off-map
    best = np.zeros((h, w)); rec = np.full((h, w), -1, np.int64)
    rr, cc = np.mgrid[0:h, 0:w]
    for dr, dc in OFFS:
        dist = math.hypot(dr * dy, dc * dx)
        drop = (z - zp[1 + dr:1 + dr + h, 1 + dc:1 + dc + w]) / dist
        nr, nc = rr + dr, cc + dc
        ok = (drop > best) & (nr >= 0) & (nr < h) & (nc >= 0) & (nc < w)
        best = np.where(ok, drop, best)
        rec = np.where(ok, nr * w + nc, rec)
    acc = np.ones(h * w); recf = rec.ravel()
    for i in np.argsort(z.ravel())[::-1]:                 # high -> low
        j = recf[i]
        if j >= 0:
            acc[j] += acc[i]
    return rec, acc.reshape(h, w)


# ----------------------------------------------------------------------------- incident history
def load_gazetteer(path=os.path.join(HERE, "gazetteer.csv")):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(l for l in f if not l.startswith("#")):
            rows.append(dict(name=r["name"], lat=float(r["lat"]), lon=float(r["lon"]), kind=r["kind"],
                             rx=re.compile(r["aliases"], re.I) if r["aliases"] else None))
    return rows


def load_incidents(xlsx):
    import openpyxl
    wb = openpyxl.load_workbook(xlsx, read_only=True, data_only=True)
    sheet = next((s for s in wb.sheetnames if "incident" in s.lower()), None)
    if not sheet:
        return []
    it = wb[sheet].iter_rows(values_only=True)
    hdr = [str(h).strip().lower() if h else "" for h in next(it)]
    return [dict(zip(hdr, r)) for r in it]


def geocode_incidents(incidents, gaz):
    """Place each incident at the first gazetteer feature its peak/trail (then summary) mentions.
    Lost/missing incidents weigh 1.0, other incidents 0.25 (a proxy for traffic)."""
    pts = []
    for d in incidents:
        lost = bool(re.search(r"lost|navigation", f"{d.get('primary_factor')} {d.get('secondary_factors')}")) \
            or d.get("outcome") == "missing" or d.get("incident_type") == "sar_no_subject_found"
        for field in (f"{d.get('peak') or ''} | {d.get('trail') or ''}", str(d.get("summary") or "")):
            hit = next((g for g in gaz if g["rx"] and g["rx"].search(field)), None)
            if hit:
                pts.append((hit["lat"], hit["lon"], 1.0 if lost else 0.25, hit["name"]))
                break
    return pts


# ----------------------------------------------------------------------------- layers
def norm01(a):
    a = np.asarray(a, float); lo, hi = np.nanmin(a), np.nanmax(a)
    return (a - lo) / (hi - lo) if hi > lo else np.zeros_like(a)


def rasterize_route(grid, pts):
    m = np.zeros(grid.z.shape, bool)
    rcs = [grid.rc(*p) for p in pts]
    if len(rcs) == 1:
        rcs = rcs * 2
    for (r0, c0), (r1, c1) in zip(rcs, rcs[1:]):
        n = max(abs(r1 - r0), abs(c1 - c0), 1)
        for t in np.linspace(0, 1, n + 1):
            r, c = int(round(r0 + t * (r1 - r0))), int(round(c0 + t * (c1 - c0)))
            if grid.inside(r, c):
                m[r, c] = True
    return m


ON_TRAIL_SHARE = 0.30      # ~1/3 of lost-person records in the DB were found on/near a trail

def distance_prior(d_m, route_len_m):
    """Mixture: ON_TRAIL_SHARE in a half-normal band (~150 m) hugging the route, the rest lognormal-fit
    to DIST_PCT_MI. Radial density is divided by the band area so each cell gets a per-area density."""
    p25, p75 = DIST_PCT_MI[25] * 1609.34, DIST_PCT_MI[75] * 1609.34
    mu = (math.log(p25) + math.log(p75)) / 2
    sg = (math.log(p75) - math.log(p25)) / (2 * 0.6745)
    d = np.maximum(d_m, 15.0)
    logn = np.exp(-(np.log(d) - mu) ** 2 / (2 * sg ** 2)) / (d * sg * math.sqrt(2 * math.pi))
    near = np.exp(-d ** 2 / (2 * 150.0 ** 2)) * math.sqrt(2 / math.pi) / 150.0
    pdf = ON_TRAIL_SHARE * near + (1 - ON_TRAIL_SHARE) * logn
    return pdf / (2 * route_len_m + 2 * math.pi * d)      # area of the band at offset d around the route


def downstream_paths(rec, route, cell_m, decay_m=DOWNHILL_DECAY_M, step=3):
    """From points on the route, follow D8 receivers downhill ("walk downstream to civilization")."""
    h, w = route.shape; out = np.zeros(h * w); recf = rec.ravel()
    starts = np.flatnonzero(route.ravel())[::step]
    max_steps = int(4 * decay_m / cell_m)
    for s in starts:
        i, k = s, 0
        while i >= 0 and k < max_steps:
            out[i] += math.exp(-k * cell_m / decay_m)
            i = recf[i]; k += 1
    return ndimage.gaussian_filter(out.reshape(h, w), 1.5)   # people don't walk the exact thalweg


def build_layers(grid, route_pts, incidents_pts):
    z = fill_depressions(grid.z)
    cell = (grid.dx + grid.dy) / 2
    rec, acc = d8(z, grid.dx, grid.dy)
    area_km2 = acc * grid.dx * grid.dy / 1e6
    streams = area_km2 >= STREAM_AREA_KM2
    gy, gx = np.gradient(grid.z, grid.dy, grid.dx)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))

    route = rasterize_route(grid, route_pts)
    sampling = (grid.dy, grid.dx)
    d_route = ndimage.distance_transform_edt(~route, sampling=sampling)
    d_stream = ndimage.distance_transform_edt(~streams, sampling=sampling)
    route_len = route.sum() * cell

    L = {}
    L["downstream"] = norm01(np.log1p(downstream_paths(rec, route, cell)))
    L["stream"] = np.exp(-d_stream / 150.0)
    L["trail"] = np.exp(-d_route / 200.0)
    L["alpine"] = (grid.z > TREELINE_M) * np.exp(-d_route / 600.0)
    trap = streams & ((slope > 25) | (area_km2 > 5))          # cascades/falls, big crossings
    L["trap"] = ndimage.gaussian_filter(trap.astype(float), 2)
    L["trap"] = norm01(L["trap"])
    hist = np.zeros(grid.z.shape)
    for lat, lon, wgt, _ in incidents_pts:
        r, c = grid.rc(lat, lon)
        if grid.inside(r, c):
            hist[r, c] += wgt
    L["history"] = norm01(ndimage.gaussian_filter(hist, 800 / cell)) if hist.any() else hist

    prior = distance_prior(d_route, route_len)
    mobility = np.clip(1.0 - np.maximum(slope - 35, 0) / 25, 0.15, 1.0)   # cliffs are hard to be on (but see trap)
    aux = dict(streams=streams, route=route, slope=slope, z=grid.z)
    return prior, mobility, L, aux


def combine(prior, mobility, L, weights=WEIGHTS):
    boost = 1 + sum(weights[k] * L[k] for k in weights)
    p = prior * mobility * boost
    return p / p.sum()


def bayes_update(prob, searched_mask, pod):
    """Cells searched with probability-of-detection `pod` and nothing found: P' = P(1-POD)/(1-ΣP·POD)."""
    p = prob.copy(); p[searched_mask] *= (1 - pod)
    return p / p.sum()


def find_hotspots(grid, prob, L, route, weights=WEIGHTS, k=10, k_trail=3, min_sep_m=500, trail_band_m=250):
    """Two lists, because the on-trail band always wins on density and gets a hasty search anyway:
    k_trail best trail segments, then k off-trail hot spots (> trail_band_m from the route)."""
    cell = (grid.dx + grid.dy) / 2
    d_route = ndimage.distance_transform_edt(~route, sampling=(grid.dy, grid.dx))
    on = d_route <= trail_band_m
    out = []
    for zone, mask, kk in (("trail", on, k_trail), ("off-trail", ~on, k)):
        sm = ndimage.gaussian_filter(np.where(mask, prob, 0), 150 / cell)
        size = max(3, int(min_sep_m / cell) | 1)
        peaks = (sm == ndimage.maximum_filter(sm, size=size)) & (sm > 0) & mask
        rr, cc = np.nonzero(peaks)
        order = np.argsort(sm[rr, cc])[::-1][:kk]
        out += _describe(grid, prob, L, weights, rr[order], cc[order], zone, cell, len(out))
    return out


def _describe(grid, prob, L, weights, rr, cc, zone, cell, start):
    rad = int(250 / cell)
    out = []
    for rank, (r, c) in enumerate(zip(rr, cc), start + 1):
        win = prob[max(0, r - rad):r + rad + 1, max(0, c - rad):c + rad + 1]
        contrib = {n: round(weights[n] * float(L[n][r, c]), 3) for n in weights}
        lat, lon = grid.latlon(r, c)
        out.append(dict(type="hotspot", t=time.time(), rank=rank, zone=zone, lat=round(lat, 6), lon=round(lon, 6),
                        row=int(r), col=int(c), elev_m=round(float(grid.z[r, c]), 1),
                        poa_250m=round(float(win.sum()), 4),
                        reasons=[n for n, v in sorted(contrib.items(), key=lambda kv: -kv[1]) if v > 0.05][:3],
                        contrib=contrib))
    return out


# ----------------------------------------------------------------------------- outputs
def save(out_dir, grid, prob, hotspots, aux, title):
    os.makedirs(out_dir, exist_ok=True)
    np.save(os.path.join(out_dir, "probability.npy"), prob)
    # georeferencing + layers for the ground-station map (ground_station/server.py)
    np.save(os.path.join(out_dir, "elevation.npy"), aux["z"].astype(np.float32))
    np.save(os.path.join(out_dir, "streams.npy"), aux["streams"].astype(np.uint8))
    np.save(os.path.join(out_dir, "route.npy"), aux["route"].astype(np.uint8))
    json.dump(dict(lat0=grid.lat0, lon0=grid.lon0, dlat=grid.dlat, dlon=grid.dlon,
                   rows=int(prob.shape[0]), cols=int(prob.shape[1]), dx=grid.dx, dy=grid.dy, title=title),
              open(os.path.join(out_dir, "grid.json"), "w"), indent=1)
    with open(os.path.join(out_dir, "hotspots.jsonl"), "w") as f:
        for h in hotspots:
            f.write(json.dumps(h) + "\n")
    gj = {"type": "FeatureCollection", "features": [
        {"type": "Feature", "geometry": {"type": "Point", "coordinates": [h["lon"], h["lat"]]},
         "properties": {k: v for k, v in h.items() if k not in ("lat", "lon")}} for h in hotspots]}
    json.dump(gj, open(os.path.join(out_dir, "hotspots.geojson"), "w"), indent=1)
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        from matplotlib.colors import LightSource
        fig, ax = plt.subplots(figsize=(9, 9))
        ax.imshow(LightSource(315, 45).hillshade(aux["z"], dx=grid.dx, dy=grid.dy), cmap="gray")
        lp = np.log10(prob + 1e-12)
        ax.imshow(np.ma.masked_less(lp, np.quantile(lp, 0.75)), cmap="inferno", alpha=0.55)
        ax.contour(aux["streams"], levels=[0.5], colors="#3aa0ff", linewidths=0.6)
        rr, cc = np.nonzero(aux["route"]); ax.plot(cc, rr, ".", ms=1.5, color="#7CFC00")
        for h in hotspots:
            col = "#7CFC00" if h["zone"] == "trail" else "cyan"
            ax.plot(h["col"], h["row"], "o", mfc="none", mec=col, ms=14, mew=2)
            ax.text(h["col"] + 6, h["row"], str(h["rank"]), color=col, fontsize=11, weight="bold")
        ax.set_title(title); ax.axis("off")
        fig.savefig(os.path.join(out_dir, "heatmap.png"), dpi=110, bbox_inches="tight"); plt.close(fig)
    except Exception as e:
        print("plot skipped:", e)


def pick_route(trail, gaz):
    byname = {g["name"]: g for g in gaz}
    key = next((k for k in TRAIL_ROUTES if trail and k.lower().split(" (")[0] in trail.lower()
                or (trail and trail.lower().split(" (")[0] in k.lower())), None)
    if not key:
        raise SystemExit(f"No route for trail '{trail}'. Known: {', '.join(TRAIL_ROUTES)}")
    return key, [(byname[n]["lat"], byname[n]["lon"]) for n in TRAIL_ROUTES[key]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--excel", help="workbook with the incident database sheet")
    ap.add_argument("--trail", default="Franconia Ridge Loop")
    ap.add_argument("--ipp", help="LAT,LON last known point (overrides trail start)")
    ap.add_argument("--dem", help="GeoTIFF elevation tile (lat/lon or projected)")
    ap.add_argument("--k", type=int, default=10)
    ap.add_argument("--out", default="out")
    a = ap.parse_args()

    if not a.dem:                                   # fall back to the shared height map (../dem.py)
        import sys; sys.path.insert(0, os.path.dirname(HERE))
        root = os.path.dirname(HERE)
        for cand in (os.environ.get("DEM_PATH", ""), os.path.join(root, "data", "height_map.tif"),
                     os.path.join(root, "dem", "height_map.tif"),
                     os.path.join(os.path.expanduser("~"), "Documents", "height_map.tif")):
            if cand and os.path.exists(cand):
                a.dem = cand; print("Using DEM:", a.dem); break
    gaz = load_gazetteer()
    if (a.trail or "").lower() in ("", "none", "point"):
        if not a.ipp:
            raise SystemExit("--trail none needs --ipp LAT,LON (the last known point)")
        lat, lon = map(float, a.ipp.split(","))
        name, route_pts = f"Last known point {lat:.4f}, {lon:.4f}", [(lat, lon)]
    else:
        name, route_pts = pick_route(a.trail, gaz)
        if a.ipp:
            lat, lon = map(float, a.ipp.split(","))
            route_pts = [(lat, lon)] + route_pts
    lats, lons = zip(*route_pts)
    kinds = {(g["lat"], g["lon"]): g["kind"] for g in gaz}
    summits = [p for p in route_pts if kinds.get(p) == "peak"]
    span_km = max(8.0, 111 * max(max(lats) - min(lats), (max(lons) - min(lons)) * 0.72) + 8)
    grid, src = load_dem(a.dem, center=(sum(lats) / len(lats), sum(lons) / len(lons)),
                         size_km=span_km, peaks=summits)
    if src == "synthetic":
        print("WARNING: synthetic terrain centered on the route - shapes are fake, use --dem for real search planning")

    incidents = load_incidents(a.excel) if a.excel else []
    pts = geocode_incidents(incidents, gaz)
    print(f"{len(incidents)} incidents, {len(pts)} geocoded, {sum(1 for p in pts if p[2] == 1.0)} lost/missing")

    prior, mob, L, aux = build_layers(grid, route_pts, pts)
    prob = combine(prior, mob, L)
    hs = find_hotspots(grid, prob, L, aux["route"], k=a.k)
    save(a.out, grid, prob, hs, aux, f"{name} - search probability ({src} terrain)")
    for h in hs:
        print(f"#{h['rank']:2d}  {h['lat']:.5f},{h['lon']:.5f}  elev {h['elev_m']:6.0f} m  "
              f"POA(250 m) {h['poa_250m']:.3f}  {h['zone']:9s} {', '.join(h['reasons'])}")


if __name__ == "__main__":
    main()
