"""Small geo helpers for the ground station (no pyproj needed)."""
import math


def utm_to_latlon(x, y, zone=19):
    """UTM (northern hemisphere, WGS84/NAD83) -> (lat, lon) in degrees. Zone 19 = EPSG:26919 / 32619."""
    a, f, k0 = 6378137.0, 1 / 298.257223563, 0.9996
    e2 = f * (2 - f); ep2 = e2 / (1 - e2)
    x = x - 500000.0
    mu = (y / k0) / (a * (1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256))
    e1 = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
    p = (mu + (3 * e1 / 2 - 27 * e1 ** 3 / 32) * math.sin(2 * mu)
         + (21 * e1 ** 2 / 16 - 55 * e1 ** 4 / 32) * math.sin(4 * mu)
         + (151 * e1 ** 3 / 96) * math.sin(6 * mu) + (1097 * e1 ** 4 / 512) * math.sin(8 * mu))
    s, c, t = math.sin(p), math.cos(p), math.tan(p)
    N1 = a / math.sqrt(1 - e2 * s * s); T1 = t * t; C1 = ep2 * c * c
    R1 = a * (1 - e2) / (1 - e2 * s * s) ** 1.5; D = x / (N1 * k0)
    lat = p - (N1 * t / R1) * (D ** 2 / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 ** 2 - 9 * ep2) * D ** 4 / 24
                               + (61 + 90 * T1 + 298 * C1 + 45 * T1 ** 2 - 252 * ep2 - 3 * C1 ** 2) * D ** 6 / 720)
    lon = (D - (1 + 2 * T1 + C1) * D ** 3 / 6
           + (5 - 2 * C1 + 28 * T1 - 3 * C1 ** 2 + 8 * ep2 + 24 * T1 ** 2) * D ** 5 / 120) / c
    return math.degrees(lat), (zone - 1) * 6 - 180 + 3 + math.degrees(lon)


def to_latlon(rec):
    """Accept {'lat','lon'} or UTM-19N {'x','y'} (metres) and return (lat, lon) or None."""
    if rec.get("lat") is not None and rec.get("lon") is not None:
        return float(rec["lat"]), float(rec["lon"])
    if rec.get("x") is not None and rec.get("y") is not None and float(rec["y"]) > 1e6:
        return utm_to_latlon(float(rec["x"]), float(rec["y"]))
    return None


def dist_m(a, b):
    return math.hypot((a[0] - b[0]) * 111_320, (a[1] - b[1]) * 111_320 * math.cos(math.radians(a[0])))


def latlon_to_utm(lat, lon, zone=19):
    """(lat, lon) degrees -> UTM northern-hemisphere (x, y) metres. Zone 19 = EPSG:26919."""
    a, f, k0 = 6378137.0, 1 / 298.257223563, 0.9996
    e2 = f * (2 - f); ep2 = e2 / (1 - e2)
    phi, lam = math.radians(lat), math.radians(lon)
    lam0 = math.radians((zone - 1) * 6 - 180 + 3)
    s, c, t = math.sin(phi), math.cos(phi), math.tan(phi)
    N = a / math.sqrt(1 - e2 * s * s); T = t * t; C = ep2 * c * c; A = c * (lam - lam0)
    M = a * ((1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256) * phi
             - (3 * e2 / 8 + 3 * e2 ** 2 / 32 + 45 * e2 ** 3 / 1024) * math.sin(2 * phi)
             + (15 * e2 ** 2 / 256 + 45 * e2 ** 3 / 1024) * math.sin(4 * phi)
             - (35 * e2 ** 3 / 3072) * math.sin(6 * phi))
    x = k0 * N * (A + (1 - T + C) * A ** 3 / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ep2) * A ** 5 / 120) + 500000.0
    y = k0 * (M + N * t * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A ** 4 / 24
                           + (61 - 58 * T + T * T + 600 * C - 330 * ep2) * A ** 6 / 720))
    return x, y
