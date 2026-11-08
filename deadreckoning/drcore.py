"""Strapdown dead reckoning (INS coast) for a drone from an MPU9250 (accel + gyro + magnetometer). Stdlib only.

Role in the pod (see SKILL.md): this is the INS that COASTS between fixes. A lidar terrain match (another
module) supplies pose_fix messages, baseline one per second; apply_fix() folds them in with a Kalman
update, which corrects velocity as well as position and keeps the covariance small. Output: ins_coast
messages (t, x, y, z, covariance). x, y, z are metres East, North, Up in the map frame; start_xy is
where the drone starts in that frame. Coordinates in and out are decimal degrees (DD).

READ THIS FIRST - what an MPU9250 can and cannot do
  Position from integrating acceleration twice has error growing with t^2. A consumer MEMS accelerometer
  (bias ~0.05-0.2 m/s^2) drifts several metres in 10 s and 100+ m in a minute. On a multirotor it is worse:
  the accelerometer reads thrust along body z, not gravity, so tilt must come from the gyro alone and each
  degree of tilt error fakes 0.17 m/s^2 of horizontal acceleration. The fixes are what keep this usable.

Frames
  body : x forward, y left, z up (MPU9250 axes with the chip flat, arrow toward the drone's nose).
  world: East, North, Up. Heading = compass bearing (deg clockwise from north).
Input units: accel in g, gyro in deg/s, magnetometer in microtesla (AK8963 axes unless mag_aligned).
"""
import json
import math
import re
from collections import deque

G = 9.80665


# ------------------------------------------------------------------ small maths
def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(v):
    return math.sqrt(dot(v, v))


def unit(v):
    n = norm(v)
    return (v[0] / n, v[1] / n, v[2] / n) if n > 1e-9 else v


def quat_to_matrix(q):
    """Rotation matrix (rows) from a unit quaternion (w, x, y, z): v_world = R @ v_body."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def matvec(R, v):
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2])


def attitude_to_enu_flu(q, convention="ned_frd"):
    """Rotation matrix taking body (x fwd, y left, z up) vectors to world (East, North, Up).

    convention: "enu_flu" - quaternion already body(FLU) -> world(ENU) (ROS REP-103).
                "ned_frd" - quaternion body(FRD) -> world(NED): PX4 and ArduPilot / MAVLink ATTITUDE_QUATERNION.
    """
    R = quat_to_matrix(q)
    if convention == "enu_flu":
        return R
    if convention != "ned_frd":
        raise ValueError("convention must be ned_frd or enu_flu")
    ned2enu = ((0, 1, 0), (1, 0, 0), (0, 0, -1))
    # R_enu_flu = ned2enu @ R_ned_frd @ diag(1, -1, -1)
    Rf = tuple((row[0], -row[1], -row[2]) for row in R)
    return tuple(tuple(sum(ned2enu[i][k] * Rf[k][j] for k in range(3)) for j in range(3)) for i in range(3))


# ------------------------------------------------------------------ coordinates (decimal degrees)
def meters_per_degree(lat_deg):
    """Metres per degree of latitude and of longitude at this latitude (WGS84 series)."""
    p = math.radians(lat_deg)
    return (111132.92 - 559.82 * math.cos(2 * p) + 1.175 * math.cos(4 * p),
            111412.84 * math.cos(p) - 93.5 * math.cos(3 * p))


def parse_coord(text, kind="lat"):
    """Decimal degrees from text. Accepts 38.8977, -77.0365, '38.8977 N', and 38 53 51.7 N / 38°53'51.7"N."""
    s = str(text).strip().upper().replace(",", " ")
    if not s:
        raise ValueError("empty coordinate")
    sign = -1.0 if (s.startswith("-") or re.search(r"[SW]\s*$", s) or re.match(r"^[SW]\s", s)) else 1.0
    nums = [float(x) for x in re.findall(r"\d+(?:\.\d+)?", s)]
    if not nums or len(nums) > 3:
        raise ValueError(f"cannot read coordinate {text!r}")
    value = sum(n / 60 ** i for i, n in enumerate(nums)) * sign
    limit = 90.0 if kind == "lat" else 180.0
    if abs(value) > limit:
        raise ValueError(f"{kind}itude {value} is outside +/-{limit:g}")
    return value


def format_dd(lat, lon):
    return f"{lat:.6f}, {lon:.6f}"


# ------------------------------------------------------------------ attitude
class Attitude:
    """"Up" direction (in body axes) and compass heading of the forward axis.

    Gyro carries both. The accelerometer pulls "up" back (tau_acc seconds, None = never) only when it is
    trustworthy; the magnetometer pulls heading (tau_mag seconds) only when allowed and undisturbed.
    """

    def __init__(self, init_heading_deg=0.0, forward=(1.0, 0.0, 0.0), tau_mag=3.0):
        self.forward, self.tau_mag = forward, tau_mag
        self.tau_acc = 0.5
        self.mag_allowed = True
        self.u = None
        self.h = math.radians(init_heading_deg)
        self.mag_ref = None
        self.mag_ok = False

    def horizontal_axes(self):
        """North and East unit vectors in body coordinates (from up and the heading of the forward axis)."""
        f = self.forward
        fh = unit(tuple(f[i] - dot(f, self.u) * self.u[i] for i in range(3)))
        p = cross(self.u, fh)                      # fh rotated 90 deg counter-clockwise about up
        s, c = math.sin(self.h), math.cos(self.h)
        north = tuple(c * fh[i] + s * p[i] for i in range(3))
        east = tuple(s * fh[i] - c * p[i] for i in range(3))
        return north, east

    def _mag_heading(self, m):
        d = tuple(m[i] - dot(m, self.u) * self.u[i] for i in range(3))
        if norm(d) < 1e-6:
            return None
        n = unit(d)
        e = cross(n, self.u)
        return math.atan2(dot(self.forward, e), dot(self.forward, n))

    def update(self, a, w, m, dt):
        an = norm(a)
        if an < 1e-6:
            return
        a_hat = (a[0] / an, a[1] / an, a[2] / an)
        if self.u is None:
            self.u = a_hat
            if m is not None and self.mag_allowed:
                hm = self._mag_heading(m)
                if hm is not None:
                    self.h = hm
                    self.mag_ref = (norm(m), dot(m, self.u) / norm(m))
            return
        wxu = cross(w, self.u)                     # a world-fixed vector turns opposite to the body
        u = (self.u[0] - wxu[0] * dt, self.u[1] - wxu[1] * dt, self.u[2] - wxu[2] * dt)
        quality = max(0.0, 1.0 - abs(an / G - 1.0) / 0.25)
        k = 0.0 if self.tau_acc is None else min(1.0, dt / self.tau_acc) * quality
        self.u = unit(tuple(u[i] * (1 - k) + a_hat[i] * k for i in range(3)))
        # Azimuth rate of the forward axis. Only equals the body's rate about "up" when level; tilted, rolling
        # about the nose does not change the heading but turning about "up" does, so use the exact expression.
        f = self.forward
        fh = tuple(f[i] - dot(f, self.u) * self.u[i] for i in range(3))
        nfh = norm(fh)
        if nfh > 0.2:                               # skip when the forward axis points nearly straight up/down
            p = cross(self.u, fh)
            p = (p[0] / nfh, p[1] / nfh, p[2] / nfh)
            rate = dot(p, cross(w, f)) / nfh       # world "up" is fixed, so only the forward axis moves
            self.h -= rate * dt                    # counter-clockwise rate lowers the compass bearing
        self.mag_ok = False
        if m is not None and self.mag_allowed:
            mn = norm(m)
            if mn > 1e-6:
                cosdip = dot(m, self.u) / mn
                if self.mag_ref is None:
                    self.mag_ref = (mn, cosdip)
                if 0.7 * self.mag_ref[0] <= mn <= 1.3 * self.mag_ref[0] and abs(cosdip - self.mag_ref[1]) < 0.25:
                    hm = self._mag_heading(m)
                    if hm is not None:
                        self.h += wrap_pi(hm - self.h) * min(1.0, dt / self.tau_mag)
                        self.mag_ok = True
        self.h %= 2 * math.pi


class StepDetector:
    """Steps as peaks of the low-passed vertical acceleration (walking). Step length is fixed or Weinberg-sized."""

    def __init__(self, thr=1.0, fc=4.0, min_interval=0.3, k=0.5, fixed=0.7):
        self.thr, self.fc, self.min_interval, self.k, self.fixed = thr, fc, min_interval, k, fixed
        self.y = 0.0
        self.armed = False
        self.peak_v = self.peak_t = 0.0
        self.lo = 0.0
        self.last_t = -9.0

    def update(self, t, dt, a_vert):
        self.y += (a_vert - self.y) * dt / (dt + 1.0 / (2 * math.pi * self.fc))
        y = self.y
        self.lo = min(self.lo, y)
        if not self.armed:
            if y > self.thr and t - self.last_t > self.min_interval:
                self.armed, self.peak_v, self.peak_t = True, y, t
            return None
        if y > self.peak_v:
            self.peak_v, self.peak_t = y, t
            return None
        if y < 0.6 * self.peak_v:
            self.armed = False
            self.last_t = self.peak_t
            length = self.fixed if self.fixed else max(0.3, min(1.3, self.k * max(self.peak_v - self.lo, 0.1) ** 0.25))
            step = {"t": self.peak_t, "length": length, "amax": self.peak_v, "amin": self.lo}
            self.lo = y
            return step
        return None


def load_mag_cal(path):
    try:
        with open(path) as f:
            d = json.load(f)
        return tuple(d["offset"]), tuple(d["scale"])
    except (OSError, ValueError, KeyError):
        return None


# ------------------------------------------------------------------ dead reckoning
class DeadReckoner:
    """Strapdown INS. Position/velocity per axis are a 2-state Kalman filter driven by the IMU."""

    def __init__(self, lat=0.0, lon=0.0, alt=0.0, start_xy=(0.0, 0.0), init_heading_deg=0.0,
                 declination_deg=0.0, mag_in_flight=False, mag_aligned=False, bias_secs=2.0,
                 accel_sigma=0.5, forward=(1.0, 0.0, 0.0), mag_cal=None, landed_agl=3.0, walk_step_m=0.7, use_mag=True):
        self.mag_in_flight, self.mag_aligned, self.mag_cal = mag_in_flight, mag_aligned, mag_cal
        self.declination = math.radians(declination_deg)
        self.bias_secs, self.forward, self.accel_sigma = bias_secs, forward, accel_sigma
        self.landed_agl = landed_agl
        self.walk_step_m = walk_step_m              # fixed step length used in walking mode, metres
        self.use_mag = use_mag                      # False: heading from the gyro only (indoors the compass is unreliable)
        self.mode = "auto"                          # auto | flying | grounded (override from the UI)
        self.bias = (0.0, 0.0, 0.0)
        self.bias_ok = False
        self.acc_scale = 1.0                        # accelerometer scale correction, learned from gravity on the ground
        self.reset(lat, lon, alt, start_xy, init_heading_deg, keep_bias=False)

    # ------------------------------------------------------------------ control
    def reset(self, lat=None, lon=None, alt=None, start_xy=None, init_heading_deg=None, keep_bias=True,
              airborne=False):
        """New start point in decimal degrees (and optionally its map x,y). Heading comes from the
        magnetometer on the ground, else from init_heading_deg. airborne=True: the drone is in the air (the
        start was changed mid-flight), so stay in flight mode with a large uncertainty and let fixes pull it in."""
        if lat is not None:
            self.start_lat, self.start_lon = float(lat), float(lon)
        if alt is not None:
            self.start_alt = float(alt)
        if start_xy is not None:
            self.start_xy = (float(start_xy[0]), float(start_xy[1]))
        if init_heading_deg is not None:
            self.init_heading_deg = float(init_heading_deg)
        self.att = Attitude(self.init_heading_deg - math.degrees(self.declination), self.forward)
        self.att.mag_allowed = self.use_mag
        self.p = [self.start_xy[0], self.start_xy[1], 0.0]      # x east, y north (map frame, m), z up from start
        self.v = [0.0, 0.0, 0.0]
        self.P = [[1.0, 0.0, 0.01] for _ in range(3)]            # per axis: var(p), cov(p,v), var(v)
        self.path = [tuple(self.p)]
        self.t_prev = self.t0 = None
        if not keep_bias:
            self.bias, self.bias_ok = (0.0, 0.0, 0.0), False
        self._calib = [] if not self.bias_ok else None
        self._calib_a = []
        self._win = deque()
        self.grounded = True
        self._still_since = None
        self.samples = 0
        self.rate = 0.0
        self.fixes = 0
        self.fixes_rejected = 0
        self.steps = 0
        self.distance = 0.0
        self.last_step_len = 0.0
        self.walking_still = False
        self.steps_det = StepDetector(fixed=self.walk_step_m)
        self._hx = self._hy = 0.0
        self.last_fix_t = None
        self.agl = None
        self.agl_in = None                          # latest lidar height above ground, set by the service
        self.R_ext = None                           # latest orientation from the flight controller (body -> ENU)
        self.R_ext_t = None
        if airborne:
            self.grounded = False
            self._calib = None
            self.P = [[2500.0, 0.0, 100.0] for _ in range(3)]

    def feed_attitude(self, t, quat, convention="ned_frd"):
        """Orientation from the drone's flight controller, quaternion (w, x, y, z). When it is fresh the
        INS uses it instead of its own gyro/accel/mag attitude filter, which is far better in flight."""
        self.R_ext = attitude_to_enu_flu(quat, convention)
        self.R_ext_t = t

    def set_mode(self, mode):
        if mode not in ("auto", "flying", "grounded", "walking"):
            raise ValueError("mode must be auto, flying, grounded or walking")
        if mode == "walking" and self.mode != "walking":
            self.steps_det = StepDetector(fixed=self.walk_step_m)
            self._hx = self._hy = 0.0
        self.mode = mode

    # ------------------------------------------------------------------ fixes from other modules
    def _update_axis(self, axis, meas, sigma, gate):
        P = self.P[axis]
        S = P[0] + sigma * sigma
        innov = float(meas) - self.p[axis]
        if gate and abs(innov) > gate * math.sqrt(S):
            return False
        k0, k1 = P[0] / S, P[1] / S
        self.p[axis] += k0 * innov
        self.v[axis] += k1 * innov
        P[2] = P[2] - k1 * P[1]
        P[1] = (1 - k0) * P[1]
        P[0] = (1 - k0) * P[0]
        return True

    def apply_fix(self, x, y, z=None, sigma=5.0, sigma_z=2.0, gate=6.0, heading=None, heading_sigma=5.0):
        """Fold in a pose_fix (lidar terrain match), x,y,z in the same map frame. sigma is the fix's 1-sigma
        error in metres. It corrects velocity too, through the filter's position-velocity correlation.
        Returns True if used, False if rejected by the innovation gate (inconsistent with the filter)."""
        ok = True
        for axis, meas, sg in ((0, x, sigma), (1, y, sigma)):
            ok &= self._update_axis(axis, meas, sg, gate)
        if z is not None:
            self._update_axis(2, float(z) - self.start_alt, sigma_z, gate)
        if ok and heading is not None:   # a lidar scan match also gives the facing direction: pull the gyro heading to it
            meas = math.radians(float(heading)) - self.declination
            gain = 0.3 * min(1.0, 5.0 / max(float(heading_sigma), 0.5))
            self.att.h = (self.att.h + wrap_pi(meas - self.att.h) * gain) % (2 * math.pi)
        if ok:
            self.fixes += 1
            self.last_fix_t = self.t_prev
        else:
            self.fixes_rejected += 1
        return ok

    def baro(self, alt_m, sigma=2.0):
        """Absolute altitude (m) from a barometer, as a vertical-only fix."""
        self._update_axis(2, float(alt_m) - self.start_alt, sigma, 0)

    # ------------------------------------------------------------------ input
    def _prep_mag(self, m):
        if m is None:
            return None
        m = tuple(m)
        if self.mag_cal:
            off, sc = self.mag_cal
            m = tuple((m[i] - off[i]) * sc[i] for i in range(3))
        if not self.mag_aligned:                    # AK8963 axes -> accel/gyro axes: (x, y, z) -> (y, x, -z)
            m = (m[1], m[0], -m[2])
        return m

    def _predict_cov(self, dt):
        q = self.accel_sigma ** 2
        for P in self.P:
            p00, p01, p11 = P
            P[0] = p00 + 2 * dt * p01 + dt * dt * p11 + q * dt ** 3 / 3
            P[1] = p01 + dt * p11 + q * dt * dt / 2
            P[2] = p11 + q * dt

    def feed(self, t, accel_g, gyro_dps, mag_uT=None, agl=None):
        """One IMU sample (t in s). agl = height above ground from the lidar, if known: it tells the filter
        when the drone is really on the ground (a smooth cruise looks motionless to an IMU)."""
        if self.t_prev is None:
            self.t_prev = self.t0 = t
            return
        dt = t - self.t_prev
        if dt <= 0 or dt > 0.2:
            self.t_prev = t
            return
        self.t_prev = t
        self.samples += 1
        self.agl = agl
        self.rate = 0.98 * self.rate + 0.02 / dt if self.rate else 1.0 / dt
        a_raw = tuple(x * G for x in accel_g)
        a = tuple(x * self.acc_scale for x in a_raw)
        w_raw = tuple(math.radians(x) for x in gyro_dps)
        m = self._prep_mag(mag_uT)

        if self._calib is not None:                 # startup: sit still on the ground to measure gyro bias + gravity
            self._calib.append(w_raw)
            self._calib_a.append(norm(a_raw))
            self.att.update(a, (0.0, 0.0, 0.0), m, dt)
            if t - self.t0 >= self.bias_secs and len(self._calib) > 10:
                n = len(self._calib)
                mean = tuple(sum(s[i] for s in self._calib) / n for i in range(3))
                spread = max(math.sqrt(sum((s[i] - mean[i]) ** 2 for s in self._calib) / n) for i in range(3))
                self.bias_ok = spread < math.radians(2.0)   # moved during calibration: do not trust it
                self.bias = mean if self.bias_ok else (0.0, 0.0, 0.0)
                if self.bias_ok:                     # the sensor's gravity reading fixes its scale (typically +-2%)
                    scale = G / (sum(self._calib_a) / len(self._calib_a))
                    if 0.9 < scale < 1.1:
                        self.acc_scale = scale
                self._calib = None
            return

        w = tuple(w_raw[i] - self.bias[i] for i in range(3))

        if self.mode == "walking":                  # on foot: step counting, not double integration
            self._feed_walk(t, dt, a, w, m)
            return

        # landed = still for 2 s AND low (or unknown) altitude; moving = flying.
        # "Still" means a steady accelerometer and almost no rotation. It looks at how much |a| varies, not
        # its absolute size, so a few percent of accelerometer scale error cannot fake motion.
        self._win.append((t, norm(w), norm(a)))
        while self._win and t - self._win[0][0] > 0.5:
            self._win.popleft()
        if self.mode != "auto":
            self.grounded = self.mode == "grounded"
        elif len(self._win) > 10:
            gm = sum(x[1] for x in self._win) / len(self._win)
            amean = sum(x[2] for x in self._win) / len(self._win)
            astd = math.sqrt(sum((x[2] - amean) ** 2 for x in self._win) / len(self._win))
            still = gm < math.radians(3.0) and astd < 0.15 and abs(amean - G) < 0.8
            low = agl is None or agl < self.landed_agl
            if still and low:
                self._still_since = self._still_since or t
                if t - self._still_since > 2.0:
                    self.grounded = True
            else:
                self._still_since = None
                if not still:
                    self.grounded = False

        # attitude: on the ground trust accel and mag; in flight the accel reads thrust, not gravity, so
        # tilt comes from the gyro alone (any accel pull-back would fake horizontal acceleration)
        ext = self.R_ext is not None and t - self.R_ext_t < 0.5
        if ext:      # the drone's own orientation wins; keep the fallback filter in step for display/handover
            R = self.R_ext
            self.att.u = (R[2][0], R[2][1], R[2][2])           # world "up" in body axes = third row of R
            self.att.h = math.atan2(R[0][0], R[1][0]) % (2 * math.pi)   # heading of the body x axis
            self.att.mag_ok = False
        else:
            self.att.tau_acc = 0.5 if self.grounded else None
            self.att.mag_allowed = (self.grounded or self.mag_in_flight) and self.use_mag
            self.att.update(a, w, m, dt)

        if self.grounded:
            self.v = [0.0, 0.0, 0.0]               # landed: not moving
            self.P = [[min(P[0], 1.0), 0.0, 0.01] for P in self.P]
            if len(self._win) > 10 and norm(w) < math.radians(1.0):   # refine gyro bias and gravity scale while still
                r = min(1.0, dt / 5.0)
                self.bias = tuple(self.bias[i] * (1 - r) + w_raw[i] * r for i in range(3))
                if 0.9 < G / max(norm(a_raw), 1e-6) < 1.1:
                    self.acc_scale += (G / norm(a_raw) - self.acc_scale) * r
            return

        if ext:
            aw = matvec(self.R_ext, a)
            acc_world = (aw[0], aw[1], aw[2] - G)
        else:
            north, east = self.att.horizontal_axes()
            acc_world = (dot(a, east), dot(a, north), dot(a, self.att.u) - G)
        for i in range(3):
            self.v[i] += acc_world[i] * dt
            self.p[i] += self.v[i] * dt
        self._predict_cov(dt)
        if self.samples % 10 == 0:
            self.path.append(tuple(self.p))
            if len(self.path) > 8000:
                self.path = self.path[::2]

    def _feed_walk(self, t, dt, a, w, m):
        """Pedestrian dead reckoning for the walking demo. Step = a peak of the vertical acceleration; each
        step moves walk_step_m along the compass heading (gyro for turns, magnetometer for absolute heading).
        Hold the board level, its x arrow pointing the way you walk. Error grows with distance walked
        (a few percent plus heading error), not with time squared."""
        self.att.tau_acc = 0.5                      # a walking person's accelerometer still senses gravity
        self.att.mag_allowed = self.use_mag
        self.att.update(a, w, m, dt)
        a_vert = dot(a, self.att.u) - G
        self._win.append((t, norm(w), norm(a)))
        while self._win and t - self._win[0][0] > 0.5:
            self._win.popleft()
        still = False
        if len(self._win) > 10:
            gm = sum(x[1] for x in self._win) / len(self._win)
            amean = sum(x[2] for x in self._win) / len(self._win)
            astd = math.sqrt(sum((x[2] - amean) ** 2 for x in self._win) / len(self._win))
            still = gm < math.radians(3.0) and astd < 0.15
        self.walking_still = still
        hd = self.att.h + self.declination
        self._hx += math.sin(hd)
        self._hy += math.cos(hd)
        if still:
            self.steps_det.y *= 0.9                 # standing still must never arm a step
            self._hx = self._hy = 0.0
            return
        step = self.steps_det.update(t, dt, a_vert)
        if not step:
            return
        heading = math.atan2(self._hx, self._hy) if (self._hx or self._hy) else hd
        self._hx = self._hy = 0.0
        L = step["length"]
        self.p[0] += L * math.sin(heading)
        self.p[1] += L * math.cos(heading)
        self.steps += 1
        self.distance += L
        self.last_step_len = L
        for P in self.P[:2]:                        # each step adds ~15% of its length of position uncertainty
            P[0] += (0.15 * L) ** 2
        self.path.append(tuple(self.p))
        if len(self.path) > 8000:
            self.path = self.path[::2]

    # ------------------------------------------------------------------ output
    def latlon(self):
        mlat, mlon = meters_per_degree(self.start_lat)
        return (self.start_lat + (self.p[1] - self.start_xy[1]) / mlat,
                self.start_lon + (self.p[0] - self.start_xy[0]) / mlon)

    def covariance(self):
        """3x3 position covariance (m^2), x/y/z. Axes are treated as independent."""
        return [[self.P[0][0], 0.0, 0.0], [0.0, self.P[1][0], 0.0], [0.0, 0.0, self.P[2][0]]]

    def sigma_h(self):
        return math.sqrt(0.5 * (self.P[0][0] + self.P[1][0]))

    def ins_coast(self):
        """The ins_coast message from SKILL.md, plus decimal-degree lat/lon."""
        lat, lon = self.latlon()
        return {"msg": "ins_coast", "t": round(self.t_prev or 0.0, 3),
                "x": round(self.p[0], 3), "y": round(self.p[1], 3), "z": round(self.p[2] + self.start_alt, 3),
                "covariance": [[round(c, 3) for c in row] for row in self.covariance()],
                "lat": round(lat, 6), "lon": round(lon, 6)}

    def state(self, path_points=600):
        lat, lon = self.latlon()
        path = self.path if len(self.path) <= path_points else self.path[:: len(self.path) // path_points + 1] + [self.path[-1]]
        return {
            "calibrating": self._calib is not None, "bias_ok": self.bias_ok,
            "mode": self.mode, "flying": not self.grounded and self.mode != "walking",
            "walking": self.mode == "walking", "steps": self.steps, "distance_m": round(self.distance, 1),
            "step_len_m": round(self.last_step_len, 2),
            "lat": round(lat, 6), "lon": round(lon, 6), "dd": format_dd(lat, lon),
            "x": round(self.p[0], 2), "y": round(self.p[1], 2), "z": round(self.p[2] + self.start_alt, 2),
            "start": {"lat": self.start_lat, "lon": self.start_lon, "alt": self.start_alt, "heading": self.init_heading_deg,
                      "x": self.start_xy[0], "y": self.start_xy[1]},
            "speed_ms": round(math.hypot(self.v[0], self.v[1]), 2), "climb_ms": round(self.v[2], 2),
            "heading_deg": round(math.degrees(self.att.h + self.declination) % 360, 1),
            "mag_ok": self.att.mag_ok, "use_mag": self.use_mag, "sigma_h_m": round(self.sigma_h(), 1),
            "sigma_z_m": round(math.sqrt(self.P[2][0]), 1),
            "fixes": self.fixes, "fixes_rejected": self.fixes_rejected,
            "gyro_bias_dps": [round(math.degrees(b), 2) for b in self.bias], "accel_scale": round(self.acc_scale, 4),
            "rate_hz": round(self.rate, 1),
            "path": [[round(e, 1), round(n, 1), round(u, 1)] for e, n, u in path],
        }
