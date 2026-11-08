"""Drone flight simulator: known 3D path -> what an MPU9250 on the airframe would read. Stdlib only.

Model: point-mass multirotor flying waypoints with limited acceleration. Thrust acts along the body z
axis, so the accelerometer reads (0, 0, |thrust|): it senses acceleration only through tilt, which is
exactly why accel-based tilt correction does not work in flight. Yaw follows the direction of travel.
Adds sensor bias/noise, motor vibration, and a magnetic disturbance from the motors in flight.
"""
import math
import random

from drcore import G, cross, dot, norm, unit


def _axes(u_axis, yaw):
    fh = (math.sin(yaw), math.cos(yaw), 0.0)
    f = unit(tuple(fh[i] - dot(fh, u_axis) * u_axis[i] for i in range(3)))
    l = cross(u_axis, f)
    return f, l, u_axis


def plan(waypoints, hz=100, vmax=5.0, amax=2.0, ground_secs=3.0):
    """Returns per-sample (pos, vel, acc, yaw, flying). waypoints: [(east, north, up), ...] after takeoff."""
    dt = 1.0 / hz
    out = []
    pos, vel, yaw = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0
    for _ in range(int(ground_secs * hz)):
        out.append((tuple(pos), tuple(vel), (0.0, 0.0, 0.0), yaw, False))
    targets = list(waypoints) + [(waypoints[-1][0], waypoints[-1][1], 0.0)]  # land under the last waypoint
    for ti, tgt in enumerate(targets):
        last = ti == len(targets) - 1
        for _ in range(int(300 * hz)):
            d = [tgt[i] - pos[i] for i in range(3)]
            dist = norm(d)
            if dist < (0.2 if last else 1.0) and (not last or norm(vel) < 0.15):
                break
            vdes_mag = min(vmax, math.sqrt(2 * 0.7 * amax * dist)) if (last or ti == len(targets) - 2) else vmax
            vdes = [d[i] / dist * vdes_mag for i in range(3)]
            acc = [(vdes[i] - vel[i]) / 0.8 for i in range(3)]
            an = norm(acc)
            if an > amax:
                acc = [x * amax / an for x in acc]
            for i in range(3):
                vel[i] += acc[i] * dt
                pos[i] += vel[i] * dt
            if math.hypot(vel[0], vel[1]) > 1.0:
                want = math.atan2(vel[0], vel[1])
                diff = (want - yaw + math.pi) % (2 * math.pi) - math.pi
                yaw += max(-math.radians(60) * dt, min(math.radians(60) * dt, diff))
            out.append((tuple(pos), tuple(vel), tuple(acc), yaw, True))
    for _ in range(int(ground_secs * hz)):
        out.append(((pos[0], pos[1], 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), yaw, False))
    return out


def simulate(waypoints, hz=100, seed=1, return_att=False, return_quat=False, att_err_deg=0.0, accel_bias=(0.08, -0.06, 0.05), gyro_bias_dps=(0.5, -0.3, 0.4),
             accel_noise=0.05, vib=0.4, gyro_noise_dps=0.1, gyro_vib_dps=0.3, mag_noise=0.5,
             motor_mag=(8.0, -5.0, 6.0), field_uT=50.0, inclination_deg=65.0, ideal=False, **plan_kw):
    """Returns (samples, truth). samples: [(t, accel_g, gyro_dps, mag_uT)], truth: [(e, n, u)] per sample."""
    rnd = random.Random(seed)
    traj = plan(waypoints, hz, **plan_kw)
    B = field_uT
    m_earth = (0.0, B * math.cos(math.radians(inclination_deg)), -B * math.sin(math.radians(inclination_deg)))
    if ideal:
        accel_bias, gyro_bias_dps, accel_noise, vib, gyro_noise_dps, gyro_vib_dps, mag_noise, motor_mag = (
            (0, 0, 0), (0, 0, 0), 0, 0, 0, 0, 0, (0, 0, 0))
    samples, truth, att, quats = [], [], [], []
    err_vec = [0.0, 0.0, 0.0]
    att_err_rad = math.radians(att_err_deg)
    Rprev = None
    for k, (pos, vel, acc, yaw, flying) in enumerate(traj):
        t = k / hz
        s = (acc[0], acc[1], acc[2] + G)                     # specific force in the world frame
        f, l, u = _axes(unit(s), yaw)
        R = (f, l, u)                                        # body axes expressed in the world frame
        if Rprev is None:
            gyro = (0.0, 0.0, 0.0)
        else:  # body rate from the skew part of R_prev^T R
            rel = [[dot(Rprev[i], R[j]) for j in range(3)] for i in range(3)]
            gyro = tuple(math.degrees(x * hz) for x in
                         ((rel[1][2] - rel[2][1]) / 2, (rel[2][0] - rel[0][2]) / 2, (rel[0][1] - rel[1][0]) / 2))
            # rel[i][j] = body_prev_i . body_j ; w = vee(skew(rel^T)) -> sign fixed by the convention below
            gyro = (-gyro[0], -gyro[1], -gyro[2])
        Rprev = R
        a_body = (0.0, 0.0, norm(s))                         # thrust only along body z
        vibn = vib if flying else 0.0
        accel = tuple((a_body[i] + accel_bias[i] + rnd.gauss(0, accel_noise) + rnd.gauss(0, vibn)) / G for i in range(3))
        gv = gyro_vib_dps if flying else 0.0
        gy = tuple(gyro[i] + gyro_bias_dps[i] + rnd.gauss(0, gyro_noise_dps) + rnd.gauss(0, gv) for i in range(3))
        mag = tuple(dot(m_earth, R[i]) + rnd.gauss(0, mag_noise) + (motor_mag[i] if flying else 0.0) for i in range(3))
        samples.append((t, accel, gy, mag))
        truth.append(pos)
        att.append(((f[2], l[2], u[2]), math.atan2(f[0], f[1])))   # world-up in body axes, compass heading
        # what a flight controller would report: quaternion body(FRD) -> world(NED), with a slowly varying error
        for i in range(3):
            err_vec[i] += -err_vec[i] * (1.0 / hz) / 20.0 + att_err_rad * math.sqrt(2.0 / hz / 20.0) * rnd.gauss(0, 1)
        Rm = [[(f[i], l[i], u[i])[j] for j in range(3)] for i in range(3)]        # columns = body axes, world ENU
        Rm = _rodrigues(err_vec, Rm)
        quats.append((t, _enu_flu_to_ned_frd_quat(Rm)))
    if return_quat:
        return samples, truth, quats
    return (samples, truth, att) if return_att else (samples, truth)


def _rodrigues(vec, Rm):
    """Apply a small world-frame rotation (rotation vector, rad) to a rotation matrix."""
    ang = math.sqrt(sum(c * c for c in vec))
    if ang < 1e-12:
        return Rm
    k = [c / ang for c in vec]
    K = ((0, -k[2], k[1]), (k[2], 0, -k[0]), (-k[1], k[0], 0))
    s, c = math.sin(ang), math.cos(ang)
    Re = [[(1 if i == j else 0) * c + s * K[i][j] + (1 - c) * k[i] * k[j] for j in range(3)] for i in range(3)]
    return [[sum(Re[i][m] * Rm[m][j] for m in range(3)) for j in range(3)] for i in range(3)]


def _enu_flu_to_ned_frd_quat(Rm):
    """Quaternion (w, x, y, z) of body(FRD) -> world(NED) from the ENU/FLU rotation matrix."""
    ned2enu = ((0, 1, 0), (1, 0, 0), (0, 0, -1))
    Rf = [[Rm[i][0], -Rm[i][1], -Rm[i][2]] for i in range(3)]
    R = [[sum(ned2enu[i][k] * Rf[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return (0.25 * s, (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s)
    i = max(range(3), key=lambda n: R[n][n])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(R[i][i] - R[j][j] - R[k][k] + 1.0) * 2
    q = [0.0] * 4
    q[1 + i] = 0.25 * s
    q[0] = (R[k][j] - R[j][k]) / s
    q[1 + j] = (R[j][i] + R[i][j]) / s
    q[1 + k] = (R[k][i] + R[i][k]) / s
    return tuple(q)
