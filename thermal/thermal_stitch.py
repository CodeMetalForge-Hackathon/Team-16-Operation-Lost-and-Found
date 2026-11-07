"""Multi-frame thermal fusion to see through occluding canopy (synthetic aperture).

A person under trees is hidden in any single thermal frame, but from a moving drone the
leaves cover different spots in each frame. Project every frame onto the same ground grid
using its pose, then combine: the warm target is stable in ground coordinates while the
canopy (which sits above the ground) smears out and is rejected by a high percentile.

Model: nadir-looking pinhole camera at (x, y, z) metres above a flat ground plane (z=0),
heading `yaw` radians. Frames are projected onto a ground grid with cv2.remap.

  python thermal_stitch.py --demo                 simulate a drone sweep, save data/stitch_demo.png
  python thermal_stitch.py --demo --pose-noise 0.5   same, with 0.5 m pose error (shows sensitivity)
  python thermal_stitch.py --selftest             pass/fail check of the whole idea
  python thermal_stitch.py --frames-dir D --poses P  fuse real frames; P is a CSV: file,x,y,z,yaw

Run from inside thermal/.
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

W, H = 256, 192          # HT-203U thermal preview size
HFOV_DEG = 50.0          # horizontal field of view (assumed; set --hfov to the real value)


def focal(hfov_deg=HFOV_DEG, width=W):
    return (width / 2) / np.tan(np.radians(hfov_deg) / 2)


class Grid:
    """Ground grid: res metres per cell, covering [x0, x0+w*res] x [y0, y0+h*res]."""

    def __init__(self, x0, y0, w, h, res):
        self.x0, self.y0, self.w, self.h, self.res = x0, y0, w, h, res
        jj, ii = np.meshgrid(np.arange(w), np.arange(h))
        self.X = (x0 + jj * res).astype(np.float32)
        self.Y = (y0 + ii * res).astype(np.float32)


def project(frame, pose, grid, f):
    """Warp one frame onto the ground grid. Returns (values, valid) both grid-shaped."""
    x, y, z, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    dx, dy = grid.X - x, grid.Y - y
    a = (dx * c + dy * s) / z
    b = (-dx * s + dy * c) / z
    u = (W / 2 + f * a).astype(np.float32)
    v = (H / 2 + f * b).astype(np.float32)
    val = cv2.remap(frame.astype(np.float32), u, v, cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=np.nan)
    valid = (u >= 0) & (u <= W - 1) & (v >= 0) & (v <= H - 1)
    val[~valid] = np.nan
    return val, valid


def fuse(frames, poses, grid, f, mode="p85"):
    """Combine frames on the ground grid. mode: mean | max | median | pNN (percentile)."""
    stack = np.stack([project(fr, p, grid, f)[0] for fr, p in zip(frames, poses)])
    cnt = np.sum(~np.isnan(stack), axis=0)
    with np.errstate(all="ignore"):
        if mode == "mean":
            out = np.nanmean(stack, axis=0)
        elif mode == "max":
            out = np.nanmax(stack, axis=0)
        elif mode == "median":
            out = np.nanmedian(stack, axis=0)
        elif mode.startswith("p"):
            out = np.nanpercentile(stack, float(mode[1:]), axis=0)
        else:
            raise ValueError(mode)
    out[cnt < 3] = np.nan  # need a few looks before trusting a cell
    return out, cnt


# --------------------------------------------------------------- simulation
def make_world(rng, size=60.0, res=0.1, cover=0.7):
    """Ground temperature field with a warm person, and a canopy occupancy mask above it."""
    n = int(size / res)
    ground = np.full((n, n), 0.20, np.float32)
    person = (size / 2, size / 2)            # world metres
    yy, xx = np.mgrid[0:n, 0:n] * res
    body = ((xx - person[0]) / 0.45) ** 2 + ((yy - person[1]) / 0.25) ** 2 < 1   # ~0.9 x 0.5 m
    ground[body] = 0.90
    ground += rng.normal(0, 0.01, ground.shape).astype(np.float32)
    noise = cv2.GaussianBlur(rng.random((n, n)).astype(np.float32), (0, 0), 6)  # ~1 m leaf clumps
    thr = np.quantile(noise, 1 - cover)
    canopy = noise >= thr
    return ground, canopy, person, res


def render(ground, canopy, res, pose, f, zc, rng, noise=0.01):
    """Thermal frame seen by the drone. Canopy at height zc blocks the ground where it is solid."""
    x, y, z, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    jj, ii = np.meshgrid(np.arange(W), np.arange(H))
    a, b = (jj - W / 2) / f, (ii - H / 2) / f

    def sample(depth, field):
        X = x + depth * (a * c - b * s)
        Y = y + depth * (a * s + b * c)
        gi = np.clip((Y / res).astype(int), 0, field.shape[0] - 1)
        gj = np.clip((X / res).astype(int), 0, field.shape[1] - 1)
        return field[gi, gj]

    g = sample(z, ground)
    solid = sample(z - zc, canopy)
    img = np.where(solid, 0.35, g)               # leaves read cooler than a person
    return (img + rng.normal(0, noise, img.shape)).astype(np.float32)


def simulate(n_frames=60, alt=20.0, canopy_h=8.0, cover=0.7, pose_noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    ground, canopy, person, res = make_world(rng, cover=cover)
    f = focal()
    # lawnmower pass over the person, with a little jitter in heading
    xs = np.linspace(person[0] - 15, person[0] + 15, n_frames)   # wide sweep = varied view angles
    ys = person[1] + 6.0 * np.sin(np.linspace(0, 3 * np.pi, n_frames))
    true = [(xs[i], ys[i], alt, rng.normal(0, 0.02)) for i in range(n_frames)]
    frames = [render(ground, canopy, res, p, f, canopy_h, rng) for p in true]
    est = [(p[0] + rng.normal(0, pose_noise), p[1] + rng.normal(0, pose_noise), p[2], p[3])
           for p in true]
    return frames, true, est, person, f


def contrast(img, grid, person, radius=0.35):
    """(person mean - background mean) / background std, over cells with data."""
    d = np.hypot(grid.X - person[0], grid.Y - person[1])
    ok = ~np.isnan(img)
    pm, bm = ok & (d < radius), ok & (d > 2.0) & (d < 8.0)
    if pm.sum() == 0 or bm.sum() == 0:
        return 0.0
    return float((img[pm].mean() - img[bm].mean()) / (img[bm].std() + 1e-6))


def run_demo(args):
    frames, true, est, person, f = simulate(args.frames, pose_noise=args.pose_noise, seed=args.seed)
    grid = Grid(person[0] - 8, person[1] - 8, 160, 160, 0.1)
    mid = len(frames) // 2
    single, _ = project(frames[mid], true[mid], grid, f)
    results = {"single frame": single}
    for m in ("mean", "p85"):
        results[f"fused {m}"], _ = fuse(frames, est, grid, f, m)
    print(f"frames={len(frames)}  pose error={args.pose_noise} m")
    for k, v in results.items():
        print(f"  {k:14s} person contrast = {contrast(v, grid, person):6.2f}")
    return results, grid, person, frames[mid]


def save_panel(results, grid, person, raw, path):
    tiles = [("raw drone frame", raw)] + list(results.items())
    out = []
    for name, im in tiles:
        im = np.nan_to_num(im, nan=0.0)
        g = cv2.resize(np.clip(im * 255, 0, 255).astype(np.uint8), (320, int(320 * im.shape[0] / im.shape[1])),
                       interpolation=cv2.INTER_NEAREST)
        g = cv2.applyColorMap(g, cv2.COLORMAP_INFERNO)
        if name != "raw drone frame":  # circle the true person location
            px = int((person[0] - grid.x0) / grid.res * 320 / grid.w)
            py = int((person[1] - grid.y0) / grid.res * g.shape[0] / grid.h)
            cv2.circle(g, (px, py), 22, (255, 255, 255), 1)
        g = cv2.copyMakeBorder(g, 28, 0, 0, 0, cv2.BORDER_CONSTANT)
        cv2.putText(g, name, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        out.append(cv2.resize(g, (320, 28 + 320)))
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), np.hstack(out))
    print("saved", path)


def load_real(frames_dir, poses_csv):
    frames, poses = [], []
    with open(poses_csv, newline="") as fh:
        for r in csv.DictReader(fh):
            img = cv2.imread(str(Path(frames_dir) / r["file"]), cv2.IMREAD_GRAYSCALE)
            if img is None:
                sys.exit(f"cannot read {r['file']}")
            frames.append(cv2.resize(img, (W, H)).astype(np.float32) / 255.0)
            poses.append((float(r["x"]), float(r["y"]), float(r["z"]), float(r["yaw"])))
    return frames, poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--pose-noise", type=float, default=0.0, help="metres of pose error (demo)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="data/stitch_demo.png")
    ap.add_argument("--frames-dir")
    ap.add_argument("--poses")
    ap.add_argument("--hfov", type=float, default=HFOV_DEG)
    ap.add_argument("--mode", default="p85")
    ap.add_argument("--res", type=float, default=0.1, help="ground grid metres per cell")
    a = ap.parse_args()

    if a.selftest:
        ok = True
        for seed in range(6):
            a.seed, a.pose_noise = seed, 0.0
            r, grid, person, _ = run_demo(a)
            cs, cm, cp = (contrast(r[k], grid, person) for k in ("single frame", "fused mean", "fused p85"))
            ok &= cp > 3 * max(cs, 0.5) and cp > cm and cp > 5
        print("SELFTEST", "PASS" if ok else "FAIL")
        sys.exit(0 if ok else 1)
    if a.demo:
        r, grid, person, raw = run_demo(a)
        save_panel(r, grid, person, raw, a.out)
        return
    if a.frames_dir and a.poses:
        frames, poses = load_real(a.frames_dir, a.poses)
        xs, ys = [p[0] for p in poses], [p[1] for p in poses]
        pad = 5.0
        grid = Grid(min(xs) - pad, min(ys) - pad, int((max(xs) - min(xs) + 2 * pad) / a.res),
                    int((max(ys) - min(ys) + 2 * pad) / a.res), a.res)
        fused, cnt = fuse(frames, poses, grid, focal(a.hfov), a.mode)
        cv2.imwrite(a.out, cv2.applyColorMap(np.clip(np.nan_to_num(fused) * 255, 0, 255).astype(np.uint8),
                                             cv2.COLORMAP_INFERNO))
        print("saved", a.out)
        return
    ap.print_help()


if __name__ == "__main__":
    main()
