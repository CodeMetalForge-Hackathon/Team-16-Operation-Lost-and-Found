"""Search planner shared by the simulator (and usable on the real pod).

Keeps a probability map of where the person is, lowers it wherever the pod's camera has looked
(Bayesian update, one factor of (1 - POD) per pass), and picks the next box to search: the 500 m
square that still holds the most probability.
"""
import json, math, os
import numpy as np
from scipy import ndimage


class Planner:
    def __init__(self, run_dir, footprint_m=60.0, pod=0.7, box_m=500.0):
        g = json.load(open(os.path.join(run_dir, "grid.json")))
        self.g, self.pod, self.box_m = g, pod, box_m
        self.prior = np.load(os.path.join(run_dir, "probability.npy")).astype(np.float64)
        self.prior /= self.prior.sum()
        self.passes = np.zeros(self.prior.shape, np.int16)
        self.cell_m = (g["dx"] + g["dy"]) / 2
        rad = max(1, int(footprint_m / self.cell_m))
        yy, xx = np.mgrid[-rad:rad + 1, -rad:rad + 1]
        self.disk = np.argwhere(yy ** 2 + xx ** 2 <= rad ** 2) - rad
        self.pass_mask = np.zeros(self.prior.shape, bool)          # cells seen during the current pass

    def rc(self, lat, lon):
        return int((lat - self.g["lat0"]) / self.g["dlat"]), int((lon - self.g["lon0"]) / self.g["dlon"])

    def latlon(self, r, c):
        return self.g["lat0"] + (r + 0.5) * self.g["dlat"], self.g["lon0"] + (c + 0.5) * self.g["dlon"]

    def see(self, lat, lon):
        """The camera footprint at this pose has been looked at during the current pass."""
        r0, c0 = self.rc(lat, lon)
        pts = self.disk + [r0, c0]
        ok = (pts[:, 0] >= 0) & (pts[:, 0] < self.prior.shape[0]) & (pts[:, 1] >= 0) & (pts[:, 1] < self.prior.shape[1])
        self.pass_mask[pts[ok, 0], pts[ok, 1]] = True

    def end_pass(self):
        """Nothing was found: everything seen this pass gets one more (1 - POD) factor."""
        self.passes[self.pass_mask] += 1
        self.pass_mask[:] = False

    def posterior(self):
        p = self.prior * (1 - self.pod) ** self.passes
        return p / p.sum()

    def searched_share(self):
        return float(1 - (self.prior * (1 - self.pod) ** self.passes).sum())

    def next_box(self):
        """Centre (lat, lon) of the box_m square holding the most remaining probability, and that probability."""
        post = self.posterior()
        size = max(3, int(self.box_m / self.cell_m))
        mass = ndimage.uniform_filter(post, size=size, mode="constant") * size * size
        r, c = np.unravel_index(int(np.argmax(mass)), mass.shape)
        lat, lon = self.latlon(r, c)
        return lat, lon, float(mass[r, c])

    def sample_location(self, rng, top_share=0.6):
        """Draw a plausible hiker location from the map (restricted to the most likely cells)."""
        p = self.prior.ravel()
        order = np.argsort(p)[::-1]
        keep = order[: max(1, int(np.searchsorted(np.cumsum(p[order]), top_share)) + 1)]
        w = p[keep] / p[keep].sum()
        i = int(rng.choice(keep, p=w))
        return self.latlon(*np.unravel_index(i, self.prior.shape))
