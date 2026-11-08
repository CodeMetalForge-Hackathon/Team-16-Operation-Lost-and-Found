"""Height grid used as the prior map. Row is y, column is x."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class HeightMap:
    z: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def width(self) -> int:
        return int(self.z.shape[1])

    @property
    def height(self) -> int:
        return int(self.z.shape[0])

    def index(self, x, y):
        ix = np.floor((np.asarray(x) - self.origin_x) / self.resolution).astype(np.int32)
        iy = np.floor((np.asarray(y) - self.origin_y) / self.resolution).astype(np.int32)
        return ix, iy

    def contains(self, ix, iy):
        return (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)

    def sample(self, x, y):
        ix, iy = self.index(x, y)
        ok = self.contains(ix, iy)
        out = np.zeros(np.broadcast(np.asarray(x), np.asarray(y)).shape, dtype=np.float64)
        if out.shape == ():
            if bool(ok):
                return float(self.z[int(iy), int(ix)]), True
            return None, False
        out[ok] = self.z[iy[ok], ix[ok]]
        return out, ok

    def cell_center(self, ix: int, iy: int):
        x = self.origin_x + (ix + 0.5) * self.resolution
        y = self.origin_y + (iy + 0.5) * self.resolution
        return float(x), float(y)


def synthetic_hill(cells: int = 200, resolution: float = 10.0) -> HeightMap:
    """Asymmetric watershed plus fixed bumps so a few-hundred-meter patch is unique."""
    cols = np.arange(cells)
    rows = np.arange(cells)
    x = (cols + 0.5) * resolution
    y = (rows + 0.5) * resolution
    xx, yy = np.meshgrid(x, y)

    ridge = 180.0 * np.exp(-((yy - 0.55 * xx - 200.0) ** 2) / (2.0 * 180.0 ** 2))
    bowl = -90.0 * np.exp(-((xx - 1400.0) ** 2 + (yy - 500.0) ** 2) / (2.0 * 220.0 ** 2))
    spur = 120.0 * np.exp(-((xx - 400.0) ** 2 + (yy - 1500.0) ** 2) / (2.0 * 150.0 ** 2))
    base = 300.0 + 0.02 * xx + 0.01 * yy
    z = base + ridge + bowl + spur

    rng = np.random.default_rng(7)
    for _ in range(40):
        cx = float(rng.uniform(80.0, 1920.0))
        cy = float(rng.uniform(80.0, 1920.0))
        amp = float(rng.uniform(-40.0, 50.0))
        sigma = float(rng.uniform(40.0, 90.0))
        z += amp * np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2.0 * sigma ** 2))

    return HeightMap(z.astype(np.float64), resolution, 0.0, 0.0)


def save_heightmap(heightmap: HeightMap, directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    npy_path = directory / "synthetic_hill.npy"
    png_path = directory / "synthetic_hill.png"
    yaml_path = directory / "synthetic_hill.yaml"
    np.save(npy_path, heightmap.z)
    z_min = float(heightmap.z.min())
    z_max = float(heightmap.z.max())
    span = max(z_max - z_min, 1.0)
    image = np.round((heightmap.z - z_min) / span * 255.0).astype(np.uint8)
    _write_png(png_path, image)
    yaml_path.write_text(
        "\n".join(
            [
                f"resolution: {heightmap.resolution}",
                f"origin_x: {heightmap.origin_x}",
                f"origin_y: {heightmap.origin_y}",
                f"width: {heightmap.width}",
                f"height: {heightmap.height}",
                f"z_min: {z_min}",
                f"z_max: {z_max}",
                "npy: synthetic_hill.npy",
                "png: synthetic_hill.png",
                "frame_id: map",
                "",
            ]
        )
    )


def load_heightmap(yaml_path: Path) -> HeightMap:
    values = {}
    for line in Path(yaml_path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, raw = line.split(":", 1)
        values[key.strip()] = raw.strip()
    npy_name = values.get("npy", "synthetic_hill.npy")
    z = np.load(Path(yaml_path).parent / npy_name)
    return HeightMap(
        z.astype(np.float64),
        float(values["resolution"]),
        float(values["origin_x"]),
        float(values["origin_y"]),
    )


def _write_png(path: Path, image: np.ndarray) -> None:
    """Grayscale PNG, no third-party image library."""
    import struct
    import zlib

    if image.ndim != 2 or image.dtype != np.uint8:
        raise ValueError("png writer expects a uint8 grayscale image")
    height, width = image.shape

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + image[row].tobytes() for row in range(height))
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9))
    png += chunk(b"IEND", b"")
    path.write_bytes(png)
