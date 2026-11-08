"""Hot-blob detector for the pod HIK UVC thermal camera.

The capture path opens the HikCamera V4L2 node, reads one raw frame, and
turns it into a thermal image. The blob decision is a pure function of that
image: one JSON detection when a compact region is hotter than the
background, and nothing when the frame has no hot outlier.

Self-test, no camera:
    python3 thermal_hotblob.py --selftest
One frame from the camera:
    python3 thermal_hotblob.py --once --stem frame1
Run the same function on that saved frame:
    python3 thermal_hotblob.py --from-capture frame1.json
"""
import argparse
import ctypes
import fcntl
import glob
import json
import mmap
import os
import signal
import sys
import time

import cv2
import numpy as np

V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_NONE = 1
V4L2_FRMSIZE_TYPE_DISCRETE = 1

# A row is the camera's rendered preview when chroma is pinned.
PREVIEW_CHROMA = 0x80
PREVIEW_FRACTION = 0.90
MIN_PREVIEW_ROWS = 8
# State/telemetry rows between the raw band and the preview are short.
MAX_STATE_ROWS = 8


def _ioc(direction, nr, size):
    return (direction << 30) | (size << 16) | (ord("V") << 8) | nr


def _ior(nr, size):
    return _ioc(2, nr, size)


def _iow(nr, size):
    return _ioc(1, nr, size)


def _iowr(nr, size):
    return _ioc(3, nr, size)


class V4L2Capability(ctypes.Structure):
    _fields_ = [
        ("driver", ctypes.c_char * 16),
        ("card", ctypes.c_char * 32),
        ("bus_info", ctypes.c_char * 32),
        ("version", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32),
        ("device_caps", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 3),
    ]


class V4L2FmtDesc(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("description", ctypes.c_char * 32),
        ("pixelformat", ctypes.c_uint32),
        ("mbus_code", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 3),
    ]


class V4L2FrmSizeDiscrete(ctypes.Structure):
    _fields_ = [("width", ctypes.c_uint32), ("height", ctypes.c_uint32)]


class V4L2FrmSizeStepwise(ctypes.Structure):
    _fields_ = [
        ("min_width", ctypes.c_uint32),
        ("max_width", ctypes.c_uint32),
        ("step_width", ctypes.c_uint32),
        ("min_height", ctypes.c_uint32),
        ("max_height", ctypes.c_uint32),
        ("step_height", ctypes.c_uint32),
    ]


class V4L2FrmSizeEnum(ctypes.Structure):
    class _U(ctypes.Union):
        _fields_ = [("discrete", V4L2FrmSizeDiscrete), ("stepwise", V4L2FrmSizeStepwise)]

    _fields_ = [
        ("index", ctypes.c_uint32),
        ("pixel_format", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("u", _U),
        ("reserved", ctypes.c_uint32 * 2),
    ]


class V4L2PixFormat(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("pixelformat", ctypes.c_uint32),
        ("field", ctypes.c_uint32),
        ("bytesperline", ctypes.c_uint32),
        ("sizeimage", ctypes.c_uint32),
        ("colorspace", ctypes.c_uint32),
        ("priv", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("ycbcr_enc", ctypes.c_uint32),
        ("quantization", ctypes.c_uint32),
        ("xfer_func", ctypes.c_uint32),
    ]


class V4L2Format(ctypes.Structure):
    class _U(ctypes.Union):
        # The kernel union is 8-byte aligned (it contains pointers), so
        # v4l2_format is 208 bytes on aarch64, not 204.
        _fields_ = [
            ("pix", V4L2PixFormat),
            ("raw", ctypes.c_uint8 * 200),
            ("_align", ctypes.c_void_p),
        ]

    _fields_ = [("type", ctypes.c_uint32), ("fmt", _U)]


class V4L2RequestBuffers(ctypes.Structure):
    _fields_ = [
        ("count", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32),
        ("flags", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class V4L2Buffer(ctypes.Structure):
    class _M(ctypes.Union):
        _fields_ = [
            ("offset", ctypes.c_uint32),
            ("userptr", ctypes.c_ulong),
            ("fd", ctypes.c_int32),
        ]

    _fields_ = [
        ("index", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("bytesused", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("field", ctypes.c_uint32),
        ("timestamp_sec", ctypes.c_long),
        ("timestamp_usec", ctypes.c_long),
        ("timecode", ctypes.c_uint32 * 4),
        ("sequence", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("m", _M),
        ("length", ctypes.c_uint32),
        ("reserved2", ctypes.c_uint32),
        ("request_fd", ctypes.c_int32),
    ]


def _request(request):
    # The direction lives in the top bits. Python's ioctl wants that as a signed int.
    request &= 0xFFFFFFFF
    if request >= 0x80000000:
        request -= 0x100000000
    return request


def _xioctl(fd, request, obj):
    raw = bytearray(ctypes.string_at(ctypes.addressof(obj), ctypes.sizeof(obj)))
    fcntl.ioctl(fd, _request(request), raw, True)
    ctypes.memmove(ctypes.addressof(obj), bytes(raw), len(raw))
    return obj


def fourcc_to_str(value):
    raw = bytes([(value >> (8 * i)) & 0xFF for i in range(4)])
    return raw.decode("ascii", "replace")


def fourcc_to_int(text):
    raw = text.encode("ascii")
    return raw[0] | (raw[1] << 8) | (raw[2] << 16) | (raw[3] << 24)


def hik_capture_nodes():
    """Capture nodes whose sysfs name is the HIK camera, not the IMX708."""
    found = []
    for path in sorted(glob.glob("/sys/class/video4linux/video*")):
        name = open(os.path.join(path, "name")).read().strip()
        if "Hik" not in name:
            continue
        node = "/dev/" + os.path.basename(path)
        found.append((node, name))
    return found


def query_modes(fd):
    modes = []
    fmt = V4L2FmtDesc()
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
    while True:
        try:
            _xioctl(fd, _iowr(2, ctypes.sizeof(fmt)), fmt)
        except OSError:
            break
        sizes = V4L2FrmSizeEnum()
        sizes.pixel_format = fmt.pixelformat
        while True:
            try:
                _xioctl(fd, _iowr(74, ctypes.sizeof(sizes)), sizes)
            except OSError:
                break
            if sizes.type == V4L2_FRMSIZE_TYPE_DISCRETE:
                modes.append((fourcc_to_str(fmt.pixelformat), int(sizes.u.discrete.width), int(sizes.u.discrete.height)))
            sizes.index += 1
        fmt.index += 1
    return modes


def _set_format(fd, fourcc, width, height):
    fmt = V4L2Format()
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
    fmt.fmt.pix.width = width
    fmt.fmt.pix.height = height
    fmt.fmt.pix.pixelformat = fourcc_to_int(fourcc)
    fmt.fmt.pix.field = V4L2_FIELD_NONE
    _xioctl(fd, _iowr(5, ctypes.sizeof(fmt)), fmt)
    return (
        fourcc_to_str(fmt.fmt.pix.pixelformat),
        int(fmt.fmt.pix.width),
        int(fmt.fmt.pix.height),
        int(fmt.fmt.pix.sizeimage),
    )


def _dqbuf(fd, buf, timeout):
    def alarm(_signum, _frame):
        raise TimeoutError("camera frame timed out")

    previous = signal.signal(signal.SIGALRM, alarm)
    signal.alarm(timeout)
    try:
        _xioctl(fd, _iowr(17, ctypes.sizeof(buf)), buf)
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, previous)


def _stream_off(fd):
    kind = ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE)
    try:
        _xioctl(fd, _iow(19, ctypes.sizeof(kind)), kind)
    except OSError:
        pass


def capture_frame(node, fourcc, width, height):
    """Open the real V4L2 node and return one raw buffer plus the size the driver set."""
    # This camera does not wake poll() when opened nonblocking, so DQBUF blocks.
    fd = os.open(node, os.O_RDWR)
    maps = []
    try:
        try:
            got = _set_format(fd, fourcc, width, height)
        except OSError as exc:
            if exc.errno != 16:
                raise
            # A previous stream can leave the device busy until it is stopped.
            _stream_off(fd)
            got = _set_format(fd, fourcc, width, height)
        req = V4L2RequestBuffers()
        req.count = 2
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = V4L2_MEMORY_MMAP
        _xioctl(fd, _iowr(8, ctypes.sizeof(req)), req)
        if req.count < 1:
            raise RuntimeError("driver granted no buffers")
        buffers = []
        for index in range(req.count):
            buf = V4L2Buffer()
            buf.index = index
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = V4L2_MEMORY_MMAP
            _xioctl(fd, _iowr(9, ctypes.sizeof(buf)), buf)
            mm = mmap.mmap(fd, buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset)
            maps.append(mm)
            buffers.append(mm)
            _xioctl(fd, _iowr(15, ctypes.sizeof(buf)), buf)
        kind = ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE)
        _xioctl(fd, _iow(18, ctypes.sizeof(kind)), kind)
        last = None
        # One long wait. Aborting DQBUF early drops the buffer the camera just filled.
        buf = V4L2Buffer()
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        buf.memory = V4L2_MEMORY_MMAP
        try:
            _dqbuf(fd, buf, 3)
        except TimeoutError:
            buf = None
        if buf is not None:
            size = buf.bytesused or buf.length
            last = bytes(buffers[buf.index][:size])
        if last is None:
            raise RuntimeError("camera produced no frame")
        return got, last
    finally:
        _stream_off(fd)
        for mm in maps:
            mm.close()
        try:
            req = V4L2RequestBuffers()
            req.count = 0
            req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            req.memory = V4L2_MEMORY_MMAP
            _xioctl(fd, _iowr(8, ctypes.sizeof(req)), req)
        except OSError:
            pass
        os.close(fd)


def _longest_true_run(flags):
    best_at, best_len, i, n = 0, 0, 0, len(flags)
    while i < n:
        if not flags[i]:
            i += 1
            continue
        j = i + 1
        while j < n and flags[j]:
            j += 1
        if j - i > best_len:
            best_at, best_len = i, j - i
        i = j
    return best_at, best_len


def _row_is_preview(row):
    odd = row[1::2]
    if odd.size == 0:
        return False
    return float(np.mean(odd == PREVIEW_CHROMA)) >= PREVIEW_FRACTION


def _trim_state_suffix(counts):
    """Drop a short trailer whose level is not the thermal image (config/state rows)."""
    n = counts.shape[0]
    if n <= MAX_STATE_ROWS:
        return counts
    body_med = float(np.median(counts[:-MAX_STATE_ROWS]))
    med_lim = max(abs(body_med) * 0.25, 50.0)
    end = n
    for i in range(n - 1, n - 1 - MAX_STATE_ROWS, -1):
        if abs(float(np.median(counts[i])) - body_med) > med_lim:
            end = i
        else:
            break
    return counts[:end]


def extract_thermal(buf, width, height):
    """Return a 2D image, higher = hotter.

    A stacked HIK frame is identifiable from the bytes: the rendered preview
    pins every chroma byte to 0x80, and the rows above that are raw counts.
    Anything else falls back to preview luminance.
    """
    raw = np.frombuffer(buf, dtype=np.uint8)
    yuyv_bytes = width * height * 2
    if raw.size == width * height:
        return raw.reshape(height, width).astype(np.float64)
    if raw.size < yuyv_bytes:
        raise ValueError(f"buffer is {raw.size} bytes, {width}x{height} YUYV needs {yuyv_bytes}")
    rows = raw[:yuyv_bytes].reshape(height, width * 2)
    preview = [_row_is_preview(rows[i]) for i in range(height)]
    start, length = _longest_true_run(preview)
    if length >= MIN_PREVIEW_ROWS and start >= 1:
        block = np.ascontiguousarray(rows[:start])
        counts = np.frombuffer(block.tobytes(), dtype="<u2").reshape(start, width)
        counts = _trim_state_suffix(counts)
        if counts.shape[0] >= 1:
            return counts.astype(np.float64)
    return rows[:, 0::2].astype(np.float64)


def _components(mask):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=4)
    blobs = []
    for i in range(1, n):
        x, y, w, h, area = (int(stats[i, k]) for k in range(5))
        blobs.append((area, x, y, w, h, labels == i))
    return blobs


def hot_blob(image, t=None):
    """One thermal detection, or None when the frame has no compact hot outlier.

    `image` is a 2D array of raw counts or luminance. Higher means hotter.
    The returned record matches the color detector's JSON line.
    """
    img = np.asarray(image, dtype=np.float64)
    if img.ndim != 2 or img.size == 0 or not np.isfinite(img).any():
        return None
    finite = np.where(np.isfinite(img), img, np.nan)
    med = float(np.nanmedian(finite))
    mad = float(np.nanmedian(np.abs(finite - med)))
    sigma = 1.4826 * mad
    peak = float(np.nanpercentile(finite, 99.5))
    if not np.isfinite(peak) or peak <= med + max(6.0 * sigma, 1e-3):
        return None
    thresh = med + max(6.0 * sigma, 0.45 * (peak - med))
    mask = (finite >= thresh).astype(np.uint8)
    if int(mask.sum()) < 16:
        return None
    best = None
    frame_area = img.shape[0] * img.shape[1]
    for area, x, y, w, h, sel in _components(mask):
        if area < 16 or area > 0.25 * frame_area:
            continue
        fill = area / float(w * h)
        if fill < 0.45:
            continue
        mean = float(finite[sel].mean())
        if best is None or mean > best[0]:
            best = (mean, x, y, w, h, area)
    if best is None:
        return None
    mean, x, y, w, h, area = best
    contrast = (mean - med) / max(peak - med, 1e-6)
    return {
        "type": "detection",
        "t": round(time.time() if t is None else float(t), 3),
        "cls": "thermal",
        "confidence": round(float(min(1.0, max(0.0, contrast))), 2),
        "bbox": [int(x), int(y), int(w), int(h)],
    }


def _synthetic_stacked():
    """YUYV buffer with a raw band, a rough state trailer, and a 0x80-chroma preview."""
    width, height = 32, 40
    counts = np.full((16, width), 1000, np.uint16)
    counts[4:9, 10:18] = 4000
    state = np.zeros((2, width), np.uint16)
    state[:, 0::2] = 7
    state[:, 1::2] = 40000
    raw_band = np.concatenate([counts, state], axis=0)
    buf = np.zeros((height, width * 2), np.uint8)
    raw_bytes = raw_band.astype("<u2").tobytes()
    buf[:18] = np.frombuffer(raw_bytes, dtype=np.uint8).reshape(18, width * 2)
    buf[18:36, 1::2] = PREVIEW_CHROMA
    buf[18:36, 0::2] = 10
    buf[36:, :] = 0xFF
    return buf.tobytes(), width, height, (10, 4, 8, 5)


def selftest():
    rng = np.random.default_rng(0)
    flat = 1000.0 + rng.normal(0, 2.0, (120, 160))
    none = hot_blob(flat, t=1.0)
    scene = flat.copy()
    scene[30:50, 70:100] = 1800.0
    det = hot_blob(scene, t=2.0)
    buf, width, height, planted = _synthetic_stacked()
    thermal = extract_thermal(buf, width, height)
    stacked = hot_blob(thermal, t=3.0)
    px, py, pw, ph = planted
    ok = (
        none is None
        and det is not None
        and det["type"] == "detection"
        and det["cls"] == "thermal"
        and isinstance(det["t"], float)
        and 0.0 <= det["confidence"] <= 1.0
        and _covers(det["bbox"], 70, 30, 30, 20)
        and stacked is not None
        and stacked["cls"] == "thermal"
        and _covers(stacked["bbox"], px, py, pw, ph)
        and thermal.shape[0] < height
    )
    print("flat detection:", none)
    print("hot detection:", det)
    print("stacked shape:", None if thermal is None else list(thermal.shape), "detection:", stacked)
    print("SELFTEST", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


def _covers(bbox, x, y, w, h):
    bx, by, bw, bh = bbox
    if bw <= 0 or bh <= 0:
        return False
    inside = bx <= x and by <= y and bx + bw >= x + w and by + bh >= y + h
    tight = bw * bh <= w * h * 4 and bw * bh >= w * h * 0.5
    return inside and tight


def open_hik():
    nodes = hik_capture_nodes()
    if not nodes:
        raise SystemExit("no HikCamera V4L2 node")
    chosen = None
    for node, name in nodes:
        fd = os.open(node, os.O_RDWR | os.O_NONBLOCK)
        try:
            cap = V4L2Capability()
            _xioctl(fd, _ior(0, ctypes.sizeof(cap)), cap)
            caps = cap.device_caps or cap.capabilities
            if caps & V4L2_CAP_VIDEO_CAPTURE:
                modes = query_modes(fd)
                chosen = (node, name, modes)
                break
        finally:
            os.close(fd)
    if chosen is None:
        raise SystemExit("HikCamera has no video-capture node")
    return chosen


def candidate_modes(modes):
    """Advertised uncompressed modes, in the order the device lists them.

    Tall frames come first when the device advertises them. The largest size
    is not always one that streams, so the caller tries these until a frame arrives.
    """
    yuyv = [m for m in modes if m[0] in ("YUYV", "YUY2")]
    stacked = [m for m in yuyv if m[2] > m[1]]
    pool = stacked or yuyv
    if not pool:
        pool = [m for m in modes if m[0] not in ("H264", "MJPG")] or list(modes)
    unique = []
    for mode in pool:
        if mode not in unique:
            unique.append(mode)
    return unique


def capture_once(stem):
    node, name, modes = open_hik()
    last_err = None
    frame = None
    got_cc = got_w = got_h = None
    for fourcc, width, height in candidate_modes(modes):
        try:
            (got_cc, got_w, got_h, _), frame = capture_frame(node, fourcc, width, height)
            break
        except (RuntimeError, OSError) as exc:
            last_err = exc
            time.sleep(0.2)
    if frame is None:
        raise last_err if last_err else RuntimeError("camera produced no frame")
    pixels = np.frombuffer(frame, dtype=np.uint8)
    info = {
        "node": node,
        "name": name,
        "fourcc": got_cc,
        "width": got_w,
        "height": got_h,
        "advertised": [{"fourcc": c, "width": w, "height": h} for c, w, h in modes],
        "bytes": len(frame),
        "min": int(pixels.min()) if pixels.size else None,
        "max": int(pixels.max()) if pixels.size else None,
        "varying": bool(pixels.size and pixels.min() != pixels.max()),
    }
    raw_path = stem + ".raw"
    with open(raw_path, "wb") as f:
        f.write(frame)
    info["raw"] = raw_path
    with open(stem + ".json", "w") as f:
        json.dump(info, f)
        f.write("\n")
    print(json.dumps(info))
    if not info["varying"]:
        raise SystemExit("captured frame is a constant value")
    return info


def detect_saved(meta_path, log_path):
    with open(meta_path) as f:
        info = json.load(f)
    with open(info["raw"], "rb") as f:
        frame = f.read()
    thermal = extract_thermal(frame, int(info["width"]), int(info["height"]))
    det = hot_blob(thermal)
    if det is None:
        line = "no-detection"
    else:
        line = json.dumps(det)
        if log_path:
            with open(log_path, "a") as f:
                f.write(line + "\n")
    print(line)
    return det


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--once", action="store_true", help="capture one frame from the HikCamera node")
    ap.add_argument("--stem", default="thermal-frame", help="path prefix for --once (.raw and .json)")
    ap.add_argument("--from-capture", metavar="META.json", help="run hot_blob on a frame --once saved")
    ap.add_argument("--log", default="", help="append a detection line here when --from-capture finds a blob")
    a = ap.parse_args()
    if a.selftest:
        selftest()
    if a.once:
        capture_once(a.stem)
    if a.from_capture:
        detect_saved(a.from_capture, a.log)
    if not (a.selftest or a.once or a.from_capture):
        ap.error("pass --selftest, --once, or --from-capture")


if __name__ == "__main__":
    main()
