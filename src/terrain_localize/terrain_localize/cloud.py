"""PointCloud2 helpers for xyz float32 clouds."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def xyz_to_cloud(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    xyz = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = int(xyz.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = xyz.tobytes()
    return msg


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    offsets = {field.name: field.offset for field in msg.fields}
    if not {"x", "y", "z"} <= set(offsets):
        return np.zeros((0, 3), dtype=np.float64)
    count = msg.width * msg.height
    if count == 0 or msg.point_step == 0:
        return np.zeros((0, 3), dtype=np.float64)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < count * msg.point_step:
        return np.zeros((0, 3), dtype=np.float64)
    raw = raw[: count * msg.point_step].reshape(count, msg.point_step)
    xyz = np.empty((count, 3), dtype=np.float32)
    for index, name in enumerate(("x", "y", "z")):
        xyz[:, index] = raw[:, offsets[name] : offsets[name] + 4].copy().view(np.float32).reshape(count)
    return xyz.astype(np.float64)
