"""Conversions used by ROS adapters."""
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
from .core import Grid, transform


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def apply_tf(points, stamped):
    t, q = stamped.transform.translation, stamped.transform.rotation
    return transform(points, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def from_grid(msg):
    q = msg.info.origin.orientation
    if (not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or
            abs(q.w-1.) > 1e-5 or any(abs(v) > 1e-5 for v in (q.x, q.y, q.z)) or
            not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0 or
            not all(math.isfinite(v) for v in (msg.info.origin.position.x, msg.info.origin.position.y))):
        raise ValueError('Only axis-aligned navigation grids are supported')
    return Grid(np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width),
                msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)


def grid_message(grid, frame, stamp):
    msg = OccupancyGrid()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.info.resolution = grid.resolution
    msg.info.height, msg.info.width = grid.data.shape
    msg.info.origin.position.x, msg.info.origin.position.y = float(grid.x), float(grid.y)
    msg.info.origin.orientation.w = 1.
    msg.data = grid.data.ravel().tolist()
    return msg
