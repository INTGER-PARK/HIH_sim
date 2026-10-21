"""Shared cooperative simulation timing and quaternion conversion."""
import math

import numpy as np


PHYSICS_HZ = 400.0


def quat_to_rpy(q_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(v) for v in q_wxyz]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return np.array([roll, pitch, yaw], dtype=float)
