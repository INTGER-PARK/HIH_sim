"""Rectangular plate CoP from the solver resultant wrench."""
import math

import numpy as np


def plate_support_cop_from_wrench(force, moment_face, half_y, half_z,
                                  threshold_n):
    """Return CoP using the full rectangular plate as the support region.

    Instantaneous MuJoCo contact-point positions are not used in this
    definition. The input wrench is payload-on-plate, expressed at the +X
    contact FACE center in plate/Body-parallel axes. Compression has Fx < 0.

        Fn       = -Fx
        y_demand =  Mz / Fn
        z_demand = -My / Fn

    The display CoP is saturated to the plate boundary; the unsaturated demand
    and utilization are returned separately so infeasibility is not hidden.
    """
    force = np.asarray(force, dtype=float).reshape(3)
    moment_face = np.asarray(moment_face, dtype=float).reshape(3)
    half_y = float(half_y)
    half_z = float(half_z)
    threshold_n = float(threshold_n)

    if not np.all(np.isfinite(force)) or not np.all(np.isfinite(moment_face)):
        raise ValueError('force and moment_face must be finite')
    if half_y <= 0.0 or half_z <= 0.0:
        raise ValueError('plate half sizes must be positive')
    if threshold_n < 0.0 or not math.isfinite(threshold_n):
        raise ValueError('threshold_n must be finite and nonnegative')

    fn = max(0.0, -float(force[0]))
    valid = fn >= threshold_n and fn > 0.0
    if not valid:
        return {
            'valid': False,
            'normal_force': fn,
            'y': 0.0,
            'z': 0.0,
            'y_demand': math.nan,
            'z_demand': math.nan,
            'y_utilization': math.nan,
            'z_utilization': math.nan,
        }

    y_demand = float(moment_face[2]) / fn
    z_demand = -float(moment_face[1]) / fn
    return {
        'valid': True,
        'normal_force': fn,
        'y': float(np.clip(y_demand, -half_y, half_y)),
        'z': float(np.clip(z_demand, -half_z, half_z)),
        'y_demand': y_demand,
        'z_demand': z_demand,
        'y_utilization': abs(y_demand) / half_y,
        'z_utilization': abs(z_demand) / half_z,
    }
