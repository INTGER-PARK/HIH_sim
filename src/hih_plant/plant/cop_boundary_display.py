"""CoP boundary packet decoding and drawing, independent of ROS/Qt imports."""

import math
from typing import Dict, Optional, Tuple

# Independent packets prevent attitude/translation controllers from overwriting
# each other's settings. Float64MultiArray layout is documented in the README:
# [master, y_enabled, z_enabled, safe_y_m, safe_z_m, source_id].
COP_BOUNDARY_SUFFIXES = {
    "attitude": "/cop_attitude/boundary",
    "translation": "/cop_translation/boundary",
}
COP_BOUNDARY_TIMEOUT_SEC = 1.0
COP_SAFETY_DEBUG_SUFFIX = "/cop_safety/debug"
COP_SAFETY_ENABLED_INDEX = 10
COP_SAFETY_SAFE_Z_INDEX = 12
COP_SAFETY_Y_ENABLED_INDEX = 19
COP_SAFETY_Z_ENABLED_INDEX = 20
COP_SAFETY_SAFE_Y_INDEX = 21
COP_SAFETY_SOURCE_ID_INDEX = 24


def cop_boundary_from_state(
    flattened: Dict[str, float]
) -> Optional[Tuple[bool, bool, bool, float, float, int]]:
    """Decode the boundary packet published even while a controller is OFF."""
    values = [flattened.get(f"data[{index}]") for index in range(6)]
    if any(value is None or not math.isfinite(value) for value in values):
        return None
    master, y_enabled, z_enabled, safe_y, safe_z, source_id = values
    if any(value not in (0.0, 1.0) for value in values[:3]):
        return None
    # Translation supports a zero boundary (centering); TOUCH is unsupported.
    if safe_y < 0.0 or safe_z < 0.0 or source_id not in (0.0, 2.0):
        return None
    return bool(master), bool(y_enabled), bool(z_enabled), safe_y, safe_z, int(source_id)


def cop_boundary_snapshot(topic_info, now: float, legacy: bool = False):
    """Do not display cached settings after the publisher stops."""
    if topic_info is None:
        return None, "unavailable"
    stamp = topic_info.latest_rx_time
    if stamp is None:
        return None, "waiting"
    age = now - stamp
    if age < 0.0 or age > COP_BOUNDARY_TIMEOUT_SEC:
        return None, "stale"
    decode = cop_safety_from_debug if legacy else cop_boundary_from_state
    state = decode(topic_info.latest_flattened)
    return state, "invalid" if state is None else ""


def render_cop_boundary(
    items, state, half_y: float, half_z: float, label: str, missing_status: str
) -> str:
    """Draw one shared boundary; contact/CoP validity is independent."""
    if state is None:
        for item in (items.z_upper, items.z_lower, items.y_positive, items.y_negative):
            item.setData([], [])
        return f"{label}: {missing_status}"
    master, y_enabled, z_enabled, safe_y, safe_z, _ = state
    show_y = master and y_enabled and math.isfinite(safe_y)
    show_z = master and z_enabled and math.isfinite(safe_z)
    items.z_upper.setData([-half_y, half_y] if show_z else [],
                          [safe_z, safe_z] if show_z else [])
    items.z_lower.setData([-half_y, half_y] if show_z else [],
                          [-safe_z, -safe_z] if show_z else [])
    items.y_positive.setData([safe_y, safe_y] if show_y else [],
                             [-half_z, half_z] if show_y else [])
    items.y_negative.setData([-safe_y, -safe_y] if show_y else [],
                             [-half_z, half_z] if show_y else [])
    parts = []
    if show_y:
        parts.append(f"|y|<={safe_y:.3f} m")
    if show_z:
        parts.append(f"|z|<={safe_z:.3f} m")
    return f"{label}: " + (", ".join(parts) if parts else "off")


def common_cop_boundary(states):
    """Combine fresh controller packets into one geometry and OR axis enables.

    A sole attitude controller must draw the same dashed boundary as a sole
    translation controller. OFF/stale peers cannot hide an active controller.
    When both control an axis, their limits must agree; do not draw a misleading
    common boundary if runtime overrides disagree.
    """
    valid = [state for state, _ in states.values() if state is not None]
    if not valid:
        missing = [status for _, status in states.values()]
        for status in ('invalid', 'stale', 'waiting', 'unavailable'):
            if status in missing:
                return None, status
        return None, 'unavailable'
    widths, enabled = [], []
    for enable_index, width_index in ((1, 3), (2, 4)):
        limits = [state[width_index] for state in valid
                  if state[0] and state[enable_index]]
        if limits and any(not math.isclose(value, limits[0], rel_tol=1e-9, abs_tol=1e-9)
                          for value in limits[1:]):
            return None, 'boundary mismatch'
        enabled.append(bool(limits))
        widths.append(limits[0] if limits else valid[0][width_index])
    return (any(state[0] for state in valid), *enabled, *widths, valid[0][5]), ''


def cop_controller_status(kind, snapshot):
    """Keep each controller's source/ON state separate from the common geometry."""
    state, missing_status = snapshot
    if state is None:
        return f"{kind}: {missing_status}"
    source = {0: 'MOB', 1: 'TOUCH', 2: 'SOLVER'}.get(state[5], f'ID{state[5]}')
    return f"{kind}[{source}]: {'on' if state[0] else 'off'}"


def cop_safety_from_debug(
    flattened: Dict[str, float]
) -> Optional[Tuple[bool, bool, bool, float, float, int]]:
    """Decode master, per-axis enables, y/z boundaries, and selected CoP source."""
    enabled = flattened.get(f"data[{COP_SAFETY_ENABLED_INDEX}]")
    safe_z = flattened.get(f"data[{COP_SAFETY_SAFE_Z_INDEX}]")
    if enabled is None or safe_z is None or not math.isfinite(safe_z) or safe_z <= 0.0:
        return None

    # Backward compatibility with the earlier z-only debug packet.
    y_enabled_raw = flattened.get(f"data[{COP_SAFETY_Y_ENABLED_INDEX}]", 0.0)
    z_enabled_raw = flattened.get(f"data[{COP_SAFETY_Z_ENABLED_INDEX}]", 1.0)
    safe_y = flattened.get(f"data[{COP_SAFETY_SAFE_Y_INDEX}]", math.nan)

    y_enabled = bool(y_enabled_raw > 0.5)
    z_enabled = bool(z_enabled_raw > 0.5)
    safe_y_value = float(safe_y) if math.isfinite(safe_y) and safe_y > 0.0 else math.nan
    source_raw = flattened.get(f"data[{COP_SAFETY_SOURCE_ID_INDEX}]", 0.0)
    source_id = int(source_raw) if math.isfinite(source_raw) else 0
    return (
        bool(enabled > 0.5), y_enabled, z_enabled,
        safe_y_value, float(safe_z), source_id
    )


