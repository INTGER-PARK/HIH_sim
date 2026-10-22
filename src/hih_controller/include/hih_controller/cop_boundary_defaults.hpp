#pragma once

// Shared compile-time safety half-widths and hysteresis for all four CoP nodes.
// CoP y/z are tangent coordinates in the Body-parallel contact plate frame [m].
// safe=0.10 describes [-0.10,+0.10] m, not a total width of 0.10 m.
// Inside the boundary, correction rates are zero and accumulated offsets hold.
// Hysteresis releases at safe-hysteresis; master/axis OFF returns offsets to zero.
// Runtime safe_y_m/safe_z_m overrides apply to each node independently.
// Rebuild hih_controller and restart its nodes after changing these defaults.

namespace hih_controller::cop_boundary_defaults
{
// All four CoP nodes refer directly to these shared defaults.
// Node members store current values; runtime overrides apply only to that node.
inline constexpr double safe_y_m = 0.10;  // y safety half-width [m].
inline constexpr double safe_z_m = 0.10;  // z safety half-width [m].
inline constexpr double safe_y_hysteresis_m = 0.005;  // y boundary-release hysteresis [m].
inline constexpr double safe_z_hysteresis_m = 0.005;  // z boundary-release hysteresis [m].
}  // namespace hih_controller::cop_boundary_defaults
