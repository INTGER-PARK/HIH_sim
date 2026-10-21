# Contact points for the existing HandinHand_rviz interface.
# C origin is the plate geom center; C axes are parallel to drone Body.
# One sample per physics step; contact positions and forces use C axes.
uint64 step
float64 sim_time

# Matching contacts, active solver constraints, and slots retained below.
uint32 contact_count
uint32 active_constraint_count
uint32 stored_contact_count
bool contact_truncated

# Normal compression sum [N]; resultant payload-on-plate wrench about C.
float64 total_normal_force
float64[3] net_force
float64[3] net_moment

# Signed solver distances [m]; NaN when no contacts exist.
float64 min_contact_dist
float64 max_contact_dist

# Compatibility fields: friction/slip/moment analysis is no longer computed.
# Unused floating fields are NaN; the reserved boolean stays false.
bool friction_cone_pyramidal
float64 global_friction_utilization
float64 max_local_friction_utilization
float64 max_tangential_relative_speed
float64 normal_force_weighted_tangential_speed

# CoP from resultant wrench at the physical +X face center.
bool solver_cop_valid
float64 solver_cop_y
float64 solver_cop_z

# Reserved legacy moment-analysis fields (NaN).
float64 normal_moment_y
float64 normal_moment_y_capacity
float64 normal_moment_y_utilization
float64 normal_moment_z
float64 normal_moment_z_capacity
float64 normal_moment_z_utilization

# Reserved legacy gap/relative-motion fields (NaN).
float64[4] corner_face_gap
float64[3] relative_position
float64[3] relative_rpy
float64[3] relative_linear_velocity
float64[3] relative_angular_velocity

# First 50 contacts in solver order, in plate-center C coordinates [m].
# Unused floating slots are NaN. efc_address=-1 means no active constraint.
float64[50] contact_pos_x
float64[50] contact_pos_y
float64[50] contact_pos_z
float64[50] contact_dist

# Reserved legacy normal-relative velocity [m/s] (NaN).
float64[50] normal_relative_velocity

# Contact-frame normal compression [N], zero for inactive contacts.
float64[50] normal_force

# Reserved legacy tangential relative speed [m/s] (NaN).
float64[50] tangential_relative_speed

# Reserved legacy friction utilization (NaN).
float64[50] local_friction_utilization

# Payload-on-plate contact force in C axes [N].
float64[50] force_x
float64[50] force_y
float64[50] force_z

# Solver constraint row; nonnegative values identify active contacts.
int32[50] efc_address
