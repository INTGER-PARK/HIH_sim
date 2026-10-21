# Torque DoB status/correction in BODY roll/pitch/yaw axes.
# Every data field starts with dob_ so CSV columns cannot be confused with MoB.
uint64 dob_step
float64 dob_sim_time
bool dob_observer_valid
# Master compensation state toggled only by keyboard [d].
bool dob_compensation_enabled
# Axis mask affects compensation only; all three disturbance torques are observed.
bool[3] dob_axis_enabled
float64[3] dob_estimated_torque
# Requested correction actually consumed by wrench_controller while fresh.
float64[3] dob_compensation_torque
float64[3] dob_compensation_limit
