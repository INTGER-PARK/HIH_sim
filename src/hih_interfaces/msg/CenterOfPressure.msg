# Center of Pressure expressed in the contact-plate frame C.
# C is located at the center of the +X contact face. +x_C is the outward
# surface normal (drone toward wall); y and z below are y_C and z_C.
float64 y
float64 z
float64 normal_force
bool valid

# Corner load cells ordered as (+y,+z), (-y,+z), (-y,-z), (+y,-z).
float64[4] corner_forces
