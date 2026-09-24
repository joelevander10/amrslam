# Wheel velocity setpoints, cmd_mux -> drive node (spec §2.1 /cmd_wheel_vel, 50 Hz).
# SI at this boundary; the drive node converts to r/min for 60FFh (D-1).
std_msgs/Header header
uint32 generation           # supervisor control generation the mux applied (0 = unsupervised bench)
float64 left_rad_s
float64 right_rad_s
