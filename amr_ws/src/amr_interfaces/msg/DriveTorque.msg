# Motor torque, drive node -> logging (2026-11-02, groundwork for towing-load sensing).
# DIAGNOSTIC ONLY: nothing may decide whether or how the vehicle moves on this.
# 6077h Torque actual value from TPDO1, one sample per /wheel_states (same stamp).
# Percent of the motor's RATED torque, estimated by the drive from current, on the MOTOR
# side of the 1:30 gearhead. Signed like /wheel_states velocity (profile invert flags
# applied): positive drives the wheel forward, negative brakes it / regenerates.
std_msgs/Header header
float32 left_torque_pct
float32 right_torque_pct
# Statusword bit 15 (TLC): output torque is at the motor maximum or the active torque limit.
bool left_torque_limited
bool right_torque_limited
# A fresh TPDO1 that carried 6077h. False = stale or absent; the torque reads 0 and means nothing.
bool left_valid
bool right_valid
