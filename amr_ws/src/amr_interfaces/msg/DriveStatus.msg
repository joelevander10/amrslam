# CiA-402 state of both drives, drive node -> diagnostics/mission (spec §2.1 /drives/status, 10 Hz).
std_msgs/Header header
uint16 left_statusword
uint16 right_statusword
uint16 left_error_code
uint16 right_error_code
string left_state
string right_state
bool operational
# Drive power (2026-10-08). link_state: disarmed | standby | armed | fault. Standby = CANopen
# up and feedback flowing, power stage off. power_reason: why power is or is not wanted
# (amr_base/arm_policy), for the operator.
string link_state
bool power_wanted
string power_reason
