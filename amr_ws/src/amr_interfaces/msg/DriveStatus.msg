# CiA-402 state of both drives, drive node -> diagnostics/mission (spec §2.1 /drives/status, 10 Hz).
std_msgs/Header header
uint16 left_statusword
uint16 right_statusword
uint16 left_error_code
uint16 right_error_code
string left_state
string right_state
bool operational
