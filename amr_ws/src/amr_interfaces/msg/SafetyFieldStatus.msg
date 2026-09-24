# nanoScan3 field status, sick driver -> diagnostics/mission (spec §2.1 /scan/field_status, 10 Hz).
# Informational only: the safety stop is the OSSD pair into the FX3, never this topic.
std_msgs/Header header
bool protective_field_clear
bool warning_field_clear
uint8 active_monitoring_case
