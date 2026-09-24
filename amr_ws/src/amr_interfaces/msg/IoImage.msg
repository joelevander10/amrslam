# The DIO island's full image from its owner, panel_node (unified plan §7). 5 Hz.
# Requested coil state and the module's readback are separate: on an output
# "off because nothing asked" and "held off" look identical and mean opposite things.
std_msgs/Header header
bool comms_ok
float64 rx_age_s              # since the last successful scan; -1 never
string[] di_names
bool[] di
string[] do_names
bool[] do_readback            # what the module reports its coils are
bool[] do_requested           # what this software is currently claiming (expired claims read false)
uint32 scans
uint32 errors
uint32 writes
string detail
