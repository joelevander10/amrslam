# Operator-relevant event from a data owner or the supervisor (unified plan §7.1).
# Bounded: only edges and transitions, never per-tick. The web keeps a ring.
uint8 INFO=0
uint8 WARN=1
uint8 ERROR=2
std_msgs/Header header
string source                 # node name
uint8 level
string code                   # machine-readable, e.g. MODE_CHANGE, DRIVE_ALARM, PANEL_LOST
string text
uint64 seq                    # per source, strictly increasing
