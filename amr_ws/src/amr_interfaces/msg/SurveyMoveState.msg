# Preset survey move state (survey_move_node), latched; on change and at 5 Hz while active.
uint8 IDLE=0
uint8 MOVING=1
uint8 SETTLING=2     # stopped; waiting for SLAM to take the last scans before the check
uint8 DONE=3
uint8 ABORTED=4

std_msgs/Header header
uint8 state
string move_id
string label                 # e.g. "forward 2.00 m", "spin right 90 deg"
string kind                  # straight | rotate
float64 target               # m or rad, signed
float64 progress             # m or rad, signed, from odometry
string reason                # why ABORTED, or the check text
bool check_done              # the SLAM check below is valid
bool check_ok
float64 correction_m         # how far SLAM moved the map pose relative to odometry during the move
float64 correction_deg
