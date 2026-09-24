# Survey session state (spec §6.2), published by mapping_session_node on change and at 1 Hz.
uint8 IDLE=0
uint8 MAPPING=1
uint8 RETURN_REVIEW=2
uint8 SAVING=3
uint8 SAVED=4

std_msgs/Header header
uint32 generation           # layer generation that produced this; consumers drop other generations
uint8 state
string map_id
uint32 revision              # of the last SAVED bundle, 0 if none
string description           # operator's description of the physical start mark
geometry_msgs/Pose2D start    # start reference in map, recorded at survey start
bool closure_available       # return evidence below is valid (RETURN_REVIEW or later)
float64 closure_dx_m         # current pose vs start reference, as SLAM estimates it
float64 closure_dy_m
float64 closure_dyaw_rad
string saved_path            # directory of the published revision, when SAVED
string message               # last result / block reason
