# Route execution state (spec §7.1), latched; on change and at 2 Hz.
uint8 IDLE=0
uint8 READY=1       # mission loaded, localisation READY, start gate satisfied; waiting for AUTO + Start
uint8 EXECUTING=2
uint8 PAUSED=3      # operator pause; progress retained
uint8 BLOCKED=4     # obstruction in the active step's swept envelope; progress retained
uint8 FAULT=5       # sensor/drive/panel/localisation fault; acknowledgement does not resume
uint8 DONE=6

std_msgs/Header header
uint32 generation           # layer generation that produced this; consumers drop other generations
uint8 state
string run_id
string mission_id
string map_id
uint32 map_revision
string route_id
uint32 route_revision
int32 step_index            # -1 before the first step
string step_id
string step_type            # straight | rotate | ""
float64 remaining_turn_rad  # signed, for a rotate in progress or paused
float64 cross_track_m       # for a straight in progress
float64 step_v_mps          # the speed a straight in progress runs at (route cap or long-straight boost); 0 otherwise
uint8 localization_state    # LocalizationState value
bool resume_prepared        # a Start edge will continue the remaining step
string hold_cause           # BLOCKED only: field | estop | obstacle | controller | pending (auto-resume plan)
bool auto_resume            # BLOCKED and this hold resumes by itself once clear
bool pose_valid
float64 pose_x
float64 pose_y
float64 pose_yaw
string reason
