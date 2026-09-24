# Supervisor operating mode (unified plan §4.2, §5.1). Informational: which
# software layer exists and whether a transition is complete. NOT a motion
# lease - that is ControlLease. Reliable / transient-local; on change and 2 Hz.
uint8 STARTING=0
uint8 IDLE=1            # base + web only; jog under physical MANUAL
uint8 MAPPING=2         # mapping layer ready, survey active
uint8 NAVIGATION=3      # navigation layer ready on exactly one map revision
uint8 TRANSITIONING=4   # old layer stopping / new layer starting; no motion
uint8 FAULT=5           # failed layer stopped; base/web retained if healthy
uint8 STOPPING=6        # service teardown
std_msgs/Header header
string instance             # supervisor instance UUID; changes on every supervisor start
uint32 generation           # control generation; rotates on every transition
uint8 mode
uint8 requested_mode
string phase                # human-readable transition phase
string operation_id         # operation in progress, "" if none
bool base_ready
bool layer_ready
string active_map_id        # NAVIGATION only
uint32 active_map_revision
string active_map_sha256
bool manual_available       # a fresh MANUAL press would be accepted now
bool autonomous_available   # executor may hold a permit now
string fault_code
string reason
string last_survey_map_id   # last SAVED survey in this service lifetime
uint32 last_survey_revision
