# The drive owner's profile-position executor (amr_base/pp.py), 10 Hz.
std_msgs/Header header
bool available              # a pp move would be admitted now (locked profile -> false)
string reason               # why not, or the last result's reason
string state                # idle | moving | halting
string run_id               # current, or the last finished run
string outcome              # of the last run: done | halted | refused | faulted | ""
int32 left_target
int32 right_target
int32 left_actual
int32 right_actual
bool left_target_reached    # 6041h bit 10 (meaningful once the set-point is acknowledged)
bool right_target_reached
bool left_following_error   # 6041h bit 13
bool right_following_error
float64 started_age_s       # since the move started, -1 if none
