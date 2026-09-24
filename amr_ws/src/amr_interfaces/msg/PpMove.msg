# A profile-position (pp) blind move, HELD not latched: the commissioning node
# re-sends it every tick while its job runs. The drive owner starts a run id once,
# and halts it the moment this stream goes stale (0.2 s), says hold=false, names
# another run, or loses the generation - see amr_base/pp.py.
std_msgs/Header header
string run_id               # one physical Start = one run id; never re-run
uint32 generation           # supervisor control generation the job was admitted under
uint64 seq
bool hold                   # false = halt now
# per wheel, driver terms (the drive's own sign); relative to the position at entry
int32 left_delta_counts
int32 right_delta_counts
uint32 left_velocity_rpm    # 6081h, motor r/min
uint32 right_velocity_rpm
uint32 left_accel_rpm_s     # 6083h
uint32 right_accel_rpm_s
uint32 left_decel_rpm_s     # 6084h
uint32 right_decel_rpm_s
float64 duration_s          # planned trapezoid time; the owner halts at 2x + 2 s
