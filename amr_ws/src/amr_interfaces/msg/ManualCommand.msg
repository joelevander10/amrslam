# One browser jog refresh (unified plan §4.2, §6.3). The web adapter publishes
# exactly one of these per accepted HTTP request; no backend timer repeats the
# last one. The mux rejects nonfinite values, a wrong instance/generation, a
# non-increasing seq within a session, and anything older than valid_for_s.
std_msgs/Header header
string instance
uint32 generation
string session              # server-issued jog session id; one owner at a time
uint64 seq
float64 valid_for_s         # remaining allowed lifetime carried across the HTTP hop
float64 v                   # body m/s
float64 w                   # body rad/s
