# The supervisor's motion lease (unified plan §4.2, §4.3). The mux and the drive
# owner each require a fresh lease (local receipt age <= 0.3 s) whose instance
# and generation match before accepting ANY command; either gate closes on its
# own if the supervisor stops making progress. Reliable / volatile depth 1, 10 Hz.
# The supervisor is the only publisher. It never publishes MotionPermit.
uint8 NONE=0
uint8 MANUAL=1          # bitmask: jog / stamped browser commands under physical MANUAL
uint8 AUTONOMOUS=2      # executor permit may be honoured (NAVIGATION only)
uint8 COMMISSIONING=4   # exclusive IDLE substate (U8)
std_msgs/Header header
string instance
uint32 generation
uint64 seq                  # strictly increasing within an instance
uint8 allowed               # bitmask of the classes above; 0 = inhibited
string inhibit_reason
