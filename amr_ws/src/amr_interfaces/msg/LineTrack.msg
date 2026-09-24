# SICK MLS track reading (node 10 TPDO1 or its SDO equivalent), published by
# drive_node on /amr/line_track for every decoded sample. Line-follow plan §1.1.
# Positions are mm across the sensor, sign by the cable outlet (2027h flips it).
# With one tape, LCP2 is the track (MLS table 17); LCP1/LCP3 fill in on diverters.

builtin_interfaces/Time stamp   # receive time on this PC
int16[3] lcp_mm                 # LCP1..LCP3; meaningful only where valid[i]
bool[3] valid                   # which LCPs #LCP says are populated
uint8 nlcp                      # #LCP: 0 none, 2 one track, 3/6 diverter, 7 three tracks
string nlcp_label
bool line_good                  # status bit 0: track strong enough
uint8 track_level               # status bits 1-3, 0..7 (manual table 19)
string polarity                 # "north" | "south"
uint8 marker                    # marker code (byte 6 bits 4-7)
bool marker_intro               # byte 6 bit 3
bool event_flag                 # status bit 7
string source                   # "tpdo" (control-rate) | "sdo" (polled, monitor only)
