# SICK MLS marker codes (manuals/mls-marker-plan.md, 2026-10-09), published by drive_node
# on /amr/line_marker. The SENSOR decodes the code (2028h, standard mode, FailSafe); this is
# one message per marker it reported, made in the bus thread at TPDO rate so a code shown
# for a single 10 ms frame is not lost to the best-effort /amr/line_track.
#
# Two kinds of message on ONE topic, like /amr/rfid:
#   heartbeat=false  one per marker: code, raw, where the track was at the read.
#   heartbeat=true   1 Hz status: markers_ok/status and the counters, seq = the latest event.
# A change of generation (bus owner restart, MLS rediscovered) breaks continuity: a
# consumer drops its cursor. RELIABLE + VOLATILE: a replayed old marker must never look new.
builtin_interfaces/Time stamp
bool heartbeat
uint32 seq               # per generation, 1 = the first marker
uint32 generation
uint8 code               # 1..7 standard mode; 0 on a heartbeat
uint8 raw                # TPDO1 byte 7 bits 3-7 as read, for diagnosis (plan B6)
int8 direction           # +1 / -1 once the bench settles how the MLS reports it; 0 = unknown
int16 lcp2_mm            # track position at the read; 0 when no track
uint8 nlcp
bool line_good
bool markers_ok          # profile on AND sensor configured as the profile expects
string status            # "ok" | "off" | "misconfigured: ..." | "no stream (sdo)" | "checking"
uint32 rejects           # codes outside mls.marker_codes since start
uint32 aborts            # "reading code" episodes that ended with no code
