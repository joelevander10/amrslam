# Line-follow layer state (dual-product plan, Increment 1), published by
# line_follow_node on /amr/line_state on change and at 1 Hz.
#
# *** The first four state values are load-bearing. *** readiness.py holds
# _line_active_locked as `line_state in (1, 2, 3)`, which feeds
# mode_fsm.Conditions.line_active and therefore the admit() rule that refuses
# to leave LINE while the follower is armed or running. ARMED/RUNNING/HOLD must
# be exactly 1..3: renumber them and leaving LINE mode is either wrongly
# refused, or wrongly ALLOWED while the vehicle is still moving. Append only.
uint8 IDLE=0            # layer up, not armed; safe to swap the layer out
uint8 ARMED=1           # pre-move warning after a Start: drives powering, alarm horn on
uint8 RUNNING=2         # following tape
uint8 HOLD=3            # stopped, progress retained; see hold_cause
uint8 DONE=4            # run finished or stopped itself; Reset clears
uint8 FAULT=5           # sensor/drive/authority fault; acknowledgement does not resume

std_msgs/Header header
uint32 generation           # layer generation that produced this; consumers drop other generations
uint8 state

# Why the follower is not driving. HOLD only: field | estop | drives | track |
# rate | authority | pending. Shares the vocabulary of RunState.hold_cause so
# both products explain a stop the same way.
string hold_cause
bool auto_resume            # HOLD and this hold clears by itself once the cause does

# What the engine is doing, straight out of the follower's diagnostic row.
string engine_state         # idle | halted | sensor_lost | coast | line_lost | run | stopping
bool has_track
float64 error_mm            # lateral error the PID acted on; 0 with no track
float64 v_mps               # commanded body speed
float64 omega_rad_s         # commanded yaw rate
float64 followed_m          # distance travelled this run

# Sensor health, the two numbers R1 says to record on the vehicle. track_hz is
# a RATE gate, not a staleness gate: the SDO fallback measured 9.8 Hz against a
# 50 Hz PID, and every staleness check ever written passes that.
float32 track_hz
float32 sample_age_s
string track_source         # "tpdo" (control-rate) | "sdo" (polled, monitor only)

string message              # last result / block reason, for the operator
string code                 # alarm catalogue code for the hold/fault in `message`; "" when driving

# Mission engine (2026-10-02). APPEND ONLY. All empty/false with no mission.
string mission              # "" = plain line following
string station              # route stage: current station id
string next_station
string branch_intent        # straight | left | right
bool slow_zone              # junction slow zone (branch_latch slow_speed): no HIGH through it
string uturn_phase          # "" | stopping | spin | center | settle
string last_tag             # last RFID encounter and what the engine did with it
string last_tag_action
string stop_where           # the station being stopped at / parked at

# Tag-table job (2026-10-08). APPEND ONLY. Empty/false with no destination mission.
# station / next_station above now carry: the stop being made, and the destination.
string destination          # the job's destination stop label; "" = none (spent at Home)
bool at_home                # last tag read was Home and the wheels have not left it
string home_detail          # why not at Home, for the readiness list
string job_result           # how the last run ended at Home: "MRU2 served" | "... MISSED ..."

# Tracked speeds (tracked-speed-plan-1, 2026-10-08). APPEND ONLY.
bool high_speed             # HIGH granted (RFID high zone), else NORMAL
string speed_reason         # why: zone entry, budget used, late exit, curve guard, hold...

# Drive power (2026-10-08). APPEND ONLY. What the follower asks of drive_node's power
# policy under AUTO: ARMED (the pre-move warning), RUNNING, and holds that resume by
# themselves. False when idle and in holds that wait for a physical Start.
bool drive_power

# MLS markers (manuals/mls-marker-plan.md, 2026-10-09). APPEND ONLY. Display only: no
# marker changes the state or the speed (yet).
bool markers_ok             # the marker stream is up and the sensor configured
uint8 last_marker           # code of the last marker read; 0 = none since start
uint32 marker_count         # markers read since the node started
uint32 marker_missed        # sequence gaps on /amr/line_marker

# Tape-graph localization (tracked plan 3, P1 teach + shadow, 2026-10-18). APPEND ONLY.
# Display only: nothing here changes the state, the stops or the branch orders yet.
string loc_health           # "" off | IDLE | WAITING (At home, waiting for READY) | HEALTHY | DEGRADED | LOST
string loc_circuit          # the shadow circuit, e.g. C-MRU
string loc_leg              # tape leg the locator is on, e.g. "J-N:right>X1"
float32 loc_s_m             # distance along the circuit's route
float32 loc_sigma_m         # its uncertainty (1 sigma)
uint32 loc_rejects          # AMCL corrections rejected since the locator started
string loc_detail           # why not running, why LOST, or the last rejection
string loc_graph            # active graph, "site103 rev4 g2"; "" = none
string teach                # the pass being recorded; "" when not recording
