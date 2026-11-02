# What the mux is doing right now (unified plan §4.2): the transition barrier
# reads this for "inhibited generation acknowledged". 10 Hz. Diagnostics only;
# never proof of physical stillness - that comes from /wheel_states.
uint8 NONE=0
uint8 TELEOP=1          # /cmd_vel_teleop (engineering keyboard input)
uint8 FOLLOW=2
uint8 ROTATE=3
uint8 MANUAL=4          # /amr/manual_command (stamped browser jog)
uint8 COMMISSIONING=5
uint8 PENDANT=6          # physical jog pendant levels carried in PanelState
uint8 LINE=7             # /amr/line_cmd, magnetic tape following (dual-product plan)
std_msgs/Header header
string instance             # lease instance the mux is applying
uint32 generation
uint8 source
bool inhibited
float64 left_rad_s          # output setpoint this tick
float64 right_rad_s
string reason
string code                 # alarm catalogue code for `reason`; "" when a source is selected

# Scanner fields as the mux applies them (2026-10-02). APPEND ONLY.
bool field_fresh            # a fresh /output_paths sample (or the sim's assume_clear)
bool protective_clear       # protective field clear; AUTO is zeroed while it is not
bool warning_active         # warning field occupied; AUTO is scaled by speed_scale
float32 speed_scale         # applied AUTO factor: 1.0 clear, ramping to 0.5 / 0.1 under warning 1 / 2, 0.0 stopped by a field
uint8 warning_level         # 0 clear, 1 warning 1 (outer, x0.5), 2 warning 2 (inner, x0.1); appended 2026-10-02
