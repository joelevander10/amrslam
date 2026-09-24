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
std_msgs/Header header
string instance             # lease instance the mux is applying
uint32 generation
uint8 source
bool inhibited
float64 left_rad_s          # output setpoint this tick
float64 right_rad_s
string reason
