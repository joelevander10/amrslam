# Encoder-only commissioning (blind-run) job (unified plan §7.2), from commissioning_node.
# Latched; on change and 2 Hz. An exclusive substate of supervisor IDLE. PV only on this
# vehicle (profile position removed 2026-10-30; blind run restored 2026-11-02).
uint8 IDLE=0
uint8 PREPARED=1     # a validated plan is loaded; a fresh physical Start under MANUAL executes it
uint8 RUNNING=2
uint8 SETTLING=3
uint8 DONE=4
uint8 ABORTED=5
std_msgs/Header header
uint8 phase
uint32 generation
string plan_id
uint32 segment           # 1-based, current or last
uint32 segments
string kind
float64 progress_left_m
float64 progress_right_m
float64 speed_mps
float64 pose_x_m
float64 pose_y_m
float64 heading_deg
float64 gyro_heading_deg   # integrated corrected yaw rate since the job started (evidence)
uint32 completed
string reason
string results_path        # evidence file after DONE/ABORTED
