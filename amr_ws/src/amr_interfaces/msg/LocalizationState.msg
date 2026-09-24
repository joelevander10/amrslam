# Runtime localisation readiness (spec §4.3), from localization_monitor_node.
# Latched; republished on change and at 2 Hz.
uint8 UNLOCALIZED=0     # no accepted initial pose, or reset
uint8 CHECKING=1        # initial pose given; waiting for convergence, stability and the operator
uint8 READY=2           # converged, stable, fresh sensors, operator confirmed scans align
uint8 LOST=3            # was READY (or CHECKING) and a stream went stale, covariance grew,
                        # or map->odom jumped more than the trigger; requires revalidation

std_msgs/Header header
uint32 generation           # layer generation that produced this; consumers drop other generations
uint8 state
bool operator_confirmed
bool can_confirm            # CHECKING and every automatic condition holds

# AMCL covariance (diagonal) of the last /amcl_pose
float64 cov_xx
float64 cov_yy
float64 cov_yaw

# Fraction of scan endpoints within match_tolerance of an occupied map cell at the
# estimated pose (spec §4.3 "consistency limits"). -1 until computed. This is what
# catches a kidnapped robot: AMCL without recovery injection stays confidently wrong.
float64 scan_match
# Fraction of beams LONGER than the saved map allows from the estimated pose, i.e.
# passing through a mapped obstacle. Unmapped clutter only shortens beams, so this
# is the unambiguous wrong-pose signature and the loss trigger. -1 until computed.
float64 scan_long

# Largest localisation correction (at the robot) seen since the last accepted initial pose
float64 last_jump_m
float64 last_jump_rad
builtin_interfaces/Time last_jump_stamp

# Ages in seconds, -1 when never received (spec §2.4 limits: scan 0.15, wheels 0.10, imu 0.20, tf 0.20)
float64 scan_age_s
float64 wheels_age_s
float64 imu_age_s
float64 tf_age_s
float64 amcl_age_s

string reason
