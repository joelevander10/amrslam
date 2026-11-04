# One image of the SICK GLS621 floor-label camera, aggregated over every code read in it
# (amr_base/gls_node, gls-wait-coding-plan §B.4). Published per image (~14.6 Hz with a label
# in view), and as valid=false at 5 Hz once no image has arrived for gls.stale_s: the camera
# sends nothing on a no-read, so absence is the only no-read signal.
std_msgs/Header header        # receive time; frame_id "gls_frame"
bool valid                    # a label read, enough codes, on a camera whose settings verified
bool comms_ok                 # the camera answers (read-only keepalive within 3 x keepalive_s)
bool config_ok                # the settings the integration relies on read back as expected
bool calibrated               # gls.xy_scale_m_per_count is measured: x_m/y_m/yaw_rad are meaningful
string label_id               # "x0000y0000": a NAME, never a coordinate; "" when not valid
# T_LC, the camera in the label frame, from the profile's scale and signs. 0 unless calibrated.
float64 x_m
float64 y_m
float64 yaw_rad
float64 spread_xy_m           # furthest code from the image's median (0 unless calibrated)
float64 spread_yaw_rad
uint16 n_codes                # codes used, after outlier rejection
uint16 n_outliers
# The same image in raw device units, always (bench work and calibration need them).
float64 x_counts              # median XMCL
float64 y_counts              # median YMCL
float64 angle_deg             # circular median, [0, 360)
float64 spread_counts
uint64 device_ts_ms           # $FAMS: groups codes into an image; device clock, not wall time
float32 fps                   # image rate over the last ~30 images
string detail                 # the link's own words: why not comms_ok / config_ok
