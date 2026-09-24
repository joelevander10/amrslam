# Operator panel image, debounced (core/panel.PanelScan semantics): the DIO adapter
# (T12) or fake_panel_node in sim. Stale (> 0.2 s) means no motion permission.
std_msgs/Header header
bool valid            # a trusted, debounced image exists
bool mode_auto        # selector: true = AUTO, false = MANUAL  ("auto" is a C++ keyword)
bool start_edge       # rising edge of Start on THIS message
bool reset_edge       # rising edge of Reset on THIS message
uint32 seq            # increments per scan, so edges are never missed by a slow consumer
# Jog pendant (core/panel.pendant_intent): direction levels, an opposing pair
# cancelled. Levels, not edges: held = keep moving.
bool pendant_fwd
bool pendant_rvs
bool pendant_left
bool pendant_right
