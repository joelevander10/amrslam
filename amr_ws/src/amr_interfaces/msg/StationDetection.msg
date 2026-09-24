# RFID station read, station_id_node -> mission (replaces the spec's QrDetection, reconciliation D-4).
# An RFID read means "somewhere in the antenna's read zone": identity, not pose.
# The node seeds /initialpose with WIDE covariance from stations.yaml; no pose here.
std_msgs/Header header
string rfid_tag          # 4 hex chars, same convention as the profile's branch_latch
string station_id        # from stations.yaml; empty if the tag is not registered
bool known
