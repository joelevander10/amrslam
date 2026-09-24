# Source this in a shell that will run HARDWARE launches (drivers/mapping/nav
# with sim:=false). amr_bringup.domains enforces it: a hardware launch in any
# other domain refuses to start. ~/.bashrc sources this by default on the vehicle.
export ROS_DOMAIN_ID=10
# The vehicle profile every node loads (config.py). Set it in ~/.bashrc on a vehicle that
# is not gvievo-01, e.g. export AGV_PROFILE=amr-qr-01 on the AMR QR.
export AGV_PROFILE="${AGV_PROFILE:-agv-01}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/agv_can/amr_ws/src/amr_bringup/config/cyclonedds-local.xml"
