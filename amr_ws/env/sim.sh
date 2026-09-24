# Source this in a shell that will run SIMULATION launches (sim.launch.py, or
# mapping/nav with sim:=true). Same loopback-only DDS; a different domain, so a
# sim on this host can never see the live drives.
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/agv_can/amr_ws/src/amr_bringup/config/cyclonedds-local.xml"
