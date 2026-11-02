"""Device drivers for everything that is not on the CAN bus.

Ethernet devices: the digital I/O island (`dio`, Modbus TCP) and the Chafon
RFID reader (`rfid`, kept for the tape product's stations). The legacy
`modbus_io` and `lidar_scan` modules went with the legacy controller (U11,
2026-09-21): the nanoScan3 is `sick_safetyscanners2` on the ROS side.

CAN-bus devices live one level down, in `agv_core.drivers.canbus`.
"""
