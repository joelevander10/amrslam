"""Device drivers for everything that is not on the CAN bus.

Ethernet devices: the digital I/O island (`dio`, Modbus TCP) and the SICK
GLS621 floor-label camera (`gls`, CoLa A over TCP: the telegram parser and
frame aggregation; `gls_link` its reconnecting, read-only socket thread;
`gls_probe` its bench tool). The legacy
`modbus_io` and `lidar_scan` modules went with the legacy controller (U11,
2026-09-21): the nanoScan3 is `sick_safetyscanners2` on the ROS side.

CAN-bus devices live one level down, in `agv_core.drivers.canbus`.
"""
