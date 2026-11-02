"""CANopen layer: frame decoders, the write deny-list and bench utilities.

`guard`, `alarms`, `bus_health`, `rpdo` and `drive_forward` are libraries — the
bus owner (`amr_base.canopen`) imports them.

`verify_bus`, `verify_drivers`, `read_imu`, `read_mls` and `lss` are ALSO
runnable on a bench against a live can0:

    python3 -m agv_core.drivers.canbus.verify_drivers

Run them with `-m` from the repo root, not as a bare file path. As plain
scripts only their own directory lands on sys.path, and the `agv_core.` imports
they now use would not resolve.
"""
