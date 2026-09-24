"""Drive health monitoring: the analogue and motion objects, polled by SDO.

*** DIAGNOSTIC AND LOGGING ONLY. *** Nothing here may decide whether it is safe
to move. The safety chain is lidar/encoders -> FX3 -> HWTO1/HWTO2 -> STO, wired
in hardware. This is the channel beside it that says what happened and warns
before a trip (manuals/can-monitoring-plan.txt section 0).

WHY ROUND-ROBIN AND NOT A BURST
-------------------------------
canworker._poll_telemetry() already fires six blocking SDO reads back to back
every 200 ms, and that burst is most of a 20 ms control tick. Eleven more
objects x two nodes as a burst would blow the tick outright, and a late tick is
a steering update the vehicle does not get.

So this yields ONE object per node per tick. The cost is bounded at two SDO
round-trips regardless of how long the table grows, and a full sweep of eleven
objects completes in about 220 ms at 50 Hz - inside the 1-5 Hz the plan asks
for Tier 3, and the values are slow-moving anyway.

The doc's own advice is the same: "do not poll aggressively by SDO when a TPDO
will do". We are not remapping TPDOs (that needs driver reconfiguration and the
bus already carries the MLS stream at 100 Hz), so the poll has to be polite.

DATA TYPES COME FROM THE MANUAL, NOT THE PLAN
---------------------------------------------
can-monitoring-plan.txt marks several types "INT?" and guesses at others. The
manual settles them, and the plan guessed WRONG on the two voltages: 40A4h and
40A3h are INT16, not INT32. Decoding a 16-bit signed value as 32-bit reads
plausible-looking garbage rather than failing, so this table cites the manual.
"""
import struct

# (index, key, label, ctype, scale, unit, tier)
#   ctype  how to unpack the SDO payload - see _decode()
#   scale  multiply the raw value by this to get `unit`
MUST, SHOULD = "must", "should"

OBJECTS = [
    # -- Tier 3, analogue health. The early-warning channel; trend these. -----
    (0x40A4, "bus_v", "Main supply voltage", "i16", 0.1, "V", MUST),
    (0x407C, "drv_c", "Driver temperature", "i16", 0.1, "C", MUST),
    (0x407D, "mtr_c", "Motor temperature", "i16", 0.1, "C", MUST),
    (0x40A3, "inv_v", "Inverter voltage", "i16", 0.1, "V", SHOULD),
    # Negative current IS the regeneration case - the plan's F-05.
    (0x409B, "cur_a", "Main supply current", "i32", 0.001, "A", SHOULD),
    (0x406B, "torque", "Torque monitor", "i16", 0.1, "%", SHOULD),
    # Rising load factor at constant duty = drivetrain drag, a dragging brake,
    # or a failing bearing. Good predictive signal.
    (0x406C, "load", "Load factor", "i32", 0.1, "%", SHOULD),

    # -- Tier 4, motion feedback. Motor-side ABZO encoder, upstream of the
    #    1:30 gearhead, and NOT safety rated - do not confuse it with the SIL 3
    #    wheel encoders feeding the FX3.
    (0x4073, "pos_dev", "Position deviation", "i32", 1.0, "", SHOULD),
    (0x4075, "spd_dev", "Speed deviation", "i32", 1.0, "", SHOULD),

    # -- Tier 5, comms health -------------------------------------------------
    (0x4056, "comm_err", "Present comms error", "u8", 1.0, "", SHOULD),

    # -- Provisional. 407Bh is UINT32 per the manual, but its bit layout is
    #    cross-referenced from the Function Edition's Modbus register tables and
    #    the plan's section 11 says to confirm it on the bench. Carried as a raw
    #    word so it is visible without asserting a decode we have not verified.
    (0x407B, "info", "Information status 1", "u32", 1.0, "", SHOULD),
]

OBJECT_BY_KEY = {o[1]: o for o in OBJECTS}


def _decode(ctype, payload):
    """SDO payload -> signed/unsigned int of the right width. None if short."""
    if payload is None:
        return None
    b = bytes(payload)
    try:
        if ctype == "u8":
            return b[0] if b else None
        if ctype == "i16":
            return struct.unpack("<h", b.ljust(2, b"\x00")[:2])[0]
        if ctype == "u16":
            return struct.unpack("<H", b.ljust(2, b"\x00")[:2])[0]
        if ctype == "i32":
            return struct.unpack("<i", b.ljust(4, b"\x00")[:4])[0]
        if ctype == "u32":
            return struct.unpack("<I", b.ljust(4, b"\x00")[:4])[0]
    except struct.error:
        return None
    return None


class MonitorPoller:
    """Round-robin cursor over OBJECTS x nodes. Pure bookkeeping - no bus.

    The caller does the actual SDO read, so this stays testable without
    hardware and cannot itself stall a control tick.
    """

    def __init__(self, nodes, objects=None):
        self._nodes = list(nodes)
        self._objects = list(objects if objects is not None else OBJECTS)
        self._i = 0
        self.values = {n: {} for n in self._nodes}
        self.sweeps = 0

    def next_object(self):
        """The (index, key, ctype) due this tick, or None if the table is empty.

        Advances one step per call. One object per node per tick keeps the added
        bus time bounded no matter how many objects are being watched.
        """
        if not self._objects:
            return None
        idx, key, _label, ctype, _scale, _unit, _tier = self._objects[self._i]
        self._i += 1
        if self._i >= len(self._objects):
            self._i = 0
            self.sweeps += 1
        return idx, key, ctype

    def store(self, node, key, raw):
        """Record one decoded reading. raw None means the read did not answer."""
        obj = OBJECT_BY_KEY.get(key)
        if obj is None or node not in self.values:
            return
        _idx, _key, label, _ctype, scale, unit, tier = obj
        self.values[node][key] = {
            "label": label, "unit": unit, "tier": tier,
            "raw": raw,
            "value": None if raw is None else round(raw * scale, 4),
        }

    def snapshot(self, thresholds=None):
        """Everything read so far, with warn/trip state applied per object.

        Thresholds live in the profile, not here - a temperature limit is a
        vehicle fact, and the two drives could differ.

        Limits are two-sided because the interesting quantities fail in both
        directions: a temperature only matters when it climbs, but bus voltage
        matters at BOTH ends - low is battery sag with stopping distance
        already degraded, high is regen the battery is refusing to take.
            {"warn": x, "trip": y}          trips when value >= x / >= y
            {"warn_low": x, "trip_low": y}  trips when value <= x / <= y
        """
        th = thresholds or {}
        out = {}
        for node, vals in self.values.items():
            node_out = {}
            for key, v in vals.items():
                d = dict(v)
                limits = th.get(key) or {}
                for k in ("warn", "trip", "warn_low", "trip_low"):
                    d[k] = limits.get(k)
                d["state"] = "ok"
                val = d["value"]
                if val is not None:
                    hi_t, hi_w = limits.get("trip"), limits.get("warn")
                    lo_t, lo_w = limits.get("trip_low"), limits.get("warn_low")
                    if (hi_t is not None and val >= hi_t) or                        (lo_t is not None and val <= lo_t):
                        d["state"] = "trip"
                    elif (hi_w is not None and val >= hi_w) or                          (lo_w is not None and val <= lo_w):
                        d["state"] = "warn"
                node_out[key] = d
            out[str(node)] = node_out
        return {"nodes": out, "sweeps": self.sweeps,
                "objects": len(self._objects)}
