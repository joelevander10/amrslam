#!/usr/bin/env python3
"""BLVD-KRD alarm and statusword decoding. Pure tables - no bus, no imports.

Standalone like the rest of canbus/: it must never import config, so the bench
scripts and the web app share one definition of what an alarm code means.

*** DIAGNOSTIC ONLY. *** Nothing decoded here may gate motion. The safety chain
is lidar/encoders -> FX3 -> HWTO1/HWTO2 -> STO with EDM wired back as hardware.
CAN is the logging and display channel beside it (can-monitoring-plan.txt s.0).

Codes are from can-monitoring-plan.txt section 3, which tiers them MUST / SHOULD
by how much they matter on this vehicle. Only those two tiers are carried here;
the limit/homing group is USEFUL-only on a free-roaming AGV and is left out
rather than padding the table with entries nobody reads.

EMCY frames are PUSHED on COB-ID 080h + Node-ID. They are events in their own
right, so unlike a polled fault bit they may be emitted one-for-one without
tripping the events.py flooding rule.
"""

# severity: "error" = vehicle down or safety-relevant, "warn" = degrading.
# Both map onto events.py levels directly.
MUST, SHOULD = "must", "should"

EMCY_CODES = {
    # -- communication (error register 11h) --------------------------------
    0x8110: ("CAN overrun", "warn", SHOULD, "bus loading too high"),
    0x8120: ("CAN error passive", "error", MUST, "wiring or termination fault"),
    0x8130: ("Heartbeat / node guarding error", "error", MUST,
             "a drive stopped answering"),
    0x8140: ("Recovered from bus-off", "error", MUST,
             "a serious bus fault occurred"),
    0x8210: ("PDO length error", "warn", SHOULD, "mapping mismatch - config bug"),

    # -- safety chain and brake (error register 81h) - highest value -------
    0xFF53: ("HWTO input circuit error", "error", MUST,
             "the drive detected a fault in its own STO input circuitry - "
             "treat as immediate vehicle-down"),
    0xFF68: ("HWTO input detection", "error", MUST,
             "HWTO went OFF (only raised when the alarm-at-HWTO-OFF parameter "
             "is enabled)"),
    0xFF50: ("Brake overcurrent", "error", MUST,
             "brake coil drawing too much - degrading coil or short"),
    0xFF55: ("Brake connection error", "error", MUST,
             "brake coil open circuit - the holding function is not in its "
             "designed state"),

    # -- power and thermal -------------------------------------------------
    0xFF22: ("Overvoltage", "error", MUST,
             "regen with nowhere to go - the BMS is refusing charge current"),
    0xFF25: ("Undervoltage", "error", MUST,
             "battery sag - stopping distance was already degraded"),
    0xFF21: ("Main circuit overheat", "error", MUST, ""),
    0xFF26: ("Motor overheat", "error", MUST, ""),
    0xFF20: ("Overcurrent", "error", MUST, "not resettable; non-excitation"),
    0xFF30: ("Overload", "warn", SHOULD, "sustained torque demand"),
    0xFF31: ("Overspeed", "error", MUST, "cross-check against the FX3 SLS limit"),

    # -- drivetrain and feedback -------------------------------------------
    0xFF10: ("Position deviation", "warn", SHOULD,
             "commanded vs actual diverged - wheel slip, a dragging brake, or "
             "gear-train trouble"),
    0xFF28: ("Encoder error", "error", MUST, ""),
    0xFF2A: ("Encoder communication error", "error", MUST, ""),
    0xFF42: ("Initial encoder error", "error", MUST, ""),
    0xFF44: ("Encoder EEPROM error", "warn", SHOULD, ""),
    0xFF45: ("Motor combination error", "error", MUST,
             "wrong motor/driver pairing - catches a wrong spare part"),

    # -- controller and config ---------------------------------------------
    0xFFF0: ("CPU error", "error", MUST, ""),
    0xFFF3: ("CPU overload", "error", MUST, ""),
    0xFF41: ("EEPROM error", "error", MUST, "parameter store corrupt"),
    0xFF70: ("Operation data error", "warn", SHOULD, ""),
    0xFF71: ("Unit setting error", "warn", SHOULD, ""),
    0xFF8C: ("Outside setting range", "warn", SHOULD, ""),
    0xFF81: ("Network bus error", "error", MUST, ""),
}

# Not a fault: the drive sends this when every error has cleared.
EMCY_CLEARED = 0x0000

# 1001h error register bits (also byte 2 of every EMCY frame).
ERROR_REGISTER_BITS = {0: "generic", 4: "communication", 7: "manufacturer"}

# NMT state, as reported in the heartbeat byte on COB-ID 700h + Node-ID.
NMT_STATES = {0x00: "Boot-up", 0x04: "Stopped", 0x05: "Operational",
              0x7F: "Pre-operational"}


def decode_emcy(data):
    """8-byte EMCY payload -> dict. Never raises; a short frame still decodes.

    byte 0-1  emergency error code, little-endian
    byte 2    error register, mirrors 1001h
    byte 3-7  zero
    """
    b = bytes(data or b"").ljust(8, b"\x00")
    code = b[0] | (b[1] << 8)
    reg = b[2]
    if code == EMCY_CLEARED:
        return {"code": code, "hex": "0000h", "name": "error reset",
                "level": "info", "tier": None, "note": "all errors cleared",
                "error_register": reg, "cleared": True,
                "register_bits": _register_bits(reg)}
    name, level, tier, note = EMCY_CODES.get(
        code, (f"unknown alarm 0x{code:04X}", "error", None,
               "not in the monitored set - look it up in the manual"))
    return {"code": code, "hex": f"{code:04X}h", "name": name, "level": level,
            "tier": tier, "note": note, "error_register": reg,
            "cleared": False, "register_bits": _register_bits(reg)}


def _register_bits(reg):
    return [n for bit, n in ERROR_REGISTER_BITS.items() if reg & (1 << bit)]


def decode_nmt(byte):
    """Heartbeat state byte -> name. Bit 7 is the toggle in node-guarding."""
    return NMT_STATES.get(byte & 0x7F, f"unknown (0x{byte:02X})")


# -- statusword (6041h) -------------------------------------------------------
# The CiA 402 state itself is decoded by bus_health.decode_state(). These are
# the flag bits beside it, which cost nothing extra because 6041h is already
# polled every telemetry pass.
# *** Bit 5 (QS) is NOT in this list, deliberately. ***
# can-monitoring-plan.txt says "bit 5 QS = quick stop active", which is
# inverted. The manual groups bits 0-6 as the CiA 402 state machine, and the
# state table bears that out: Operation enabled is 0x27 with bit 5 SET, Quick
# stop active is 0x07 with bit 5 CLEAR. Read as an active-high flag it would
# light permanently in normal running and go dark during an actual quick stop -
# exactly backwards. bus_health.decode_state() already reports "Quick stop
# active" correctly by matching the whole masked pattern, so there is nothing
# to add here.
SW_FLAGS = [
    (3,  "FAULT", "error", "alarm active"),
    (7,  "WARN", "warn", "warning or alarm occurred"),
    (11, "ILA", "warn",
     "internal limit active - a software limit, FW/RV-LS, FW/RV-BLK, STOP, "
     "QSTOP or CLR is asserted. This is how the FX3 quick stop becomes "
     "visible over CAN."),
]


def decode_statusword_flags(sw):
    """Statusword -> the flag bits that are set, most significant first."""
    if sw is None:
        return []
    return [{"bit": bit, "name": name, "level": level, "note": note}
            for bit, name, level, note in SW_FLAGS if sw & (1 << bit)]
