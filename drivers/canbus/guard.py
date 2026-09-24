#!/usr/bin/env python3
"""Which CANopen objects the navigation PC is allowed to WRITE.

can-monitoring-plan.txt section 8, and the doc calls it the most important
section in the file. CANopen is bidirectional, and the objects below can defeat
safety behaviour from a single stray frame:

  403Eh  Driver input command. R-IN6 defaults to FREE. Writing bit 6 RELEASES
         THE HOLDING BRAKE on BOTH drive wheels and de-excites the motors, with
         600 kg behind the hitch. A bug, a stale frame or a bad mask is enough.
  40D0h  Clear ETO. Re-excites the motor. Called automatically after a
         protective field clears, this IS an automatic restart - prohibited by
         ISO 3691-4.
  40C0h  Alarm reset, and 6040h bit 7 (fault reset). An alarm that clears
         itself is an alarm nobody learns about; repeated auto-reset of an
         overvoltage hides a real regen problem until it becomes a stoppage.
  1010h  Store parameters / 1011h Restore defaults. 1011h wipes configuration
         back to factory, including the safety-relevant stop parameters.
  40C6h  and the 4000h-4FFFh parameter block generally. Writing quick-stop
         rate, QSTOP action, ETO reset action or the overvoltage threshold from
         the nav stack is how the two drives silently diverge.

The posture is READ-MOSTLY: permit only the writes motion genuinely needs, deny
everything else by index, and assert it in a test. The deny-list is documented
because an assessor will ask for it.

Deliberately dependency-free, like the rest of canbus/.
"""


class ForbiddenWrite(Exception):
    """A write to an object the navigation PC must never touch."""


# Everything motion actually needs, and nothing else.
#   6040h controlword     - CiA 402 state machine (bit 7 masked, see below)
#   6060h modes           - set to pv at arm time
#   6083h/6084h ramps     - profile accel/decel, written per mode at arm
#   60FFh target velocity - the setpoint itself
#   1017h heartbeat       - producer heartbeat time; enabling it is what makes
#                           a dead drive distinguishable from an idle one, and
#                           it cannot influence motion.
ALLOWED = {
    0x6040: "controlword",
    0x6060: "modes of operation",
    0x6083: "profile acceleration",
    0x6084: "profile deceleration",
    0x60FF: "target velocity",
    0x1017: "producer heartbeat time",
    # 1016h consumer heartbeat: the drive's OWN response to losing the PC (spec
    # §3.3). The ROS drive node produces a heartbeat and tells each drive to
    # expect it; if the PC dies the drive raises 8130h and applies its fault
    # reaction (605Eh) with nothing on the PC involved. It cannot start motion,
    # and the alarm it raises is cleared by an operator, not by us (40C0h).
    0x1016: "consumer heartbeat time",
    # Profile position (pp) blind moves: the SET-POINT of a move and nothing
    # about how the drive reacts to one. Only reached while profile pp.enabled
    # is true. The pp safety configuration - 6072h max torque, 6065h following
    # error window, 6067h position window, 605Dh halt option, 605Eh fault
    # reaction, 6085h quick stop, 60F2h positioning option - is deliberately NOT
    # here: a human sets it with MEXE02 (RUNBOOK section 3), and the
    # drive owner only READS it back and refuses a move on any difference.
    0x607A: "target position",
    0x6081: "profile velocity",
}

# Named so a test can assert that none of them ever becomes writable.
PP_CONFIG_READ_ONLY = (0x6072, 0x6065, 0x6067, 0x605D, 0x605E, 0x6085, 0x60F2)

# Named purely so a refusal can say WHY, rather than "not allowed".
FORBIDDEN = {
    0x403E: "driver input command - bit 6 is FREE and releases both brakes",
    0x40D0: "clear ETO - re-excites the motor; automatic restart is prohibited",
    0x40C0: "alarm reset - alarms must never be cleared automatically",
    0x1010: "store parameters",
    0x1011: "restore default parameters - wipes the safety-relevant config",
    0x40C6: "configuration",
}

# 6040h is allowed, but bit 7 of it is Fault reset, which is 40C0h by another
# name. Masked rather than refused, so the ordinary state-machine writes the
# arm sequence performs are unaffected.
CONTROLWORD_FAULT_RESET = 1 << 7

# ---------------------------------------------------------------------------
# PDO CONFIGURATION (CiA 301), and the asymmetry that matters
# ---------------------------------------------------------------------------
# Migrating 60FFh from a blocking SDO write to RPDO1, and enabling the MLS's
# yaw-rate TPDO, both need these ranges. They were refused by the "not on the
# permitted-write list" branch, which is correct-by-default but too blunt to
# build on.
#
# *** AN RPDO MAPPING IS A WRITE PATH BY ANOTHER NAME. *** Map 403Eh into an
# RPDO and a two-byte CAN frame releases the holding brake on both drive
# wheels - with no SDO write anywhere, and every deny-list check above passed.
# That is not a hypothetical hole; it is the same hazard 403Eh is on the list
# for, reachable by a route the list did not cover.
#
# So the ranges are NOT simply added to ALLOWED. Each is admitted on its own
# terms:
#
#   RPDO mapping   permitted only when the MAPPED OBJECT is itself writable.
#                  The deny-list is applied recursively to index (value >> 16),
#                  so a forbidden object cannot be smuggled in behind a PDO.
#   TPDO mapping   permitted for any object. A TPDO is a READ path - the device
#                  transmits, we receive - and the posture has always been
#                  read-mostly, not read-nothing. Refusing these would forbid
#                  reading a temperature by PDO while permitting it by SDO.
#   comm params    permitted. COB-ID and transmission type decide WHERE and WHEN
#                  a PDO goes, never WHAT it carries, so they cannot reach an
#                  object the mapping rules above have not already cleared.
PDO_RANGES = (
    (0x1400, 0x15FF, "rpdo_comm"),
    (0x1600, 0x17FF, "rpdo_map"),
    (0x1800, 0x19FF, "tpdo_comm"),
    (0x1A00, 0x1BFF, "tpdo_map"),
)

# Sub 0 of a mapping object is the ENTRY COUNT (0-8), not an object reference.
# Writing 0 to it is how a mapping is disabled before being rewritten, which is
# the first step of every remap - so it must not be decoded as a mapping entry,
# where the count 2 would read as index 0000h and be refused.
MAPPING_COUNT_SUB = 0


def _pdo_kind(index):
    for lo, hi, kind in PDO_RANGES:
        if lo <= index <= hi:
            return kind
    return None


def _check_rpdo_mapping(index, value, sub):
    """An RPDO mapping entry, validated against the deny-list it would bypass.

    A mapping entry is a u32: index << 16 | subindex << 8 | bit length. Zero is
    an empty slot, which is how a mapping is shortened, and carries no object.
    """
    if sub == MAPPING_COUNT_SUB:
        return None                     # the entry count, not an object
    if sub is None:
        raise ForbiddenWrite(
            f"write to {index:04X}h refused: an RPDO mapping needs its "
            f"subindex to be checked - sub 0 is the entry count, sub 1-8 are "
            f"object references, and they cannot be told apart from the value")
    if value is None:
        raise ForbiddenWrite(
            f"write to {index:04X}h:{sub:02X} refused: an RPDO mapping entry "
            f"cannot be checked without its value")
    if value == 0:
        return None                     # empty slot; maps nothing
    mapped = (value >> 16) & 0xFFFF
    try:
        check(mapped)
    except ForbiddenWrite as e:
        raise ForbiddenWrite(
            f"RPDO mapping {index:04X}h:{sub:02X} would map {mapped:04X}h, "
            f"which is not writable: {e}. An RPDO is a write path - mapping a "
            f"forbidden object into one would let a plain CAN frame do what a "
            f"direct SDO write is refused (can-monitoring-plan.txt section 8)"
        ) from None
    return None


def check(index, value=None, sub=None):
    """Raise ForbiddenWrite unless this object may be written. Else return None.

    Called on EVERY write path. A denied write raises rather than being quietly
    dropped: silently ignoring a command that a caller believed had landed is
    its own hazard.

    `sub` is required for RPDO mapping objects and ignored everywhere else - see
    PDO_RANGES for why a mapping entry cannot be judged without it.
    """
    if index in FORBIDDEN:
        raise ForbiddenWrite(
            f"write to {index:04X}h refused: {FORBIDDEN[index]} "
            f"(can-monitoring-plan.txt section 8)")
    # The whole manufacturer parameter block. 40D0h/40C0h/40C6h are named above
    # for a better message; this catches every other 4xxxh index, which is what
    # keeps the left/right configuration from diverging.
    if 0x4000 <= index <= 0x4FFF:
        raise ForbiddenWrite(
            f"write to {index:04X}h refused: driver parameters are changed by a "
            f"controlled maintenance procedure, not by the vehicle controller "
            f"(can-monitoring-plan.txt section 8)")
    # PDO configuration, admitted per-range on its own terms. Placed BEFORE the
    # ALLOWED check because these ranges are permitted by rule rather than by
    # being listed - ALLOWED stays the set of objects motion writes directly.
    kind = _pdo_kind(index)
    if kind == "rpdo_map":
        return _check_rpdo_mapping(index, value, sub)
    if kind is not None:
        return None                     # comm params and TPDO mappings
    if index not in ALLOWED:
        raise ForbiddenWrite(
            f"write to {index:04X}h refused: not on the permitted-write list "
            f"{sorted(f'{i:04X}h' for i in ALLOWED)}")
    if index == 0x6040 and value is not None and value & CONTROLWORD_FAULT_RESET:
        raise ForbiddenWrite(
            "controlword bit 7 (fault reset) refused: an alarm must be cleared "
            "by a deliberate operator acknowledgment, never by the nav stack "
            "(can-monitoring-plan.txt section 8)")
    return None


def is_allowed(index, value=None, sub=None):
    """Boolean form, for tests and for reporting the list in the UI."""
    try:
        check(index, value, sub)
        return True
    except ForbiddenWrite:
        return False
