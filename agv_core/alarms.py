"""The alarm catalogue: one code, two audiences.

Every stop, hold or fault in the stack carries a CODE. This module maps that
code to what the OPERATOR is told - a title in plain words, and the one action
that clears it - while the node's own `reason` string stays as the engineer's
detail. The operator surface (`/api/alarms`, the Home page, the Alarms page)
never shows a string this catalogue does not know.

`clears_by` is the load-bearing field: it decides which button the Home page
offers, so a code with the wrong `clears_by` sends the operator to the wrong
lever.

    auto            clears itself once the cause goes away; no button
    start_button    the physical Start button on the panel, after AUTO
    ack             Acknowledge on the Run page (an executor/line FAULT)
    recover         Recover on the Status page (a supervisor FAULT)
    operator        an operator act on a page (set the pose, pick a map)
    power_cycle     switch the drives off and on (a latched drive alarm)
    restart_service the service must be restarted (the base cannot be rebuilt)
    engineer        nothing on the panel clears it; call the engineer

Severity is the operator's urgency, not the log level: `info` = it will pass,
`warn` = it needs you but nothing is broken, `error` = it is stopped until acted on.

Deliberately dependency-free: the ROS nodes, the web app and the RUNBOOK
generator all import it, and none of them may pull in the others.

RUNBOOK section 4 is generated from these rows (`python3 -m agv_core.alarms --md`),
so the screen and the paper cannot drift apart. `--card` prints the laminated
one-page card for the vehicle.
"""

from __future__ import annotations

from dataclasses import dataclass

INFO, WARN, ERROR = "info", "warn", "error"
CLEARS = ("auto", "start_button", "ack", "operator", "recover", "power_cycle", "restart_service", "engineer")
# Worst first: the operator surface sorts standing alarms by this, so the one
# sentence on the Home page is the one that actually holds the vehicle.
SEVERITY_ORDER = {ERROR: 0, WARN: 1, INFO: 2}


@dataclass(frozen=True)
class Alarm:
    code: str
    severity: str
    clears_by: str
    title: str  # what the operator reads first; a state, not a diagnosis
    action: str  # the ONE thing to do, in the imperative
    hint: str  # the engineer's note: where to look, what it means


def _rows() -> tuple[Alarm, ...]:
    a = Alarm
    return (
        # ---- safety stops and holds: the vehicle is fine, something is in the way ----
        a("FIELD_BLOCKED", INFO, "auto", "Stopped: something is in the safety field",
          "Clear the area. The vehicle starts again by itself.",
          "Scanner OSSD -> FX3 -> STO. Executor/line hold cause 'field'; auto-resume after the hold window."),
        a("ESTOP", WARN, "start_button", "Stopped: emergency stop or safety chain",
          "Release the E-stop, then press Reset and Start on the panel.",
          "Hold cause 'estop'. Auto-resume only if auto_resume_estop is set in the profile."),
        a("PATH_BLOCKED", WARN, "auto", "Stopped: the way ahead is blocked",
          "Clear the route. The vehicle tries again by itself.",
          "Nav2 controller aborted the step (hold cause 'controller'); bounded retries, then FAULT."),
        a("WAITING_FOR_PREREQ", INFO, "auto", "Waiting: a sensor or the panel is not ready yet",
          "Wait a moment. If it stays, look at the Alarms page.",
          "Hold cause 'pending': a prerequisite went stale inside the grace window."),
        a("TRACK_LOST", WARN, "start_button", "Stopped: the tape is not under the sensor",
          "Push the vehicle back onto the tape, then press Start.",
          "Line layer hold cause 'track': MLS reports no track for longer than the loss grace."),
        a("RFID_LINK_LOST", WARN, "start_button", "Stopped: the RFID tag reader is not reporting",
          "Check the reader and its cable; once it is back, check where the vehicle is, then press Start.",
          "Line layer hold cause 'rfid': on a mission with tags the RFID stream broke while driving "
          "(link down, reconnect, encounter buffer overrun), so a stop or the U-turn may have been "
          "driven past. Never resumes by itself; Start is refused while the link is down."),
        a("TRACK_RATE_LOW", WARN, "engineer", "Stopped: the tape sensor is too slow",
          "Call the engineer: the tape sensor is not keeping up.",
          "Line hold cause 'rate': track_hz below the PID's rate gate (SDO fallback reads ~10 Hz)."),
        a("AUTHORITY_LOST", WARN, "start_button", "Stopped: the vehicle lost permission to drive",
          "Put the selector back to AUTO and press Start.",
          "Line hold cause 'authority': lease/permit withdrawn or the selector left AUTO."),
        a("DRIVES_NOT_READY", WARN, "start_button", "Stopped: the motors are not powered",
          "MANUAL: switch Manual Arm (DI08) on. AUTO: press Start (after the cabinet's safety "
          "reset if the E-stop was used).",
          "DriveStatus not operational while something wanted to move: the hold cause 'drives', "
          "or the mux gating a command. Since 2026-10-08 the drives are powered only on demand "
          "(DriveStatus.power_reason says why not)."),

        # ---- the drives ----
        a("SAFETY_RESET_NEEDED", WARN, "start_button", "Motors are off: the safety circuit needs a reset",
          "Press the blue Reset button on the cabinet.",
          "Both drives in 'Switch on disabled' (statusword 0x1270): STO held by the FX3, arming retries "
          "every 2 s and succeeds the moment the chain closes."),
        a("DRIVE_ALARM", ERROR, "power_cycle", "Drive alarm",
          "Switch the drives off and on, then press Recover.",
          "CANopen EMCY latched; 40C0h (alarm reset) is on the write deny-list, so only a power cycle "
          "clears it. See the code in the detail (e.g. 8130h = heartbeat lost)."),
        a("DRIVE_SILENT", ERROR, "power_cycle", "A drive stopped answering",
          "Switch the drives off and on, then press Recover.",
          "No TPDO/heartbeat from a node for driver_timeout_s while armed. Check CAN wiring and drive "
          "logic power before blaming the PC."),
        a("DRIVE_STOP_UNCONFIRMED", ERROR, "power_cycle", "The vehicle cannot confirm it stopped",
          "Stay clear. Switch the drives off and on, then press Recover.",
          "Fault stop without positive standstill evidence: the PC heartbeat is withheld on purpose so "
          "each drive's own 1016h trips."),
        a("DRIVE_FAULT", ERROR, "power_cycle", "A drive is in fault",
          "Switch the drives off and on, then press Recover.",
          "CiA-402 Fault state on a node while armed."),

        # ---- the laser scanner ----
        a("SCANNER_SILENT", WARN, "engineer", "The safety scanner is not sending data",
          "The vehicle can still be driven by hand. Call the engineer before running a mission.",
          "No /scan at all: the nanoScan3 driver could not reach 192.168.3.2:6060 (cable, power, "
          "netplan) or AMR_LIDAR=false. The driver is an OPTIONAL launch member, so nothing faults "
          "and nothing else reports it - this row is the only place it shows. The vehicle's STOP is "
          "the scanner's OSSD pair into the FX3 and is unaffected by the data link; mapping, "
          "navigation and localisation are dead without it."),
        a("SCANNER_STALE", WARN, "engineer", "The safety scanner stopped sending data",
          "The vehicle can still be driven by hand. Call the engineer before running a mission.",
          "/scan arrived and then stopped: driver died mid-run, or the Ethernet link dropped. "
          "Expected rate is 34 Hz."),

        # ---- the line sensor (manuals/mls-diagnostics-plan.md) ----
        a("MLS_FIELD_WEAK", INFO, "auto", "Weak tape signal under the line sensor",
          "Nothing to do now; the engineer checks the tape where this keeps happening.",
          "The MLS raised its magnetic-field event (2080h:04 = 1, manual 8.8): the field fell below "
          "the critical strength, so the unfiltered LCP is not within data-sheet spec. Worn, lifted "
          "or dirty tape, or the sensor too high. Only counted with a track under the sensor on the "
          "sample before; at most one per 2 s (the text counts the rest). History only, no motion "
          "change. Per-position record: ~/.amr/reports/tape_health (tools/tape_health_report.py)."),
        a("MLS_TEMP", WARN, "engineer", "The line sensor is outside its temperature range",
          "Call the engineer: the line sensor is too hot or too cold.",
          "2070h:01 outside the sensor's own thresholds 2070h:07..2070h:06 (manual 8.7), read every "
          "60 s and at once after a temperature event. Clears itself when back inside."),

        # ---- the panel and the command mux ----
        a("PANEL_STALE", ERROR, "engineer", "The control panel is not answering",
          "Call the engineer: the panel wiring or the I/O island is down.",
          "No fresh, valid PanelState: Modbus DIO at 192.168.1.30 lost, or panel_node down. No motion "
          "authority at all without it."),
        a("NOT_LEASED", WARN, "auto", "Waiting for the supervisor",
          "Wait. If it stays, press Restart on the Home page.",
          "Mux has no fresh ControlLease (supervisor down, starting or in a transaction)."),
        a("INHIBITED", INFO, "auto", "Motion is held by the supervisor",
          "Wait for the mode change to finish.",
          "Lease allowed = 0: a transaction, a save, STARTING, FAULT or STOPPING."),
        a("GENERATION_MISMATCH", WARN, "auto", "Motion is held after a mode change",
          "Wait. If it stays, press Restart on the Home page.",
          "A command or permit carries a superseded generation; the mux drops it by design."),
        a("NO_SOURCE", INFO, "auto", "Nothing is asking the vehicle to move",
          "Nothing to do. Press Start, or jog from the Manual page.",
          "Selector position is fine but no fresh command stream is selected."),
        a("SOURCE_TIMED_OUT", WARN, "auto", "The command stopped arriving",
          "Press and hold again. If a page is jogging, check the Wi-Fi link.",
          "The selected stream went stale (cmd_timeout_s 0.2 s): closed tab, dropped Wi-Fi or a "
          "released button - indistinguishable to the vehicle, by design."),
        a("NO_PERMIT", INFO, "auto", "Waiting for a mission step",
          "Nothing to do; the executor drives this.",
          "AUTO with no MotionPermit, or a permit whose source has no fresh /cmd_vel."),
        a("FIELD_PROTECTIVE", WARN, "auto", "Stopped: something is in the protective field",
          "Clear the area in front of the vehicle; it continues by itself.",
          "The mux zeroes every AUTO source while /output_paths says the protective field is "
          "violated - the software side of a stop the scanner's OSSD pair already makes."),
        a("FIELD_WARNING", INFO, "auto", "Slowed or stopped: something is in a warning field",
          "Clear the area ahead; auto speed returns by itself when the warning fields clear.",
          "cmd_mux scales every AUTO source while a warning field is occupied: warning 1 "
          "(outer, 1.77 m ahead) x0.5, warning 2 (inner, 0.97 m) a deceleration stop within 0.40 m, the stricter winning; the "
          "slowdown and the recovery ramp over 1 s. Paths and factors: amr_bringup/config/scanner_fields.yaml."),
        a("MARKERS_LOST", WARN, "start_button", "Stopped: the line sensor's markers are not reporting",
          "Check where the vehicle is (the home marker may have been passed), then press Start.",
          "Line layer hold cause 'markers': a marker run (mission marker_run, no RFID) lost the MLS "
          "marker stream, or marker events, while driving, so the U-turn or home marker may have been "
          "driven past. Never resumes by itself; Start is refused while the stream is down."),
        a("AT_STATION", INFO, "start_button", "Parked at a station",
          "Press Start when the station work is done; it moves after the start delay.",
          "Tape mission stop row (role always, or the job's destination): a measured stop over "
          "stop_distance_m after the tag, then a LineState HOLD with cause 'station'. The Home "
          "stop ends the run (DONE) instead."),
        a("LINE_MISSION", INFO, "auto", "Tape mission event",
          "Nothing to do; this records what the mission engine did with a tag.",
          "amr_line.tape_run events: stops, passed destinations, speed toggles, branch orders, "
          "U-turns, the run's result at Home. The text names the tag and the action."),
        a("LINE_MARKER", INFO, "auto", "Track marker read",
          "Nothing to do; this records a marker code the line sensor read beside the tape.",
          "MLS marker codes (manuals/mls-marker-plan.md): the sensor decodes the code, drive_node "
          "publishes /amr/line_marker, the line follower logs it. The mission's high-zone codes "
          "also set the speed (2026-10-09); no marker changes the run's state. Also logged: the "
          "marker stream going up or down, and markers lost on the topic (warn)."),
        a("LINE_LOC", INFO, "auto", "Tape localization event",
          "Nothing to do in shadow mode; a LOST or unexpected-fork entry is worth a look on the Teach page.",
          "amr_line.tape_runtime (tracked plan 3, P1 shadow): where the locator would have stopped, "
          "ordered a junction or approached the U-turn, and whether its uncertainty allowed it; "
          "LOST and re-acquired; teach passes and stations. Nothing here changes motion yet."),
        a("FIELD_UNKNOWN", WARN, "engineer", "Stopped: no safety-scanner field data",
          "Check the scanner cable and that its driver is running (Status page).",
          "Supervised AUTO is zeroed with no fresh /output_paths: a field state nobody can see "
          "is not clear. The OSSD stop path does not depend on this data link."),

        # ---- localisation ----
        a("LOC_LOST", ERROR, "operator", "The vehicle lost its position",
          "Set the initial pose on the Run page and confirm the scans line up.",
          "Umbrella code for a LocalizationState LOST without a more specific cause."),
        a("LOC_SCAN_MISMATCH", ERROR, "operator", "The vehicle is not where it thinks it is",
          "Set the initial pose on the Run page and confirm the scans line up.",
          "Too many beams pass THROUGH mapped obstacles (scan_long over the limit): the unambiguous "
          "wrong-pose signature."),
        a("LOC_COV_GREW", ERROR, "operator", "The vehicle is unsure where it is",
          "Set the initial pose on the Run page and confirm the scans line up.",
          "AMCL covariance over the limit for longer than cov_hold_s."),
        a("LOC_STREAM_STALE", ERROR, "engineer", "A sensor the vehicle navigates by went quiet",
          "Call the engineer: a sensor stopped (see the detail for which).",
          "scan/wheels/imu/tf/amcl age over the spec 2.4 limits."),
        a("LOC_JUMP", ERROR, "operator", "The vehicle's position jumped",
          "Set the initial pose on the Run page and confirm the scans line up.",
          "map->odom correction over the jump trigger since the last accepted pose."),
        a("LOC_GATE_FAILED", ERROR, "engineer", "The position check stopped running",
          "Call the engineer: the map/scan check failed.",
          "Scan-consistency gate or the transform it needs failed; no evidence either way, so LOST."),
        a("LOC_NOT_SET", WARN, "operator", "The vehicle does not know where it is yet",
          "On the Run page: pick the map, set the initial pose, then confirm.",
          "LocalizationState UNLOCALIZED: no accepted initial pose since the layer started."),
        a("LOC_NOT_CONFIRMED", INFO, "operator", "Waiting for you to confirm the position",
          "On the Run page, check the scans line up with the map and press Confirm.",
          "LocalizationState CHECKING with can_confirm true."),

        # ---- the route executor ----
        a("EXEC_OFF_PATH", ERROR, "ack", "The vehicle drifted off its route",
          "Press Acknowledge on the Run page, then reload the mission.",
          "Cross-track over the route's allowance; the step was interrupted."),
        a("EXEC_OVERSHOOT", ERROR, "ack", "The vehicle went past the step's end",
          "Press Acknowledge on the Run page, then reload the mission.",
          "Travel past the straight's length, or a turn overshoot, beyond tolerance."),
        a("EXEC_TURN_FAILED", ERROR, "ack", "The turn did not come out right",
          "Press Acknowledge on the Run page, then reload the mission.",
          "Turn centre drift, wrong direction of rotation, or remaining angle out of tolerance."),
        a("EXEC_ENDPOINT_MISSED", ERROR, "ack", "The step finished in the wrong place",
          "Press Acknowledge on the Run page, then reload the mission.",
          "Endpoint position/heading error over tolerance at step completion."),
        a("EXEC_ACTION_FAILED", ERROR, "ack", "The navigation software refused a step",
          "Press Acknowledge on the Run page. If it repeats, call the engineer.",
          "Nav2 action aborted/rejected/lost, or its server was unavailable."),
        a("EXEC_PREREQ_LOST", ERROR, "ack", "A sensor or the panel dropped out mid-step",
          "Press Acknowledge on the Run page, then check the Alarms page.",
          "A prerequisite stayed bad past prereq_grace_s, or wheel feedback stopped without a safety stop."),
        a("EXEC_POSE_INJECTED", ERROR, "ack", "The position was changed while the vehicle was moving",
          "Press Acknowledge on the Run page, then reload the mission.",
          "An /initialpose arrived during EXECUTING: progress is no longer trustworthy."),
        a("EXEC_ROUTE_INVALID", ERROR, "operator", "This mission cannot be run",
          "Pick another mission, or call the engineer to fix this one.",
          "Route/mission failed to compile or validate against the active map revision."),

        # ---- mapping ----
        a("MAP_SAVE_FAILED", ERROR, "operator", "The map was not saved",
          "Try Save again on the Maps page. If it fails twice, call the engineer.",
          "mapping_session save returned an error; the survey is still open."),
        a("SURVEY_NOT_READY", WARN, "operator", "The vehicle is not ready to survey",
          "Fix what the Alarms page lists, then start the survey again.",
          "mapping_session refused: sensor ages or mode preconditions not met."),

        # ---- the disk ----
        a("DISK_LOW", WARN, "engineer", "The vehicle is running out of storage",
          "Call the engineer: old maps and reports need deleting.",
          "Under 1 GB free on the state or maps filesystem. Surveys and saves still run; "
          "at 200 MB they are refused (DISK_FULL)."),
        a("DISK_FULL", ERROR, "engineer", "No storage left: new maps cannot be saved",
          "Call the engineer: the disk is full. A route already running is not affected.",
          "Under 200 MB free. Survey start and map save are refused; navigation and LINE are "
          "deliberately NOT gated - a full disk must not stop a vehicle that is already moving."),

        # ---- the supervisor and the service ----
        a("BASE_NOT_READY", WARN, "recover", "Not ready: the vehicle's basics did not come up",
          "Press the safety reset on the cabinet. The vehicle becomes ready by itself.",
          "Boot budget expired with drives/panel/mux/wheel feedback missing. Since 2026-09-22 the "
          "supervisor leaves FAULT on its own once the base reports ready."),
        a("BASE_EXITED", ERROR, "restart_service", "Needs service: the vehicle software stopped",
          "Switch the drives off and on, then press Restart on the Home page.",
          "The base process group died. Recovery cannot rebuild it: the drives must be re-armed."),
        a("BOOT_ERROR", ERROR, "restart_service", "Needs service: the vehicle software failed to start",
          "Press Restart on the Home page. If it repeats, call the engineer.",
          "Exception while spawning the base/web/foxglove groups; see journalctl -u amr.service."),
        a("SPAWN_FAILED", ERROR, "restart_service", "Needs service: a program would not start",
          "Press Restart on the Home page. If it repeats, call the engineer.",
          "Group.spawn failed (missing executable, bad environment)."),
        a("LAYER_EXITED", ERROR, "recover", "The mapping or navigation software stopped",
          "Press Recover on the Status page, then load the mission again.",
          "The mode layer's process group exited unrequested; the base is untouched."),
        a("LAYER_NOT_EMPTY", ERROR, "restart_service", "Needs service: old software would not go away",
          "Press Restart on the Home page.",
          "STOP_OLD found surviving members of the previous layer's process group."),
        a("LAYER_STOP_TIMEOUT", ERROR, "restart_service", "Needs service: software would not stop",
          "Press Restart on the Home page.",
          "The layer did not exit inside its stop budget."),
        a("STOP_TIMEOUT", ERROR, "restart_service", "Needs service: shutdown did not finish",
          "Press Restart on the Home page.",
          "Teardown exceeded its budget; systemd's TimeoutStopSec (55 s) is the backstop."),
        a("LAYER_START_TIMEOUT", ERROR, "recover", "The new software did not start in time",
          "Press Recover on the Status page, then try the mode change again.",
          "The new layer did not report ready inside layer_start_s; see ~/.amr/logs/layer.log."),
        a("LAYER_NO_READINESS", ERROR, "recover", "The new software never reported ready",
          "Press Recover on the Status page, then try the mode change again.",
          "The layer produced no readiness signal at all: it probably died during entry."),
        a("SAVE_UNKNOWN_OUTCOME", WARN, "operator", "The map may or may not have been saved",
          "Check the Maps page for a new revision before saving again.",
          "The save RPC passed its deadline after the request was accepted; a late answer is ignored."),
        a("MUX_ACK_TIMEOUT", ERROR, "recover", "The mode change did not finish",
          "Press Recover on the Status page.",
          "The mux never acknowledged the new generation inside the barrier budget."),
        a("LIFECYCLE_STARTUP", ERROR, "recover", "The new software did not become ready",
          "Press Recover on the Status page, then try the mode change again.",
          "A lifecycle node failed to configure/activate during a transition."),
        a("COORDINATOR_UNAVAILABLE", ERROR, "recover", "The survey software is not answering",
          "Press Recover on the Status page, then start the survey again.",
          "mapping_session service missing when the supervisor needed it."),
        a("SURVEY_RPC_TIMEOUT", ERROR, "recover", "The survey command got no answer",
          "Check the Maps page for a new revision, then press Recover.",
          "returned/abort/save RPC passed its deadline; a late answer is ignored."),
        a("SURVEY_START_REFUSED", WARN, "operator", "The survey would not start",
          "Fix what the Alarms page lists, then start the survey again.",
          "mapping_session refused the start request."),
        a("LOOP_ERROR", ERROR, "recover", "Internal error in the vehicle software",
          "Press Recover on the Status page. If it repeats, save a report and call the engineer.",
          "The supervisor caught an exception in its loop and failed the running operation."),
        a("UNCLEAN_SHUTDOWN", WARN, "power_cycle", "The vehicle lost power last time",
          "Switch the drives off and on, then press the safety reset on the cabinet.",
          "A running.json marker survived from before this boot (amr_bringup/uptime.py). The drives "
          "lost the PC heartbeat, so 8130h is latched and only a power cycle clears it. A marker from "
          "the SAME boot is a service crash instead and needs no drive action."),
        a("SERVICE_CRASHED", WARN, "auto", "The vehicle software restarted itself",
          "Nothing to do. Tell the engineer if it keeps happening.",
          "A marker from THIS boot survived: the supervisor died and systemd's Restart=on-failure "
          "or the 30 s watchdog brought it back. The drives were disarmed by the teardown or by "
          "their own 1016h, so no power cycle is needed. journalctl -u amr.service has the reason."),
        a("SUPERVISOR_DOWN", ERROR, "restart_service", "Needs service: the vehicle software is not running",
          "Press Restart on the Home page. If it repeats, call the engineer.",
          "No ModeState/lease reaching the web. systemd restarts on failure and on a watchdog timeout."),
        a("WEB_DOWN", ERROR, "restart_service", "Needs service: this page's server keeps stopping",
          "Press Restart on the Home page.",
          "The supervisor respawned the web group three times and gave up."),
        # ---- informational events: not alarms, but they carry a code and the
        # operator surface refuses to show a code it does not know. ----
        a("MODE_CHANGE", INFO, "auto", "The vehicle changed mode",
          "Nothing to do.", "Supervisor FSM transition; the text carries from -> to."),
        a("MUX_SOURCE", INFO, "auto", "The vehicle is taking commands from somewhere else",
          "Nothing to do.", "Command-source edge in the mux (pendant, manual, follow, line...)."),
        a("PANEL", INFO, "auto", "The control panel changed",
          "Nothing to do.", "Selector, Start/Reset edge or pendant change from panel_node."),
        a("BOOT_READY", INFO, "auto", "The vehicle is ready",
          "Nothing to do.", "End-of-boot report from the supervisor; the text lists anything missing."),
        a("RUN_SUMMARY", INFO, "auto", "A mission finished",
          "Nothing to do.", "One line per run at DONE/ABORT/FAULT: duration, distance, holds by cause."),
        a("PP_START", INFO, "auto", "A blind move started",
          "Nothing to do.", "Profile-position (blind-run) move accepted by the drive owner."),
        a("PP_REFUSED", WARN, "auto", "A blind move was refused",
          "Nothing to do; ask the engineer if it was expected.",
          "Blind-run move refused by the drive owner."),
        a("PP_VETOED", WARN, "auto", "A blind move was stopped before it started",
          "Nothing to do; ask the engineer if it was expected.",
          "Blind-run move vetoed at start: authority re-read after the drive setup (audit R02) "
          "no longer allowed it; the veto's controlwords were sent."),
        a("PP_FAULTED", WARN, "ack", "A blind move failed",
          "Press Acknowledge on the Run page.", "Blind-run move faulted mid-flight."),
        a("PP_ABANDONED", WARN, "auto", "A blind move was abandoned",
          "Nothing to do.", "Blind-run move dropped (authority lost or the owner disarmed)."),
        a("WEB_BUG", WARN, "auto", "A page did not work",
          "Reload the page. Save a report if it keeps happening.",
          "Unhandled exception in the Flask app or the page script; the reference id is in the web log."),
    )


CATALOGUE: dict[str, Alarm] = {row.code: row for row in _rows()}

UNKNOWN = Alarm(
    "UNKNOWN", ERROR, "engineer", "Something stopped the vehicle",
    "Save a report from the Alarms page and call the engineer.",
    "A code no catalogue row knows: whoever emitted it must be given a row in agv_core/alarms.py.",
)


def get(code: str) -> Alarm:
    """The row for `code`, or a safe UNKNOWN row carrying it. Never raises: an
    unknown code must still reach the operator as words, not as a blank panel."""
    row = CATALOGUE.get((code or "").strip().upper())
    if row is not None:
        return row
    return UNKNOWN if not code else Alarm(code, UNKNOWN.severity, UNKNOWN.clears_by,
                                          UNKNOWN.title, UNKNOWN.action, UNKNOWN.hint)


def describe(code: str, detail: str = "", since: float | None = None) -> dict:
    """One standing-alarm row for the operator surface / `/api/alarms`."""
    row = get(code)
    return {
        "code": row.code,
        "level": row.severity,
        "title": row.title,
        "action": row.action,
        "clears_by": row.clears_by,
        "detail": detail or "",
        "since": since,
        "hint": row.hint,
    }


# hold_cause (RunState / LineState) -> catalogue code. Both products share the hold
# vocabulary, so both products explain a stop with the same words.
HOLD_CODES = {
    "station": "AT_STATION",
    "field": "FIELD_BLOCKED",
    "estop": "ESTOP",
    "controller": "PATH_BLOCKED",
    "pending": "WAITING_FOR_PREREQ",
    "drives": "DRIVES_NOT_READY",
    "track": "TRACK_LOST",
    "rate": "TRACK_RATE_LOW",
    "authority": "AUTHORITY_LOST",
    "rfid": "RFID_LINK_LOST",
    "markers": "MARKERS_LOST",
}


def hold_code(cause: str) -> str:
    return HOLD_CODES.get((cause or "").strip().lower(), "WAITING_FOR_PREREQ")


# The buttons an operator surface may offer, keyed by clears_by. "" = no button.
BUTTON = {
    "auto": "",
    "start_button": "",
    "ack": "Acknowledge",
    "operator": "Set position",
    "recover": "Recover",
    "power_cycle": "Recover",
    "restart_service": "Restart",
    "engineer": "",
}

CARD_CODES = (
    "FIELD_BLOCKED", "ESTOP", "SAFETY_RESET_NEEDED", "BASE_NOT_READY",
    "DRIVE_ALARM", "LOC_LOST", "PANEL_STALE", "BASE_EXITED",
)


def markdown() -> str:
    """RUNBOOK section 4, generated. Grouped by what clears it, worst first."""
    out = ["<!-- generated by python3 -m agv_core.alarms --md; do not edit by hand -->",
           "Every code the vehicle can show, what the operator does about it, and the",
           "engineer's note. Generated from `agv_core/alarms.py`.", ""]
    for how in CLEARS:
        rows = [r for r in CATALOGUE.values() if r.clears_by == how]
        if not rows:
            continue
        rows.sort(key=lambda r: (SEVERITY_ORDER[r.severity], r.code))
        out += [f"### Cleared by: {how.replace('_', ' ')}", "",
                "| Code | Level | Operator sees | Operator does | Engineer's note |",
                "|---|---|---|---|---|"]
        out += [f"| `{r.code}` | {r.severity} | {r.title} | {r.action} | {r.hint} |" for r in rows]
        out.append("")
    return "\n".join(out)


def card() -> str:
    """The laminated one-pager: the eight codes most likely to meet an operator."""
    out = ["# If the vehicle stops", "",
           "| It says | Do this |", "|---|---|"]
    out += [f"| **{CATALOGUE[c].title}** | {CATALOGUE[c].action} |" for c in CARD_CODES if c in CATALOGUE]
    out += ["",
            "Buttons: **Restart** is on the Home page, **Recover** on Status, "
            "**Acknowledge** on Run. Reset and Start are on the cabinet.",
            "", "Anything else: save a report from the Alarms page and call the engineer.", ""]
    return "\n".join(out)


def _main(argv: list[str]) -> int:
    if "--card" in argv:
        print(card())
    elif "--md" in argv:
        print(markdown())
    else:
        print(f"{len(CATALOGUE)} codes. Use --md (RUNBOOK section) or --card (operator card).")
    return 0


if __name__ == "__main__":  # pragma: no cover - a generator, exercised by tests
    import sys

    raise SystemExit(_main(sys.argv[1:]))
