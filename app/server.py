"""Flask front end for the AGV: a manual jog pad and diagnostic pages.

Read the safety model before changing anything here:

  * THE WEB APP CANNOT START THE VEHICLE. There is no /api/arm and no
    /api/arm: the physical panel owns entering every state - the selector
    sitting in MANUAL is the arm command. The only thing here that can
    produce motion is /api/drive, and only while the vehicle is already armed
    in MANUAL - which only the panel can bring about.
  * A manual direction is HELD, not latched. The browser re-POSTs /api/drive
    about every 100 ms while the button or key is down; the bus thread zeros
    the setpoint if it misses three in a row. Closing the tab, losing Wi-Fi and
    letting go all look the same to the AGV, which is the point.
  * Everything else here only ever stops: /api/disarm de-energises, /api/stop
    zeroes the setpoint, and /api/restart de-energises and then ends the
    process. None of them can produce motion, and the last is refused outright
    while the vehicle is armed.
  * /api/blind/plan and /api/blind/clear only store or discard a blind-run
    PLAN. Nothing here runs it: PB Start on the panel does, with the selector
    in MANUAL, and the DI scan - not this page's poll - is what keeps that run
    alive. See canworker._panel_scan().
"""
import os
import subprocess
import sys
import threading

from flask import Flask, jsonify, redirect, render_template, request

# This file lives in app/, so the repo root is its PARENT. Anchoring to the root
# rather than to this file is what makes `python3 main.py`, a systemd unit with
# an absolute path, and an import from any cwd all resolve the same modules.
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ("", "core", "drivers", os.path.join("drivers", "canbus")):
    sys.path.insert(0, os.path.join(_ROOT, _d) if _d else _ROOT)

# Bare imports throughout - see the note in canworker.py on why the layer
# directories go on sys.path instead of becoming packages. templates/ and
# static/ are siblings of THIS file, which is exactly where Flask looks.
import config  # noqa: E402
import events  # noqa: E402
import motion  # noqa: E402
from canworker import Controller  # noqa: E402

# guard holds the permitted/forbidden write lists from the monitoring plan's
# section 8; the monitor page displays them because an assessor will ask.
import guard  # noqa: E402
import ownerlock  # noqa: E402

app = Flask(__name__)
ctl = Controller()


def _fail(msg, code=409):
    return jsonify({"ok": False, "error": str(msg)}), code


# ---- pages ----------------------------------------------------------------

@app.get("/")
def index():
    """The bare address lands on /manual, which is the only page that drives.

    A redirect rather than rendering it here, so the address bar names the page
    it is showing and a bookmark of the landing page is a bookmark of /manual.

    It used to land on /auto, which watched a tape-following run. That page is
    gone with the feature; when navigation lands, its page becomes the natural
    landing target again.
    """
    return redirect("/manual")


@app.get("/manual")
def manual():
    # No `table`, `full` or `half`: the pad no longer prints the 60FFh setpoints
    # under each arrow. They are still reported by /api/config, which is where
    # something reading them programmatically should look.
    return render_template(
        "manual.html", page="manual", pad=motion.PAD, labels=motion.LABELS,
        glyphs=motion.GLYPHS, keymap=motion.KEYMAP,
        rfid_ip=f"{config.RFID_IP}:{config.RFID_PORT}",
        watchdog_ms=int(config.MANUAL_WATCHDOG_S * 1000))


@app.get("/monitor")
def monitor():
    return render_template(
        "monitor.html", page="monitor",
        heartbeat_ms=config.CAN_HEARTBEAT_MS,
        bitrate_kbps=config.CAN_BITRATE // 1000,
        nodes={str(n): config.NODES[n] for n in config.NODES},
        allowed=sorted(f"{i:04X}h" for i in guard.ALLOWED),
        scan_period_s=config.LOOP_PERIOD_S,
        imu_enabled=config.IMU_ENABLED,
        imu_node=config.SENSOR_NODE,
        imu_period_ms=round(1000 * config.IMU_PERIOD_S),
        forbidden=sorted(f"{i:04X}h" for i in guard.FORBIDDEN))


@app.get("/blind")
def blind():
    """Encoder-only test moves. The page SETS a plan; only PB Start runs it."""
    return render_template(
        "blind.html", page="blind", track_m=config.TRACK_M,
        max_distance_m=config.BLIND_MAX_DISTANCE_M,
        max_rpm=int(config.BLIND_MAX_RPM),
        max_segments=config.BLIND_MAX_SEGMENTS,
        max_mps=round(config.BLIND_MAX_RPM * config.MPS_PER_RPM, 3),
        imu_enabled=config.IMU_ENABLED)


@app.get("/io")
def io():
    """Digital I/O lamps. Read-only: nothing here can energise an output."""
    return render_template(
        "io.html", page="io",
        di_names=config.DIO_DI_NAMES, do_names=config.DIO_DO_NAMES,
        dio_ip=f"{config.DIO_IP}:{config.DIO_PORT}",
        scan_hz=round(1.0 / config.DIO_SCAN_PERIOD_S))


@app.get("/alarms")
def alarms():
    """The event log and everything standing against the vehicle right now.

    Notably it cannot CLEAR anything: a latched fault is cleared at the panel
    with Reset, by somebody who can see the vehicle. The one control on the page
    is the service restart, which is the opposite of silencing an alarm - it
    de-energises the drives, is refused while armed, and throws the log away
    rather than tidying it.
    """
    return render_template("alarms.html", page="alarms",
                           max_events=events.MAX_EVENTS, unit=SERVICE_UNIT)


@app.get("/params")
def params():
    """Every tunable the vehicle is running on. Read-only, and unavoidably so:
    there is no endpoint here that writes a profile, because a parameter that
    can be changed from a browser is a parameter that can be changed while
    somebody is standing next to the vehicle. A profile is edited in the JSON
    and the service is restarted, which is also what makes the value on this
    page the value the bus thread is actually using.

    The content is generated from config's schema rather than listed here - see
    config.describe(). A hand-written list would be a second copy of the
    profile format, and the copy that goes stale is the one on the screen.
    """
    return render_template(
        "params.html", page="params",
        sections=config.describe(),
        profile=config.PROFILE_NAME, path=config.PROFILE_PATH_LOADED,
        env_var=config.PROFILE_ENV_VAR,
        enabled=[(name, config.__dict__[f"{name.upper()}_ENABLED"])
                 for name in ("dio", "panel", "rfid", "monitor")])


# ---- api ------------------------------------------------------------------

@app.get("/api/state")
def api_state():
    """Vehicle state. Read-only - polling this cannot keep anything alive.

    *** It used to carry an opt-in heartbeat (?hb=1) that fed the auto
    watchdog. *** That existed because a tape-following run was LATCHED motion:
    something had to prove an operator was still watching, and only the page
    driving the run was allowed to claim it.

    Nothing here is latched any more. Manual jogging is HELD - the browser
    re-POSTs /api/drive about every 100 ms and the bus thread zeroes the
    setpoint if it misses several - so the drive path carries its own liveness
    and this endpoint needs none.

    *** When an autonomous mode returns, the opt-in property has to come back
    with it. *** Making the heartbeat unconditional is the tempting shortcut and
    it is the bug that was already fixed once: a monitoring page on a second
    screen then holds a run alive after the driving page is closed.
    """
    return jsonify(ctl.snapshot())


@app.post("/api/preflight")
def api_preflight():
    try:
        return jsonify(ctl.submit("preflight"))
    except Exception as e:
        return _fail(e)


# There is deliberately no /api/arm.
#
# The web app may not put the vehicle into motion by any route except the manual
# jog arrows below, and only while it is already armed in manual. Arming belongs
# to the physical panel - the selector resting in MANUAL is the arm command - so
# a browser left open on a bench cannot move a 150 kg vehicle.
#
# *** Keep this property when navigation lands. *** The temptation will be a
# "go" button on a page; the panel owning every entry into motion is what makes
# a browser incapable of starting the vehicle.
#
# What remains here only ever STOPS: disarm de-energises, stop zeroes the
# setpoint, and drive is a dead-man that the operator must keep holding.
@app.post("/api/disarm")
def api_disarm():
    try:
        return jsonify(ctl.submit("disarm"))
    except Exception as e:
        return _fail(e)


# ---- restarting this service ----------------------------------------------
#
# The controller restarts itself by DYING, not by asking systemd to restart it.
# `systemctl restart` is not available: this process runs as an unprivileged
# user, `sudo -n` wants a password and polkit answers "authorization requires
# authentication" for org.freedesktop.systemd1.manage-units. A web request has
# no terminal to answer either with.
#
# So it de-energises the drives and exits non-zero, and the unit's own
# Restart=on-failure brings it back after RestartSec. That inverts the usual
# reading of an exit code - a deliberate restart is recorded in the journal as a
# failure - and it is the price of needing no privilege at all.
#
# *** THIS DEPENDS ON THE UNIT'S RESTART POLICY. *** Without it, exiting is not
# a restart, it is a shutdown of the only thing that can stop the vehicle. So
# the policy is READ at request time rather than assumed: a unit edited to
# Restart=no, or a process started by hand from a shell, refuses instead.
SERVICE_UNIT = "agv_controller.service"


def _restart_policy():
    """(policy, seconds) from systemd, or (None, None) if it cannot be known.

    `systemctl show` is a read and needs no privilege - unlike `systemctl
    restart`, which is the whole reason this route works the way it does.
    """
    try:
        out = subprocess.run(
            ["systemctl", "show", "-p", "Restart", "-p", "RestartUSec",
             SERVICE_UNIT],
            capture_output=True, text=True, timeout=4.0)
    except Exception:                       # noqa: BLE001 - no systemd, no policy
        return None, None
    if out.returncode != 0:
        return None, None
    fields = dict(line.split("=", 1) for line in out.stdout.splitlines()
                  if "=" in line)
    return fields.get("Restart"), fields.get("RestartUSec")


@app.post("/api/restart")
def api_restart():
    """Restart the controller process. De-energises the drives on the way out.

    Refused while armed: this is a stop, and a stop the operator did not ask for
    is exactly what the panel's Reset exists to make deliberate. Disarm first,
    which is itself a de-energise, and then the restart costs nothing that was
    not already given up.
    """
    snap = ctl.snapshot()
    if snap.get("armed"):
        return _fail("the vehicle is armed - disarm before restarting the "
                     "controller, so the stop is deliberate rather than a "
                     "side effect")

    policy, usec = _restart_policy()
    if policy not in ("on-failure", "always"):
        return _fail(
            f"this process would not come back: {SERVICE_UNIT} reports "
            f"Restart={policy or 'unknown'}. The restart button relies on "
            f"systemd restarting a process that exits non-zero, because it "
            f"cannot call systemctl itself.")

    events.warn("controller restart requested from /alarms - de-energising")

    def _bye():
        # Off the request thread, so the response reaches the browser before the
        # process stops answering. shutdown() joins the bus thread, whose finally
        # runs _do_disarm() - that is what actually de-energises.
        import time
        time.sleep(0.4)
        try:
            ctl.shutdown()
        finally:
            # _exit, not sys.exit: this is not the main thread, so an exception
            # would simply end this thread and leave the process running with a
            # shut-down controller - the one outcome worse than either restarting
            # or not.
            os._exit(1)

    threading.Thread(target=_bye, name="restart", daemon=True).start()
    return jsonify({"ok": True, "restart_usec": usec, "policy": policy})


@app.post("/api/drive")
def api_drive():
    direction = (request.json or {}).get("dir", "stop")
    try:
        ctl.drive(direction)
    except Exception as e:
        return _fail(e)
    return jsonify({"ok": True, "dir": direction})


@app.post("/api/stop")
def api_stop():
    ctl.halt()
    return jsonify({"ok": True})


# A blind-run PLAN only. Storing one moves nothing; PB Start runs it.
@app.post("/api/blind/plan")
def api_blind_plan():
    try:
        return jsonify({"ok": True, "plan": ctl.set_blind_plan(request.json or {})})
    except Exception as e:
        return _fail(e)


@app.post("/api/blind/clear")
def api_blind_clear():
    try:
        ctl.clear_blind_plan()
    except Exception as e:
        return _fail(e)
    return jsonify({"ok": True})


@app.get("/api/events")
def api_events():
    """Operator events newer than ?since=<seq>. since=0 returns the whole ring.

    Polled only when /api/state reports an event_seq ahead of what the page
    holds, so a quiet vehicle costs nothing.
    """
    try:
        since = int(request.args.get("since", 0))
    except (TypeError, ValueError):
        since = 0
    latest, items = events.since(since)
    return jsonify({"seq": latest, "events": items})


@app.get("/api/can")
def api_can():
    """Drive monitoring detail. Read-only.

    Split from /api/state so a monitoring page can poll as often as it likes
    and carry far more detail than a status line.
    """
    snap = ctl.snapshot()
    return jsonify({
        "can": snap.get("can"),
        "nodes": snap.get("nodes"),
        "health": snap.get("health"),
        "imu": snap.get("imu"),
        "connected": snap.get("connected"),
        "how": snap.get("how"),
        "error": snap.get("error"),
    })


@app.get("/api/config")
def api_config():
    return jsonify({
        "profile": config.PROFILE_NAME,
        "profile_path": config.PROFILE_PATH_LOADED,
        "full_rpm": config.MANUAL_FULL_RPM, "half_rpm": config.MANUAL_HALF_RPM,
        "driver_ramp": config.RAMP,   # 6083h/6084h, per mode
        "manual_watchdog_ms": int(config.MANUAL_WATCHDOG_S * 1000),
        "invert_left": config.INVERT_LEFT, "invert_right": config.INVERT_RIGHT,
        "use_rpdo": config.CAN_USE_RPDO,
        "table": {d: motion.velocities(d) for d in motion.PAD},
    })


def main():
    """Entry point for both the shell and the agv_controller systemd unit.

    *** THIS MOVES HARDWARE, and it has no authentication. *** Anyone who can
    reach the port can drive the AGV. Keep it on a trusted network.

    The CAN bus is opened once, on a dedicated thread, when the server starts -
    see canworker.py for why nothing else is allowed to touch it.
    """
    import argparse

    ap = argparse.ArgumentParser(
        description=main.__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--debug", action="store_true",
                    help="Flask autoreload. Off by default: a reload would open "
                         "can0 twice and orphan an armed driver.")
    args = ap.parse_args()

    try:
        ctl.start()
    except ownerlock.OwnerBusy as e:
        # The ROS stack (drive_node / panel_node) holds can0 or the DIO island.
        # Refuse cleanly rather than become a second owner - see core/ownerlock.py.
        print(f"refusing to start: {e}. Stop the AMR service first.", file=sys.stderr)
        return 1
    print(f"AGV web UI on http://{args.host}:{args.port}/  "
          f"(manual: /manual)")
    try:
        app.run(host=args.host, port=args.port, debug=args.debug,
                use_reloader=args.debug, threaded=True)
    finally:
        # Reached via KeyboardInterrupt, which is why the unit sends SIGINT
        # rather than the default SIGTERM - see the service file.
        print("shutting down - de-energising motors")
        ctl.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
