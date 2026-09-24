"""The web tier: the watchdog opt-in and the read-only monitor page."""
import math
import re
import os
import pathlib
import struct
import sys
import threading

from helpers import FAIL, ROOT, check

import config
import kinematics
import motion

def test_no_page_can_hold_the_vehicle_alive():
    """*** Polling /api/state must not keep anything running. ***

    It used to carry an opt-in heartbeat (?hb=1) feeding the auto watchdog,
    because a tape-following run was LATCHED motion and something had to prove
    an operator was still watching. Only the page driving the run was allowed to
    claim it - a monitor page on a second screen holding a run open was a real
    bug that was fixed once.

    Nothing is latched now. Manual jogging is HELD, by the /api/drive re-POST,
    which carries its own liveness. So the heartbeat is gone entirely - and this
    pins that it did not come back as an unconditional one, which is the shape
    the original bug had.
    """
    import server as webapp   # app/server.py; see the note on the rename
    print("\nno page can hold the vehicle alive")

    check("the Controller has no keepalive to call",
          not hasattr(webapp.ctl, "keepalive"))
    src = (ROOT / "app" / "server.py").read_text(encoding="utf-8")
    # The CODE, not the prose - the docstring names ?hb=1 to explain what was
    # removed and why the opt-in property mattered, and that is worth keeping.
    body = src[src.index("def api_state"):src.index("def api_preflight")]
    check("/api/state reads no heartbeat argument",
          "request.args" not in body and "ctl.keepalive" not in body)
    check("no page claims a heartbeat any more",
          not any("CLAIM_HEARTBEAT" in (ROOT / "app" / "static" / f).read_text(encoding="utf-8")
                  for f in ("common.js", "monitor.js", "manual.js")))
    # The property that must survive if an autonomous mode returns.
    check("the opt-in rule is recorded for whoever adds navigation",
          "opt-in" in src[src.index("def api_state"):src.index("def api_preflight")])

def test_web_cannot_start_the_vehicle():
    """The web app may produce motion by exactly one route: the manual arrows.

    Arming and starting an auto run belong to the physical panel, so the routes
    that did those are GONE rather than guarded - a stale browser tab gets a 404
    instead of a moving vehicle.
    """
    import server as webapp   # app/server.py; see the note on the rename
    print("\nthe web app cannot start the vehicle")
    c = webapp.app.test_client()

    for route in ("/api/arm", "/api/auto/run"):
        check(f"POST {route} is gone", c.post(route, json={}).status_code == 404,
              str(c.post(route, json={}).status_code))

    # What survives only ever stops, or is a dead-man the operator must hold.
    for route in ("/api/disarm", "/api/stop", "/api/drive"):
        check(f"POST {route} still resolves",
              c.post(route, json={"dir": "stop"}).status_code != 404)

    # /api/restart belongs in that group - it de-energises and ends the process,
    # so the worst it can do is stop the vehicle - but it is checked through the
    # URL map rather than by calling it. On a machine where the unit exists and
    # nothing is armed, both guards pass and the handler does exactly what it
    # says: os._exit(1), taking this test run with it.
    rules = {r.rule: r.methods for r in webapp.app.url_map.iter_rules()}
    check("POST /api/restart is registered", "POST" in rules.get("/api/restart", set()))
    check("...and is POST-only, so a link cannot trigger it",
          "GET" not in rules.get("/api/restart", set()))

    src = (ROOT / "app" / "server.py").read_text(encoding="utf-8")
    check("no arm route is defined at all", '"/api/arm"' not in src)
    check("no auto-run route is defined at all", '"/api/auto/run"' not in src)

    check("the retired auto page is not served", c.get("/auto").status_code == 404)
    # The lidar page went with drivers/lidar.py: the ROS driver owns the
    # scanner's UDP port, and a page fed by a listener that cannot bind is a
    # page that always reads "stale".
    check("the retired lidar page is not served", c.get("/lidar").status_code == 404)
    check("...and neither is its cloud endpoint", c.get("/api/lidar").status_code == 404)

    # The manual page keeps DISARM - removing a stop path is the wrong
    # direction - but must not offer ARM.
    man = c.get("/manual").data.decode()
    check("the manual page keeps DISARM", 'id="disarm"' in man)
    check("...but cannot arm", 'id="arm"' not in man)
    check("...and says arming happens at the panel", "Arm at the panel" in man)

    # A dangling handler for a deleted button is a ReferenceError that kills the
    # whole onState callback and silently freezes the page's telemetry. The auto
    # page it used to guard is gone, so the surviving scripts carry the rule.
    for name in ("manual.js", "common.js"):
        js = (ROOT / "app" / "static" / name).read_text(encoding="utf-8")
        for dead in ("runBtn", "armBtn", "/api/arm", "/api/auto/run"):
            check(f"{name} has no reference to {dead}", dead not in js)


def test_manual_page_shows_the_rfid_tag():
    """Reading a tag's value means jogging over it, which happens on /manual."""
    import server as webapp   # app/server.py; see the note on the rename
    print("\nthe manual page shows the station tag")
    c = webapp.app.test_client()

    man = c.get("/manual").get_data(as_text=True)
    for el in ('id="r-tag"', 'id="r-age"', 'id="r-count"', 'id="r-link"'):
        check(f"/manual carries {el}", el in man)
    check("...and says what it is", "Station tags" in man)

    # One definition, not two. A copy in both files is a copy that gets fixed
    # in one of them - which is the whole reason showRfid lives in common.js
    # rather than being duplicated per page.
    common = (ROOT / "app" / "static" / "common.js").read_text(encoding="utf-8")
    check("showRfid is defined in common.js", "function showRfid" in common)
    check("common.js calls it from the shared poll", "showRfid(s.rfid" in common)
    check("...and nothing references the retired branch readout",
          not any("r-branch" in (ROOT / "app" / "static" / f).read_text(encoding="utf-8")
                  for f in ("common.js", "manual.js", "monitor.js")))

    check("/manual still cannot arm", 'id="arm"' not in man)
    check("/manual still keeps DISARM", 'id="disarm"' in man)


def test_the_shared_rail_is_on_every_page():
    """Battery, state and alarm travel with every page.

    And the three diagnostics that used to sit there do NOT: loop timing,
    frames-per-tick and the watchdog countdown are for somebody tuning the
    control loop, and they were taking four of the six rail slots on a screen an
    operator reads at arm's length.
    """
    import server as webapp   # app/server.py; see the note on the rename
    print("\nthe shared rail carries what an operator needs")
    c = webapp.app.test_client()

    OPERATOR = ('id="batt"', 'id="vstate"', 'id="alarm"')
    DIAGNOSTIC = ('id="loop-work"', 'id="loop-frames"', 'id="wd"')

    for page in ("/manual", "/io", "/alarms", "/params"):
        body = c.get(page).get_data(as_text=True)
        for el in OPERATOR:
            check(f"{page} carries {el}", el in body)
        for el in DIAGNOSTIC:
            check(f"{page} does NOT carry {el}", el not in body)

    mon = c.get("/monitor").get_data(as_text=True)
    for el in OPERATOR + DIAGNOSTIC:
        check(f"/monitor carries {el}", el in mon)

    # The tiles moved, so every write to them has to be guarded. An unguarded
    # getElementById(...).textContent for a tile that is not on this page throws
    # inside poll() and silently freezes ALL of the page's telemetry.
    common = (ROOT / "app" / "static" / "common.js").read_text(encoding="utf-8")
    check("common.js writes the watchdog through a guarded setter",
          "setText('wd'" in common
          and "document.getElementById('wd').textContent" not in common)
    check("...and the setpoint too", "setText('setpoint'" in common)
    check("renderLoopHealth returns early when its tiles are absent",
          "if (!work || !lp) return;" in common)

    # The verdicts are computed once, on the server. Four pages each deciding
    # what "alarm" means is four chances for one of them to say all is well.
    worker = (ROOT / "canworker.py").read_text(encoding="utf-8")
    check("the alarm verdict is computed server-side", "def _alarm(" in worker)
    check("the battery summary is computed server-side", "def _battery(" in worker)
    check("both reach the browser through the state snapshot",
          '"alarm": self._alarm(' in worker and '"battery": _battery(' in worker)


def test_alarms_page_records_but_cannot_clear():
    """Colour-coded log plus what is standing. It must not be able to silence."""
    import server as webapp   # app/server.py; see the note on the rename
    print("\nthe alarms page records and cannot clear")
    c = webapp.app.test_client()

    body = c.get("/alarms").get_data(as_text=True)
    check("the alarms page renders", "Standing" in body and "Log" in body)
    check("it is reachable from every page",
          'href="/alarms"' in c.get("/manual").get_data(as_text=True))
    # Said on the standing-fault row itself rather than in a banner at the top -
    # the banner was removed by request, and the row is where somebody reading
    # about a live fault is actually looking.
    check("a latched fault says it is cleared at the panel",
          "clear with panel Reset"
          in (ROOT / "app" / "static" / "alarms.js").read_text(encoding="utf-8"))

    # The one thing this page must never do. An acknowledge button on a browser
    # silences an alarm for somebody standing somewhere else.
    #
    # This used to be asserted as "no <button> anywhere on the page", which was
    # broader than the rule it was protecting - the page now carries a service
    # restart, which is the opposite of silencing: it de-energises the drives
    # and throws the log away rather than tidying it. So the checks name the
    # forbidden thing instead of forbidding all controls.
    js = (ROOT / "app" / "static" / "alarms.js").read_text(encoding="utf-8")
    srv = (ROOT / "app" / "server.py").read_text(encoding="utf-8")
    for banned in ("id=\"ack\"", "id=\"clear\"", "acknowledge", "Acknowledge"):
        check(f"no {banned} control on the page", banned not in body)
    check("the only endpoint the page POSTs to is the restart",
          [c for c in re.findall(r"api\('([^']+)'", js)] == ["/api/restart"],
          str(re.findall(r"api\('([^']+)'", js)))
    check("there is no endpoint that clears a fault or an alarm",
          "/api/clear" not in srv and "/api/ack" not in srv)
    check("events.clear is not reachable from the web",
          "clear" not in srv.split("def api_events")[1][:400])
    # The restart is a stop, so it must be refused while the vehicle is armed -
    # a control that de-energises on a whim is the hazard, not the button.
    check("the restart is refused while armed",
          'snap.get("armed")' in srv and "disarm before restarting" in srv)
    check("...and refuses when nothing would restart the process",
          '("on-failure", "always")' in srv)
    check("the page says it de-energises the drives",
          "de-energises the drives" in body)

    # Three levels, three colours, and the level is never carried by colour
    # alone - a colour-only scheme vanishes in a photograph of the screen,
    # which is how a fault usually reaches somebody who was not there.
    css = (ROOT / "app" / "static" / "app.css").read_text(encoding="utf-8")
    for cls in ("lv-info", "lv-warn", "lv-error"):
        check(f"{cls} is styled", f".{cls}" in css)
    check("the level is also printed as text", 'class="al-lv"' in js)
    check("colour is carried by a left bar, not a fill alone",
          "border-left-color:var(--stop)" in css
          and "border-left-color:var(--hazard)" in css)

    # Errors cannot be filtered away.
    check("info and warn are filterable", 'id="f-info"' in body and 'id="f-warn"' in body)
    check("errors are not", "disabled" in body and "always shown" in body)
    check("the filter always keeps errors", "e.level === 'error'" in js)

    # Event text is operator-visible and can contain anything an exception's
    # str() produced, so it must never be interpolated into HTML.
    check("event text is set through textContent, not innerHTML",
          ".textContent = shown[i].msg" in js)


def test_params_page_displays_and_cannot_edit():
    """The parameters page shows the whole tuning surface and can change none
    of it.

    Two failures it is built against. The first is the ordinary one: a value
    that can be edited from a browser is a value that can be edited while
    somebody is standing next to the vehicle, so there is no write endpoint to
    guard - there is no write endpoint at all. The second is quieter and is why
    the page is GENERATED from config's schema: a hand-written parameters page
    stops matching the profile the moment a key is added, and what it then
    shows is a value the vehicle is not running on.
    """
    import server as webapp   # app/server.py; see the note on the rename
    print("\nthe parameters page displays and cannot edit")
    c = webapp.app.test_client()

    body = c.get("/params").get_data(as_text=True)
    # The "display only" banner was removed by request. Read-onlyness is a fact
    # about the endpoints, not about a paragraph, and it is asserted as such a
    # few lines down - there is nothing to press and nothing to POST to.
    check("the params page renders", "Profile" in body and "pm-list" in body)
    check("it names the profile and the file it came from",
          config.PROFILE_NAME in body and config.PROFILE_PATH_LOADED in body)
    check("it is reachable from every page",
          'href="/params"' in c.get("/manual").get_data(as_text=True))

    # Nothing to press, nothing to POST, and no route that writes a profile.
    check("there are no controls", "<button" not in body)
    js = (ROOT / "app" / "static" / "params.js").read_text(encoding="utf-8")
    check("the page makes no POST", "api(" not in js)
    check("...and no request of any kind", "fetch(" not in js
          and "apiGet(" not in js)
    src = (ROOT / "app" / "server.py").read_text(encoding="utf-8")
    check("there is no endpoint that writes a profile",
          "/api/params" not in src and "config.load(" not in src)

    # The whole profile reaches the screen. Generated, so this cannot pass by
    # somebody having remembered to add a row.
    absent = [f"{sec}.{key}" for sec, fields in config._SCHEMA.items()
              for key, (const, _t) in fields.items() if const not in body]
    check("every parameter in the schema reaches the page", not absent,
          str(absent))
    check("derived values are shown apart, as underivable from the JSON",
          "derived" in body and "MPS_PER_RPM" in body
          and "cannot be edited" in body)

    # The prose is config.py's, parsed out of it. If it were copied into the
    # template there would be two of every explanation and one would rot.
    note = "Both drivers take a POSITIVE 60FFh to travel forward"
    tpl = (ROOT / "app" / "templates" / "params.html").read_text(encoding="utf-8")
    check("a tuning note reaches the page", note in body)
    check("...from config.py, not from the template",
          note in (ROOT / "config.py").read_text(encoding="utf-8") and note not in tpl)


def test_params_page_reads_speed_first():
    """Speed is the most-consulted and most-edited part of the profile, and it
    was spread across three sections. It is gathered into one at the top.

    The invariant that matters is not the order - it is that gathering a row
    MOVES it rather than copying it. A parameter printed in two places is a
    parameter that can be read as two parameters, and the one somebody edits
    will be the one the vehicle is not using.
    """
    import server as webapp   # app/server.py; see the note on the rename
    print("\nthe parameters page reads speed first")
    c = webapp.app.test_client()

    secs = config.describe()
    names = [s["name"] for s in secs]
    # The synthetic blocks lead; the schema's own sections follow in the order
    # _SECTION_ORDER names. Asserted separately so adding another synthetic
    # block does not look like the schema order breaking.
    check("the gathered blocks lead", names[0] == "speed", str(names[:3]))
    schema_order = [n for n in names if n in config._SCHEMA]
    check("then the geometry it runs on",
          schema_order[0] == "vehicle", str(schema_order[:3]))

    speed = next(s for s in secs if s["name"] == "speed")
    keys = [r["key"] for r in speed["rows"]]
    check("both manual jog speeds are there",
          "manual.full_rpm" in keys and "manual.half_ratio" in keys, str(keys))
    check("and the driver ramps that shape them",
          len([k for k in keys if k.startswith("drivers.ramp.")]) == 4)
    check("every gathered row names the section it lives in, so it can be found "
          "in the JSON", all("." in k for k in keys), str(keys))

    # The real guard. Not order - duplication.
    consts = [r["const"] for s in secs for r in s["rows"] if r["const"]]
    dupes = sorted({c for c in consts if consts.count(c) > 1})
    check("no parameter is printed twice", not dupes, str(dupes))

    # And the mirror of it: nothing may be lost on the way. Suppressing a row
    # from its home section and forgetting to gather it would be silent.
    shown = set(consts)
    missing = [f"{sec}.{key}" for sec, fields in config._SCHEMA.items()
               for key, (const, _t) in fields.items() if const not in shown]
    check("...and none is lost by being moved", not missing, str(missing))

    # A section emptied by the gathering must not leave a bare heading behind.
    check("an emptied section is dropped, not shown blank",
          all(s["rows"] for s in secs) and "manual" not in names, str(names))

    body = c.get("/params").get_data(as_text=True)
    # Headings, not bare words - "vehicle" appears in the intro prose above
    # every section, so a plain index() comparison passes for the wrong reason.
    import re as _re
    heads = _re.findall(r"<h2[^>]*>\s*([a-z ]+)", body)
    check("the page renders the gathered section first",
          heads and heads[0].strip() == "speed", str(heads[:3]))
    check("...and says where those keys actually live",
          "drivers.ramp.auto.accel" in body)


def test_params_does_not_list_retired_rules():
    """The station-tag rule table went with tape following.

    Two rules were loaded from the profile - the branch latch and the
    stop-until-Start tags - and both were route features on a fixed tape path.
    The RFID reader itself is NOT gone; what is gone is anything consuming a tag.
    """
    import server as webapp   # app/server.py; see the note on the rename
    print("\n/params no longer lists retired tag rules")

    body = webapp.app.test_client().get("/params").get_data(as_text=True)
    for gone in ("branch latch", "stop until start button", "auto_rpm",
                 "k_ratio", "line_loss_grace"):
        check(f"/params does not mention {gone}", gone not in body)
    check("the RFID section itself is still shown", "RFID_IP" in body)
    check("...and the profile still carries the reader",
          "192.168.1.200" in body)

def test_landing_page_is_manual_and_the_pill_says_armed():
    """The bare address lands on the only page that can drive.

    It used to land on /auto, which watched a tape-following run and claimed
    that run's heartbeat. Both are gone, so the landing page is /manual - and a
    redirect rather than a render, so the address bar names the page it shows.
    """
    import server as webapp   # app/server.py; see the note on the rename
    print("\nthe landing page and the armed pill")

    c = webapp.app.test_client()
    r = c.get("/")
    check("/ redirects rather than rendering", r.status_code in (301, 302),
          str(r.status_code))
    check("...to /manual", "/manual" in r.headers.get("Location", ""),
          r.headers.get("Location", ""))
    check("the retired auto page is gone", c.get("/auto").status_code == 404)
    check("...and so is its script",
          not (ROOT / "app" / "static" / "auto.js").exists())
    check("...and its template",
          not (ROOT / "app" / "templates" / "auto.html").exists())
    check("the nav no longer offers it",
          '/auto' not in (ROOT / "app" / "templates" / "base.html").read_text(encoding="utf-8"))

TESTS = [
    test_params_page_displays_and_cannot_edit,
    test_params_page_reads_speed_first,
    test_params_does_not_list_retired_rules,
    test_landing_page_is_manual_and_the_pill_says_armed,
    test_web_cannot_start_the_vehicle,
    test_no_page_can_hold_the_vehicle_alive,
    test_manual_page_shows_the_rfid_tag,
    test_the_shared_rail_is_on_every_page,
    test_alarms_page_records_but_cannot_clear,
]
