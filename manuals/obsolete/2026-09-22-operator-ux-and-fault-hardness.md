# Improvement plan: operator friendliness and fault hardness

Date 2026-09-22 · branch `slam-roadmap` · status: proposal, nothing implemented.

Two priorities, one root cause. Today every state, reason and alarm in the stack is
written for the developer. The operator's question is only "does it work, and if not,
what do I press?". The code answers a different question ("cross-track +0.14 m exceeds
0.10 m at 3.2 m along", "MUX_ACK_TIMEOUT", "8130h"). Hardness has the same gap from the
other side: when something dies, the *operator surface* often does not notice or
cannot recover it, even where the software is internally sound.

## 1. What the audit found (facts, from the code on 2026-09-22)

Operator surface
- The landing page is Status with a 10-tab engineering nav (Monitor, I/O, Params,
  Commission, Routes, Maps …). Nothing separates an operator from an engineer.
- "Standing alarms" are computed client-side in `alarms.html` from `/api/state`; the
  rail, Status and Run pages each re-derive their own wording. No single source.
- Every reason string is free text from the node that produced it. Only the supervisor
  has fault *codes*. Executor, localisation monitor, mux, line layer and mapping session
  have text only, so nothing can be mapped to an action.
- Drive alarms reach the operator as CANopen hex (`8130h`) with the meaning in RUNBOOK §4.

Events
- `/amr/events` is published by three nodes only: supervisor (MODE_CHANGE), panel_node
  (PANEL), drive_node (PP_* blind-run events only). **No event for a drive alarm, an
  executor FAULT/BLOCKED, localisation LOST, a mux inhibit, a line-layer hold or a
  mapping-session failure.** The Alarms page "Events" list therefore misses most of
  what stops the vehicle.
- The event ring lives in the web adapter's memory (300 entries) and is lost on every
  service restart. There is no on-disk event log, so "it stopped yesterday" has only
  `journalctl` and `~/.amr/logs/*.log` as evidence, both engineer-only.

Hardness
- The web server is an "optional group": if it exits, the supervisor logs a warning and
  does **not** restart it (`supervisor_node.py:874`). The operator loses the UI with no
  recovery short of `systemctl restart`.
- A dead base is "restart required" (correct: it needs the drives re-armed), but the
  operator sees a FAULT with an engineering sentence and has no button.
- `amr.service` has `Restart=on-failure` but no `WatchdogSec`; a hung supervisor
  (alive process, dead loop) is never restarted.
- The web has no Flask error handler and no JS global error handler: a bug in a page
  script leaves buttons silently dead; a bug in a handler returns an HTML stack trace.

Things already right and to keep: owner-published diagnostics; supervisor recovery
transaction; `Restart=on-failure`; bounded holds with auto-resume; all safety semantics
(deny-listed drive resets, motion only from the panel). Nothing in this plan touches
gains, lease rules or the port (`autopilot.py`, `branch.py`).

## 2. Design rule for everything below

One catalogue, two audiences. Every stop, hold or fault carries a **code**. The
catalogue maps a code to what the operator sees; the original text stays as the
engineer's detail. The operator surface never shows a string the catalogue does not know.

```
code            → severity, clears_by, operator_title, operator_action, engineer_hint
FIELD_BLOCKED   → info,  auto,      "Stopped: something is in the safety field",
                                    "Clear the area. The vehicle resumes by itself.", ...
DRIVE_ALARM     → error, power_cycle,"Drive alarm", "Switch the drives off and on, then press Recover.", ...
LOC_LOST        → error, operator,  "Vehicle lost its position",
                                    "Set the initial pose on the Run page and confirm the scans align.", ...
```

`clears_by` ∈ {auto, start_button, ack, recover, power_cycle, restart_service, engineer}.
That single field drives which button the Home page shows.

## 3. Phases

### Phase 0 — Alarm catalogue and codes everywhere (foundation)

1. `agv_core/alarms.py`: the catalogue as data (dataclass rows, ~40 codes). A test
   asserts every code used anywhere in `src/` exists in the catalogue, and that every
   catalogue code appears in RUNBOOK §4 (or the runbook section is generated from it).
2. Codes at the source. Append-only message fields (one colcon build):
   - `RunState.fault_code`, `LocalizationState.code`, `LineState.code`,
     `MuxState.code`. `reason` stays the engineer detail.
   - Executor: `_fault(code, detail)` at all 17 sites; `_hold(cause, ...)` already has a
     cause, map cause → code (`field`→FIELD_BLOCKED, `estop`→ESTOP, `controller`→
     PATH_BLOCKED, `pending`→WAITING_FOR_PREREQ, `drives`/`rate`/`track` in LINE).
   - Localisation readiness `_lose(code, detail)`: LOC_SCAN_MISMATCH, LOC_COV_GREW,
     LOC_STREAM_STALE, LOC_JUMP, LOC_GATE_FAILED.
   - Mux `gating.select()` reasons → code enum (NO_SOURCE, PANEL_STALE, INHIBITED_GEN,
     SOURCE_TIMED_OUT, NOT_LEASED …).
   - Drive node: decode the CANopen emergency/error code through a small table
     (`canmon`) → DRIVE_ALARM with `detail="8130h heartbeat lost (left)"`.
3. Events from every owner on every transition (bounded, edges only, as Event.msg
   already demands): executor (state change + fault code), localisation monitor
   (state change), mux (source change / inhibit edge), line node (state + hold cause),
   mapping session (save ok/failed), drive node (alarm set/cleared, operational
   edge). Text = catalogue title + detail.
4. `/api/alarms` server-side: the standing list computed once in the adapter from state
   + catalogue (`[{code, level, title, action, clears_by, detail, since}]`). The rail,
   Status, Run and Alarms pages consume it; the client-side derivation in `alarms.html`
   is deleted. Tests move from JS harness to `test_server.py`.

Deliverable: nothing looks different yet, but every surface has a code and an action.

### Phase 1 — Operator home and role split

1. Role: `data-role="operator|engineer"` next to the existing `data-ui`. Default on the
   panel (coarse pointer) = operator. Engineer via `?role=engineer` + a 4-digit PIN from
   the profile (`web_engineer_pin`), stored in localStorage; a button in the header
   returns to operator. Operator nav = **Home · Run · Alarms**. Everything else hidden
   (still reachable by URL for you; the PIN is a mistake guard, not security).
2. `home.html`, new landing page for the operator role, fits 1280×720 at `l`:
   - One big line: the vehicle's state in catalogue words
     (READY · RUNNING mission X, step 3/9 · STOPPED (will resume) · STOPPED (needs you)
     · NOT READY · NEEDS SERVICE).
   - One sentence: the top standing alarm's `operator_action`.
   - **One primary button** chosen from `clears_by` (Ack / Recover / Set pose / none),
     plus Pause and Abort always visible when a run exists. Nothing else.
   - A "more" panel (collapsed) with the standing list.
   - The mission selector and Load only when state is IDLE with no run.
3. Run page: the localisation tab gets a guided 3-step strip
   (1 pick map → 2 set pose (or "survey mark") → 3 confirm). Steps grey out when done.
   Current buttons stay underneath for the engineer role.
4. Wording pass on every reason the operator can see (catalogue only). Engineer role
   keeps the current text plus the code in a small mono span.
5. Alarms page for the operator: standing list with title/action; history with
   "what happened / when / how it cleared". Engineer role: today's page plus codes.

### Phase 2 — Evidence that survives (post-mortem without an engineer)

1. Event log on disk: the web adapter appends every event to
   `~/.amr/logs/events.jsonl` (one line per event, size-rotated at 5 MB × 5). On start
   it reloads the tail into the ring, so the Alarms history survives a restart.
2. "Save report" button (both roles) → `/api/report` builds a zip in `~/.amr/reports/`:
   events tail, `journalctl -u amr.service --since -2h` (needs a sudoers line for
   `journalctl`, the user installs), profile, active map/route ids, RunState snapshot,
   diagnostics snapshot. The Alarms page lists the reports; the operator hands you the
   file name. Reports are what "verbose" should mean: verbose on disk, short on screen.
3. Run summary event at DONE/ABORT/FAULT: mission, revision, duration, distance, peak
   cross-track, holds by cause. One line the operator can read back to you.

### Phase 3 — Hardness: stay up, or fail loudly with the right button

1. **Web group restart.** In `_reap`, a `web` (and `foxglove`) exit is respawned with
   a bounded backoff (5 s, 10 s, 20 s, then give up with WEB_DOWN event). The web is the
   operator's only window; today it is the least protected process.
2. **systemd watchdog.** `Type=notify`, `WatchdogSec=30`; the supervisor calls
   `sd_notify("READY=1")` after boot and `WATCHDOG=1` each loop tick (python-systemd
   or a 10-line socket writer). A hung loop is restarted like a crash. The stop path is
   already bounded (55 s), so motion inhibition holds through the restart.
3. **Base death → one button.** Keep "restart required" semantically (drives must be
   re-armed) but give the operator the action: Home shows "NEEDS SERVICE: press Restart
   after switching the drives off and on"; `/api/service/restart` triggers
   `systemctl restart amr.service` through a sudoers line (user installs). The button
   is refused while any wheel moves or a run is not IDLE/FAULT.
4. **Error boundaries in the web.** Flask `@app.errorhandler(Exception)` → JSON
   `{error: "internal", code: "WEB_BUG", ref: <uuid>}` + an event + full trace in the
   web log. `window.onerror` / `unhandledrejection` in `amr.js` → red banner "Page
   error, reload" with the ref, and the rail keeps polling. A JS bug no longer leaves
   silent dead buttons.
5. **Boot report.** One event at the end of boot (or budget): "Base ready" or
   "Not ready: drives, panel" from `base_missing`, with each missing item mapped to a
   catalogue action ("drives: press the safety reset on the cabinet" — the 70 s case
   from 2026-09-22 becomes a sentence, not a FAULT screen).
6. **Fault-injection suite** (opt-in sim tests, mechanism only per the memory rule):
   kill each process group in turn (base, layer, web, foxglove), stall the panel,
   freeze `/scan`, and assert three things: the expected code appears on `/api/alarms`
   within 2 s, the event is on disk, and the documented button clears it. This is the
   regression guard for everything above.

### Phase 4 — Operator documentation generated, not written

- `RUNBOOK` §4 "Fault recovery" is regenerated from the catalogue (`python3 -m
  agv_core.alarms --md`), so the doc and the screen never diverge. A one-page laminated
  card (same generator, `--card`) with the 8 most likely codes and their actions.

## 4. Order and size

| Phase | Depends on | Touches | Size |
|---|---|---|---|
| 0 catalogue + codes + events + `/api/alarms` | – | agv_core, 4 msgs, 6 nodes, adapter, tests | ~3 days |
| 1 home + role + wording | 0 | amr_web templates/js/css, profile key | ~2 days |
| 2 event log + report + run summary | 0 | adapter, server, executor, sudoers | ~1 day |
| 3 hardness (web restart, watchdog, restart button, error boundaries, boot report, fault injection) | 0 for codes; otherwise independent | supervisor, unit file, server, amr.js, sim tests | ~2 days |
| 4 generated docs | 0 | agv_core, RUNBOOK | ½ day |

Phase 3 items 1, 2 and 4 can go first if you want hardness before UX: they need no
catalogue. Phase 0 is the one message rebuild in the plan; everything after it is
symlink-install code plus a service restart.

## 5. Decisions needed from you

1. Codes as new message fields (one rebuild, clean) or as a `CODE: detail` prefix in the
   existing `reason` strings (no rebuild, parsing). **Recommend fields.**
2. Engineer gate: PIN from the profile, or just `?role=engineer` with no PIN.
   **Recommend PIN**; operators tap things.
3. Operator-triggered service restart button (needs a sudoers line) — yes or no.
   **Recommend yes**, refused unless stopped; it is what you would do by SSH anyway.
4. Web auto-restart backoff limits (proposed 5/10/20 s then give up).

## 6. Vehicle checks after each phase

- Phase 0/1: walk the acceptance file's fault steps (field block, estop, selector to
  MANUAL, lost pose, drive power off) and read the Home line aloud each time; PASS when
  a person who has not read the runbook can say what to press.
- Phase 3: pull the web process (`kill`) and the layer; watch Home recover; pull the
  supervisor's loop (SIGSTOP) and confirm systemd restarts it within 60 s with the
  drives disarmed throughout.
